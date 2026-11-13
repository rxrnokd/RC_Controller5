"""One HTTP + WebSocket ROS node. No account service, broker, or rosbridge."""
import asyncio
import contextlib
import ipaddress
import json
import math
import re
import time
import uuid
from pathlib import Path

from aiohttp import web, WSMsgType
import rclpy
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from tf2_ros import Buffer, TransformListener, TransformException
from slam_toolbox.srv import SaveMap, SerializePoseGraph
from ament_index_python.packages import get_package_share_directory

from .nodes import Base, LATEST
from .core import dumps, number


def yaw(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))


class Bridge(Base):
    def __init__(self):
        super().__init__('web_bridge')
        self.requests = self.create_publisher(String, '/rc/request', 20)
        self.latest, self.times = {}, {}
        self.clients = {}
        self.grid, self.grid_id = None, 0
        self.map_error = None
        for topic, key in (('/rc/state', 'vehicle'), ('/rc/command', 'command'), ('/rc/odom_status', 'odometry_status')):
            self.create_subscription(String, topic, lambda msg, k=key: self.cache(k, json.loads(msg.data)), LATEST)
        self.create_subscription(String, '/rc/ack', self.ack, 20)
        self.create_subscription(Odometry, '/odom', self.odom, 5)
        self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.map, qos)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.save_client = self.create_client(SaveMap, '/slam_toolbox/save_map')
        self.serialize_client = self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
        self.map_busy = False

    def cache(self, key, value):
        self.latest[key], self.times[key] = value, time.monotonic()

    def ack(self, msg):
        ack = json.loads(msg.data)
        queue = self.clients.get(ack.get('session'))
        if queue is not None:
            if queue.full():
                # Never block ROS spinning for a slow browser.
                return
            queue.put_nowait(ack)

    def odom(self, msg):
        self.cache('odom', {'x': msg.pose.pose.position.x, 'y': msg.pose.pose.position.y,
                            'yaw': yaw(msg.pose.pose.orientation), 'speed_m_s': msg.twist.twist.linear.x,
                            'frame': msg.header.frame_id})

    def scan(self, msg):
        self.cache('scan', {'frame': msg.header.frame_id, 'angle_min': msg.angle_min,
                            'angle_increment': msg.angle_increment,
                            'ranges': [v if math.isfinite(v) and msg.range_min <= v <= msg.range_max else None for v in msg.ranges]})

    def map(self, msg):
        if msg.header.frame_id != 'map' or msg.info.width * msg.info.height > self.cfg['network']['max_map_cells']:
            self.map_error = 'map frame/size rejected'
            return
        if msg.info.resolution <= 0 or len(msg.data) != msg.info.width * msg.info.height:
            self.map_error = 'invalid occupancy grid'
            return
        self.grid_id += 1
        self.grid = {'type': 'map', 'version': self.grid_id, 'frame': 'map',
                     'width': msg.info.width, 'height': msg.info.height, 'resolution': msg.info.resolution,
                     'origin': [msg.info.origin.position.x, msg.info.origin.position.y, yaw(msg.info.origin.orientation)],
                     'data': list(msg.data)}
        self.map_error = None
        self.times['map'] = time.monotonic()

    def telemetry(self, session):
        now = time.monotonic()
        age = {k: now-v for k, v in self.times.items()}
        pose = None
        try:
            tf = self.buffer.lookup_transform('map', 'base_link', Time())
            tf_age = abs((self.get_clock().now() - Time.from_msg(tf.header.stamp)).nanoseconds / 1e9)
            if tf_age < 1.0 and age.get('odom', 999) < 0.6:
                pose = {'x': tf.transform.translation.x, 'y': tf.transform.translation.y,
                        'yaw': yaw(tf.transform.rotation), 'frame': 'map', 'tf_age_s': tf_age}
        except TransformException:
            pass
        odom_valid = (age.get('odometry_status', 999) < 0.6 and self.latest.get('odometry_status', {}).get('valid', False)
                      and age.get('vehicle', 999) < 0.6 and age.get('odom', 999) < 0.6)
        ready = bool(odom_valid and pose and self.grid and age.get('scan', 999) < 0.6)
        return {'v': 1, 'type': 'state', 'server_time': now, 'session': session,
                'data': self.latest, 'age_s': age, 'map_pose': pose if ready else None,
                'slam_ready': ready, 'map_error': self.map_error, 'auto_available': False}

    def request(self, session, payload):
        self.send(self.requests, dict(payload, session=session, received=time.monotonic()))

    async def save_map(self, session, payload):
        command = self.latest.get('command', {})
        if command.get('owner') != session or command.get('armed') or time.monotonic() - self.times.get('command', 0) > 0.5:
            raise ValueError('claim and disarm before saving map')
        if not self.telemetry(session)['slam_ready']:
            raise ValueError('SLAM/TF/odometry not ready')
        if self.map_busy:
            raise ValueError('map save already in progress')
        name = payload.get('name', '')
        if not re.fullmatch(r'[A-Za-z0-9_-]{1,64}', name):
            raise ValueError('map name: 1..64 letters, digits, underscore, hyphen')
        directory = Path(self.cfg['slam']['maps_dir']).resolve()
        directory.mkdir(parents=True, exist_ok=True)
        target = directory / name
        if any(directory.glob(name + '.*')):
            raise ValueError('map name exists; choose a new name')
        if not self.save_client.service_is_ready() or not self.serialize_client.service_is_ready():
            raise ValueError('SLAM save/serialize service unavailable')
        self.map_busy = True
        completed = []
        try:
            save = SaveMap.Request()
            save.name.data = str(target)
            serialize = SerializePoseGraph.Request()
            serialize.filename = str(target)
            for client, request, label in ((self.save_client, save, 'occupancy'), (self.serialize_client, serialize, 'posegraph')):
                future = client.call_async(request)
                deadline = time.monotonic() + 20
                while not future.done():
                    if time.monotonic() > deadline:
                        raise ValueError(f'{label} service timeout; files may exist, inspect maps directory')
                    await asyncio.sleep(0.02)
                response = future.result()
                if response is None or response.result != 0:
                    raise ValueError(f'{label} save failed; completed={completed}')
                completed.append(label)
            return {'name': name, 'saved': completed}
        finally:
            self.map_busy = False


async def serve(node):
    cfg = node.cfg['network']
    networks = [ipaddress.ip_network(c) for c in cfg['allowed_cidrs']]
    static = Path(get_package_share_directory('rc_car')) / 'web'

    @web.middleware
    async def local_only(request, handler):
        try:
            address = ipaddress.ip_address(request.remote or '')
            if address.version == 6 and address.ipv4_mapped:
                address = address.ipv4_mapped
            allowed = any(address in n for n in networks)
        except ValueError:
            allowed = False
        if not allowed:
            raise web.HTTPForbidden(text='Wi-Fi subnet is not allowed in vehicle.yaml')
        origin = request.headers.get('Origin')
        if origin and origin != f'http://{request.host}':
            raise web.HTTPForbidden(text='Cross-origin access rejected')
        response = await handler(request)
        response.headers['Cache-Control'] = 'no-store'
        response.headers['X-Content-Type-Options'] = 'nosniff'
        response.headers['Content-Security-Policy'] = "default-src 'self'; connect-src 'self'; style-src 'self'; script-src 'self'"
        return response

    async def asset(request):
        name = request.match_info.get('name', 'index.html')
        if name not in ('index.html', 'app.js', 'style.css'):
            raise web.HTTPNotFound()
        return web.FileResponse(static / name)

    async def websocket(request):
        if len(node.clients) >= cfg['max_clients']:
            raise web.HTTPServiceUnavailable(text='client limit')
        ws = web.WebSocketResponse(heartbeat=2, max_msg_size=4096)
        await ws.prepare(request)
        session = uuid.uuid4().hex  # routing/ownership ID, not an authentication token
        queue = asyncio.Queue(maxsize=32)
        node.clients[session] = queue
        seq, window, count = -1, time.monotonic(), 0

        async def sender():
            map_version, map_at = -1, 0.0
            hello = {'type': 'hello', 'v': 1, 'session': session,
                     'limits': {'pwm': node.cfg['hardware']['max_pwm'],
                                'rpm': [m['max_rpm'] or 0 for m in node.cfg['hardware']['motors']],
                                'steering': node.cfg['servo']['max_angle_rad'] or 0}}
            await ws.send_str(dumps(hello))
            while not ws.closed:
                while not queue.empty():
                    await asyncio.wait_for(ws.send_str(dumps(queue.get_nowait())), 0.3)
                await asyncio.wait_for(ws.send_str(dumps(node.telemetry(session))), 0.3)
                if node.grid and map_version != node.grid_id and time.monotonic() - map_at >= cfg['map_period_s']:
                    map_version, map_at = node.grid_id, time.monotonic()
                    await asyncio.wait_for(ws.send_str(dumps(node.grid)), 0.5)
                await asyncio.sleep(1 / cfg['state_hz'])

        async def run_sender():
            try:
                await sender()
            finally:
                node.request(session, {'type': 'release', 'seq': -1})
                await ws.close()

        task = asyncio.create_task(run_sender())
        save_task = None

        async def save(payload):
            try:
                result = await node.save_map(session, payload)
                reply = {'ok': True, **result}
            except Exception as exc:
                reply = {'ok': False, 'error': str(exc)}
            if not queue.full():
                queue.put_nowait(dict(type='ack', seq=payload['seq'], **reply))

        try:
            async for message in ws:
                if message.type != WSMsgType.TEXT:
                    break
                payload = {}
                try:
                    payload = json.loads(message.data, parse_constant=lambda s: (_ for _ in ()).throw(ValueError(s)))
                    if not isinstance(payload, dict):
                        raise ValueError('JSON object required')
                    now = time.monotonic()
                    if now - window >= 1:
                        window, count = now, 0
                    count += 1
                    if count > cfg['max_messages_per_s']:
                        raise ValueError('message rate exceeded')
                    if payload.get('v') != 1 or type(payload.get('seq')) is not int or payload['seq'] <= seq:
                        raise ValueError('protocol version/sequence rejected')
                    seq = payload['seq']
                    kind = payload.get('type')
                    if kind not in ('estop', 'stop'):
                        # Echo most recently observed server_time. No cross-machine clock sync needed.
                        number(payload.get('server_time'), 'stale server_time', now - cfg['max_command_age_s'], now + 0.01)
                    if kind == 'save_map':
                        if save_task and not save_task.done():
                            raise ValueError('save already pending')
                        save_task = asyncio.create_task(save(payload))
                    else:
                        node.request(session, payload)
                except (ValueError, TypeError, KeyError) as exc:
                    node.request(session, {'type': 'release', 'seq': seq})
                    if not queue.full():
                        queue.put_nowait({'type': 'ack', 'seq': payload.get('seq') if isinstance(payload, dict) else None, 'ok': False, 'error': str(exc)})
        finally:
            node.request(session, {'type': 'release', 'seq': seq + 1})
            node.clients.pop(session, None)
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError, Exception):
                await task
            # A save already handed to ROS may complete after disconnect. Do not claim cancellation.
        return ws

    app = web.Application(middlewares=[local_only])
    app.router.add_get('/', asset)
    app.router.add_get('/ws', websocket)
    app.router.add_get('/{name}', asset)
    runner = web.AppRunner(app)
    await runner.setup()
    await web.TCPSite(runner, cfg['bind'], cfg['port']).start()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            await asyncio.sleep(0.002)
    finally:
        await runner.cleanup()


def main():
    rclpy.init()
    node = Bridge()
    try:
        asyncio.run(serve(node))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
