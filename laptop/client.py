"""WebSocket transport shared by the observer and laptop parking worker.
Requires aiohttp on the algorithm laptop; ordinary browser use needs no installation.
"""
import asyncio
import time
from aiohttp import ClientSession, WSMsgType


class VehicleClient:
    def __init__(self, url):
        self.url = url.rstrip('/') + '/ws'
        self.http = self.ws = self.reader = None
        self.seq = 0
        self.server_time = None
        self.received_at = 0
        self.state = self.map = None
        self.map_parts = None
        self.pending = {}
        self.owns_control = False
        self.hello = {}

    async def __aenter__(self):
        self.http = ClientSession()
        self.ws = await self.http.ws_connect(self.url, max_msg_size=16*1024*1024)
        self.reader = asyncio.create_task(self.receive())
        for _ in range(50):
            if self.server_time is not None:
                return self
            await asyncio.sleep(0.1)
        await self.__aexit__(None, None, None)
        raise TimeoutError('vehicle telemetry unavailable')

    async def receive(self):
        async for message in self.ws:
            if message.type != WSMsgType.TEXT:
                break
            payload = message.json()
            if payload['type'] == 'hello':
                self.hello = payload
            elif payload['type'] == 'state':
                self.state = payload
                self.server_time = payload['server_time']
                self.received_at = time.monotonic()
            elif payload['type'] == 'map':
                self.map = payload
            elif payload['type'] == 'map_chunk':
                if (self.map_parts is None or self.map_parts['version'] != payload['version']
                        or self.map_parts['received'] != payload['offset']):
                    self.map_parts = dict(version=payload['version'], received=0, data=[],
                                          width=payload['width'], height=payload['height'],
                                          resolution=payload['resolution'], origin=payload['origin'],
                                          frame=payload['frame'])
                if self.map_parts['received'] == payload['offset']:
                    self.map_parts['data'].extend(payload['data'])
                    self.map_parts['received'] += len(payload['data'])
                    if self.map_parts['received'] == payload['total']:
                        self.map = dict(self.map_parts, type='map')
                        self.map.pop('received')
                        self.map_parts = None
            elif payload['type'] == 'ack':
                future = self.pending.get(payload['seq'])
                if future and not future.done():
                    future.set_result(payload)
        for future in self.pending.values():
            if not future.done():
                future.set_exception(ConnectionError('vehicle disconnected'))

    async def command(self, kind, **values):
        if kind not in ('stop', 'estop') and time.monotonic() - self.received_at > 0.25:
            raise TimeoutError('fresh telemetry required')
        self.seq += 1
        sequence = self.seq
        future = asyncio.get_running_loop().create_future()
        self.pending[sequence] = future
        try:
            await self.ws.send_json({'v': 1, 'seq': sequence, 'type': kind,
                                     'server_time': self.server_time, **values})
            ack = await asyncio.wait_for(future, 25 if kind == 'save_map' else 1)
            if not ack['ok']:
                raise RuntimeError(ack['error'])
            if kind == 'claim':
                self.owns_control = True
            elif kind == 'release':
                self.owns_control = False
            return ack
        finally:
            self.pending.pop(sequence, None)

    async def drive(self, speed_m_s, steering_rad):
        # Call only after explicit claim / PI / mode / arm acknowledgement.
        return await self.command('drive', speed_m_s=speed_m_s, steering_rad=steering_rad)

    async def __aexit__(self, *args):
        try:
            if self.owns_control and self.ws and not self.ws.closed:
                await self.command('stop')
                await self.command('release')
        finally:
            if self.http:
                await self.http.close()
            if self.reader:
                self.reader.cancel()


async def observe(url):
    async with VehicleClient(url) as vehicle:
        for _ in range(10):
            print(vehicle.state)
            await asyncio.sleep(1)


if __name__ == '__main__':
    import sys
    asyncio.run(observe(sys.argv[1]))
