"""Trailer geometry, swept footprint and Hybrid A*. SI units, yaw CCW, map frame."""
import heapq
import itertools
import math
import time
from dataclasses import dataclass
from typing import NamedTuple

import numpy as np
from scipy.ndimage import distance_transform_edt


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class Pose(NamedTuple):
    x: float
    y: float
    yaw: float
    beta: float  # car yaw minus trailer yaw


class Knot(NamedTuple):
    pose: Pose
    ds: float     # signed distance from previous knot
    steer: float


@dataclass(frozen=True)
class Rig:
    wheelbase: float
    steer: float
    slew: float
    hitch: float
    bar: float
    front: float
    back: float
    width: float
    tfront: float
    tback: float
    twidth: float
    beta: float
    speed: float
    accel: float
    margin: float
    pos_tol: float
    yaw_tol: float
    stopped: float
    deviation: float

    @classmethod
    def from_config(cls, config):
        p = config['parking']
        names = ('hitch_back_m drawbar_m car_front_m car_back_m car_width_m trailer_front_m '
                 'trailer_back_m trailer_width_m max_beta_rad max_speed_m_s max_accel_m_s2 '
                 'clearance_m pos_tol_m yaw_tol_rad stop_speed_m_s deviation_m').split()
        fields = {'wheelbase_m': config['geometry']['wheelbase_m'],
                  'max_angle_rad': config['servo']['max_angle_rad'], 'slew_rad_s': config['servo']['slew_rad_s'],
                  **{name: p[name] for name in names}}
        for key, value in fields.items():
            if type(value) not in (float, int) or not math.isfinite(value) or value < 0 or (value == 0 and key != 'hitch_back_m'):
                raise ValueError(f'missing/invalid calibration: {key}')
        result = cls(*fields.values())
        if not (result.steer < math.pi/2 and result.beta < math.pi/2 and result.stopped < result.speed):
            raise ValueError('invalid angle/speed limits')
        return result

    def axle(self, p):
        yaw = p.yaw - p.beta
        return (p.x-self.hitch*math.cos(p.yaw)-self.bar*math.cos(yaw),
                p.y-self.hitch*math.sin(p.yaw)-self.bar*math.sin(yaw), wrap(yaw))

    def goal_ok(self, p, goal):
        return (math.dist(p[:2], goal[:2]) <= self.pos_tol and
                math.dist(self.axle(p)[:2], self.axle(goal)[:2]) <= self.pos_tol and
                abs(wrap(p.yaw-goal.yaw)) <= self.yaw_tol and
                abs(wrap((p.yaw-p.beta)-(goal.yaw-goal.beta))) <= self.yaw_tol)

    def step(self, p, ds, steer):
        k = math.tan(steer)/self.wheelbase
        def beta_rate(b):
            return k - (math.sin(b)-self.hitch*k*math.cos(b))/self.bar
        mid = p.yaw + 0.5*ds*k
        beta_mid = p.beta + 0.5*ds*beta_rate(p.beta)
        return Pose(p.x+ds*math.cos(mid), p.y+ds*math.sin(mid), wrap(p.yaw+ds*k),
                    wrap(p.beta+ds*beta_rate(beta_mid)))


class World:
    """Cover both rectangles and coupling with discs. Unknown/outside are occupied."""
    def __init__(self, msg, rig, cancel=None):
        self.r = rig
        self.cancel = cancel
        self.res = float(msg['resolution'])
        self.origin = msg['origin']
        if not (self.res > 0 and math.isfinite(self.res) and all(map(math.isfinite, self.origin))):
            raise ValueError('invalid map geometry')
        if msg.get('frame', 'map') != 'map':
            raise ValueError('expected map frame')
        grid = np.asarray(msg['data']).reshape(msg['height'], msg['width'])
        self.h, self.w = grid.shape
        self.dist = distance_transform_edt(np.pad(grid == 0, 1))[1:-1, 1:-1] * self.res
        self.cos, self.sin = math.cos(self.origin[2]), math.sin(self.origin[2])
        self.car = self.discs(-max(rig.back, rig.hitch), rig.front, rig.width)
        self.trailer = self.discs(-rig.tback, max(rig.tfront, rig.bar), rig.twidth)
        k = math.tan(rig.steer)/rig.wheelbase
        self.tip_rate = max(1+k*(rig.front+rig.back+rig.hitch+rig.width),
                            (1+rig.hitch*k)*(1+(rig.tfront+rig.tback+rig.twidth+rig.bar)/rig.bar))

    @staticmethod
    def discs(lo, hi, width):
        n = max(1, math.ceil((hi-lo)/width))
        step = (hi-lo)/n
        radius = math.hypot(step/2, width/2)
        return [(lo+(i+0.5)*step, radius) for i in range(n)]

    def clearance(self, p):
        if abs(p.beta) >= self.r.beta:
            return -math.inf
        best = math.inf
        for (x,y,angle), discs in ((p[:3], self.car), (self.r.axle(p), self.trailer)):
            dx, dy = x-self.origin[0], y-self.origin[1]
            gx = (dx*self.cos+dy*self.sin)/self.res
            gy = (-dx*self.sin+dy*self.cos)/self.res
            a = angle-self.origin[2]
            c, s = math.cos(a)/self.res, math.sin(a)/self.res
            for offset, radius in discs:
                ix, iy = math.floor(gx+offset*c), math.floor(gy+offset*s)
                if not (0 <= ix < self.w and 0 <= iy < self.h):
                    return -math.inf
                # EDT centre-to-centre, subtract the two cell half diagonals.
                best = min(best, self.dist[iy,ix]-math.sqrt(2)*self.res-radius)
        return best

    def advance(self, p, ds, steer):
        if self.cancel is not None and self.cancel.is_set():
            raise ValueError('planning cancelled')
        n = max(1, math.ceil(abs(ds)*self.tip_rate / self.res))
        step = ds/n
        reserve = abs(step)*self.tip_rate
        minimum = math.inf
        for _ in range(n):
            p = self.r.step(p, step, steer)
            minimum = min(minimum, self.clearance(p)-reserve)
            if minimum < self.r.margin:
                return None, minimum
        return p, minimum


class HybridAStar:
    def __init__(self, world, max_nodes=16000, seconds=15):
        if type(max_nodes) is not int or max_nodes < 1 or not math.isfinite(seconds) or seconds <= 0:
            raise ValueError('invalid planning budget')
        self.w, self.budget, self.seconds = world, max_nodes, seconds

    def plan(self, start, goal):
        w, r = self.w, self.w.r
        for name, p in [('start', start), ('goal', goal)]:
            if not all(map(math.isfinite, p)) or w.clearance(p) < r.margin:
                raise ValueError(f'{name}: footprint, map bounds or articulation limit')
        length = max(2*w.res, min(r.wheelbase/2, 2*r.pos_tol))
        def key(p, direction):
            return (round(p.x/(length/2)), round(p.y/(length/2)),
                    round(wrap(p.yaw)/0.09), round(p.beta/0.09), direction)
        def heuristic(p):
            return max(math.dist(p[:2], goal[:2]),
                       abs(wrap(p.yaw-goal.yaw))*r.wheelbase/math.tan(r.steer))
        # Parent indices retain the continuous state of the actual winning primitive.
        nodes = [(Knot(start, 0, 0), -1, 0.0)]
        queue = [(heuristic(start), 0)]
        costs = {key(start, 0): 0}
        deadline = time.monotonic()+self.seconds
        expanded = 0
        while queue:
            _, i = heapq.heappop(queue)
            knot, parent, cost = nodes[i]
            p = knot.pose
            previous = int(np.sign(knot.ds))
            if cost > costs[key(p, previous)]:
                continue
            if r.goal_ok(p, goal) and previous <= 0:
                path = []
                while i >= 0:
                    step, i, _ = nodes[i]
                    path.append(step)
                return path[::-1]
            expanded += 1
            if expanded > self.budget or time.monotonic() > deadline:
                raise ValueError(f'Hybrid A* search budget reached ({expanded} expansions)')
            for direction, steer in itertools.product((-1, 1), (0., -r.steer, r.steer, -r.steer/2, r.steer/2)):
                next_p, gap = w.advance(p, direction*length, steer)
                if next_p is None:
                    continue
                score = cost + length*(1 if direction < 0 else 1.6) + 0.08*length*abs(steer-knot.steer)
                if previous and direction != previous:
                    score += 4*length
                k = key(next_p, direction)
                if score >= costs.get(k, math.inf):
                    continue
                costs[k] = score
                nodes.append((Knot(next_p, direction*length, steer), i, score))
                heapq.heappush(queue, (score+1.4*heuristic(next_p), len(nodes)-1))
        raise ValueError('Hybrid A*: no feasible route')

