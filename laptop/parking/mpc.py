"""Finite-control-set, receding-horizon planning and feedback tracking (laptop only)."""
import bisect
import math
from typing import NamedTuple

from .planner import Knot, Pose, wrap


def error(rig, p, ref):
    dx = math.dist(p[:2], ref[:2])
    trailer = math.dist(rig.axle(p)[:2], rig.axle(ref)[:2])
    return dx*dx + 2*trailer*trailer + rig.bar**2 * (
        wrap(p.yaw-ref.yaw)**2 + 2*wrap(p.beta-ref.beta)**2)


class Sample(NamedTuple):
    t: float
    pose: Pose
    speed: float
    steer: float


class PlanningMPC:
    """Spatial shooting horizon with steering perturbations; time-scale the accepted path."""
    def __init__(self, world):
        self.w = world

    def optimize(self, path, goal):
        r = self.w.r
        result = [path[0]]
        for i in range(1, len(path)):
            horizon = path[i:i+5]
            choices = []
            for shift in (0., -0.12*r.steer, 0.12*r.steer, -0.3*r.steer, 0.3*r.steer):
                p, prev, score = result[-1].pose, result[-1].steer, 0
                for j, k in enumerate(horizon):
                    steer = max(-r.steer, min(r.steer, k.steer+shift*(1-j/len(horizon))))
                    p, gap = self.w.advance(p, k.ds, steer)
                    if p is None:
                        break
                    if j == 0:
                        first = Knot(p, k.ds, steer)
                    score += (40*error(r, p, k.pose) + 0.04*(steer-prev)**2
                              + 0.005*steer**2 + 0.002/max(0.01, gap))
                    prev = steer
                else:
                    terminal = goal if i+len(horizon) == len(path) else horizon[-1].pose
                    choices.append((score+100*error(r,p,terminal), first))
            if not choices:
                raise ValueError('Planning MPC: no collision-free horizon')
            result.append(min(choices, key=lambda item:item[0])[1])
        if not r.goal_ok(result[-1].pose, goal):
            # Original steering sequence is a feasible control candidate, including terminal set.
            result = path
        return self.profile(result)

    def profile(self, knots):
        r = self.w.r
        # Bisect every primitive so even a single stop-to-stop segment has a speed peak.
        edges = []
        p = knots[0].pose
        for k in knots[1:]:
            for _ in range(2):
                p, _ = self.w.advance(p, k.ds/2, k.steer)
                if p is None:
                    raise ValueError('Planning MPC: time profile collision')
                edges.append(Knot(p, k.ds/2, k.steer))
        speeds = [r.speed]*(len(edges)+1)
        speeds[0] = speeds[-1] = 0
        for i in range(1,len(edges)):
            if edges[i-1].ds*edges[i].ds < 0 or abs(edges[i-1].steer-edges[i].steer) > 1e-8:
                speeds[i] = 0  # switch gear / steer at rest
        for indices, sign in ((range(len(edges)),1), (range(len(edges),0,-1),-1)):
            for i in indices:
                j = i+sign
                speeds[j] = min(speeds[j], math.sqrt(speeds[i]**2+2*r.accel*abs(edges[min(i,j)].ds)))
        ref = [Sample(0., knots[0].pose, 0., knots[0].steer)]
        for i,k in enumerate(edges):
            prev = ref[-1]
            if abs(k.steer-prev.steer) > 1e-8:
                prev = Sample(prev.t+abs(k.steer-prev.steer)/r.slew, prev.pose, 0., k.steer)
                ref.append(prev)
            dt = 2*abs(k.ds)/(speeds[i]+speeds[i+1])
            ref.append(Sample(prev.t+dt, k.pose, math.copysign(speeds[i+1],k.ds), k.steer))
        return ref


def interpolate(ref, times, t):
    i = bisect.bisect_right(times,t)
    if i == 0 or i == len(ref):
        return ref[0] if i == 0 else ref[-1]
    a,b = ref[i-1:i+1]
    f = (t-a.t)/(b.t-a.t)
    # Constant acceleration within moving segments; position follows travelled distance.
    g = f if abs(a.speed+b.speed)<1e-10 else f*(2*a.speed+f*(b.speed-a.speed))/(a.speed+b.speed)
    return Sample(t, Pose(a.pose.x+g*(b.pose.x-a.pose.x), a.pose.y+g*(b.pose.y-a.pose.y),
                          wrap(a.pose.yaw+g*wrap(b.pose.yaw-a.pose.yaw)),
                          wrap(a.pose.beta+g*wrap(b.pose.beta-a.pose.beta))),
                  a.speed+f*(b.speed-a.speed), a.steer+f*(b.steer-a.steer))


class TrackingMPC:
    """Bounded beam search over speed and steering sequences. Apply only the first input."""
    def __init__(self, world, reference, goal):
        self.w, self.ref, self.goal = world, reference, goal
        self.times = [s.t for s in reference]

    def compute(self, pose, speed, steer, elapsed, dt=0.10):
        r = self.w.r
        if not all(map(math.isfinite, (*pose,speed,steer,elapsed,dt))) or not 0 < dt <= 0.25:
            raise ValueError('invalid tracking feedback/time')
        if abs(speed)>r.speed+1e-6 or abs(steer)>r.steer+1e-6 or self.w.clearance(pose)<r.margin:
            raise ValueError('tracking feedback outside safe envelope')
        target = interpolate(self.ref,self.times,elapsed)
        deviation = math.dist(r.axle(pose)[:2],r.axle(target.pose)[:2])
        if (max(deviation, math.dist(pose[:2],target.pose[:2])) > r.deviation
            or abs(wrap(pose.beta-target.pose.beta)) > r.beta/2
            or abs(wrap(pose.yaw-target.pose.yaw)) > max(r.yaw_tol, math.atan2(r.deviation,r.bar))):
            raise ValueError('path deviation: stop and replan')
        if elapsed >= self.times[-1] and r.goal_ok(pose,self.goal) and abs(speed)<=r.stopped:
            return 0.,0.,deviation,True
        if elapsed > self.times[-1]+1:
            raise ValueError('final pose missed: stop and replan')
        beam = [(0.,pose,speed,steer,None)]
        for j in range(5):
            ref = interpolate(self.ref,self.times,elapsed+(j+1)*dt)
            candidates = []
            for cost,p,v,d,first in beam:
                def options(current, desired, rate, limit):
                    lo = max(-limit,current-rate*dt)
                    hi = min(limit,current+rate*dt)
                    return {lo,hi,max(lo,min(hi,desired))}
                for vn in options(v,ref.speed,r.accel,r.speed):
                    if vn*v < -1e-9 or vn*ref.speed < -1e-9 or (v==0 and ref.speed==0 and vn!=0):
                        continue
                    for dn in options(d,ref.steer,r.slew,r.steer):
                        pn,gap = self.w.advance(p,(vn+v)*dt/2,(dn+d)/2)
                        if pn is None:
                            continue
                        score = (cost + (2+j)*error(r,pn,ref.pose) + 0.5*(vn-ref.speed)**2
                                 + 0.1*(dn-ref.steer)**2 + 0.02*(dn-d)**2 + 0.0005/max(0.01,gap))
                        candidates.append((score,pn,vn,dn,first or (vn,dn)))
            if not candidates:
                raise ValueError('Tracking MPC: no feasible input')
            beam = sorted(candidates,key=lambda c:c[0])[:5]
        velocity, angle = beam[0][-1]
        return velocity, angle, deviation, False

