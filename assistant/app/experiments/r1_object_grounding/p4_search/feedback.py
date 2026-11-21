"""Reusable heading feedback law. Caller owns rate loop and STOP in finally.
Input is measured unwrapped heading, not integrated command velocity.
"""
import math


def unwrap(previous,wrapped):
    if previous is None:return wrapped
    return previous+(wrapped-previous+180)%360-180


def yaw_rate(target,heading,kp,max_rate,tolerance):
    values=[target,heading,kp,max_rate,tolerance]
    if not all(math.isfinite(v) for v in values) or min(kp,max_rate,tolerance)<=0:
        raise ValueError('invalid heading controller input')
    error=target-heading
    return 0. if abs(error)<=tolerance else max(-max_rate,min(max_rate,kp*error))


def turn_to(target,read_state,send_rate,now,sleep,cancelled,*,kp=2.,max_rate=15.,
            tolerance=3.,speed_tolerance=2.,max_age=.2,timeout=20.,dt=.02,settle_samples=3):
    """Closed-loop turn primitive for an owner adapter; send_rate accepts deg/s.
    read_state returns time_s/body_heading_deg/body_speed_deg_s (unwrapped).
    Never derives heading from commanded rate. Explicit zero rate on every exit.
    """
    begin=now();good=0;last=None
    try:
        while now()-begin<=timeout:
            if cancelled():raise InterruptedError('cancelled turn')
            s=read_state()
            if not all(math.isfinite(s[k]) for k in ['time_s','body_heading_deg','body_speed_deg_s']):
                raise ValueError('nonfinite heading state')
            if not 0<=now()-s['time_s']<=max_age:raise TimeoutError('stale heading')
            rate=yaw_rate(target,s['body_heading_deg'],kp,max_rate,tolerance)
            send_rate(rate)
            if s['time_s']!=last:
                last=s['time_s']
                good=good+1 if rate==0 and abs(s['body_speed_deg_s'])<=speed_tolerance else 0
                if good>=settle_samples:return s
            sleep(dt)
        raise TimeoutError('turn timeout')
    finally:send_rate(0.)
