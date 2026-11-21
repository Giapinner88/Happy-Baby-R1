"""Read-only robot state; this module never creates ChannelPublisher."""
import math
import threading
import time
from control import IDL,JOINTS


class StateReader:
    def __init__(self,interface,timeout_s):
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize,ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
        self.lock=threading.Lock();self.state=None
        def callback(msg):
            if len(msg.motor_state)<=max(IDL):return
            q=[float(msg.motor_state[i].q) for i in IDL];dq=[float(msg.motor_state[i].dq) for i in IDL]
            if not all(math.isfinite(v) for v in q+dq):return
            tick=int(msg.tick)
            item={'robot_tick':tick,'received_monotonic_s':time.monotonic(),'positions_rad':q,'velocities_rad_s':dq,
                  'joints_rad':dict(zip(JOINTS,q)),'mode_machine':int(msg.mode_machine)}
            with self.lock:
                if self.state is not None and tick==self.state['robot_tick']:return
                self.state=item
        ChannelFactoryInitialize(0,interface)
        self.sub=ChannelSubscriber('rt/lowstate',LowState_);self.sub.Init(callback,10)
        end=time.monotonic()+timeout_s
        while time.monotonic()<end:
            with self.lock:ready=self.state is not None
            if ready:return
            time.sleep(.01)
        raise RuntimeError('no fresh LowState; nothing sent')

    def latest(self,max_age_s):
        with self.lock:item=self.state
        if item is None or time.monotonic()-item['received_monotonic_s']>max_age_s:
            raise RuntimeError('LowState timeout; release target stream')
        return item
