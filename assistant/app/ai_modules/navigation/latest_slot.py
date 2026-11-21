"""One shared latest value, bounded memory; slow readers skip old versions."""
import multiprocessing as mp
import time


class LatestSlot:
    def __init__(self,capacity,context=None):
        if type(capacity) is not int or capacity<=0:raise ValueError('invalid capacity')
        ctx=context or mp.get_context('spawn');self.capacity=capacity
        self.buffer=ctx.RawArray('B',capacity);self.size=ctx.Value('i',0,lock=False)
        self.sequence=ctx.Value('Q',0,lock=False);self.lock=ctx.Lock();self.changed=ctx.Event()

    def publish(self,value):
        if not isinstance(value,bytes) or len(value)>self.capacity:raise ValueError('slot requires bounded bytes')
        with self.lock:
            memoryview(self.buffer).cast('B')[:len(value)]=value
            self.size.value=len(value);self.sequence.value+=1;self.changed.set()
            return self.sequence.value

    def read(self,after=0,timeout=0.):
        until=time.monotonic()+timeout
        while True:
            with self.lock:
                sequence=self.sequence.value
                if sequence>after:return sequence,bytes(memoryview(self.buffer).cast('B')[:self.size.value])
                self.changed.clear()
            remaining=until-time.monotonic()
            if remaining<=0:return None
            self.changed.wait(remaining)
