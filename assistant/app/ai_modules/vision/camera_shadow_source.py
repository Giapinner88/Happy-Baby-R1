"""Bounded framing reader for the read-only SSH camera worker."""
import hashlib
import json
import os
import select
import struct
import time
from concurrent.futures import ThreadPoolExecutor


class OneAheadCapture:
    """Overlap one sensor request with compute, never queue multiple frames."""
    def __init__(self, capture, count):
        self.capture=capture;self.count=count;self.pool=ThreadPoolExecutor(max_workers=1)
        self.index=0;self.pending=self.pool.submit(capture,0) if count else None

    def next(self):
        if self.pending is None:raise StopIteration()
        value=self.pending.result();self.index+=1
        self.pending=self.pool.submit(self.capture,self.index) if self.index<self.count else None
        return value

    def close(self):
        if self.pending is not None:self.pending.cancel()
        self.pool.shutdown(wait=True)


def read_exact(fd, count, deadline):
    result = bytearray()
    while len(result) < count:
        remaining = deadline-time.monotonic()
        if remaining <= 0 or not select.select([fd], [], [], max(0, remaining))[0]:
            raise TimeoutError('camera worker response timeout')
        block = os.read(fd, count-len(result))
        if not block:
            raise EOFError('camera worker closed wire')
        result.extend(block)
    return bytes(result)


def read_packet(fd, timeout_s):
    deadline = time.monotonic()+timeout_s
    length = struct.unpack('!I', read_exact(fd, 4, deadline))[0]
    if not 0 < length <= 65536:
        raise ValueError('invalid worker header size')
    header = json.loads(read_exact(fd, length, deadline))
    size = header.get('bytes')
    if type(size) is not int or not 0 <= size <= 4*1024*1024:
        raise ValueError('invalid worker image size')
    if header.get('read_only') is not True or type(header.get('sequence')) is not int:
        raise ValueError('unexpected camera protocol')
    payload = read_exact(fd, size, deadline)
    if header.get('ok') is True and ((not size and header.get('event') not in ('pose','odom')) or hashlib.sha256(payload).hexdigest() != header.get('sha256')):
        raise ValueError('worker payload hash mismatch')
    return header, payload
