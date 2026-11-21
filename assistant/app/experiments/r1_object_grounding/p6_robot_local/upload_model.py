"""Resume a model over SSH with one acknowledged block, no bulk write queue."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import select
import shlex
import struct
import subprocess
import time

REMOTE=r'''
import hashlib,json,os,pathlib,struct,sys
meta=json.loads(sys.stdin.buffer.readline());dest=pathlib.Path(meta['destination']);dest.parent.mkdir(parents=True,exist_ok=True);temp=dest.with_name(dest.name+'.uploading')
size=temp.stat().st_size if temp.exists() else 0
h=hashlib.sha256()
if size:
 with temp.open('rb') as f:
  for block in iter(lambda:f.read(1048576),b''):h.update(block)
print(json.dumps({'offset':size,'prefix_sha256':h.hexdigest()}),flush=True)
mode=json.loads(sys.stdin.buffer.readline())
if mode.get('reset'):size=0;h=hashlib.sha256()
with temp.open('ab' if size else 'wb') as f:
 while True:
  prefix=sys.stdin.buffer.read(4)
  if len(prefix)!=4:raise EOFError('sender disconnected')
  n=struct.unpack('!I',prefix)[0]
  if n==0:break
  if n>65536 or size+n>meta['size']:raise ValueError('block exceeds bounds')
  data=sys.stdin.buffer.read(n)
  if len(data)!=n:raise EOFError('short block')
  f.write(data);f.flush();h.update(data);size+=n
  print(json.dumps({'offset':size}),flush=True)
 f.flush();os.fsync(f.fileno())
if size!=meta['size'] or h.hexdigest()!=meta['sha256']:raise ValueError('model hash mismatch')
temp.replace(dest);print(json.dumps({'complete':True,'sha256':h.hexdigest()}),flush=True)
'''


def main():
    p=argparse.ArgumentParser();p.add_argument('--robot',required=True);p.add_argument('--source',type=Path,required=True);p.add_argument('--destination',required=True);p.add_argument('--block-kib',type=int,default=16);p.add_argument('--ack-timeout-s',type=float,default=30.);a=p.parse_args()
    if a.robot.startswith('-') or not a.destination.startswith('/home/unitree/'):raise ValueError('invalid destination')
    if not 1<=a.block_kib<=64 or not 1<=a.ack_timeout_s<=60:raise ValueError('invalid transfer bounds')
    sha=hashlib.sha256(a.source.read_bytes()).hexdigest();size=a.source.stat().st_size
    child=subprocess.Popen(['ssh','-T','-o','BatchMode=yes','-o','ConnectTimeout=5','-o','ServerAliveInterval=10','-o','ServerAliveCountMax=3',a.robot,'python3 -u -c '+shlex.quote(REMOTE)],stdin=subprocess.PIPE,stdout=subprocess.PIPE)
    buffer=b''
    def reply():
        nonlocal buffer
        deadline=time.monotonic()+a.ack_timeout_s
        while b'\n' not in buffer:
            left=deadline-time.monotonic()
            if left<=0 or not select.select([child.stdout],[],[],left)[0]:raise TimeoutError('block ACK timeout')
            part=os.read(child.stdout.fileno(),4096)
            if not part:raise EOFError('upload closed')
            buffer+=part
        line,buffer=buffer.split(b'\n',1);return json.loads(line)
    try:
        child.stdin.write((json.dumps({'destination':a.destination,'size':size,'sha256':sha})+'\n').encode());child.stdin.flush();r=reply();offset=r['offset']
        with a.source.open('rb') as stream:
            digest=hashlib.sha256();remaining=offset if offset<=size else 0
            while remaining:
                data=stream.read(min(remaining,1048576));digest.update(data);remaining-=len(data)
            reset=offset>size or digest.hexdigest()!=r['prefix_sha256']
            child.stdin.write((json.dumps({'reset':reset})+'\n').encode());child.stdin.flush()
            if reset:offset=0;stream.seek(0)
            blocks=0
            while offset<size:
                block=stream.read(a.block_kib*1024);child.stdin.write(struct.pack('!I',len(block))+block);child.stdin.flush()
                offset+=len(block)
                if reply()['offset']!=offset:raise ValueError('upload ACK offset')
                blocks+=1
                if blocks%64==0:print(json.dumps({'uploaded':offset,'total':size}),flush=True)
            child.stdin.write(struct.pack('!I',0));child.stdin.flush();print(json.dumps(reply()),flush=True)
        child.stdin.close();child.wait(timeout=5)
        if child.returncode:raise RuntimeError('upload worker failed')
    finally:
        if child.poll() is None:
            child.terminate()
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:child.kill();child.wait()
        if not child.stdin.closed:child.stdin.close()
        child.stdout.close()


if __name__=='__main__':main()
