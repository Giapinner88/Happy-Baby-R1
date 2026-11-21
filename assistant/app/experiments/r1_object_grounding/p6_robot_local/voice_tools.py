"""HB voice tools: robot-local HTTP task admission, never direct motor calls."""
import asyncio
import json
import fcntl
from pathlib import Path
from urllib import request,error

PORT=5594
MARKER=Path('/etc/hb/local_approach_voice')
ACTIVE=Path('/run/hb-approach/active')


def enabled():return MARKER.is_file()


def claim_camera(path='/run/hb-approach/camera.lock'):
    file=open(path,'a')
    try:fcntl.flock(file,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BaseException:file.close();raise
    return file


def call(path,payload):
    data=json.dumps(payload,ensure_ascii=False).encode()
    req=request.Request('http://127.0.0.1:%d%s'%(PORT,path),data=data,headers={'Content-Type':'application/json'},method='POST')
    try:
        with request.urlopen(req,timeout=2) as response:return json.load(response)
    except error.HTTPError as exc:
        with exc:return json.load(exc)


async def start(params):
    try:result=await asyncio.to_thread(call,'/approach',{'query':params.arguments.get('query','')})
    except Exception:result={'status':'unavailable','answer':'Chức năng đi tới vật chưa sẵn sàng.'}
    await params.result_callback(result)


async def cancel(params):
    try:result=await asyncio.to_thread(call,'/cancel',{})
    except Exception:result={'status':'unavailable','answer':'Chưa xác nhận được lệnh dừng. Hãy dùng tay cầm.'}
    await params.result_callback(result)


async def progress(params):
    try:result=await asyncio.to_thread(call,'/status',{})
    except Exception:result={'status':'unavailable','answer':'Chưa đọc được trạng thái tác vụ.'}
    await params.result_callback(result)


def tools():
    from pipecat.adapters.schemas.function_schema import FunctionSchema
    return [FunctionSchema(name='approach_object',description='Khi người dùng yêu cầu đi tới hoặc tiến lại gần một vật, gọi với mô tả vật. Chỉ báo tác vụ đã nhận nếu accepted, không nói đã tới vật. Nếu busy không xếp hàng hoặc tự thử lại; nếu shadow chỉ nói đang quan sát, chưa đi. Không gọi đồng thời find_and_point/inspect_camera cho cùng task.',properties={'query':{'type':'string','description':'Một vật đích, ví dụ bình cứu hỏa.'}},required=['query'],handler=start),
            FunctionSchema(name='cancel_approach',description='Khi người dùng nói dừng đi hoặc hủy tiếp cận, gọi ngay. Đây không thay thế E-stop.',properties={},required=[],handler=cancel),
            FunctionSchema(name='approach_status',description='Đọc trạng thái tác vụ tiếp cận. Chỉ nói đã tới khi kết quả thực xác nhận; không suy diễn từ accepted hoặc ACK.',properties={},required=[],handler=progress)]
