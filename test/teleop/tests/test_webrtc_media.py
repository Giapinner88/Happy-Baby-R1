"""Optional media dependency checks; core deploy tests run without aiortc."""
import asyncio
import io
from pathlib import Path
import sys
from types import SimpleNamespace
import numpy as np
import pytest
from PIL import Image
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts/teleop'))
from robot_camera_webrtc import decode_jpeg,CameraSource,make_app


def test_jpeg_rejects_truncation_and_accepts_complete_image():
    buf=io.BytesIO()
    Image.new('RGB',(32,16),(120,60,30)).save(buf,format='JPEG')
    raw=buf.getvalue()
    assert decode_jpeg(raw,64,36).shape==(36,64,3)
    with pytest.raises(ValueError,match='incomplete'):
        decode_jpeg(raw[:-10],64,36)


def test_webrtc_frames_two_view_cycles_and_no_idle_capture():
    pytest.importorskip('aiortc')
    from aiortc import RTCPeerConnection,RTCConfiguration,RTCSessionDescription
    from aiohttp import ClientSession,web

    class Synthetic(CameraSource):
        def read(self):
            return np.full((self.args.height,self.args.width,3),self.sequence%255,dtype=np.uint8),str(self.sequence).encode()

    async def check():
        args=SimpleNamespace(width=160,height=90,capture_hz=0,stale_s=1,interface='unused',log=None)
        source=Synthetic(args)
        runner=web.AppRunner(make_app(args,source))
        await runner.setup()
        site=web.TCPSite(runner,'127.0.0.1',0)
        await site.start()
        url=f'http://127.0.0.1:{site._server.sockets[0].getsockname()[1]}'
        try:
            async with ClientSession() as session:
                await asyncio.sleep(.1)
                assert source.sequence==0
                for _ in range(2):
                    async with session.post(url+'/view',json={'enabled':True}) as response:
                        assert response.status==200
                    pc=RTCPeerConnection(RTCConfiguration(iceServers=[]))
                    tracks=[]
                    pc.on('track',lambda track: tracks.append(track))
                    pc.addTransceiver('video',direction='recvonly')
                    pc.addTransceiver('audio',direction='recvonly')
                    try:
                        await pc.setLocalDescription(await pc.createOffer())
                        async with session.post(url+'/offer',json={'sdp':pc.localDescription.sdp,'type':'offer'}) as response:
                            assert response.status==200
                            answer=await response.json()
                        await pc.setRemoteDescription(RTCSessionDescription(**answer))
                        assert len(tracks)==1
                        frames=[]
                        for _ in range(12):
                            frames.append(await asyncio.wait_for(tracks[0].recv(),5))
                        assert all(a.pts<b.pts for a,b in zip(frames,frames[1:]))
                        async with session.post(url+'/view',json={'enabled':False}) as response:
                            assert response.status==200
                        async with session.get(url+'/health') as response:
                            health=await response.json()
                            assert health['peers']==0 and not health['enabled']
                        sequence=source.sequence
                        await asyncio.sleep(.1)
                        assert sequence==source.sequence
                    finally:
                        await pc.close()
        finally:
            await runner.cleanup()
    asyncio.run(check())
