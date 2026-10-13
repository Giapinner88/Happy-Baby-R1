"""Exercise real HTTP/WS routes without an IK, DDS or motor publisher."""
import asyncio
import json
import sys
from multiprocessing import Value
from pathlib import Path
from types import SimpleNamespace
import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts/teleop'
sys.path.insert(0, str(SCRIPTS))


def test_tracking_routes_lease_legacy_rearm_and_bootstrap(tmp_path):
    pytest.importorskip('vuer')
    from aiohttp import web, WSServerHandshakeError
    from aiohttp.test_utils import TestClient, TestServer
    from quest_camera_webrtc import build_camera_wrapper
    from test_xr_frame_gate import frame

    (tmp_path / 'index.html').write_text('<html><head></head><body></body></html>')

    class Vendor:
        def __init__(self):
            self.routes = []
            self.vuer = SimpleNamespace(client_root=str(tmp_path), socket_index=lambda request: None,
                                        _add_route=lambda path, handler, method: self.routes.append((method, path, handler)))
            self.accepted = []
            self.left_arm_pose_shared = [1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]
            for side in ('left', 'right'):
                for field in ('trigger', 'triggerValue', 'aButton'):
                    setattr(self, side + '_ctrl_' + field + '_shared', Value('d', 0))

        async def on_controller_move(self, event, session, fps):
            self.accepted.append(event.value)

        async def on_cam_move(self, event, session, fps):
            pass

        def _vuer_run(self):
            pass

    module = SimpleNamespace(TeleVuer=Vendor)
    module.TeleVuerWrapper = lambda **kwargs: module.TeleVuer()
    tv = build_camera_wrapper(module, SimpleNamespace(media_port=8513, width=640, height=480))
    tv._vuer_run()

    async def exercise():
        app = web.Application()
        app.router.add_get('/', tv.vuer.socket_index)
        for method, path, handler in tv.routes:
            app.router.add_route(method, path, handler)
        async with TestClient(TestServer(app)) as client:
            html = await (await client.get('/')).text()
            assert '<head><script>' in html and 'xr-frame-v3' in html
            assert 'XRFrame' in html
            assert (await client.post('/tracking/diagnostic', json={'stage': 'installed', 'version': 'xr-frame-v3'})).status == 200
            status = await (await client.get('/tracking/status')).json()
            assert status['browser_status']['stage'] == 'installed'
            assert status['tracking_protocol'] == 'xr-frame-v3'
            await tv.on_controller_move(SimpleNamespace(value=frame(1, 1, True)), None)
            assert not tv.accepted  # Legacy Vuer events cannot overwrite XR frames.
            first = await client.ws_connect('/tracking?client=one')
            with pytest.raises(WSServerHandshakeError) as conflict:
                await client.ws_connect('/tracking?client=two')
            assert conflict.value.status == 409  # Initial lease works before first packet.
            await first.send_json(frame(1))
            await first.send_json(frame(2, 1, True))
            await asyncio.sleep(.03)
            assert tv.right_ctrl_trigger_shared.value
            for sequence, analog in enumerate((.452, .411, .379, .216), 3):
                await first.send_json(frame(sequence, analog))
            await asyncio.sleep(.03)
            assert tv.right_ctrl_trigger_shared.value
            partial = frame(7, 1, True)
            partial.update(arm_tracking={'left': False, 'right': True}, left=None)
            await first.send_json(partial)
            await asyncio.sleep(.03)
            assert tv.right_ctrl_trigger_shared.value
            assert list(tv.arm_tracking[:]) == [False, True]
            await first.send_json({'sequence': 8, 'valid': False, 'reason': 'right_pose_missing'})
            await asyncio.sleep(.03)
            assert not tv.right_ctrl_trigger_shared.value
            await first.send_json(frame(9, 1, True))
            await asyncio.sleep(.03)
            assert not tv.right_ctrl_trigger_shared.value
            await first.send_json(frame(10))
            await first.send_json(frame(11, 1, True))
            await asyncio.sleep(.03)
            assert tv.right_ctrl_trigger_shared.value
            # A short estimate has no controller callback or IK pose update.
            # Real position recovery resumes only after its stability window.
            from test_position_pause_recovery import tracked, estimated
            await first.send_json(tracked(12, False))
            await first.send_json(tracked(13))
            await asyncio.sleep(.03)
            accepted_before_pause = len(tv.accepted)
            await first.send_json(estimated(14))
            await asyncio.sleep(.03)
            assert not tv.right_ctrl_trigger_shared.value
            assert not tv.tracking[3]  # no physical release needed for this pause
            assert len(tv.accepted) == accepted_before_pause
            status = await (await client.get('/tracking/status')).json()
            assert status['browser_status']['stage'] == 'installed'
            assert status['tracking_server_status']['reason'] == 'right_position_paused'
            await first.send_json(tracked(15))
            await asyncio.sleep(.03)
            assert not tv.right_ctrl_trigger_shared.value
            assert len(tv.accepted) == accepted_before_pause
            await asyncio.sleep(.09)
            await first.send_json(tracked(16))
            await asyncio.sleep(.03)
            assert tv.right_ctrl_trigger_shared.value
            assert len(tv.accepted) == accepted_before_pause + 1
            await asyncio.sleep(.55)
            second = await client.ws_connect('/tracking?client=two')
            assert not tv.right_ctrl_trigger_shared.value
            await second.send_json(frame(1, 1, True))
            await asyncio.sleep(.03)
            assert not tv.right_ctrl_trigger_shared.value
            await second.send_json(frame(2))
            await second.send_json(frame(3, 1, True))
            await asyncio.sleep(.03)
            assert tv.right_ctrl_trigger_shared.value  # Closing stale owner did not STOP new one.
            await second.close()
            await asyncio.sleep(.03)
            assert not tv.right_ctrl_trigger_shared.value
            await first.close()
    asyncio.run(exercise())
