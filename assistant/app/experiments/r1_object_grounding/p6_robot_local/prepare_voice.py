"""Prepare hash-guarded canonical HB voice integration; never edit robot live."""
import argparse
import hashlib
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent


def replace(text,old,new):
    if text.count(old)!=1:raise ValueError('voice integration anchor changed: '+old[:50])
    return text.replace(old,new)


def main():
    p=argparse.ArgumentParser();p.add_argument('--canonical',type=Path,required=True);p.add_argument('--output-dir',type=Path,required=True);a=p.parse_args()
    names=['voice/hb_voice/vision_tools.py','object_search/experiments/r1_object_grounding/p5_voice_search/voice_overlay/sitecustomize.py']
    originals={n:(a.canonical/n).read_bytes() for n in names};text=originals[names[0]].decode()
    # Keep native camera capture as a tool, serialize its actual RPC with approach.
    anchor='    def prefetch(self) -> bool:\n        """Start one bounded capture when transcript hints at a visual question."""\n'
    text=replace(text,anchor,anchor+'        from . import approach_tools\n        if approach_tools.enabled() and approach_tools.ACTIVE.exists():\n            return False\n')
    text=replace(text,'        async with self._capture_lock:\n            return await asyncio.to_thread(self._capture_frame)',
        '''        async with self._capture_lock:
            from . import approach_tools
            lease = approach_tools.claim_camera() if approach_tools.enabled() else None
            try:
                return await asyncio.to_thread(self._capture_frame)
            finally:
                if lease is not None:
                    lease.close()''')
    start=text.index('    @property\n    def tools(self) -> list:');end=text.index('\n    async def inspect_camera',start)
    block=text[start:end]
    block=replace(block,'        return [','        available = [')
    block+='''
        from . import approach_tools
        if approach_tools.enabled():
            available.extend(approach_tools.tools())
        return available
'''
    text=text[:start]+block+text[end:]
    text+='''

from . import approach_tools as _approach_tools
if _approach_tools.enabled():
    VISION_TOOL_INSTRUCTIONS += (
        "\\n- Khi người dùng yêu cầu đi tới vật, gọi approach_object; không điều khiển khớp trực tiếp. "
        "accepted chỉ là nhận task, shadow chưa đi, busy không xếp hàng hoặc tự thử lại. "
        "Dùng approach_status để đọc kết quả; không nói đã tới vật nếu chưa xác nhận. "
        "Khi người dùng nói dừng đi, gọi cancel_approach ngay."
    )
'''
    changed={names[0]:text.encode(),'voice/hb_voice/approach_tools.py':(HERE/'voice_tools.py').read_bytes()}
    text=originals[names[1]].decode()
    text=replace(text,'import asyncio\n','import asyncio\nfrom pathlib import Path\n')
    text=replace(text,"if os.environ.get('R1_P5_VOICE_ENABLED') == '1':", "if os.environ.get('R1_P5_VOICE_ENABLED') == '1' and not Path('/etc/hb/local_approach_voice').is_file():")
    changed[names[1]]=text.encode()
    out=a.output_dir;out.mkdir(parents=True,exist_ok=False)
    for name,data in changed.items():
        p=out/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(data)
    (out/'manifest.json').write_text(json.dumps({'base_sha256':{n:hashlib.sha256(d).hexdigest() for n,d in originals.items()},'output_sha256':{n:hashlib.sha256(d).hexdigest() for n,d in changed.items()},'mode':'staged; marker opt-in; no motor changes'},indent=2)+'\n')
    print(out)


if __name__=='__main__':main()
