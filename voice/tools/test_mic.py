"""Manual test: record from the Unitree R1 mic bridge and save a WAV file.

Mirrors the subprocess pattern in hb_voice/output.py but for the mic (input)
direction, with a live byte counter so you can see immediately whether any
audio is arriving before waiting for the whole recording to finish.

Usage:
    uv run python tools/test_mic.py [network_interface] [seconds] [out_wav_path]
"""

import asyncio
import sys
import wave
from pathlib import Path

VOICE_ROOT = Path(__file__).resolve().parents[1]
BRIDGE_PATH = VOICE_ROOT / "unitree_bridge" / "build" / "r1_bridge"


async def record(network_interface: str, seconds: int, out_path: Path) -> None:
    if not BRIDGE_PATH.exists():
        print(f"r1_bridge not built: {BRIDGE_PATH}")
        return

    process = await asyncio.create_subprocess_exec(
        str(BRIDGE_PATH),
        "mic",
        network_interface,
        str(seconds),
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE,
    )

    chunks = []
    total_bytes = 0
    while True:
        chunk = await process.stdout.read(4096)
        if not chunk:
            break
        chunks.append(chunk)
        total_bytes += len(chunk)
        print(f"\rreceived {total_bytes} bytes", end="", flush=True)
    print()

    return_code = await process.wait()
    stderr = (await process.stderr.read()).decode(errors="replace")
    if stderr:
        print(stderr, file=sys.stderr)
    print(f"r1_bridge exited with code {return_code}")

    if total_bytes == 0:
        print("No audio received - mic stream is silent.")
        return

    with wave.open(str(out_path), "wb") as wav_file:
        wav_file.setnchannels(1)
        wav_file.setsampwidth(2)
        wav_file.setframerate(16000)
        wav_file.writeframes(b"".join(chunks))
    print(f"Saved {out_path}")


if __name__ == "__main__":
    iface = sys.argv[1] if len(sys.argv) > 1 else "eth10"
    seconds = int(sys.argv[2]) if len(sys.argv) > 2 else 5
    out = Path(sys.argv[3]) if len(sys.argv) > 3 else Path("/tmp/r1_mic_test.wav")
    asyncio.run(record(iface, seconds, out))
