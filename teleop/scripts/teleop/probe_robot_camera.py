#!/usr/bin/env python3
"""Read-only DDS camera check; retry malformed/transient frames, never motor I/O."""
import argparse
import time
from robot_camera_webrtc import decode_jpeg


def probe(client, attempts=3):
    errors = []
    for _ in range(attempts):
        try:
            code, payload = client.GetImageSample()
            if int(code) != 0 or not payload:
                raise RuntimeError(f'videohub code={int(code)}, bytes={len(payload) if payload else 0}')
            decode_jpeg(bytes(payload), 640, 360)
            return True, f'camera frame decoded; attempts={len(errors) + 1}'
        except Exception as exc:
            errors.append(f'{type(exc).__name__}: {exc}')
    return False, '; '.join(errors)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('interface')
    args = parser.parse_args()
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.go2.video.video_client import VideoClient
        ChannelFactoryInitialize(0, args.interface)
        client = VideoClient()
        client.SetTimeout(1.0)
        client.Init()
        ready, detail = probe(client)
    except Exception as exc:
        ready, detail = False, f'{type(exc).__name__}: {exc}'
    print(detail, flush=True)
    return 0 if ready else 1


if __name__ == '__main__':
    raise SystemExit(main())
