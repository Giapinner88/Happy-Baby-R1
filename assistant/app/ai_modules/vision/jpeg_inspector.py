"""Persistent isolated native JPEG validation, without per-frame Python startup."""
import json
import os
from pathlib import Path
import select
import subprocess
import sys
import tempfile
import time


def _line(fd, timeout):
    result = bytearray()
    deadline = time.monotonic()+timeout
    while len(result) < 65536:
        remaining = deadline-time.monotonic()
        if remaining <= 0 or not select.select([fd], [], [], max(remaining, 0))[0]:
            raise TimeoutError('JPEG inspector timeout')
        char = os.read(fd, 1)
        if not char:
            raise EOFError('JPEG inspector closed')
        if char == b'\n':
            return json.loads(result)
        result.extend(char)
    raise ValueError('oversized JPEG inspector response')


class JpegInspector:
    def __init__(self, timeout_s=5.):
        self.timeout = timeout_s
        self.process = subprocess.Popen([sys.executable, '-u', str(Path(__file__).resolve()), '--worker'],
                                        stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        try:
            if _line(self.process.stdout.fileno(), timeout_s) != {'ready': True}:
                raise ValueError('invalid JPEG inspector handshake')
        except BaseException:
            self.close()
            raise

    def inspect(self, path):
        self.process.stdin.write((json.dumps({'path': str(path)})+'\n').encode())
        self.process.stdin.flush()
        return _line(self.process.stdout.fileno(), self.timeout)

    def close(self):
        try:
            self.process.stdin.close()
            self.process.wait(timeout=2)
        except (OSError, subprocess.TimeoutExpired):
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        finally:
            self.process.stdout.close()


def worker():
    import cv2
    print(json.dumps({'ready': True}), flush=True)
    for line in sys.stdin:
        request = json.loads(line)
        with tempfile.TemporaryFile() as errors:
            saved = os.dup(2)
            message = None
            try:
                os.dup2(errors.fileno(), 2)
                image = cv2.imread(request['path'])
            except cv2.error as exc:
                image, message = None, str(exc)
            finally:
                os.dup2(saved, 2)
                os.close(saved)
            errors.seek(0)
            warning = errors.read().decode('utf-8', errors='replace').strip()
        print(json.dumps({'decoded': image is not None,
                          'shape': list(image.shape) if image is not None else None,
                          'decoder_stderr': warning, 'error': message,
                          'accepted': image is not None and not warning and not message}), flush=True)


if __name__ == '__main__':
    if sys.argv[1:] != ['--worker']:
        raise SystemExit('internal worker only')
    worker()
