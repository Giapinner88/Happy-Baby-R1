"""Bounded control mailboxes and diagnostic I/O outside the control loop."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import queue
import sys
import threading
import time

MAX_LINE_BYTES = 65536


class ReceivedLine(str):
    def __new__(cls, text: str):
        value = super().__new__(cls, text)
        value.received_monotonic_s = time.monotonic()
        return value


class ControlInputError(ValueError):
    pass


class LatestControlInbox:
    """Keep latest pose and last STOP/reset in order, never an unbounded FIFO.

    Validate every sequence before compacting so a discarded pose cannot hide
    a replay or malformed envelope. A fault is sticky and delivered before
    further motion. Consumers take one atomic batch per control tick.
    """
    def __init__(self, enabled_key: str):
        self.enabled_key = enabled_key
        self._lock = threading.Lock()
        self._pending = []
        self._last_sequence = -1
        self._fault = None
        self._closed = False
        self.compacted = 0

    def put(self, line):
        if line is None:
            with self._lock:
                self._closed = True
            return
        if not isinstance(line, str) or len(line.encode('utf-8')) > MAX_LINE_BYTES:
            self.fail('oversized control line')
            return
        received = ReceivedLine(line)
        try:
            payload = json.loads(line)
            if not isinstance(payload, dict):
                raise ValueError('control envelope must be an object')
            sequence = payload.get('sequence_id')
            if type(sequence) is not int or sequence < 0:
                raise ValueError('invalid control sequence')
            if type(payload.get(self.enabled_key)) is not bool:
                raise ValueError('missing boolean deadman')
        except (TypeError, ValueError) as error:
            self.fail(str(error))
            return
        with self._lock:
            if self._fault is not None or self._closed:
                return
            if sequence <= self._last_sequence:
                self._fault = 'non-increasing control sequence'
                self._pending.clear()
                return
            self._last_sequence = sequence
            self._pending.append((received, payload))
            keep = {len(self._pending) - 1}
            for key in ('stop', 'reset'):
                for i in range(len(self._pending) - 1, -1, -1):
                    data = self._pending[i][1]
                    if (data[self.enabled_key] is False if key == 'stop'
                            else data.get('reset_requested')):
                        keep.add(i)
                        break
            self.compacted += len(self._pending) - len(keep)
            self._pending = [self._pending[i] for i in sorted(keep)]

    def fail(self, reason):
        with self._lock:
            if self._fault is None:
                self._fault = reason
            self._pending.clear()

    def take(self):
        with self._lock:
            if self._fault is not None:
                raise ControlInputError(self._fault)
            batch = [line for line, _ in self._pending]
            self._pending.clear()
            if self._closed:
                batch.append(None)
            return batch

    def qsize(self):
        with self._lock:
            return len(self._pending)


def read_control_stream(lines, source=None):
    source = sys.stdin if source is None else source
    try:
        while True:
            line = source.readline(MAX_LINE_BYTES + 1)
            if not line:
                lines.put(None)
                return
            if len(line.encode('utf-8')) > MAX_LINE_BYTES:
                lines.fail('oversized control line')
                return
            if line.strip():
                lines.put(line)
    except Exception as error:
        lines.fail(f'control reader failed: {error}')


def control_batch(lines):
    if isinstance(lines, LatestControlInbox):
        return lines.take()
    # Compatibility with replay/test FIFOs: bound work per send tick.
    batch = []
    for _ in range(64):
        try:
            batch.append(lines.get_nowait())
        except queue.Empty:
            break
        if batch[-1] is None:
            break
    return batch


class AsyncFileLog:
    """Best effort bounded diagnostics; a slow disk cannot hold a motor lease.

    Opening errors are reported at startup. Later write errors/drop counts are
    exposed in stats. Only evidence may be dropped, never control samples.
    """
    def __init__(self, path: Path, mode='w', capacity=256):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._stream = self.path.open(mode, encoding='utf-8')
        self._queue = queue.Queue(maxsize=capacity)
        self._closing = threading.Event()
        self.dropped = 0
        self.error = None
        self._worker = threading.Thread(target=self._run, name='teleop-log', daemon=True)
        self._worker.start()

    def write(self, text):
        if self._closing.is_set() or self.error is not None or len(text.encode('utf-8')) > MAX_LINE_BYTES:
            self.dropped += 1
            return len(text)
        try:
            self._queue.put_nowait(text)
        except queue.Full:
            self.dropped += 1
        return len(text)

    def flush(self):
        # Disk flush belongs to the worker, never to the control caller.
        pass

    def _run(self):
        try:
            while not self._closing.is_set() or not self._queue.empty():
                try:
                    text = self._queue.get(timeout=.05)
                except queue.Empty:
                    continue
                self._stream.write(text)
                self._stream.flush()
        except Exception as error:
            self.error = str(error)
        finally:
            try:
                self._stream.close()
            except Exception as error:
                self.error = self.error or str(error)

    def stats(self):
        return dict(dropped=self.dropped, pending=self._queue.qsize(), error=self.error,
                    worker_alive=self._worker.is_alive())

    def close(self, timeout_s=.2):
        self._closing.set()
        self._worker.join(timeout_s)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tee-log', type=Path, required=True)
    args = parser.parse_args()
    log = AsyncFileLog(args.tee_log)
    try:
        for line in sys.stdin:
            sys.stdout.write(line)
            sys.stdout.flush()
            log.write(line)
    except BrokenPipeError:
        return 2
    finally:
        log.close()
        stats = log.stats()
        if stats['dropped'] or stats['error'] or stats['pending']:
            print(f'[LOG] {args.tee_log}: {json.dumps(stats)}', file=sys.stderr)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
