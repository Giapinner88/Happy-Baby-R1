#!/usr/bin/env python3
"""One JSON request to the R1 agent on loopback; clocks and files are local."""
import argparse
import json
from pathlib import Path
import re
import socket
import sys


def call_agent(payload, port=5572, timeout_s=55):
    if not 1 <= port <= 65535:
        raise ValueError('invalid agent port')
    with socket.create_connection(('127.0.0.1', port), 3) as connection:
        connection.settimeout(timeout_s)
        connection.sendall((json.dumps(payload) + '\n').encode('utf-8'))
        line = connection.makefile('r').readline(2000000)
        if not line:
            raise RuntimeError('agent disconnected')
    result = json.loads(line)
    if result.get('error'):
        raise RuntimeError(str(result['error']))
    return result['result']


def bridge(payload, port=5572):
    method = payload.get('method')
    if method not in {'clock', 'state', 'rest_arms', 'move_head', 'move_body',
                      'capture', 'execute_target', 'stop', 'release'}:
        raise ValueError('unknown agent method')
    result = call_agent(payload, port)
    if method == 'capture':
        directory = result.get('capture_dir', '')
        if not re.fullmatch(r'/tmp/r1_p4_capture_[a-f0-9]+', directory):
            raise ValueError('unexpected capture directory')
        path = Path(directory)
        if not (path/'frame.jpg').is_file() or not (path/'capture.json').is_file():
            raise FileNotFoundError('capture files incomplete')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=5572)
    args = parser.parse_args()
    try:
        print(json.dumps({'result': bridge(json.load(sys.stdin), args.port)}))
    except Exception as exc:
        print(json.dumps({'error': str(exc)}))


if __name__ == '__main__':
    main()
