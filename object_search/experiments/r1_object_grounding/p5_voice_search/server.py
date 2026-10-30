#!/usr/bin/env python3
"""Loopback voice-to-P4 bridge on the workstation; one search at a time."""
import argparse
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import unicodedata

ROOT = Path(__file__).resolve().parents[3]
P4 = ROOT / 'experiments/r1_object_grounding/p4_search/run.py'
LOCK = threading.Lock()
ACTIVE = None
PENDING = None


def _plain(value):
    return ''.join(ch for ch in unicodedata.normalize('NFD',value.lower())
                   if unicodedata.category(ch)!='Mn').replace('đ','d')


def choose_cached_candidate(query,candidates):
    """Resolve only an explicit left/right or large/small reply."""
    words=_plain(query).split()
    terms=set(words)
    centers=[(c['bbox']['x_min']+c['bbox']['x_max'])/2 for c in candidates]
    if 'trai' in terms and 'phai' not in terms:
        return centers.index(min(centers)) if abs(centers[0]-centers[1])>=5 else None
    if 'phai' in terms and 'trai' not in terms:
        return centers.index(max(centers)) if abs(centers[0]-centers[1])>=5 else None
    areas=[(c['bbox']['x_max']-c['bbox']['x_min'])*
           (c['bbox']['y_max']-c['bbox']['y_min']) for c in candidates]
    if min(areas)/max(areas)>.8:return None
    if 'to' in terms or 'lon' in terms:
        return areas.index(max(areas))
    if 'nho' in terms or 'be' in terms:
        return areas.index(min(areas))
    return None


def _same_requested_object(query,original):
    skip={'tim','chi','vao','cai','chiec','con','vat','ben','trai','phai','to','nho','lon','be','va'}
    nouns={word for word in _plain(original).split() if len(word)>=3 and word not in skip}
    return bool(nouns & set(_plain(query).split()))


def progress_snapshot(active):
    """Report only completed scan rounds; a captured frame is not a verdict."""
    if not active:
        return {'status': 'idle', 'front_scan_complete': False}
    events = active['output'] / 'events.jsonl'
    front_scan_complete = False
    if events.is_file():
        for line in events.read_text().splitlines():
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                continue
            if (event.get('state') == 'SCAN_ROUND_RESULT'
                    and event.get('body_offset_deg') == 0
                    and event.get('decision') == 'not_found'
                    and event.get('views_processed') == 3):
                front_scan_complete = True
    return {'status': 'running', 'front_scan_complete': front_scan_complete}


def run_search(query, *, robot=None, adapter_argv=None, runtime_root=None,
               bbox_config=None, vision_config=None, output_root,
               head_only=False, body_turns=1, body_step_deg=None,
               batch_head_capture=False):
    """Execute bounded P4; result.json is authoritative even if CLI exits nonzero."""
    if not isinstance(query, str) or not 1 <= len(query.strip()) <= 180:
        return {'status': 'rejected', 'answer': 'Yêu cầu tìm vật chưa rõ.'}
    global PENDING
    pending=PENDING
    if pending and time.monotonic()-pending['saved_at']>60:
        PENDING=None;pending=None
    choice=None
    full_request=bool({'tim','chi'} & set(_plain(query).split()))
    if pending and (not full_request or _same_requested_object(query,pending['query'])):
        choice=choose_cached_candidate(query,pending['candidates'])
        if choice is None:
            # A new full request supersedes the clarification; an unclear short reply does not.
            if len(query.split())<=5 and not full_request:
                return {'status':'needs_clarification','answer':pending['question'],
                        'run':str(pending['run']),'cached_candidates':2}
            PENDING=None;pending=None
    elif pending:
        PENDING=None;pending=None
    if choice is None and not os.environ.get('OPENAI_API_KEY', '').strip():
        return {'status': 'unavailable', 'answer': 'Máy xử lý ảnh chưa có OpenAI API key.'}
    output_root = Path(output_root).resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    name = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ') + '_voice'
    output = output_root / name
    cancel = output_root / (name + '.cancel')
    command = [sys.executable, str(P4), '--query', query.strip(),
               '--cancel-file', str(cancel), '--output-dir', str(output)]
    if choice is not None:
        command.extend(['--resume-run',str(pending['run']),'--candidate-index',str(choice)])
    if adapter_argv:
        command.extend(['--adapter-argv', str(adapter_argv)])
    elif robot:
        command.extend(['--robot', robot])
    else:
        return {'status': 'unavailable', 'answer': 'Chưa cấu hình agent tìm vật.'}
    if runtime_root:
        command.extend(['--runtime-root', str(runtime_root)])
    if bbox_config:
        command.extend(['--bbox-config', str(bbox_config)])
    if vision_config:
        command.extend(['--vision-config', str(vision_config)])
    if head_only:
        command.append('--head-only')
    else:
        command.extend(['--body-turns', str(body_turns)])
    if body_step_deg is not None:
        command.extend(['--body-step-deg',str(body_step_deg)])
    if batch_head_capture:command.append('--batch-head-capture')
    global ACTIVE
    try:
        process = subprocess.Popen(command, cwd=ROOT, stdin=subprocess.DEVNULL,
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                   start_new_session=True)
        ACTIVE = {'process': process, 'cancel': cancel, 'output': output}
        try:
            transcript, _ = process.communicate(timeout=230)
        except subprocess.TimeoutExpired:
            cancel.touch()
            try:
                transcript, _ = process.communicate(timeout=12)
            except subprocess.TimeoutExpired:
                process.terminate()
                transcript, _ = process.communicate(timeout=5)
        (output_root / (name + '.log')).write_text(transcript[-100000:])
        result_path = output / 'result.json'
        if not result_path.is_file():
            return {'status': 'failed', 'answer': 'Tìm kiếm lỗi trước khi có kết quả; xem log.', 'run': str(output)}
        result = json.loads(result_path.read_text())
        status = result.get('status')
        if choice is not None:PENDING=None
        elif status=='needs_clarification':
            prior=result.get('pending_candidates')
            if prior and len(prior.get('candidates',[]))==2:
                PENDING={'run':output,'candidates':prior['candidates'],
                         'query':query,
                         'question':result.get('reason') or 'Bạn chọn vật nào?',
                         'saved_at':time.monotonic()}
        if status == 'found' and not isinstance(result.get('action'), dict):
            status = 'failed'
        if status == 'found' and result['action'].get('status') not in ('tracking_completed','command_completed'):
            status = 'failed'
        reason=str(result.get('reason') or '')
        failed_answer=('Mình đã xoay về phía mục tiêu nhưng cơ thể chưa ổn định để chụp và chỉ tay.'
                       if any(word in reason for word in ('waist yaw','motion did not settle','stale state'))
                       else 'Chưa hoàn tất tìm và chỉ vật; cần xem kết quả thử nghiệm.')
        answers = {
            'found': 'Đã tìm thấy vật, quay đầu và chỉ tay theo mục tiêu.',
            'needs_clarification': result.get('reason') or 'Bạn muốn chọn vật nào?',
            'needs_body_turn': result.get('reason'),
            'target_unreachable': result.get('reason'),
            'target_lost_after_recenter': result.get('reason'),
            'not_found_in_observed_views': 'Chưa thấy vật trong những hướng đã quét.',
            'cancelled': 'Đã dừng tìm kiếm.',
            'failed': failed_answer,
        }
        return {'status': status, 'answer': answers.get(status, 'Tìm kiếm không có kết quả hợp lệ.'),
                'run': str(output), 'observations': result.get('observations', 0),
                'cached_candidates':len(PENDING['candidates']) if status=='needs_clarification' and PENDING else 0,
                'planned_body_travel_deg': result.get('body_travel_deg', 0)}
    finally:
        ACTIVE = None
        cancel.unlink(missing_ok=True)


class Handler(BaseHTTPRequestHandler):
    def do_POST(self):
        global ACTIVE
        if self.path not in ('/find', '/cancel', '/progress'):
            return self.reply(404, {'status': 'unknown_request'})
        if self.headers.get('Content-Type') != 'application/json':
            return self.reply(415, {'status': 'invalid_content_type'})
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= 4096:
                raise ValueError('invalid body length')
            payload = json.loads(self.rfile.read(length))
        except (ValueError, json.JSONDecodeError):
            return self.reply(400, {'status': 'invalid_request'})
        if self.path == '/progress':
            return self.reply(200, progress_snapshot(ACTIVE))
        if self.path == '/cancel':
            active = ACTIVE
            if active:
                active['cancel'].touch()
            return self.reply(200, {'status': 'cancelling' if active else 'idle'})
        if not LOCK.acquire(blocking=False):
            return self.reply(409, {'status': 'busy', 'answer': 'Robot đang tìm vật khác.'})
        try:
            result = run_search(payload.get('query'), robot=self.server.robot,
                                adapter_argv=self.server.adapter_argv,
                                runtime_root=self.server.runtime_root,
                                bbox_config=self.server.bbox_config,
                                vision_config=self.server.vision_config,
                                output_root=self.server.output_root,
                                head_only=self.server.head_only,
                                body_turns=self.server.body_turns,
                                body_step_deg=self.server.body_step_deg,
                                batch_head_capture=self.server.batch_head_capture)
            return self.reply(200, result)
        except Exception:
            return self.reply(500, {'status': 'failed', 'answer': 'Lỗi kết nối tìm vật.'})
        finally:
            LOCK.release()

    def reply(self, code, value):
        data = json.dumps(value, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', default='unitree@10.42.0.114')
    parser.add_argument('--adapter-argv', type=Path,
                        help='Use local process adapter instead of SSH robot bridge')
    parser.add_argument('--runtime-root', type=Path)
    parser.add_argument('--bbox-config', type=Path)
    parser.add_argument('--vision-config', type=Path)
    parser.add_argument('--port', type=int, default=5590)
    parser.add_argument('--output-root', type=Path, default=ROOT/'data/runs/r1_object_grounding/p5_voice')
    parser.add_argument('--head-only', action='store_true')
    parser.add_argument('--body-turns', type=int, default=1)
    parser.add_argument('--body-step-deg', type=float)
    parser.add_argument('--batch-head-capture', action='store_true')
    args = parser.parse_args()
    if args.body_turns < 1 or args.body_turns > 5:
        parser.error('--body-turns must be 1..5')
    if args.body_step_deg is not None and not 0<args.body_step_deg<=180:
        parser.error('--body-step-deg must be in (0,180]')
    if args.adapter_argv:
        if not args.adapter_argv.is_file():parser.error('local adapter argv file missing')
        if not args.runtime_root or not args.bbox_config:parser.error('local adapter requires --runtime-root and --bbox-config')
        if not args.bbox_config.is_file():parser.error('bbox config missing')
        if args.vision_config and not args.vision_config.is_file():parser.error('vision config missing')
    server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    server.robot, server.output_root = args.robot, args.output_root
    server.adapter_argv, server.runtime_root = args.adapter_argv, args.runtime_root
    server.bbox_config, server.vision_config = args.bbox_config, args.vision_config
    server.head_only, server.body_turns = args.head_only, args.body_turns
    server.body_step_deg=args.body_step_deg
    server.batch_head_capture=args.batch_head_capture
    server.serve_forever()


if __name__ == '__main__':
    main()
