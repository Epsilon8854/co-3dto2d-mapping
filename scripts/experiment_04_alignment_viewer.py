#!/usr/bin/env python3
"""Local live inspection of the exact patches used by startup registration.

Reads already-saved search cases; subscribes only to the small alignment status.
Never subscribes to point clouds or occupancy maps and never changes alignment.
"""
import argparse
import base64
from collections import OrderedDict
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import signal
import threading
import time
from urllib.parse import parse_qs, urlsplit
import webbrowser

import numpy as np

from experiment_04_dds import prepare_environment

ROOT = Path(__file__).resolve().parents[1]
PAGE = ROOT / 'web/experiment_04_startup_alignment.html'


class Inspection:
    def __init__(self, directory):
        self.directory = directory
        self.lock = threading.Lock()
        self.live = {}
        self.received = None
        self.cache = OrderedDict()

    def status(self, message):
        values = {}
        for status in message.status:
            for item in status.values:
                try:
                    values[item.key] = json.loads(item.value)
                except ValueError:
                    values[item.key] = item.value
        with self.lock:
            self.live = values
            self.received = time.monotonic()

    def index(self):
        with self.lock:
            live = dict(self.live)
            age = time.monotonic() - self.received if self.received is not None else None
        cases = []
        paths = [p for p in (self.directory / 'alignment_cases').glob('*/result.json') if p.parent.name.isdigit()]
        for path in sorted(paths, key=lambda p: int(p.parent.name)):
            if not all((path.parent / f'r{i}_patch.npz').is_file() for i in (0, 1)):
                continue
            try:
                result = json.loads(path.read_text())
                cases.append(dict(id=path.parent.name, generation=result.get('generation'),
                                  accepted=result.get('accepted', False), reason=result.get('reason'),
                                  stamp_ns=result.get('stamps', [None])[0]))
            except (OSError, ValueError):
                continue
        config = {}
        try:
            config = json.loads((self.directory / 'effective_central_config.json').read_text())
        except (OSError, ValueError):
            pass
        return dict(run=str(self.directory), live=live, status_age_sec=age,
                    cases=cases, config=config, server_time=time.time())

    def case(self, identifier):
        if not identifier.isdigit() or len(identifier) > 12:
            raise ValueError('Invalid case identifier')
        directory = self.directory / 'alignment_cases' / identifier
        paths = [directory / 'result.json', directory / 'r0_patch.npz', directory / 'r1_patch.npz']
        signature = tuple(p.stat().st_mtime_ns for p in paths)
        with self.lock:
            cached = self.cache.get(identifier)
            if cached and cached[0] == signature:
                self.cache.move_to_end(identifier)
                return cached[1]
        data = dict(id=identifier, result=json.loads(paths[0].read_text()), patches=[])
        for path in paths[1:]:
            with np.load(path, allow_pickle=False) as saved:
                grid = np.asarray(saved['grid'], dtype=np.int8)
                data['patches'].append(dict(width=grid.shape[1], height=grid.shape[0],
                    resolution=float(saved['resolution']), origin=saved['origin'].tolist(),
                    grid=base64.b64encode(grid.tobytes()).decode('ascii'),
                    boundary=np.round(saved['boundary'], 4).tolist()))
        encoded = json.dumps(data, allow_nan=False, separators=(',', ':')).encode()
        with self.lock:
            self.cache[identifier] = (signature, encoded)
            self.cache.move_to_end(identifier)
            while len(self.cache) > 4:
                self.cache.popitem(last=False)
        return encoded


class Handler(BaseHTTPRequestHandler):
    def __init__(self, *args, inspection, **kwargs):
        self.inspection = inspection
        super().__init__(*args, **kwargs)

    def do_GET(self):
        request = urlsplit(self.path)
        try:
            if request.path == '/':
                data, content_type = PAGE.read_bytes(), 'text/html; charset=utf-8'
            elif request.path == '/api/index':
                data = json.dumps(self.inspection.index(), allow_nan=False).encode()
                content_type = 'application/json'
            elif request.path == '/api/case':
                identifier = parse_qs(request.query).get('id', [''])[0]
                data, content_type = self.inspection.case(identifier), 'application/json'
            elif request.path == '/favicon.ico':
                self.send_response(204); self.end_headers(); return
            else:
                self.send_error(404); return
        except (OSError, ValueError, KeyError) as exc:
            self.send_error(503, str(exc)); return
        self.send_response(200)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, *_):
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-directory', type=Path, required=True,
                        help='Fusion output directory containing alignment_cases/')
    parser.add_argument('--port', type=int, default=0)
    parser.add_argument('--no-open-browser', action='store_true')
    parser.add_argument('--owner-pid', type=int, help='stop the attached viewer when this launcher exits')
    args = parser.parse_args()
    directory = args.run_directory.expanduser().resolve()
    if not directory.is_dir():
        parser.error('run-directory must already exist')
    if not 0 <= args.port <= 65535:
        parser.error('invalid port')
    if not PAGE.is_file():
        parser.error(f'viewer page missing: {PAGE}')
    owner_stat = Path(f'/proc/{args.owner_pid}/stat') if args.owner_pid is not None else None
    try:
        owner_start = owner_stat.read_text().split()[21] if owner_stat is not None else None
    except OSError:
        parser.error('the owning launcher has already stopped')
    prepare_environment()
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node = rclpy.create_node(f'experiment04_alignment_viewer_{os.getpid()}')
    inspection = Inspection(directory)
    subscription = node.create_subscription(DiagnosticArray, '/multi_agent/alignment_status', inspection.status,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    def spin():
        while not stop.is_set():
            rclpy.spin_once(node, timeout_sec=.2)
    ros_thread = threading.Thread(target=spin, daemon=True)
    server = ThreadingHTTPServer(('127.0.0.1', args.port), partial(Handler, inspection=inspection))
    server.timeout = .25
    ros_thread.start()
    url = f'http://127.0.0.1:{server.server_port}/'
    location = dict(url=url, pid=os.getpid(), run_directory=str(directory))
    (directory / 'alignment_viewer.json').write_text(json.dumps(location, indent=2))
    print('Live startup alignment monitor: ' + url, flush=True)
    print('Reads saved registration patches; only alignment_status is subscribed.', flush=True)
    if not args.no_open_browser:
        threading.Thread(target=lambda: webbrowser.open(url), daemon=True).start()
    try:
        while not stop.is_set():
            if owner_stat is not None:
                try:
                    if owner_stat.read_text().split()[21] != owner_start:
                        break
                except OSError:
                    break
            server.handle_request()
    finally:
        stop.set(); server.server_close(); ros_thread.join(timeout=2)
        node.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()
