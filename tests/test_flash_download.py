# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
"""Release download against a local server: redirect, cache, signature and HTTP errors."""
import argparse
import os
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import sys
import tempfile
import threading

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools/update'))
import flash_board as fb


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('package', type=Path)
    parser.add_argument('public_key', type=Path)
    args = parser.parse_args()
    key = fb.load_key(args.public_key)
    good = args.package.read_bytes()
    bad = good[:-1] + bytes([good[-1] ^ 1])
    state = {'hits': 0}

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            state['hits'] += 1
            if self.path == '/releases/latest/download/infrabot_firmware.ifw':
                self.send_response(302)
                self.send_header('Location', '/releases/download/v0.1.1/infrabot_firmware.ifw')
                self.end_headers()
            elif self.path == '/releases/download/v0.1.1/infrabot_firmware.ifw':
                self.send_response(200)
                self.send_header('Content-Length', str(len(good)))
                self.end_headers()
                self.wfile.write(good)
            elif self.path == '/releases/download/v0.1.2/infrabot_firmware.ifw':
                self.send_response(200)
                self.send_header('Content-Length', str(len(bad)))
                self.end_headers()
                self.wfile.write(bad)
            else:
                self.send_response(404)
                self.end_headers()

        def log_message(self, *_):
            pass

    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    base = f'http://127.0.0.1:{server.server_address[1]}/releases'
    os.environ['no_proxy'] = os.environ['NO_PROXY'] = '127.0.0.1'
    tmp = tempfile.TemporaryDirectory()
    cache = Path(tmp.name) / 'cache'
    try:
        cached = fb.download_release('latest', key, base, cache)
        assert cached.name == 'infrabot_firmware.ifw' and cached.parent.name == 'v0.1.1'
        assert cached.read_bytes() == good
        hits = state['hits']
        again = fb.download_release('v0.1.1', key, base, cache)
        assert again == cached and state['hits'] == hits, 'a cached release must not be downloaded again'
        try:
            fb.download_release('v0.1.2', key, base, cache)
        except ValueError as exc:
            assert 'authentication' in str(exc), exc
        else:
            raise AssertionError('a tampered package must be rejected')
        try:
            fb.download_release('v9.9.9', key, base, cache)
        except ValueError as exc:
            assert 'HTTP 404' in str(exc), exc
        else:
            raise AssertionError('a missing release must be reported')
        try:
            fb.download_release('latest-beta', key, base, cache)
        except ValueError as exc:
            assert '版本号' in str(exc), exc
        else:
            raise AssertionError('a malformed version must be rejected before any request')
    finally:
        server.shutdown()
        tmp.cleanup()
    print('Release download: latest redirect, cache, tampered package, missing release and version checks passed.')


if __name__ == '__main__':
    main()
