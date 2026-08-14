"""완성된 화면을 브라우저로 확인할 때 쓰는 개발 서버.

**기본 `http.server`를 그냥 쓰면 안 된다.** charset을 안 붙여서 한글이 windows-1252로
깨진다. 화면이 통째로 이상해 보여 코드를 뒤지게 되는데 원인은 서버다.

세션 스크래치패드에 두었다가 몇 번을 다시 만들었다(그 디렉터리는 지워진다). 리포에 둔다.

사용: python3 core/serve.py [프로젝트]     기본 capstone-pick, 포트 8731
"""
import functools
import http.server
import os
import pathlib
import socketserver
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
PORT = int(os.environ.get('DASH_PORT', 8731))


class Handler(http.server.SimpleHTTPRequestHandler):
    def guess_type(self, path):
        t = super().guess_type(path)
        return t + '; charset=utf-8' if t.startswith('text/') else t

    def log_message(self, *a):
        pass                       # 프레임마다 찍히면 로그가 화면을 덮는다


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else 'capstone-pick'
    home = ROOT / 'projects' / name
    if not (home / 'dashboard.html').exists():
        raise SystemExit(f'빌드된 화면이 없다: {home / "dashboard.html"}\n'
                         f'  python3 dash.py {name} all')
    socketserver.TCPServer.allow_reuse_address = True
    handler = functools.partial(Handler, directory=str(home))
    with socketserver.TCPServer(('127.0.0.1', PORT), handler) as s:
        print(f'http://127.0.0.1:{PORT}/dashboard.html  ({home})')
        s.serve_forever()


if __name__ == '__main__':
    main()
