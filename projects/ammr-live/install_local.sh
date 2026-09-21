#!/usr/bin/env bash
# Install one committed release; no Pi commands, ROS, or robot motion.
set -euo pipefail
repo_root=$(git -C "$(dirname "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)
git -C "$repo_root" diff --quiet HEAD -- projects/ammr-live deploy || {
  echo '배포 파일을 먼저 검증하고 커밋하세요.' >&2; exit 1;
}
if [[ -n "$(git -C "$repo_root" ls-files --others --exclude-standard -- projects/ammr-live deploy)" ]]; then
  echo '배포 경로의 미추적 파일을 먼저 확인하세요.' >&2; exit 1
fi
revision=$(git -C "$repo_root" rev-parse HEAD)
app_root="$HOME/.local/share/robot-dashboard"
unit_root="$HOME/.config/systemd/user"
if [[ -e "$app_root/current" && ! -L "$app_root/current" ]]; then
  echo 'current가 심볼릭 링크가 아닙니다. 기존 경로를 덮어쓰지 않습니다.' >&2; exit 1
fi
mkdir -p "$app_root/releases" "$unit_root"
release=$(mktemp -d "$app_root/releases/${revision:0:12}.XXXXXX")
git -C "$repo_root" archive HEAD projects/ammr-live deploy | tar -x -C "$release"
python3 - "$release/projects/ammr-live" "$revision" <<'PY'
import hashlib
import json
import pathlib
import sys
folder = pathlib.Path(sys.argv[1])
files = {name: hashlib.sha256((folder / name).read_bytes()).hexdigest()
         for name in ('dashboard.html', 'dashboard.js', 'web_server.py')}
(folder / 'version.json').write_text(json.dumps({
    'repository': 'mmporong/robot-dashboard', 'version': sys.argv[2],
    'files': files}, indent=2) + '\n', encoding='utf-8')
PY
systemd-analyze --user verify "$release/deploy/systemd/robot-dashboard.service" \
  "$release/deploy/systemd/robot-dashboard-link.service"
install -m644 "$release/deploy/systemd/robot-dashboard.service" "$unit_root/robot-dashboard.service"
install -m644 "$release/deploy/systemd/robot-dashboard-link.service" "$unit_root/robot-dashboard-link.service"
install -Dm644 "$release/deploy/robot-dashboard.desktop" "$HOME/.local/share/applications/robot-dashboard.desktop"
ln -s "$release" "$app_root/current.next.$$"
mv -Tf "$app_root/current.next.$$" "$app_root/current"
systemctl --user daemon-reload
systemctl --user enable robot-dashboard.service robot-dashboard-link.service
systemctl --user restart robot-dashboard-link.service robot-dashboard.service
python3 - "$revision" <<'PY'
import json
import sys
import time
from urllib.error import URLError
from urllib.request import urlopen
for attempt in range(10):
    try:
        with urlopen('http://127.0.0.1:8090/version', timeout=1) as response:
            version = json.load(response).get('version')
        if version == sys.argv[1]:
            break
    except (OSError, URLError, ValueError):
        pass
    time.sleep(0.3)
else:
    raise SystemExit('실행 버전 확인 실패: robot-dashboard.service 로그를 확인하세요.')
PY
echo "설치한 버전: $revision"
echo '접속 주소: http://127.0.0.1:8090'
echo '현재 버전: http://127.0.0.1:8090/version'
