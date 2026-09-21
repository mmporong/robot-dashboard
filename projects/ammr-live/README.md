# AMMR 기본 관제

2026-09-21에 승인한 화이트 화면을 이 저장소의 기본 관제로 사용한다.
HTML·JavaScript·파이 데이터 수집기는 이 디렉터리에서 관리한다.
기준 소스는 `mmporong/bimanual-robot` 작업 커밋 `43d4afc`의 `tools/ammr_dashboard/`다.

## 평소에는 주소만 연다

`http://127.0.0.1:8090` 또는 앱 목록의 **AMMR 관제**를 연다.
현재 설치 버전과 화면 파일 해시는 `http://127.0.0.1:8090/version`에서 확인한다.

- 노트북의 `robot-dashboard.service`가 고정 화면을 제공한다. ROS 설치가 필요 없다.
- `robot-dashboard-link.service`가 파이 데이터를 SSH로 연결하고 끊기면 재시도한다.
- 파이가 꺼져 있어도 화면은 열린다. 실시간 수치·영상만 미수신으로 표시한다.
- 기존 파이 `jdamr-ammr-dashboard.service`는 센서와 주행 상태를 읽는다. 관제는 로봇 명령을 보내지 않는다.

노트북이 꺼져 있으면 이 주소에는 접속할 수 없다. 이 구성은 로컬 관제 웹앱이며
외부 공개 사이트가 아니다. GitHub에는 코드와 설치 정의를 보관하고, 카메라 영상이나
ROS 데이터를 올리지 않는다. GitHub에 코드를 push하는 것과 실행 중인 화면 배포는 별개다.

## 설치와 업데이트

```bash
cd "$HOME/robot-dashboard"
bash projects/ammr-live/install_local.sh
```

작업용 worktree에서는 그 worktree로 이동한 뒤 같은 스크립트를 실행한다.
커밋된 파일만 릴리스 디렉터리에 설치하므로 브랜치를 바꾸거나 실험 파일을 고쳐도
실행 중인 레이아웃은 달라지지 않는다. 검증 후 커밋하고 설치할 때만 화면이 갱신된다.
이전 릴리스 디렉터리는 자동 삭제하지 않는다.

기존 수동 SSH 터널이 8090을 쓰면 그 연결만 종료한 뒤 설치한다. 설치 스크립트가
임의의 프로세스를 찾아 종료하지 않는다. 파이·베이스·센서·모터 서비스도 재시작하지 않는다.

설치 위치:

- 실행본: `$HOME/.local/share/robot-dashboard/releases/<commit>.<release>/`
- 현재 실행본 링크: `$HOME/.local/share/robot-dashboard/current`
- 사용자 서비스: `$HOME/.config/systemd/user/robot-dashboard*.service`
- 앱 실행 항목: `$HOME/.local/share/applications/robot-dashboard.desktop`

SSH 대상은 `jdamr.local`이고 로컬 사용자와 같은 사용자명으로 연결한다.
mDNS가 되는 같은 네트워크, 기존에 검증한 호스트 키, 비대화형 SSH 인증이 필요하다.
연결만 localhost:18091에서 파이 localhost:8090으로 전달한다. 외부 인터페이스에는 바인딩하지 않는다.
사용자 서비스는 로그인 때 실행되며 lingering이 활성화된 PC에서는 로그인 전에도 실행될 수 있다.

## 화면을 바꿀 때

수정할 곳은 `dashboard.html`과 `dashboard.js`다. 임시 HTML을 만들어 기본 화면 대신
띄우지 않는다. 데이터 부족 때문에 레이아웃을 바꾸지 않고 기존 패널에 미수신을 표시한다.
예전 MCAP 재생 화면은 `projects/capstone-pick/`, SLAM·팔의 실험 브랜치에 보존한다.

Pi 수집기의 설치 경로는 Pi의 `$HOME/ammr_dashboard/`다. `ammr_dashboard.py`와
`host_metrics.py`도 이 저장소를 원본으로 사용하며 Pi 서비스 정의는
`deploy/systemd/jdamr-ammr-dashboard.service`에 있다. 이 unit은 현재 Pi 사용자 `lim`의
경로를 사용하므로 다른 기기에 복사할 때 사용자·경로를 확인한다. 수집기 변경이 있을 때만
검증한 파일을 Pi에 옮기고 관제 서비스 하나를 재시작한다. 화면 변경은 노트북 설치만으로
반영되며 Pi 쪽 HTML을 별도 수정하지 않는다. 이미 배포된 예전 HTML은 별도 진입점으로 안내하지 않는다.

```bash
cd "$HOME/robot-dashboard"
python3 -m unittest discover -s projects/ammr-live -p 'test_*.py'
node --test projects/ammr-live/test_dashboard_ui.cjs
node --check projects/ammr-live/dashboard.js
bash -n projects/ammr-live/install_local.sh
```

수집기 단위 테스트에는 NumPy·OpenCV가 필요하다. 웹 서버 자체는 Python 표준 라이브러리만 쓴다.
상하 정렬, 모바일 가로 넘침, 노드·토픽 내부 스크롤, 연결 단절 표시를 브라우저에서 확인한다.
프로세스 자동 복구와 부팅 설정 확인은 실제 재부팅 시험과 구분해 기록한다.

## 상태와 복구

```bash
systemctl --user status robot-dashboard.service robot-dashboard-link.service
journalctl --user -u robot-dashboard.service -u robot-dashboard-link.service -n 40 --no-pager
```

실행본이 잘못 바뀌었다면 확인한 이전 릴리스로 `current` 링크를 되돌린 뒤
`systemctl --user restart robot-dashboard.service`로 웹 화면만 다시 실행한다.
서버·포트 설정까지 바뀐 버전은 그 버전의 unit 파일도 함께 복원한다.

## 표시의 범위

파이 CPU·온도·디스크·서비스, RGB-D·라이다, 주행 차단 근거, ROS 노드·토픽을 표시한다.
수신한 경우에만 점유 지도·위치·계획 경로를 그린다. 지도 생성이나 3D SLAM을 대신 수행하지 않는다.
ROS 목록에 토픽이 있다고 메시지 발행·수신을 보장하지 않는다. 같은 이름의 노드는
관측 수를 표시하지만 중복 프로세스라고 단정하지 않는다.

CPU는 전체 코어 합산 100% 기준이다. 온도 경고 기준은 80°C, 디스크는 사용률 90%다.
낮은 주기의 수치 조회뿐 아니라 RGB-D 수신과 영상 변환에도 파이 부하가 발생한다.
웹 화면의 영속성과 센서 데이터 녹화·장기 보관은 별개다.
