# robot-dashboard 작업 규칙

- 기본 관제는 `projects/ammr-live/dashboard.html` + `dashboard.js`이며 2026-09-21 승인한 화이트 레이아웃을 유지한다.
- 요청이 없으면 기본 화면을 다른 프로젝트의 템플릿·실험 화면으로 교체하지 않는다. 새 센서도 이 화면의 기존 패널 구조에 연결한다.
- 기본 설치 진입점은 `projects/ammr-live/install_local.sh`, 접속 주소는 localhost:8090이다. 현재 실행 버전은 `/version`으로 확인한다.
- `web_server.py`는 노트북의 읽기 전용 화면·프록시이고 `ammr_dashboard.py`는 Pi의 ROS 수집기다. UI 수정 때문에 베이스·센서·주행 서비스를 재시작하지 않는다.
- 배포는 검증한 커밋의 릴리스 스냅샷으로만 한다. 실행 디렉터리에서 수동 편집하거나 데이터로 새 기본 HTML을 생성하지 않는다.
- 기존 프로젝트·실험 브랜치·미커밋 파일은 보존한다. 이번 정본 전환을 이유로 팔 제어·기록 기능을 삭제하거나 실행하지 않는다.
- 화면 변경은 Python/JavaScript 관련 테스트와 데스크톱·모바일 브라우저 검증 후 별도 reviewer/verifier 승인을 받는다.
- 원격 push와 로봇 이동은 구분한다. 관제 변경 요청은 실물 이동 명령 권한이 아니다.
