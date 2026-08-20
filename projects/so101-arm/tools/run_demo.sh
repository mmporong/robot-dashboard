#!/usr/bin/env bash
# SO-101 픽앤플레이스 데모 일괄 실행 (2026-08-20)
#
#   휴지(접힘) → 펴기 → 파지 → 운반 → 통 투하 → 휴지(접힘·그리퍼 다묾)
#   + 녹화 3종: 손목캠 · 뎁스캠 정면 · MuJoCo 미러
#
# 전제: 패널 서버(8765)·뎁스 데몬(8766) 가동, 팔은 휴지 자세(토크 무관),
#       빨간 체스말이 픽업 존에 놓여 있을 것 (뎁스캠 검출을 스스로 확인한다)
# 사용: bash ~/so101_tools/run_demo.sh
# 중단: Ctrl-C → 팔 정지(토크 유지) 후 녹화 마감까지 하고 종료
set -u
TOOLS="$HOME/so101_tools"
OUT="$TOOLS/media/$(date +%Y-%m-%d)"
TS="$(date +%H%M%S)"
SIMPY="$HOME/miniforge3/envs/rlwalk/bin/python"
API="http://127.0.0.1:8765"
mkdir -p "$OUT"
PIDS=()

say() { echo; echo "══ $*"; }

finalize() {
    say "녹화 마감"
    for p in "${PIDS[@]:-}"; do kill -INT "$p" 2>/dev/null; done
    sleep 3
    for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null; done
    if [ -d "$OUT/demo_${TS}_simframes" ]; then
        ffmpeg -y -loglevel error -framerate 10 \
            -i "$OUT/demo_${TS}_simframes/f%05d.png" \
            -c:v libx264 -pix_fmt yuv420p "$OUT/demo_${TS}_sim.mp4" \
            && rm -rf "$OUT/demo_${TS}_simframes"
    fi
    echo "산출물:"
    ls -la "$OUT"/demo_${TS}_*.mp4 2>/dev/null || echo "  (영상 없음)"
}

on_int() {
    echo "⚠ 중단 요청 — 팔 정지(토크 유지)"
    curl -s -m 5 -X POST "$API/cmd" -H 'Content-Type: application/json' \
         -d '{"op":"stop"}' >/dev/null
    finalize
    exit 1
}
trap on_int INT TERM

say "사전 점검"
st=$(curl -s -m 5 "$API/state") || { echo "패널 서버(8765) 응답 없음"; exit 1; }
echo "$st" | grep -q '"connected": true' || {
    echo "팔 미연결 — 연결 시도"
    curl -s -m 30 -X POST "$API/cmd" -H 'Content-Type: application/json' \
         -d '{"op":"connect"}' >/dev/null; sleep 4
    curl -s -m 5 "$API/state" | grep -q '"connected": true' \
        || { echo "연결 실패 — 전원·USB 확인"; exit 1; }
}
curl -s -m 5 "$API/blob" | grep -q '"u"' \
    || { echo "빨간 물체 미검출 — 체스말을 픽업 존에 놓고 다시 실행"; exit 1; }
echo "연결·물체 검출 OK"

say "녹화 시작 (통째로 — 마감 전에도 유효한 fragmented mp4)"
FR="-movflags +frag_keyframe+empty_moov"
ffmpeg -y -loglevel error -f mpjpeg -i "$API/cam" -t 900 \
       -c:v libx264 -pix_fmt yuv420p $FR "$OUT/demo_${TS}_wrist.mp4" & PIDS+=($!)
ffmpeg -y -loglevel error -f mpjpeg -i "$API/rgb" -t 900 \
       -c:v libx264 -pix_fmt yuv420p $FR "$OUT/demo_${TS}_rgb.mp4" & PIDS+=($!)
( cd "$TOOLS/sim" && exec "$SIMPY" -u sim_view.py \
      --record "$OUT/demo_${TS}_simframes" --seconds 900 ) \
      > "$OUT/demo_${TS}_simrec.log" 2>&1 & PIDS+=($!)
sleep 3

fail=""
run_stage() {
    local name="$1"; shift
    say "$name"
    if ! "$@"; then fail="$name"; return 1; fi
}

run_stage "토크 ON" curl -sf -m 15 -X POST "$API/cmd" \
    -H 'Content-Type: application/json' -d '{"op":"torque","on":true}' -o /dev/null \
&& sleep 2 \
&& run_stage "펴기 (unfold_safe)" timeout 420 python3 "$TOOLS/unfold_safe.py" \
&& run_stage "파지 (pick_demo lying)" timeout 420 python3 "$TOOLS/pick_demo.py" lying \
&& run_stage "운반·투하 (drop_to_box)" timeout 240 python3 "$TOOLS/drop_to_box.py" \
&& run_stage "파킹 (park)" timeout 420 python3 "$TOOLS/park.py"

finalize
if [ -n "$fail" ]; then
    echo
    echo "⚠ '$fail' 단계에서 실패 — 팔은 각 스크립트의 안전 규약대로 정지(토크 유지)"
    echo "  상태 확인: curl -s $API/state | python3 -m json.tool | head -30"
    exit 1
fi
echo
echo "✅ 데모 완주 — 팔은 휴지 자세(토크 OFF·그리퍼 다묾)"
