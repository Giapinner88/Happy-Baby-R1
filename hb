#!/usr/bin/env bash
# hb — một lệnh duy nhất để build, deploy và điều khiển service Happy Baby R1.
#
# Lớp mỏng: logic và các guard an toàn (DISARMED, preflight, SHA, xác thực
# sudo/SSH) nằm trong integration/scripts/* mà lệnh này gọi tới.
# Tự nhận biết đang chạy trên robot (aarch64) hay máy trạm; ép bằng HB_MODE=robot|dev.
# Có thể symlink vào PATH: ln -s "$PWD/hb" ~/.local/bin/hb
set -euo pipefail

HB_ROOT="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
OPS="$HB_ROOT/integration/scripts"
HIGH_DIR="$HB_ROOT/controller"
UV="${UV:-$HOME/.local/bin/uv}"
export HB_ROOT

case "${HB_MODE:-}" in
    robot|dev) MODE="$HB_MODE" ;;
    "") case "$(uname -m)" in aarch64|arm64) MODE=robot ;; *) MODE=dev ;; esac ;;
    *) echo "hb: HB_MODE phải là robot hoặc dev" >&2; exit 2 ;;
esac

usage() {
    cat <<USAGE
Usage: ./hb <lệnh> [đối tượng] [tùy chọn]        (đang chạy ở chế độ: $MODE)

HIỆN TRƯỜNG  (qua Tailscale, dùng được ở mọi mạng; HB_ROBOT_TAILSCALE=<ip> để đổi)
  field [--no-restart]         chờ robot online trên Tailscale -> deploy toàn bộ -> status
  ssh [lệnh...]                mở shell (hoặc chạy lệnh) trên robot qua Tailscale

DEPLOY  (máy trạm -> robot; robot tự dò, hoặc ROBOT=unitree@<ip>)
  deploy [all]                 backup + sync + build ARM64 + preflight + restart an toàn + teleop
  deploy all --no-restart      như trên nhưng không restart service nào
  deploy policy [--dry-run]    bundle walk/teleop + locomotion.yaml + gestures.yaml, restart high-level khi DISARMED
  deploy dance  [--dry-run]    policy + dance.yaml + policies/dance, một lần restart
  deploy high [--no-restart]   controller C++: sync + build + restart (cần DISARMED)
  deploy voice | integration | presets   từng phần, không đụng high-level
  deploy teleop                sync teleop + restart runtime Quest (không cần DISARMED)
  diff                         xem trước thay đổi, không ghi gì
  rollback                     khôi phục bản backup deploy gần nhất trên robot
  pull [--dry-run]             kéo cấu hình đã sửa trên robot về máy trạm

SERVICE  (từ máy trạm: qua SSH; trên robot khi debug: tại chỗ)
  start | stop | restart <all|high|integration|voice|presets|teleop>
  status
  logs [all|high|integration|voice|presets|teleop]      (mặc định: all)

BUILD / KIỂM TRA
  build [all|high|voice|integration] [--clean]
                               robot: build ARM64 như lúc deploy
                               máy trạm: build controller (run_r1 + test) cho preflight/test
  preflight                    kiểm tra model/config/asset, không xuất lệnh động cơ
  test [all|cpp|py]            ctest + pytest (HB_TELEOP_PYTHON=<python có aiohttp> cho test teleop)

CÀI ĐẶT TRÊN ROBOT (sudo)
  install [all|services|teleop]   cài/refresh unit systemd; teleop = runtime Quest
  teleop-check                 đọc rt/lowstate, không tạo publisher

KHÁC
  policy list                  liệt kê bundle policy đi bộ, đánh dấu walk (R2+A) / teleop (R2+X)
  policy add <file.onnx> <tên> [--contract C] [--teleop]
                               tạo controller/policies/locomotion/<tên>/ (chọn bằng locomotion.yaml)
  dance list | prune [--apply] | restore <tên>
  find-robot                   dò robot, in ROBOT=...
USAGE
}

die() { echo "hb: $*" >&2; exit 2; }
need_dev()   { [[ "$MODE" == dev ]]   || die "'$1' chạy từ máy trạm (HB_MODE=dev để ép)"; }
need_robot() { [[ "$MODE" == robot ]] || die "'$1' chạy trên robot (HB_MODE=robot để ép)"; }

ort_lib_path() {
    case "$(uname -m)" in
        aarch64|arm64) echo "$HIGH_DIR/thirdparty/onnxruntime_aarch64/lib" ;;
        *)             echo "$HIGH_DIR/thirdparty/onnxruntime/lib" ;;
    esac
}

# ------------------------------------------------------------------ build ----

build_dev_controller() {
    cmake -S "$HIGH_DIR" -B "$HIGH_DIR/build" -DCMAKE_BUILD_TYPE=Release
    if [[ "${1:-}" == "--clean" ]]; then
        cmake --build "$HIGH_DIR/build" --target clean
    fi
    cmake --build "$HIGH_DIR/build" --parallel "${HB_BUILD_JOBS:-$(nproc)}"
    [[ -x "$HIGH_DIR/build/run_r1" ]] || die "build xong nhưng thiếu $HIGH_DIR/build/run_r1"
}

cmd_build() {
    local target="all" clean=""
    for arg in "$@"; do
        case "$arg" in
            all|high|voice|integration) target="$arg" ;;
            --clean) clean="--clean" ;;
            *) die "usage: ./hb build [all|high|voice|integration] [--clean]" ;;
        esac
    done
    if [[ "$MODE" == robot ]]; then
        if [[ -n "$clean" ]]; then
            case "$target" in
                all) rm -rf "$HIGH_DIR/build" "$HB_ROOT/voice/unitree_bridge/build" "$HB_ROOT/integration/build" ;;
                high) rm -rf "$HIGH_DIR/build" ;;
                voice) rm -rf "$HB_ROOT/voice/unitree_bridge/build" ;;
                integration) rm -rf "$HB_ROOT/integration/build" ;;
            esac
        fi
        exec bash "$OPS/build_on_robot.sh" "$target"
    fi
    case "$target" in
        all|high) build_dev_controller "$clean" ;;
        integration)
            [[ -z "$clean" ]] || rm -rf "$HB_ROOT/integration/build"
            bash "$OPS/build_on_robot.sh" integration ;;
        voice) die "voice bridge cần SDK audio R1; build trên robot (./hb deploy voice)" ;;
    esac
}

# ------------------------------------------------------------------- test ----

test_cpp() {
    build_dev_controller
    ctest --test-dir "$HIGH_DIR/build" --output-on-failure
}

test_py() {
    local rc=0 teleop_py="${HB_TELEOP_PYTHON:-}"
    local runtime_py="$HOME/.local/share/hb/teleop/miniforge3/envs/hb_teleop/bin/python"
    # Teleop tests need the runtime deps (aiohttp, numpy): HB_TELEOP_PYTHON, then
    # the robot runtime env, then python3.
    if [[ -z "$teleop_py" ]]; then
        if [[ -x "$runtime_py" ]]; then teleop_py="$runtime_py"; else teleop_py=python3; fi
    fi
    (cd "$HB_ROOT/teleop" && PYTHONPATH="src:scripts:scripts/teleop:$HB_ROOT" \
        "$teleop_py" -m pytest tests -q -p no:cacheprovider --continue-on-collection-errors) || rc=1
    (cd "$HB_ROOT" && python3 -m pytest voice_presets/tests -q -p no:cacheprovider) || rc=1
    (cd "$HB_ROOT" && PYTHONPATH=voice "$UV" run --frozen --project voice --with pytest \
        python -m pytest voice/tests integration/tests -q -p no:cacheprovider) || rc=1
    return "$rc"
}

cmd_test() {
    local rc=0
    case "${1:-all}" in
        cpp) test_cpp ;;
        py)  test_py ;;
        all) test_cpp || rc=1; test_py || rc=1; return "$rc" ;;
        *)   die "usage: ./hb test [all|cpp|py]" ;;
    esac
}

cmd_preflight() {
    if [[ "$MODE" == robot ]]; then
        exec bash "$OPS/preflight.sh" all
    fi
    [[ -x "$HIGH_DIR/build/run_r1" ]] || build_dev_controller
    LD_LIBRARY_PATH="$(ort_lib_path)${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
        HB_PROJECT_DIR="$HIGH_DIR" exec "$HIGH_DIR/build/run_r1" --preflight
}

# ----------------------------------------------------------------- deploy ----

require_tmp_space() {
    # Full deploy runs a local model preflight; it needs room in /tmp.
    local min_mb="${HB_DEPLOY_MIN_TMP_MB:-2048}" avail_kb
    avail_kb="$(df -Pk /tmp | awk 'NR == 2 {print $4}')"
    if [[ ! "$avail_kb" =~ ^[0-9]+$ ]] || (( avail_kb < min_mb * 1024 )); then
        die "cần ít nhất ${min_mb} MiB trống trong /tmp cho preflight model; dọn /tmp trước"
    fi
}

cmd_deploy() {
    need_dev deploy
    local target="${1:-all}"
    [[ $# -gt 0 ]] && shift
    case "$target" in
        all)
            case "${1:-}" in
                "")           require_tmp_space; exec "$OPS/deploy_stack.sh" deploy --accept-policy ;;
                --no-restart) exec "$OPS/deploy_stack.sh" deploy --no-restart ;;
                *)            die "usage: ./hb deploy [all] [--no-restart]" ;;
            esac ;;
        policy|dance)
            # Local preflight uses the dev build of run_r1; build it when missing.
            [[ -x "$HIGH_DIR/build/run_r1" ]] || build_dev_controller
            if [[ "$target" == dance ]]; then
                exec "$OPS/deploy_policy.sh" --with-dance "$@"
            fi
            exec "$OPS/deploy_policy.sh" "$@" ;;
        high)        exec "$OPS/deploy_stack.sh" deploy-high "$@" ;;
        voice|integration|presets)
            [[ $# -eq 0 ]] || die "usage: ./hb deploy $target"
            exec "$OPS/deploy_stack.sh" "deploy-$target" ;;
        teleop)
            [[ $# -eq 0 ]] || die "usage: ./hb deploy teleop"
            exec "$OPS/deploy_stack.sh" deploy-robot-teleop ;;
        *) die "không rõ đối tượng deploy '$target' (./hb help)" ;;
    esac
}

# ---------------------------------------------------------------- service ----

# Robot: stack_ctl.sh tại chỗ. Dev: cùng script qua SSH (deploy_stack.sh).
run_ctl() {
    if [[ "$MODE" == robot ]]; then
        exec bash "$OPS/stack_ctl.sh" "$1"
    fi
    exec "$OPS/deploy_stack.sh" "$1"
}

cmd_service() {
    local action="$1" target="${2:-}"
    case "$target" in
        all|high|integration|voice|presets|teleop) run_ctl "$action-$target" ;;
        *) die "usage: ./hb $action <all|high|integration|voice|presets|teleop>" ;;
    esac
}

cmd_logs() {
    local target="${1:-all}"
    case "$target" in
        all|high|integration|voice|presets|teleop) run_ctl "logs-$target" ;;
        *) die "usage: ./hb logs [all|high|integration|voice|presets|teleop]" ;;
    esac
}

cmd_install() {
    need_robot install
    local target="${1:-all}"
    case "$target" in
        all|services|teleop) ;;
        *) die "usage: ./hb install [all|services|teleop]" ;;
    esac
    if [[ "$target" == all || "$target" == services ]]; then
        sudo HB_ROOT="$HB_ROOT" bash "$OPS/activate_services.sh"
    fi
    if [[ "$target" == all || "$target" == teleop ]]; then
        bash "$HB_ROOT/teleop/scripts/bootstrap_robot_runtime.sh"
        sudo HB_ROOT="$HB_ROOT" bash "$HB_ROOT/teleop/scripts/install_robot_runtime.sh" --start
    fi
}

# ------------------------------------------------------------ hiện trường ----

# Robot qua Tailscale: IP cố định trong tailnet, tới được dù robot ở LAN/hotspot nào.
tailscale_robot() {
    local host="${HB_ROBOT_TAILSCALE:-100.82.165.36}"
    command -v tailscale >/dev/null 2>&1 || die "máy này chưa cài/bật tailscale"
    if ! tailscale ping -c 1 --timeout=3s "$host" >/dev/null 2>&1; then
        echo "⏳ Chờ robot online trên Tailscale ($host)... (Ctrl+C để dừng)"
        until tailscale ping -c 1 --timeout=3s "$host" >/dev/null 2>&1; do sleep 3; done
    fi
    echo "✅ Robot online trên Tailscale: $host"
    TS_ROBOT="unitree@$host"
}

cmd_field() {
    need_dev field
    case "${1:-}" in ""|--no-restart) ;; *) die "usage: ./hb field [--no-restart]" ;; esac
    tailscale_robot
    # ROBOT là ưu tiên cao nhất của find_robot.sh: deploy đi đúng đường Tailscale,
    # không thử các IP LAN/hotspot cũ trong ~/.config/hb/robot.env.
    export ROBOT="$TS_ROBOT"
    if [[ "${1:-}" == --no-restart ]]; then
        "$OPS/deploy_stack.sh" deploy --no-restart
    else
        require_tmp_space
        "$OPS/deploy_stack.sh" deploy --accept-policy
    fi
    "$OPS/deploy_stack.sh" status
}

cmd_ssh() {
    need_dev ssh
    tailscale_robot
    exec ssh -t "$TS_ROBOT" "$@"
}

cmd_find_robot() {
    need_dev find-robot
    # shellcheck disable=SC1091
    source "$OPS/find_robot.sh"
    HB_ROBOT_SCAN_ONCE="${HB_ROBOT_SCAN_ONCE:-1}" find_robot
    echo "ROBOT=$ROBOT"
}

# ------------------------------------------------------------------- main ----

COMMAND="${1:-help}"
[[ $# -gt 0 ]] && shift
case "$COMMAND" in
    deploy)                  cmd_deploy "$@" ;;
    diff|rollback)           need_dev "$COMMAND"; exec "$OPS/deploy_stack.sh" "$COMMAND" ;;
    pull)                    need_dev pull; exec "$OPS/deploy_stack.sh" pull "$@" ;;
    start|stop|restart)      cmd_service "$COMMAND" "$@" ;;
    status)                  run_ctl status ;;
    logs)                    cmd_logs "$@" ;;
    build)                   cmd_build "$@" ;;
    preflight)               cmd_preflight ;;
    test)                    cmd_test "$@" ;;
    install)                 cmd_install "$@" ;;
    teleop-check)
        need_robot teleop-check
        cd "$HB_ROOT" && PYTHONPATH=teleop/src exec python3 -m teleop.hardware.run_teleop \
            --interface "${UNITREE_NETWORK_INTERFACE:-eth10}" --timeout-s 5 ;;
    dance)                   exec bash "$OPS/dance_assets.sh" "$@" ;;
    policy)                  exec bash "$OPS/policy_bundles.sh" "$@" ;;
    find-robot)              cmd_find_robot ;;
    field)                   cmd_field "$@" ;;
    ssh)                     cmd_ssh "$@" ;;
    help|-h|--help)          usage ;;
    *)                       usage >&2; exit 2 ;;
esac
