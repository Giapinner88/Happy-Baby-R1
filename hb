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

DEPLOY  (máy trạm -> robot; robot tự dò, hoặc ROBOT=unitree@<ip>). Bảng: docs/deploy.md
  deploy                       tự thấy phần nào đổi, chỉ deploy phần đó (dùng hằng ngày)
  deploy controller            code C++ + config + policy + dance; restart high-level, cần DISARMED
  deploy voice                 voice + presets + integration; không đụng high-level
  deploy teleop                runtime Quest; không cần DISARMED
  deploy object-search         chỉ đồng bộ mã tìm vật, không restart service
  deploy all                   ép làm toàn bộ (lần đầu, sửa systemd, nghi robot lệch)
    [--no-restart]             chép + build + preflight, chưa restart (./hb deploy sau để kích hoạt)
    [--dry-run]                chỉ xem kế hoạch
  diff                         = deploy --dry-run
  rollback                     trả bản trước lần deploy gần nhất (không gồm dance)
  pull [--dry-run]             kéo cấu hình đã sửa trên robot về máy trạm
  field [đối tượng] [...]      như deploy nhưng đi qua Tailscale (HB_ROBOT_TAILSCALE=<ip>)
  ssh [lệnh...]                mở shell (hoặc chạy lệnh) trên robot qua Tailscale

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

GHI DỮ LIỆU  (chỉ nghe DDS, không tạo publisher; dùng song song run_r1 được)
  record [--duration S] [--decimate N] [--all-motors] [--no-lowcmd] [--no-bms]
                               CSV mọi motor (q/dq/ddq/tau/nhiệt/áp/lỗi) + IMU + tay cầm
                               + lowcmd + pin, ~1 kHz; Ctrl-C để dừng. Robot: ~/hb_logs;
                               máy trạm: chạy qua Tailscale rồi kéo về \${HB_LOG_DIR:-~/hb_logs}

KHÁC
  policy list                  liệt kê bundle policy đi bộ, đánh dấu walk (R2+A) / teleop (R2+X)
  policy add <file.onnx> <tên> [--contract C]
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

# In "<đối tượng> [cờ...]" chuẩn hoá; tên cũ là bí danh của lệnh mới.
deploy_args() {
    local target=auto
    case "${1:-}" in
        ""|--*) ;;
        *) target="$1"; shift ;;
    esac
    case "$target" in
        auto|all|controller|voice|teleop) ;;
        policy|dance|high)
            echo "hb: 'deploy $target' nay là 'deploy controller' (code + config + policy + dance)" >&2
            target=controller ;;
        presets|integration)
            echo "hb: 'deploy $target' nay là 'deploy voice' (voice + presets + integration)" >&2
            target=voice ;;
        *) die "không rõ đối tượng deploy '$target' (./hb help)" ;;
    esac
    for arg in "$@"; do
        case "$arg" in
            --no-restart|--dry-run) ;;
            *) die "usage: ./hb deploy [auto|controller|voice|teleop|all] [--no-restart] [--dry-run]" ;;
        esac
    done
    DEPLOY_ARGS=("$target" "$@")
}

cmd_deploy() {
    need_dev deploy
    if [[ "${1:-}" == object-search ]]; then
        shift
        [[ $# -eq 0 || ( $# -eq 1 && "$1" == --dry-run ) ]] || die "usage: ./hb deploy object-search [--dry-run]"
        exec "$OPS/deploy_stack.sh" deploy-object-search "$@"
    fi
    deploy_args "$@"
    exec "$OPS/deploy_stack.sh" deploy "${DEPLOY_ARGS[@]}"
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
    deploy_args "$@"
    tailscale_robot
    # ROBOT là ưu tiên cao nhất của find_robot.sh: deploy đi đúng đường Tailscale,
    # không thử các IP LAN/hotspot cũ trong ~/.config/hb/robot.env.
    export ROBOT="$TS_ROBOT"
    exec "$OPS/deploy_stack.sh" deploy "${DEPLOY_ARGS[@]}"
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

# ----------------------------------------------------------------- record ----

# Robot: build sensor_log khi thiếu (build mặc định chỉ có run_r1) rồi chạy với nice
# để không tranh CPU với vòng điều khiển. Máy trạm: cùng lệnh qua SSH rồi scp về.
cmd_record() {
    local name="sensors_$(date +%Y%m%d_%H%M%S)" out="" arg
    local args=()
    while (( $# )); do
        case "$1" in
            --out) out="${2:?--out cần đường dẫn}"; shift 2 ;;
            --duration|--decimate) args+=("$1" "${2:?$1 cần giá trị}"); shift 2 ;;
            --all-motors|--no-lowcmd|--no-bms) args+=("$1"); shift ;;
            *) die "usage: ./hb record [--duration S] [--decimate N] [--all-motors] [--no-lowcmd] [--no-bms]" ;;
        esac
    done
    if [[ "$MODE" == robot ]]; then
        [[ -n "$out" ]] || out="$HOME/hb_logs/$name.csv"
        mkdir -p "$(dirname "$out")"
        if [[ ! -x "$HIGH_DIR/build/sensor_log" || "$HIGH_DIR/tools/sensor_log.cpp" -nt "$HIGH_DIR/build/sensor_log" ]]; then
            cmake -S "$HIGH_DIR" -B "$HIGH_DIR/build" -DCMAKE_BUILD_TYPE=Release >/dev/null
            cmake --build "$HIGH_DIR/build" --target sensor_log
        fi
        exec nice -n 10 "$HIGH_DIR/build/sensor_log" \
            --iface "${UNITREE_NETWORK_INTERFACE:-eth10}" --out "$out" "${args[@]}"
    fi
    [[ -z "$out" ]] || die "--out chỉ dùng trên robot; máy trạm lưu vào \${HB_LOG_DIR:-~/hb_logs}"
    tailscale_robot
    # Không phụ thuộc bản deploy trên robot (có thể cũ, và không deploy khi đang vận hành):
    # chép riêng nguồn tool vào ~/hb_tools/sensor_log, build ở đó nếu nguồn đổi, rồi chạy.
    # Không đụng /home/unitree/HB, run_r1 hay service nào.
    local tool_dir="/home/unitree/hb_tools/sensor_log"
    local remote_csv="/home/unitree/hb_logs/$name.csv"
    local local_dir="${HB_LOG_DIR:-$HOME/hb_logs}"
    local rc=0
    # Luôn có ít nhất "--out <file>": printf '%q ' trên mảng RỖNG vẫn in ra '' (một tham số
    # rỗng) và sensor_log từ chối nó.
    local remote_args
    remote_args="$(printf '%q ' --out "$remote_csv" "${args[@]}")"
    tar -C "$HIGH_DIR" -cf - tools/sensor_log.cpp src/config/RobotSpec.hpp \
        | ssh "$TS_ROBOT" "mkdir -p $tool_dir/new && tar -C $tool_dir/new -xf -" \
        || die "không chép được nguồn sensor_log lên robot"
    ssh -t "$TS_ROBOT" "set -e; cd $tool_dir
        if ! cmp -s new/tools/sensor_log.cpp tools/sensor_log.cpp 2>/dev/null \\
           || ! cmp -s new/src/config/RobotSpec.hpp src/config/RobotSpec.hpp 2>/dev/null \\
           || [ ! -x build/sensor_log ]; then
            rm -rf tools src && mv new/tools new/src .
            printf '%s\\n' 'cmake_minimum_required(VERSION 3.10)' 'project(sensor_log CXX)' \\
                'set(CMAKE_CXX_STANDARD 17)' 'find_package(unitree_sdk2 REQUIRED)' \\
                'add_executable(sensor_log tools/sensor_log.cpp)' \\
                'target_link_libraries(sensor_log unitree_sdk2 pthread)' > CMakeLists.txt
            echo 'Build sensor_log tren robot...'
            cmake -S . -B build -DCMAKE_BUILD_TYPE=Release >/dev/null && cmake --build build
        fi
        rm -rf new; mkdir -p /home/unitree/hb_logs
        exec nice -n 10 build/sensor_log --iface \${UNITREE_NETWORK_INTERFACE:-eth10} $remote_args" \
        || rc=$?
    # Ctrl-C trả mã khác 0 nhưng vẫn có file; chỉ coi là lỗi khi robot không có CSV.
    ssh "$TS_ROBOT" "test -s $(printf %q "$remote_csv")" \
        || die "sensor_log trên robot thoát mã $rc và không tạo $remote_csv (xem thông báo phía trên)"
    mkdir -p "$local_dir"
    scp "$TS_ROBOT:$remote_csv" "$TS_ROBOT:$remote_csv.meta.json" "$local_dir/" \
        || die "không kéo được $remote_csv về máy trạm"
    echo "Đã lưu $local_dir/$name.csv (+ .meta.json); bản gốc trên robot: $remote_csv"
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
    record)                  cmd_record "$@" ;;
    find-robot)              cmd_find_robot ;;
    field)                   cmd_field "$@" ;;
    ssh)                     cmd_ssh "$@" ;;
    help|-h|--help)          usage ;;
    *)                       usage >&2; exit 2 ;;
esac
