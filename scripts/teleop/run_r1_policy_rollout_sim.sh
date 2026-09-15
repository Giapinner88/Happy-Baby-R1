#!/usr/bin/env bash
# Chạy một policy LeRobot đã huấn luyện trong Isaac, không cần kính và không
# cần người. Cùng cách chia process như pilot teleop — policy đứng đúng chỗ mà
# solver upstream vẫn đứng — nên phía Isaac chạy nguyên đường đã có bằng chứng.
set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"

CHECKPOINT="${CHECKPOINT:?Đặt CHECKPOINT trỏ tới .../checkpoints/<step>/pretrained_model}"
DATASET_ROOT="${DATASET_ROOT:-data/lerobot/r1_d002_20260908T144820Z_recovered_8fps}"
DURATION_S="${DURATION_S:-120}"
OBS_PORT="${OBS_PORT:-5557}"
POLICY_HZ="${POLICY_HZ:-8}"
CONTROL_HZ="${CONTROL_HZ:-30}"
DEVICE="${DEVICE:-cuda:0}"
POLICY_DEVICE="${POLICY_DEVICE:-cuda:0}"
# Chỉ đổi camera của cửa sổ Isaac cho người xem. Policy luôn nhận head_camera
# gốc qua OBS_PORT, độc lập với lựa chọn head/perspective ở đây.
VIEWPORT_CAMERA="${VIEWPORT_CAMERA:-head}"
# Chữ số mục tiêu do bạn chọn, ví dụ TARGET_DIGIT=7 hoặc TARGET_DIGIT=7,3,9 cho
# nhiều episode liên tiếp. Bỏ trống thì randomizer tự bốc như cũ. Vị trí Ô vẫn
# được xáo mỗi lượt: cố định cả số lẫn ô thì không còn phân biệt được policy
# ĐỌC chữ số hay chỉ học thuộc một chỗ trên bàn.
TARGET_DIGIT="${TARGET_DIGIT:-}"
# Chế độ thao tác tay trước khi chạy: thứ tự bốn số là trái -> phải trong ảnh
# camera đầu. Prompt literal có đúng một chữ số 1-9 sẽ tự đặt TARGET_DIGIT;
# prompt ngữ nghĩa (không ghi số) và template {n} cần TARGET_DIGIT để chấm.
# Bỏ trống để random như trước.
TABLE_DIGITS="${TABLE_DIGITS:-}"
PROMPT_TEXT="${PROMPT_TEXT:-}"

test -d "$CHECKPOINT" || { echo "[FAIL] không có checkpoint: $CHECKPOINT" >&2; exit 2; }
[[ "$VIEWPORT_CAMERA" == "head" || "$VIEWPORT_CAMERA" == "perspective" ]] || {
  echo "[FAIL] VIEWPORT_CAMERA phải là head hoặc perspective: $VIEWPORT_CAMERA" >&2
  exit 2
}

# Một Isaac cũ vừa giữ nhiều GB VRAM vừa có thể giữ OBS_PORT. Khi đó policy của
# run mới kết nối nhầm publisher cũ trong khi lệnh lại đi vào simulator mới —
# run vẫn có vẻ hoạt động nhưng bằng chứng sai. Khóa giữ suốt cây process và
# preflight bên dưới chặn cả process được khởi động bởi phiên script cũ.
ROLLOUT_LOCK_FILE="/tmp/happy_baby_r1_policy_rollout.lock"
exec 9>"$ROLLOUT_LOCK_FILE"
if ! flock -n 9; then
  echo "[FAIL] một run_r1_policy_rollout_sim.sh khác đang giữ khóa rollout." >&2
  exit 2
fi
ACTIVE_SIM_PIDS="$(pgrep -f '^python scripts/teleop/run_r1_quest3_live.py ' || true)"
if [[ -n "$ACTIVE_SIM_PIDS" ]]; then
  echo "[FAIL] đã có Isaac R1 đang chạy; dừng mềm run cũ trước khi mở run mới:" >&2
  for ACTIVE_PID in $ACTIVE_SIM_PIDS; do
    ps -p "$ACTIVE_PID" -o pid=,etime=,stat=,cmd= >&2
  done
  echo "       dùng stop file ghi trong --stop-file của đúng PID, không dừng job training." >&2
  exit 2
fi
if ss -H -ltn "sport = :$OBS_PORT" | rg -q .; then
  echo "[FAIL] OBS_PORT=$OBS_PORT đang được một tiến trình khác sử dụng." >&2
  ss -H -ltnp "sport = :$OBS_PORT" >&2 || true
  exit 2
fi
if [[ -n "$PROMPT_TEXT" ]]; then
  if [[ "$PROMPT_TEXT" == *'{n}'* ]]; then
    [[ -n "$TARGET_DIGIT" ]] || {
      echo "[FAIL] PROMPT_TEXT chứa {n} nên cần TARGET_DIGIT để thay thế." >&2
      exit 2
    }
  else
    PROMPT_TARGETS=()
    for DIGIT in {1..9}; do
      if [[ "$PROMPT_TEXT" == *"$DIGIT"* ]]; then
        PROMPT_TARGETS+=("$DIGIT")
      fi
    done
    if [[ -z "$TARGET_DIGIT" ]]; then
      [[ ${#PROMPT_TARGETS[@]} -eq 1 ]] || {
        echo "[FAIL] không suy ra được một mục tiêu duy nhất từ PROMPT_TEXT; hãy đặt TARGET_DIGIT." >&2
        exit 2
      }
      TARGET_DIGIT="${PROMPT_TARGETS[0]}"
    elif [[ ${#PROMPT_TARGETS[@]} -eq 1 && "$TARGET_DIGIT" != "${PROMPT_TARGETS[0]}" ]]; then
      echo "[FAIL] TARGET_DIGIT=$TARGET_DIGIT nhưng PROMPT_TEXT ghi số ${PROMPT_TARGETS[0]}." >&2
      exit 2
    fi
  fi
fi
if [[ -n "$TARGET_DIGIT" ]]; then
  [[ "$TARGET_DIGIT" =~ ^[1-9](,[1-9])*$ ]] || {
    echo "[FAIL] TARGET_DIGIT phải là các chữ số 1-9 ngăn cách bởi dấu phẩy, nhận: $TARGET_DIGIT" >&2
    exit 2
  }
fi
if [[ -n "$TABLE_DIGITS" ]]; then
  [[ "$TARGET_DIGIT" =~ ^[1-9]$ ]] || {
    echo "[FAIL] TABLE_DIGITS yêu cầu đúng một TARGET_DIGIT từ 1-9." >&2
    exit 2
  }
  IFS=',' read -r -a TABLE_DIGIT_VALUES <<< "$TABLE_DIGITS"
  [[ ${#TABLE_DIGIT_VALUES[@]} -eq 4 ]] || {
    echo "[FAIL] TABLE_DIGITS phải có đúng bốn số, ví dụ 5,4,8,2." >&2
    exit 2
  }
  SEEN_DIGITS=","
  TARGET_ON_TABLE=false
  for DIGIT in "${TABLE_DIGIT_VALUES[@]}"; do
    [[ "$DIGIT" =~ ^[1-9]$ ]] || {
      echo "[FAIL] mỗi phần tử TABLE_DIGITS phải là một chữ số 1-9: $DIGIT" >&2
      exit 2
    }
    [[ "$SEEN_DIGITS" != *",$DIGIT,"* ]] || {
      echo "[FAIL] TABLE_DIGITS không được lặp chữ số: $DIGIT" >&2
      exit 2
    }
    SEEN_DIGITS+="$DIGIT,"
    [[ "$DIGIT" != "$TARGET_DIGIT" ]] || TARGET_ON_TABLE=true
  done
  [[ "$TARGET_ON_TABLE" == true ]] || {
    echo "[FAIL] TARGET_DIGIT=$TARGET_DIGIT không nằm trong TABLE_DIGITS=$TABLE_DIGITS." >&2
    exit 2
  }
fi
# Tính tên sau bước suy target từ prompt để run prompt-only vẫn có d8 trong tên.
RUN_ID="${RUN_ID:-rollout_${TARGET_DIGIT:+d${TARGET_DIGIT//,/-}_}$(date -u +%Y%m%dT%H%M%SZ)}"
OUT_DIR="${OUT_DIR:-experiments/r1_dataset/quest3_sim_v1/D002/rollouts/$RUN_ID}"
test -d "$DATASET_ROOT/meta" || { echo "[FAIL] không phải dataset LeRobot: $DATASET_ROOT" >&2; exit 2; }
# KHÔNG tạo $OUT_DIR: run_r1_quest3_live.py từ chối chạy nếu thư mục bằng chứng
# đã tồn tại, và đó là một guard đúng — nó ngăn hai run ghi đè lên nhau. Chỉ tạo
# thư mục cha, rồi để Isaac tự tạo thư mục run.
mkdir -p "$(dirname "$OUT_DIR")"
test ! -e "$OUT_DIR" || { echo "[FAIL] $OUT_DIR đã tồn tại; chọn RUN_ID khác" >&2; exit 2; }
POLICY_STATS="$(dirname "$OUT_DIR")/${RUN_ID}_policy_rollout_stats.json"
STOP_FILE="/tmp/${RUN_ID}.stop"
rm -f "$STOP_FILE"

echo "[rollout] run=$RUN_ID out=$OUT_DIR ckpt=$CHECKPOINT target=${TARGET_DIGIT:-<ngẫu nhiên>}" >&2

SIM_TASK_ARGS=()
[[ -z "$TARGET_DIGIT" ]] || SIM_TASK_ARGS+=(--target-digit-priority "$TARGET_DIGIT")
[[ -z "$TABLE_DIGITS" ]] || SIM_TASK_ARGS+=(--fixed-digit-layout "$TABLE_DIGITS")
[[ -z "$PROMPT_TEXT" ]] || SIM_TASK_ARGS+=(--policy-prompt "$PROMPT_TEXT")

# Policy chạy thêm một khoảng so với Isaac: nó chỉ bắt đầu đếm giờ sau khi nhận
# quan sát đầu tiên, mà Isaac mất hàng chục giây để nạp cảnh. Nếu policy dừng
# trước, Isaac sẽ đứng im với vector cuối và cả run thành vô nghĩa.
POLICY_DURATION_S="$(python3 -c "print(float('$DURATION_S') + 60.0)")"

set +e
conda run --no-capture-output -n lerobot \
  python scripts/teleop/run_r1_policy_rollout.py \
    --checkpoint "$CHECKPOINT" \
    --dataset-root "$DATASET_ROOT" \
    --obs-port "$OBS_PORT" \
    --policy-hz "$POLICY_HZ" \
    --command-hz "$CONTROL_HZ" \
    --duration-s "$POLICY_DURATION_S" \
    --device "$POLICY_DEVICE" \
    --stats-path "$POLICY_STATS" \
| conda run --no-capture-output -n unitree_sim_env \
  python scripts/teleop/run_r1_quest3_live.py \
    --output-dir "$OUT_DIR" \
    --duration-s "$DURATION_S" \
    --physics-hz 200.0 \
    --control-hz "$CONTROL_HZ" \
    --stop-file "$STOP_FILE" \
    --disable-self-collisions \
    --idle-stop-s 0.0 \
    --upstream-joint-stream-config \
      experiments/r1_teleop/quest3_sim_v1/T007/config/r1_t007_upstream_stream_live.json \
    --dataset-scene-config \
      experiments/r1_dataset/quest3_sim_v1/D002/config/r1_d002_policy_rollout.json \
    --device "$DEVICE" \
    --no-video \
    --viewport-camera "$VIEWPORT_CAMERA" \
    --policy-obs-port "$OBS_PORT" \
    "${SIM_TASK_ARGS[@]}"
PIPE_STATUS_SIM=$?
set -e

# Isaac báo lỗi khi tắt ("shutdown did not return within 30 s") dù run đã ghi đủ
# bằng chứng, nên trạng thái thoát KHÔNG được phép làm mất file stats của policy.
# Chuyển file trước, báo cáo trạng thái sau.
if [[ -d "$OUT_DIR" && -f "$POLICY_STATS" ]]; then
  mv "$POLICY_STATS" "$OUT_DIR/policy_rollout_stats.json"
fi
echo "[rollout] xong (sim exit=$PIPE_STATUS_SIM); bằng chứng ở $OUT_DIR" >&2
