#!/usr/bin/env bash
# Kiểm tra đường kết nối Quest -> vuer -> teleop TRƯỚC khi đeo headset.
#
# Mỗi lần lỗi kết nối thường mất vài phút mới nhận ra vì phải đeo kính lên mới
# biết. Script này kiểm tra sẵn những thứ hay hỏng: IP host, chứng chỉ, môi
# trường Python, và cổng 8012.
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TELEOP_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE="${HB_WORKSPACE:-$(cd "$TELEOP_DIR/../.." && pwd)}"

if [[ -r /etc/hb/teleop.env ]]; then set -a; source /etc/hb/teleop.env; set +a; fi
HOST_IP="${HB_TELEOP_HOST_IP:-10.42.0.1}"
CERT="${HB_TELEOP_CERT_FILE:-$HOME/.config/xr_teleoperate/t001_10_42/cert.pem}"
KEY="${HB_TELEOP_KEY_FILE:-$HOME/.config/xr_teleoperate/t001_10_42/key.pem}"
VUER_ENV="${HB_TELEOP_VUER_ENV:-tv}"
PORT=8012

FAILED=0
ok()   { echo "[OK] $*"; }
warn() { echo "[WARN] $*" >&2; }
bad()  { echo "[FAIL] $*" >&2; FAILED=1; }

# 1. Host IP phải thực sự nằm trên một interface, nếu không Quest không nối được.
if ip -4 addr show | grep -q "inet $HOST_IP/"; then
    IFACE="$(ip -4 -o addr show | awk -v ip="$HOST_IP/" '$4 ~ ip {print $2}')"
    ok "host IP $HOST_IP có trên $IFACE"
else
    bad "host IP $HOST_IP KHÔNG có trên máy này. Bật HB-Hotspot hoặc sửa HB_TELEOP_HOST_IP."
fi

# 2. Chứng chỉ: phải tồn tại, còn hạn, và SAN phải khớp host IP.
if [[ -r "$CERT" && -r "$KEY" ]]; then
    ok "thấy cert và key"
    if openssl x509 -in "$CERT" -noout -ext subjectAltName 2>/dev/null | grep -q "IP Address:$HOST_IP"; then
        ok "SAN của cert khớp $HOST_IP"
    else
        bad "SAN của cert KHÔNG chứa $HOST_IP -> Quest Browser sẽ từ chối."
    fi
    if ! openssl x509 -in "$CERT" -noout -checkend 0 >/dev/null 2>&1; then
        bad "cert đã HẾT HẠN ($(openssl x509 -in "$CERT" -noout -enddate | cut -d= -f2))"
    elif ! openssl x509 -in "$CERT" -noout -checkend 604800 >/dev/null 2>&1; then
        warn "cert hết hạn trong vòng 7 ngày ($(openssl x509 -in "$CERT" -noout -enddate | cut -d= -f2))"
    else
        ok "cert còn hạn tới $(openssl x509 -in "$CERT" -noout -enddate | cut -d= -f2)"
    fi
    PERM="$(stat -c '%a' "$KEY")"
    [[ "$PERM" == "600" || "$PERM" == "400" ]] || warn "private key đang ở quyền $PERM; nên đặt 600"
else
    bad "thiếu cert/key: $CERT / $KEY"
fi

# 3. Cổng 8012 phải trống, nếu không vuer sẽ không bind được.
if ss -tln 2>/dev/null | grep -q ":$PORT "; then
    bad "cổng $PORT đang bị chiếm; tắt bridge cũ trước."
else
    ok "cổng $PORT còn trống"
fi

# 4. Môi trường vuer phải import được TeleVuer.
TELEVUER_SRC="$WORKSPACE/third_party/xr_teleoperate/teleop/televuer/src"
if [[ -d "$TELEVUER_SRC" ]]; then
    if conda run -n "$VUER_ENV" python -c "
import sys; sys.path.insert(0,'$TELEVUER_SRC')
import vuer
from televuer import TeleVuerWrapper
" >/dev/null 2>&1; then
        ok "env '$VUER_ENV' import được vuer + TeleVuer"
    else
        bad "env '$VUER_ENV' KHÔNG import được vuer/TeleVuer."
    fi
else
    bad "không thấy televuer: $TELEVUER_SRC"
fi

echo
if ((FAILED)); then
    echo "❌ Đường kết nối Quest chưa sẵn sàng." >&2
    exit 1
fi
cat <<MSG
✅ Sẵn sàng. Mở trên Quest Browser:

    https://$HOST_IP:$PORT/?ws=wss://$HOST_IP:$PORT

Chấp nhận chứng chỉ, rồi PHẢI bấm vào VR (immersive session) mới có dữ liệu
điều khiển: chỉ mở trang thôi thì bridge sẽ đứng chờ ở motion_data_ready=0.
MSG
