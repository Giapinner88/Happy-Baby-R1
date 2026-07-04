"""
chatbot_subscriber.py  –  Não Bộ của Robot G1

CHẾ ĐỘ HIỆN TẠI : LOCAL (2 terminal trên cùng máy, card mạng "lo")
CHẾ ĐỘ ROBOT    : Tìm các block "# [ROBOT THẬT]" và làm theo hướng dẫn
"""

import time
import sys
import os
import threading
import warnings
warnings.filterwarnings("ignore", category=FutureWarning)  # tắt warning của google.generativeai cũ
from dotenv import load_dotenv
import google.generativeai as genai
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from chat_data import ChatData

# [ROBOT THẬT] Bỏ comment 3 dòng dưới khi kết nối robot thật
# from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
# from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
# from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient, action_map

# ==============================================================
#  CẤU HÌNH
# ==============================================================

# [ROBOT THẬT] Đổi "lo" -> tên card mạng LAN nối robot (vd: "eth0")
NETWORK_INTERFACE = "lo"

# Danh sách model thử lần lượt khi model trước bị quota
GEMINI_MODELS = [
    "gemini-1.5-flash",
    "gemini-1.5-flash-8b",
    "gemini-1.0-pro",
]

API_TIMEOUT_SEC  = 10   # giây: nếu Gemini không trả lời trong thời gian này => bỏ qua
MAX_RETRIES      = 3    # số lần thử lại khi gặp lỗi 429 quota
RETRY_DELAY_SEC  = 5    # giây chờ giữa các lần retry

SYSTEM_PROMPT = (
    "Bạn là trợ lý robot Unitree G1 thông minh và thân thiện. "
    "Hãy trả lời cực kỳ ngắn gọn, tự nhiên, dưới 3 câu."
)

# ==============================================================
#  KHỞI TẠO GEMINI API
# ==============================================================
load_dotenv()
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
if not GEMINI_API_KEY:
    print("[CHATBOT] LỖI: Không tìm thấy GEMINI_API_KEY trong file .env!")
    sys.exit(-1)

genai.configure(api_key=GEMINI_API_KEY)

# ==============================================================
#  HÀM GỌI GEMINI (có timeout cứng, tự fallback model)
# ==============================================================

def _call_api(model_name: str, prompt: str, result: dict):
    """Chạy trong thread phụ. Ghi kết quả vào dict dùng chung."""
    try:
        m = genai.GenerativeModel(model_name)
        resp = m.generate_content(prompt)
        result["text"] = resp.text.strip()
    except Exception as e:
        result["error"] = str(e)


def get_gemini_response(user_text: str) -> str:
    """
    Gọi Gemini với timeout cứng.
    Tự động thử model tiếp theo khi gặp lỗi quota (429).
    """
    prompt = SYSTEM_PROMPT + "\nCâu hỏi: " + user_text

    for model_name in GEMINI_MODELS:
        for attempt in range(1, MAX_RETRIES + 1):
            result = {}
            t = threading.Thread(target=_call_api, args=(model_name, prompt, result), daemon=True)
            t.start()
            t.join(timeout=API_TIMEOUT_SEC)

            if t.is_alive():
                # Thread vẫn còn chạy => timeout
                print(f"[CHATBOT] ⚠ Timeout sau {API_TIMEOUT_SEC}s với model {model_name}. Bỏ qua.")
                break  # thử model tiếp

            if "text" in result:
                return result["text"]

            err = result.get("error", "")
            if "429" in err or "quota" in err.lower() or "Resource" in err:
                print(f"[CHATBOT] ⚠ Quota hết ({model_name}), thử lại sau {RETRY_DELAY_SEC}s... ({attempt}/{MAX_RETRIES})")
                time.sleep(RETRY_DELAY_SEC)
            else:
                print(f"[CHATBOT] ✗ Lỗi API ({model_name}): {err[:120]}")
                break  # lỗi khác => thử model tiếp

        else:
            # Hết MAX_RETRIES, thử model tiếp
            print(f"[CHATBOT] ✗ Hết lượt thử cho model {model_name}, chuyển model...")
            continue

    return "Xin lỗi, não bộ tôi đang bị gián đoạn kết nối."


# ==============================================================
#  MAIN
# ==============================================================

def main():
    # Cho phép ghi đè card mạng qua argument dòng lệnh
    # Ví dụ: python3 chatbot_subscriber.py eth0
    network_interface = NETWORK_INTERFACE
    if len(sys.argv) > 1:
        network_interface = sys.argv[1]

    print(f"[CHATBOT] Khởi tạo mạng DDS trên '{network_interface}'...")
    ChannelFactoryInitialize(0, network_interface)

    # ----------------------------------------------------------
    # [ROBOT THẬT] Bỏ comment block dưới để bật Loa/Tay/Chân
    #              (đã bỏ comment 3 dòng import ở đầu file chưa?)
    # ----------------------------------------------------------
    # print("[CHATBOT] Đang đánh thức Loa, Tay, Chân...")
    # audio = AudioClient();  audio.SetTimeout(5.0);  audio.Init()
    # audio.SetVolume(85)
    # loco  = LocoClient();   loco.SetTimeout(5.0);   loco.Init()
    # arm   = G1ArmActionClient(); arm.SetTimeout(5.0); arm.Init()
    # audio.LedControl(0, 255, 0)   # đèn xanh lá = sẵn sàng
    # ----------------------------------------------------------

    sub = ChannelSubscriber("g1_chat_topic", ChatData)
    sub.Init()
    print("[CHATBOT] ✔ Sẵn sàng! Đang chờ câu hỏi từ Bàn Phím...\n")

    try:
        while True:
            msg = sub.Read()
            if msg is not None:
                user_text = msg.text_data.strip()
                if not user_text:
                    continue

                print(f"[CHATBOT] >>> Nhận được: '{user_text}'")

                # [ROBOT THẬT] Bỏ comment dòng dưới
                # audio.LedControl(255, 255, 0)   # đèn vàng = đang suy nghĩ

                print("[CHATBOT] Đang hỏi Gemini...")
                bot_response = get_gemini_response(user_text)

                print(f"[CHATBOT] <<< Trả lời: '{bot_response}'\n")

                # ----------------------------------------------------------
                # [ROBOT THẬT] Bỏ comment block dưới để phát loa + điều tay
                # ----------------------------------------------------------
                # audio.LedControl(0, 0, 255)   # đèn xanh dương = đang nói
                #
                # if "chào" in user_text.lower():
                #     arm.ExecuteAction(action_map.get("high wave"))
                # elif "buồn" in user_text.lower() or "chán" in user_text.lower():
                #     arm.ExecuteAction(action_map.get("hug"))
                #
                # audio.TtsMaker(bot_response, 0)
                # time.sleep(4)
                # arm.ExecuteAction(action_map.get("release arm"))
                # audio.LedControl(0, 255, 0)   # đèn xanh lá = sẵn sàng lại
                # ----------------------------------------------------------

            time.sleep(0.05)   # polling nhanh hơn, tránh quá tải CPU

    except KeyboardInterrupt:
        print("\n[CHATBOT] Đã tắt.")
    finally:
        # [ROBOT THẬT] Bỏ comment dòng dưới
        # audio.LedControl(255, 0, 0)   # đèn đỏ = tắt
        sub.Close()


if __name__ == "__main__":
    main()
