import time
import speech_recognition as sr
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from chat_data import ChatData
import sys

def main():
    # 1. Khởi tạo mạng DDS
    # Nếu chạy local cùng trên 1 máy, dùng "lo"
    network_interface = "lo" 
    if len(sys.argv) > 1:
        network_interface = sys.argv[1]
        
    print(f"[VOICE_PUB] Đang khởi tạo kết nối mạng trên {network_interface}...")
    ChannelFactoryInitialize(0, network_interface)

    # 2. Tạo Kênh Phát (Publisher)
    pub = ChannelPublisher("g1_chat_topic", ChatData)
    pub.Init()

    # 3. Cài đặt Microphone
    recognizer = sr.Recognizer()
    mic = sr.Microphone()

    print("[VOICE_PUB] Đang điều chỉnh tạp âm môi trường... Vui lòng giữ im lặng.")
    with mic as source:
        recognizer.adjust_for_ambient_noise(source, duration=2)
    print("[VOICE_PUB] Sẵn sàng lắng nghe! (Bấm Ctrl+C để thoát)")

    try:
        while True:
            with mic as source:
                print("\n[VOICE_PUB] Xin mời nói...")
                audio = recognizer.listen(source)

            try:
                print("[VOICE_PUB] Đang nhận dạng giọng nói...")
                # Sử dụng Google Speech API miễn phí cho tiện lợi
                text = recognizer.recognize_google(audio, language="vi-VN")
                print(f"[VOICE_PUB] Bạn đã nói: '{text}'")

                # 4. Gửi dữ liệu qua DDS
                msg = ChatData(text_data=text)
                if pub.Write(msg, 0.5):
                    print("[VOICE_PUB] Đã gửi sang cho Não Bộ thành công!")
                else:
                    print("[VOICE_PUB] [LỖI] Đã gửi nhưng Không có ai đang nghe (Subscriber chưa bật?)")

            except sr.UnknownValueError:
                print("[TAI NGHE] Xin lỗi, mình nghe không rõ. Bạn nói lại nhé.")
            except sr.RequestError as e:
                print(f"[TAI NGHE] Lỗi kết nối Google API: {e}")

    except KeyboardInterrupt:
        print("\n[TAI NGHE] Đã tắt.")
    finally:
        pub.Close()

if __name__ == "__main__":
    main()
