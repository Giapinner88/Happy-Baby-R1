import time
import json
import threading
import numpy as np

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

robot_state = None
state_lock = threading.Lock()

def state_handler(msg: LowState_):
    global robot_state
    with state_lock:
        robot_state = msg

def main():
    global robot_state
    
    print("Khởi tạo DDS Subscriber...")
    ChannelFactoryInitialize(1, "lo")
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(state_handler, 10)
    
    print("\n" + "="*60)
    print(" HƯỚNG DẪN RECORD KEYFRAME CÁNH TAY PHẢI (G1)")
    print("="*60)
    print("1. Chạy simulator ở Terminal 1: python unitree_mujoco2.py")
    print("2. Cửa sổ MuJoCo hiện lên -> bấm phím [Space] để Pause.")
    print("3. Dùng chuột kéo các khớp CÁNH TAY PHẢI (Khớp 23->29) tạo dáng.")
    print("4. Quay lại Terminal này (đang chạy record_wave.py):")
    print("   - Bấm [Enter] (để trống) để chụp lại tư thế hiện tại.")
    print("   - Kéo tay sang dáng mới -> Bấm [Enter] chụp tiếp.")
    print("   - Gõ phím 's' rồi [Enter] để Lưu & Thoát.")
    print("="*60 + "\n")
    
    keyframes = []
    
    while True:
        try:
            cmd = input(">>> Nhấn [Enter] để chụp Keyframe, hoặc 's' + [Enter] để lưu: ").strip().lower()
            
            if cmd == 's':
                if len(keyframes) == 0:
                    print("Chưa có Keyframe nào được lưu! Thoát...")
                else:
                    with open("wave_animation/wave_keyframes.json", "w") as f:
                        json.dump({"right_arm_keyframes": keyframes}, f, indent=4)
                    print(f"Đã lưu thành công {len(keyframes)} keyframes vào wave_animation/wave_keyframes.json")
                break
                
            else:
                with state_lock:
                    rs = robot_state
                
                if rs is None:
                    print("[LỖI] Chưa nhận được tín hiệu từ MuJoCo. Hãy chắc chắn đang chạy unitree_mujoco2.py")
                    continue
                    
                # Trích xuất 7 khớp cánh tay phải (từ index 23 đến 29)
                right_arm_q = []
                for i in range(23, 30):
                    right_arm_q.append(float(rs.motor_state[i].q))
                    
                keyframes.append(right_arm_q)
                print(f" [+] Đã chụp thành công Keyframe thứ {len(keyframes)}: {np.round(right_arm_q, 2)}")
        
        except KeyboardInterrupt:
            print("\nĐã hủy. Thoát chương trình.")
            break

if __name__ == '__main__':
    main()
