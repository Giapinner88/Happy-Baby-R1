import mujoco
import mujoco.viewer
import time
import json
import numpy as np
import threading
import sys
import select

is_running = True
keyframes = []

def input_thread(model, data):
    global is_running
    print("\n" + "="*65)
    print(" 🛠️  POSE EDITOR: TẠO DÁNG VẪY TAY CHO G1")
    print("="*65)
    print("1. Robot đang lơ lửng (Không trọng lực).")
    print("2. Dùng chuột TRÁI kéo các khớp CÁNH TAY PHẢI (Khớp 23->29) tạo dáng.")
    print("   👉 MẸO: Nháy đúp chuột (Double-click) vào khớp để mở thanh trượt (Slider) bên phải màn hình. Kéo thanh trượt sẽ dễ và chính xác hơn rất nhiều!")
    print("3. Tại cửa sổ Terminal này:")
    print("   - Gõ phím [Enter] (để trống) để LƯU tư thế tay phải thành 1 khung hình (Keyframe).")
    print("   - Gõ phím 's' rồi [Enter] để XUẤT toàn bộ khung hình ra file JSON & Thoát.")
    print("="*65 + "\n")
    
    while is_running:
        try:
            cmd = input(">>> Nhấn [Enter] lưu Keyframe, 's' + [Enter] để thoát: ").strip().lower()
            if not is_running:
                break
                
            if cmd == 's':
                if len(keyframes) == 0:
                    print("Chưa có Keyframe nào! Đang thoát...")
                else:
                    with open("wave_animation/wave_keyframes.json", "w") as f:
                        json.dump({"right_arm_keyframes": keyframes}, f, indent=4)
                    print(f"Đã lưu {len(keyframes)} keyframes vào wave_animation/wave_keyframes.json")
                is_running = False
                break
            else:
                # Danh sách tên các khớp tay phải theo chuẩn Unitree G1
                right_arm_joints = [
                    "right_shoulder_pitch_joint",
                    "right_shoulder_roll_joint",
                    "right_shoulder_yaw_joint",
                    "right_elbow_joint",
                    "right_wrist_roll_joint",
                    "right_wrist_pitch_joint",
                    "right_wrist_yaw_joint"
                ]
                
                frame = []
                for j_name in right_arm_joints:
                    j_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j_name)
                    qpos_adr = model.jnt_qposadr[j_id]
                    val = float(data.qpos[qpos_adr])
                    frame.append(val)
                    
                keyframes.append(frame)
                print(f" [+] Đã chụp thành công Keyframe thứ {len(keyframes)}: {np.round(frame, 2)}")
                
        except EOFError:
            break
        except Exception as e:
            print(f"Lỗi: {e}")
            break

def main():
    global is_running
    xml_path = "../unitree_robots/g1/scene.xml"
    try:
        m = mujoco.MjModel.from_xml_path(xml_path)
    except Exception as e:
        print(f"Không thể mở {xml_path}: {e}")
        return
        
    d = mujoco.MjData(m)
    
    # --- THIẾT LẬP MÔI TRƯỜNG ĐỂ TẠO DÁNG ---
    # Tắt trọng lực
    m.opt.gravity[:] = [0, 0, 0]
    
    # Danh sách tên các khớp tay phải
    right_arm_joints = [
        "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
        "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"
    ]
    right_arm_dofs = []
    for j_name in right_arm_joints:
        j_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j_name)
        if j_id >= 0:
            right_arm_dofs.append(m.jnt_dofadr[j_id])
    
    # Đóng băng toàn bộ các khớp (Damping = 100), CHỈ CHO PHÉP cánh tay phải cử động (Damping = 0.5)
    for i in range(m.nv):
        if i in right_arm_dofs:
            m.dof_damping[i] = 0.5
        else:
            m.dof_damping[i] = 100.0
        
    # Đặt robot đứng cao lên một chút
    d.qpos[2] = 0.8  # Z
    d.qpos[3:7] = [1, 0, 0, 0] # Quaternion
    
    mujoco.mj_forward(m, d)
    
    t_input = threading.Thread(target=input_thread, args=(m, d))
    t_input.daemon = True
    t_input.start()
    
    try:
        with mujoco.viewer.launch_passive(m, d) as viewer:
            while viewer.is_running() and is_running:
                # --- KHÓA CHẾT THÂN ROBOT LẠI ---
                # Ép tọa độ của Floating Base (Khớp 0->6) luôn cố định
                d.qpos[0:3] = [0.0, 0.0, 0.8]  # x, y, z
                d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0] # quaternion (w, x, y, z)
                d.qvel[0:6] = 0.0 # Vận tốc của thân bằng 0
                
                # Chạy mj_step để cho phép kéo thả vật lý (có damping giữ lại)
                mujoco.mj_step(m, d)
                viewer.sync()
                time.sleep(0.02)
    except Exception as e:
        print(f"Viewer Error: {e}")
        
    is_running = False
    print("\nPose Editor đã đóng.")

if __name__ == '__main__':
    main()
