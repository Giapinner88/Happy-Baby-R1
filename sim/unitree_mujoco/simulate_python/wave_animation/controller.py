import os
import json
import numpy as np

class WaveController:
    def __init__(self, keyframes_path="wave_animation/wave_keyframes.json", wave_speed=0.8, blend_duration=1.5):
        self.keyframes = None
        self.is_waving = False
        self.blend_waving = 0.0
        self.wave_time = 0.0
        self.wave_speed = wave_speed
        self.blend_duration = blend_duration
        
        self.load_keyframes(keyframes_path)
        
    def load_keyframes(self, path):
        if os.path.exists(path):
            try:
                with open(path, "r") as f:
                    data = json.load(f)
                    self.keyframes = np.array(data.get("right_arm_keyframes", []))
                    print(f"[WaveController] Đã tải thành công {len(self.keyframes)} khung hình vẫy tay từ {path}.")
            except Exception as e:
                print(f"[WaveController] Lỗi khi đọc {path}: {e}")
        else:
            print(f"[WaveController] Chưa tìm thấy file keyframes tại {path}. Hãy chạy wave_animation/record_wave.py trước.")
            
    def toggle(self):
        if self.keyframes is not None and len(self.keyframes) > 0:
            self.is_waving = not self.is_waving
            state_str = "BẬT" if self.is_waving else "TẮT"
            print(f">>> {state_str} CHẾ ĐỘ VẪY TAY!")
        else:
            print("[WaveController] Lỗi: Không có dữ liệu keyframe để vẫy tay. Vui lòng record trước.")
            
    def update_and_get_target_q(self, dt, current_target_q):
        """
        Cập nhật trạng thái và trả về target_q mới (đã được ghi đè cho cánh tay phải nếu đang vẫy).
        current_target_q là mảng numpy copy (29 phần tử).
        Trục cánh tay phải: index 22 đến 28.
        """
        if self.keyframes is None or len(self.keyframes) == 0:
            return current_target_q
            
        # Cập nhật hệ số blend mượt mà (chống giật khi bật/tắt)
        if self.is_waving:
            self.blend_waving = min(1.0, self.blend_waving + dt / self.blend_duration)
            self.wave_time += dt
        else:
            self.blend_waving = max(0.0, self.blend_waving - dt / self.blend_duration)
            # Khi tắt hẳn thì reset thời gian về 0 để lần sau vẫy lại từ đầu
            if self.blend_waving == 0.0:
                self.wave_time = 0.0
            
        if self.blend_waving > 0.0:
            num_frames = len(self.keyframes)
            if num_frames == 1:
                interp_q = self.keyframes[0]
            else:
                total_anim_time = (num_frames - 1) * self.wave_speed
                cycle_time = 2 * total_anim_time
                t_mod = self.wave_time % cycle_time
                
                if t_mod < total_anim_time:
                    # Đang đi tới (Forward)
                    t_anim = t_mod
                else:
                    # Đang đi lùi (Backward - Ping pong)
                    t_anim = cycle_time - t_mod
                    
                frame_idx = int(t_anim / self.wave_speed)
                alpha = (t_anim % self.wave_speed) / self.wave_speed
                frame_idx = min(frame_idx, num_frames - 2)
                
                frame_A = self.keyframes[frame_idx]
                frame_B = self.keyframes[frame_idx + 1]
                interp_q = (1 - alpha) * frame_A + alpha * frame_B
            
            # Blend giữa lệnh RL (current_target_q) và lệnh Waving (interp_q) cho khớp 22->28
            current_target_q[22:29] = (1.0 - self.blend_waving) * current_target_q[22:29] + self.blend_waving * interp_q
            
        return current_target_q
