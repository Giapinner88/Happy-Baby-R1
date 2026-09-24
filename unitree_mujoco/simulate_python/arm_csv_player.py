import os
import csv
import numpy as np

class ArmCSVPlayer:
    def __init__(self, folder="motions", blend_duration=0.5, retract_duration=1.5):
        self.folder = folder
        self.motions = {}          # name -> numpy array (num_frames, 14)
        self.active_motion = None  # tên động tác đang kích hoạt
        self.play_idx = 0
        self.is_playing = False
        self.is_retracting = False # trạng thái đang thu tay về (fade out)
        self.blend_weight = 0.0
        self.blend_duration = blend_duration  # Thời gian chuyển tiếp mượt mà khi bật lên (giây)
        self.retract_duration = retract_duration  # Thời gian chuyển tiếp mượt mà khi thu tay về (giây)
        
        # Nạp trước 3 file CSV cho khớp tay
        self.load_motion("vaytay", "unitree_g1_vaytay.csv")
        self.load_motion("traitim", "unitree_g1_traitim.csv")
        self.load_motion("battay", "unitree_g1_battay.csv")
        
    def load_motion(self, name, filename):
        path = os.path.join(self.folder, filename)
        if not os.path.exists(path):
            print(f"[ArmCSVPlayer] Không tìm thấy file: {path}")
            return
        try:
            frames = []
            with open(path, "r") as f:
                reader = csv.reader(f)
                for r in reader:
                    if r:
                        # Chuyển đổi dòng thành float
                        row_floats = [float(x) for x in r]
                        # Khớp tay của G1 là từ index 15 đến 28 (tương ứng cột 22 đến 35 trong file CSV)
                        arm_joints = row_floats[22:36]
                        frames.append(arm_joints)
            self.motions[name] = np.array(frames, dtype=np.float32)
            print(f"[ArmCSVPlayer] Đã tải thành công '{name}' từ {filename} ({len(frames)} khung hình).")
        except Exception as e:
            print(f"[ArmCSVPlayer] Lỗi nạp chuyển động '{name}' từ {filename}: {e}")
            
    def trigger(self, name):
        if name not in self.motions:
            print(f"[ArmCSVPlayer] Không có dữ liệu chuyển động '{name}'!")
            return
            
        # Nếu đang chạy đúng động tác đó (cho dù đang chơi hay đang giữ tư thế cuối) -> thu tay về
        if self.active_motion == name and not self.is_retracting:
            self.is_playing = False
            self.is_retracting = True
            print(f">>> YÊU CẦU THU TAY VỀ: {name.upper()}")
        else:
            # Bật động tác tay mới (hoặc chơi lại từ đầu nếu đang thu về)
            self.active_motion = name
            self.play_idx = 0
            self.is_playing = True
            self.is_retracting = False
            print(f">>> BẮT ĐẦU CHƠI ĐỘNG TÁC TAY: {name.upper()}")
            
    def update_and_blend(self, dt, current_target_q):
        """
        Cập nhật tiến trình và ghi đè góc khớp tay (khớp 15->28) vào target_q.
        current_target_q: mảng numpy (29 phần tử).
        """
        # Cập nhật hệ số blend mượt mà
        if self.active_motion is not None and not self.is_retracting:
            self.blend_weight = min(1.0, self.blend_weight + dt / self.blend_duration)
        else:
            self.blend_weight = max(0.0, self.blend_weight - dt / self.retract_duration)
            if self.blend_weight == 0.0:
                self.active_motion = None
                self.is_retracting = False
                
        # Thực hiện ghi đè và nội suy mượt mà nếu có động tác đang active
        if self.blend_weight > 0.0 and self.active_motion is not None:
            motion_data = self.motions[self.active_motion]
            num_frames = len(motion_data)
            
            # Đảm bảo chỉ số khung hình an toàn
            if self.active_motion == "vaytay":
                actual_idx = self.play_idx
            elif self.active_motion == "traitim":
                actual_idx = min(self.play_idx, 87)
            elif self.active_motion == "battay":
                actual_idx = min(self.play_idx, 78)
            else:
                actual_idx = min(self.play_idx, num_frames - 1)
                
            # Lấy góc khớp tay tại khung hình hiện tại
            target_arms = motion_data[actual_idx]
            
            # Ghi đè vào khớp tay 15->28 (tổng cộng 14 khớp)
            current_target_q[15:29] = (1.0 - self.blend_weight) * current_target_q[15:29] + self.blend_weight * target_arms
            
            # Tiến tới khung hình tiếp theo nếu đang chơi
            if self.is_playing:
                self.play_idx += 1
                if self.active_motion == "vaytay":
                    limit_end = min(112, num_frames - 1)
                    limit_start = min(70, limit_end)
                    if self.play_idx > limit_end:
                        self.play_idx = limit_start
                elif self.active_motion == "traitim":
                    limit_peak = min(87, num_frames - 1)
                    if self.play_idx >= limit_peak:
                        self.play_idx = limit_peak
                        self.is_playing = False
                        print(f">>> Đã đạt tư thế tối đa của động tác '{self.active_motion}'. Giữ nguyên tư thế. Nhấn phím lần 2 để thu tay về.")
                elif self.active_motion == "battay":
                    limit_peak = min(78, num_frames - 1)
                    if self.play_idx >= limit_peak:
                        self.play_idx = limit_peak
                        self.is_playing = False
                        print(f">>> Đã đạt tư thế tối đa của động tác '{self.active_motion}'. Giữ nguyên tư thế. Nhấn phím lần 2 để thu tay về.")
                else:
                    if self.play_idx >= num_frames:
                        self.play_idx = num_frames - 1
                        self.is_playing = False
                        print(f">>> Đã đạt tư thế cuối của động tác '{self.active_motion}'. Nhấn phím lần 2 để thu tay về.")
                        
        return current_target_q
