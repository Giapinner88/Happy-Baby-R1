import math
import numpy as np
import time

class G1FallDetector:
    """
    Bộ phát hiện trạng thái ngã cho Unitree G1 dựa trên dữ liệu IMU.
    """
    def __init__(self, tilt_threshold_deg=50, flip_tilt_deg=30, gyro_threshold=6.0, accel_threshold=30.0, dq_threshold=50.0):
        # Chuyển đổi góc nghiêng sang ngưỡng trục Z của projected gravity
        # Z = -cos(theta). Góc càng lớn, Z càng tiến về 0.
        self.tilt_threshold = -math.cos(math.radians(tilt_threshold_deg))
        self.tilt_30_thresh = -math.cos(math.radians(flip_tilt_deg))
        self.lay_down_thresh = -math.cos(math.radians(85)) # ~ -0.08
        self.gyro_threshold = gyro_threshold
        self.accel_threshold = accel_threshold
        self.dq_threshold = dq_threshold
        
        self.is_fallen = False
        self.is_lay_down = False
        self.fall_time = 0.0
        self.has_impacted = False
        
    def _get_joint_name(self, idx):
        names = [
            "L_Hip_Pitch", "L_Hip_Roll", "L_Hip_Yaw", "L_Knee", "L_Ankle_Pitch", "L_Ankle_Roll",
            "R_Hip_Pitch", "R_Hip_Roll", "R_Hip_Yaw", "R_Knee", "R_Ankle_Pitch", "R_Ankle_Roll",
            "Waist_Roll", "Waist_Yaw",
            "L_Shoulder_Pitch", "L_Shoulder_Roll", "L_Shoulder_Yaw", "L_Elbow", "L_Wrist",
            "R_Shoulder_Pitch", "R_Shoulder_Roll", "R_Shoulder_Yaw", "R_Elbow", "R_Wrist"
        ]
        if 0 <= idx < len(names):
            return names[idx]
        return f"Khớp_{idx}"
        
    def check(self, current_time: float, projected_gravity: np.ndarray, gyro: np.ndarray, accel: np.ndarray, dq: np.ndarray):
        """
        Kiểm tra trạng thái ngã.
        Trả về (is_fallen, is_lay_down, danh_sách_nguyên_nhân_nếu_vừa_ngã)
        """
        accel_norm = np.linalg.norm(accel)
        if accel_norm > self.accel_threshold:
            self.has_impacted = True
            
        gz = projected_gravity[2]
        
        # Chỉ kích hoạt trạng thái nằm hẳn (ngắt momen) nếu góc > 85 độ VÀ đã có ghi nhận va chạm
        lay_down = (gz >= self.lay_down_thresh) and self.has_impacted
        
        new_lay_down = lay_down and not self.is_lay_down
        self.is_lay_down = lay_down

        if self.is_fallen:
            if new_lay_down:
                return True, True, [f"Robot nằm hẳn (90 độ) + Va chạm (>{self.accel_threshold}m/s^2) -> Ngắt toàn bộ momen"]
            return True, self.is_lay_down, []
            
        fall_by_tilt = (gz > self.tilt_threshold)
        # Nếu nghiêng > 30 độ VÀ vận tốc xoay rất nhanh (> 6 rad/s), thì chắc chắn là đang bị lật/vấp ngã mạnh
        fall_by_flip = (gz > self.tilt_30_thresh) and (np.linalg.norm(gyro) > self.gyro_threshold)
        
        fall_by_impact = False # Không dùng va chạm đơn thuần để kích hoạt ngã (tránh đi bộ gây ngã)
        
        # Tay chân vung loạn xạ (vận tốc khớp vượt ngưỡng)
        violating_joints = np.where(np.abs(dq) > self.dq_threshold)[0]
        fall_by_flailing = len(violating_joints) > 0
        
        reasons = []
        if fall_by_tilt: reasons.append(f"Nghiêng quá mức (gz={gz:.2f})")
        if fall_by_flip: reasons.append(f"Lật nhanh (gyro={np.linalg.norm(gyro):.2f})")
        if fall_by_impact: reasons.append(f"Va chạm sàn (accel={accel_norm:.2f} m/s^2)")
        if fall_by_flailing: 
            max_dq = np.max(np.abs(dq))
            violating_names = [self._get_joint_name(i) for i in violating_joints]
            reasons.append(f"Khớp vung loạn xạ (max dq={max_dq:.2f} rad/s ở {', '.join(violating_names)})")
        
        if fall_by_tilt or fall_by_flip or fall_by_impact or fall_by_flailing:
            self.is_fallen = True
            self.fall_time = time.perf_counter()
            return True, self.is_lay_down, reasons
            
        return False, False, []
        
    def reset(self):
        self.is_fallen = False
        self.is_lay_down = False
        self.has_impacted = False
        self.fall_time = 0.0
