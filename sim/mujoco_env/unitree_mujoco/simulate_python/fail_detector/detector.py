import math
import numpy as np
import time

class G1FailDetector:
    """
    Bộ phát hiện trạng thái thất bại (ngã, lật, vung chân tay loạn xạ) cho Unitree G1 dựa trên IMU và vận tốc khớp.
    """
    def __init__(self, tilt_threshold_deg=50, flip_tilt_deg=30, gyro_threshold=6.0, accel_threshold=30.0):
        # Chuyển đổi góc nghiêng sang ngưỡng trục Z của projected gravity
        # Z = -cos(theta). Góc càng lớn, Z càng tiến về 0.
        self.tilt_threshold = -math.cos(math.radians(tilt_threshold_deg))
        self.tilt_30_thresh = -math.cos(math.radians(flip_tilt_deg))
        self.lay_down_thresh = -math.cos(math.radians(85)) # ~ -0.08
        self.gyro_threshold = gyro_threshold
        self.accel_threshold = accel_threshold
        
        self.is_failed = False
        self.is_lay_down = False
        self.fail_time = 0.0
        self.has_impacted = False
        self.dq_threshold = 30.0 # Ngưỡng vận tốc khớp (rad/s) để phát hiện vung loạn xạ
        
    def check(self, projected_gravity: np.ndarray, gyro: np.ndarray, accel: np.ndarray, dq: np.ndarray = None):
        """
        Kiểm tra trạng thái thất bại.
        Trả về (is_failed, is_lay_down, danh_sách_nguyên_nhân_nếu_vừa_thất_bại)
        """
        accel_norm = np.linalg.norm(accel)
        if accel_norm > self.accel_threshold:
            self.has_impacted = True
            
        gz = projected_gravity[2]
        
        # Chỉ kích hoạt trạng thái nằm hẳn (ngắt momen) nếu góc > 85 độ VÀ đã có ghi nhận va chạm
        lay_down = (gz >= self.lay_down_thresh) and self.has_impacted
        
        new_lay_down = lay_down and not self.is_lay_down
        self.is_lay_down = lay_down

        if self.is_failed:
            if new_lay_down:
                return True, True, [f"Robot nằm hẳn (90 độ) + Va chạm (>{self.accel_threshold}m/s^2) -> Ngắt toàn bộ momen"]
            return True, self.is_lay_down, []
            
        fail_by_tilt = (gz > self.tilt_threshold)
        # Nếu nghiêng > 30 độ VÀ vận tốc xoay rất nhanh (> 6 rad/s), thì chắc chắn là đang bị lật/vấp ngã mạnh
        fail_by_flip = (gz > self.tilt_30_thresh) and (np.linalg.norm(gyro) > self.gyro_threshold)
        
        fail_by_impact = False # Không dùng va chạm đơn thuần để kích hoạt ngã (tránh đi bộ gây ngã)
        
        fail_by_flailing = False
        if dq is not None:
            max_dq = np.max(np.abs(dq))
            if max_dq > self.dq_threshold:
                fail_by_flailing = True
        
        reasons = []
        if fail_by_tilt: reasons.append(f"Nghiêng quá mức (gz={gz:.2f})")
        if fail_by_flip: reasons.append(f"Lật nhanh (gyro={np.linalg.norm(gyro):.2f})")
        if fail_by_impact: reasons.append(f"Va chạm sàn (accel={accel_norm:.2f} m/s^2)")
        if fail_by_flailing: reasons.append(f"Tay chân vung loạn xạ (max dq={max_dq:.2f} rad/s)")
        
        if fail_by_tilt or fail_by_flip or fail_by_impact or fail_by_flailing:
            self.is_failed = True
            self.fail_time = time.perf_counter()
            return True, self.is_lay_down, reasons
            
        return False, False, []
        
    def reset(self):
        self.is_failed = False
        self.is_lay_down = False
        self.has_impacted = False
        self.fail_time = 0.0
