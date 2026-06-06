import os
import time
import threading
import collections
import numpy as np
import math
import matplotlib
matplotlib.use('Agg') # Bắt buộc dùng Agg để an toàn khi vẽ đồ thị ngầm (Thread)
import matplotlib.pyplot as plt
from datetime import datetime

class IMULogger:
    """
    Module ghi nhận tín hiệu IMU dạng vòng lặp (Circular Buffer) để không gây nghẽn CPU.
    Tự động xuất đồ thị Snapshot khi phát hiện ngã, chạy hoàn toàn trong Background Thread.
    """
    def __init__(self, output_dir="fail_detector/plots", hz=50, window_before=3, window_after=3):
        self.output_dir = os.path.abspath(output_dir)
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir, exist_ok=True)
            
        # Tính toán kích thước bộ đệm dựa trên Hz và thời gian trước + sau khi ngã
        self.hz = hz
        self.window_before = window_before
        self.window_after = window_after
        self.total_time = window_before + window_after
        self.max_len = int(self.total_time * self.hz)
        
        # Buffer lưu trữ: (time, gz, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z)
        self.buffer = collections.deque(maxlen=self.max_len)
        
        # Ngưỡng vẽ đồ thị
        self.thresh_gz = -math.cos(math.radians(50))
        self.thresh_gyro = 6.0
        self.thresh_accel = 30.0
        
        self.is_waiting_for_aftermath = False
        self.fall_trigger_time = 0.0
        
    def log_step(self, current_time, projected_gravity, gyro, accel):
        """
        Gọi mỗi vòng lặp điều khiển. Rất nhanh (O(1)), hoàn toàn không tốn CPU.
        """
        gz = projected_gravity[2]
        self.buffer.append((
            current_time, 
            gz, 
            gyro[0], gyro[1], gyro[2], 
            accel[0], accel[1], accel[2]
        ))
        
        # Nếu đang trong quá trình ghi nhận thêm 10s sau khi ngã
        if self.is_waiting_for_aftermath:
            if (current_time - self.fall_trigger_time) >= self.window_after:
                # Đã thu thập đủ dữ liệu. Tiến hành xuất ảnh ở Thread riêng biệt.
                data_snapshot = list(self.buffer)
                self.is_waiting_for_aftermath = False
                
                # Chạy ngầm tiến trình vẽ đồ thị
                thread = threading.Thread(target=self._plot_and_save, args=(data_snapshot, self.fall_trigger_time))
                thread.daemon = True
                thread.start()
                
    def trigger_fail_event(self, current_time):
        """Đánh dấu thời điểm ngã hoặc vung loạn xạ"""
        if not self.is_waiting_for_aftermath:
            self.is_waiting_for_aftermath = True
            self.fail_trigger_time = current_time
            print(f">>> [IMU Logger] Đã đánh dấu thời điểm FAIL. Sẽ xuất đồ thị sau {self.window_after} giây...")
            
    def reset(self):
        """
        Reset trạng thái khi người dùng phục hồi robot bằng phím R.
        Lưu ý: Không reset is_waiting_for_aftermath để quá trình chụp ảnh vẫn được tiếp tục (ghi nhận cả lúc đứng dậy).
        """
        # Không clear trạng thái chờ chụp ảnh để lúc nào cũng lưu được ảnh
        pass

    def _plot_and_save(self, data, fail_time):
        """
        Hàm thực hiện vẽ đồ thị Matplotlib. Chạy ở luồng riêng nên không block vòng lặp chính.
        """
        try:
            times = np.array([d[0] for d in data])
            gzs = np.array([d[1] for d in data])
            gyros = np.array([d[2:5] for d in data])
            accels = np.array([d[5:8] for d in data])
            
            # Đưa mốc thời gian về 0 tại thời điểm ngã
            rel_times = times - fail_time
            
            fig, axs = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
            
            # 1. Góc nghiêng (Tilt Angle)
            gzs_clipped = np.clip(gzs, -1.0, 1.0)
            tilt_angles = np.degrees(np.arccos(-gzs_clipped))
            
            axs[0].plot(rel_times, tilt_angles, label='Góc nghiêng (Độ)', color='blue', linewidth=2)
            axs[0].axvline(x=0, color='red', linestyle='--', linewidth=2, label='Thời điểm FAIL')
            axs[0].hlines(y=50.0, xmin=rel_times[0], xmax=0, color='purple', linestyle='--', linewidth=2, label='Ngưỡng ngã (50°)')
            axs[0].set_ylabel('Góc (Độ)')
            axs[0].set_title('Góc Nghiêng Của Robot (0° = Đứng thẳng, 90° = Nằm ngang)')
            axs[0].legend(loc='upper right')
            axs[0].grid(True, alpha=0.5)
            
            # 2. Vận tốc góc Gyro
            gyro_mag = np.linalg.norm(gyros, axis=1)
            axs[1].plot(rel_times, gyro_mag, label='Độ lớn Gyro (Norm)', color='orange', linewidth=2)
            axs[1].axvline(x=0, color='red', linestyle='--', linewidth=2)
            axs[1].hlines(y=self.thresh_gyro, xmin=rel_times[0], xmax=0, color='purple', linestyle='--', linewidth=2, label='Ngưỡng vi phạm (6.0)')
            axs[1].set_ylabel('Gyro (rad/s)')
            axs[1].set_title('Tốc độ xoay lật của Robot')
            axs[1].legend(loc='upper right')
            axs[1].grid(True, alpha=0.5)
            
            # 3. Gia tốc tuyến tính Accel
            accel_mag = np.linalg.norm(accels, axis=1)
            axs[2].plot(rel_times, accel_mag, label='Độ lớn Accel (Norm)', color='green', linewidth=2)
            axs[2].axvline(x=0, color='red', linestyle='--', linewidth=2)
            axs[2].hlines(y=self.thresh_accel, xmin=rel_times[0], xmax=0, color='purple', linestyle='--', linewidth=2, label='Ngưỡng vi phạm (30.0)')
            axs[2].set_ylabel('Accel (m/s²)')
            axs[2].set_xlabel('Thời gian so với thời điểm FAIL (giây)')
            axs[2].set_title('Gia tốc (Va chạm sàn sinh ra gia tốc lớn)')
            axs[2].legend(loc='upper right')
            axs[2].grid(True, alpha=0.5)
            
            plt.suptitle(f'Phân Tích IMU Quá Trình FAIL ({self.window_before}s trước và {self.window_after}s sau)', fontsize=16, fontweight='bold')
            plt.tight_layout()
            
            # Lưu file
            now_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            filename = os.path.join(self.output_dir, f"fail_imu_log_{now_str}.png")
            plt.savefig(filename, dpi=150)
            plt.close(fig)
            
            print(f"\n>>> [IMU Logger] HOÀN TẤT! Đã lưu đồ thị phân tích FAIL tại: {filename}\n")
        except Exception as e:
            print(f"[IMU Logger] Lỗi khi vẽ đồ thị: {e}")
