import sys
import mujoco
import numpy as np

def verify_ai_environment():
    print("=== KIỂM TRA MÔI TRƯỜNG AI / SIMULATION ===")
    print(f"Trình thông dịch Python: {sys.executable}")
    print(f"Phiên bản MuJoCo: {mujoco.__version__}")
    print(f"Phiên bản NumPy: {np.__version__}")

    # Kiểm tra tính cô lập: Đảm bảo rclpy KHÔNG tồn tại trong không gian này
    try:
        import rclpy
        print("❌ LỖI NGHIÊM TRỌNG: Rò rỉ môi trường ROS 2 vào Conda!")
    except ImportError:
        print("✅ CÔ LẬP THÀNH CÔNG: Môi trường không bị ô nhiễm bởi ROS 2.")

if __name__ == '__main__':
    verify_ai_environment()