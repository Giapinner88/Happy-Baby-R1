import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

class DummyPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        self.publisher_ = self.create_publisher(String, 'r1_internal_test', 10)
        self.timer = self.create_timer(0.5, self.timer_cb)
        self.count = 0

    def timer_cb(self):
        msg = String()
        msg.data = f'Gói tin điều khiển #{self.count}'
        self.publisher_.publish(msg)
        self.count += 1

class DummySubscriber(Node):
    def __init__(self):
        super().__init__('test_subscriber')
        self.subscription = self.create_subscription(
            String, 'r1_internal_test', self.listener_cb, 10)

    def listener_cb(self, msg):
        self.get_logger().info(f'DDS Nhận: "{msg.data}"')

def main():
    print("=== KHỞI ĐỘNG KIỂM THỬ DDS MIDDLEWARE ===")
    rclpy.init()
    pub_node = DummyPublisher()
    sub_node = DummySubscriber()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(pub_node)
    executor.add_node(sub_node)
    
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    try:
        executor_thread.join(timeout=3.0)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        print("=== HOÀN THÀNH KIỂM THỬ ===")

if __name__ == '__main__':
    main()