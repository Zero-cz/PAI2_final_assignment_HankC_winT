import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor')
        
        # 速度指令のパブリッシャ（緊急停止用）
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # センサー監視用サブスクライバ
        self.subscription_scan = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
            
        self.emergency_stop = False
        self.get_logger().info('Safety Monitor Node has been started.')

    def scan_callback(self, msg):
        if len(msg.ranges) > 0:
            # 正面付近のデータをチェック
            front_ranges = msg.ranges[0:10] + msg.ranges[350:360]
            valid_ranges = [r for r in front_ranges if not math.isinf(r) and not math.isnan(r)]
            
            if valid_ranges:
                min_dist = min(valid_ranges)
                
                # 危険距離（例: 20cm以内）に迫ったら緊急停止フラグをON
                if min_dist < 0.20:
                    if not self.emergency_stop:
                        self.get_logger().warn(f'EMERGENCY STOP! Obstacle too close: {min_dist:.2f}m')
                        self.emergency_stop = True
                    
                    # 停止コマンドを強制パブリッシュ
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.publisher_.publish(twist)
                else:
                    if self.emergency_stop:
                        self.get_logger().info('Danger cleared. Resuming normal operation.')
                        self.emergency_stop = False

def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
