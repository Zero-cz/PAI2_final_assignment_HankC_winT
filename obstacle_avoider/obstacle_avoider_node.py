import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        
        # パブリッシャとサブスクライバ
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription_scan = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.subscription_odom = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # 状態管理
        self.state = 'FORWARD' # FORWARD, AVOID, RETURN
        self.front_distance = float('inf')
        
        # 位置情報
        self.start_x = 0.0
        self.start_y = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.odom_initialized = False

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        if not self.odom_initialized:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.odom_initialized = True

    def scan_callback(self, msg):
        # 前方0度あたりのレーザー距離を取得（配列の中央付近）
        if len(msg.ranges) > 0:
            # 簡易的に正面の数点最小値を取る
            front_ranges = msg.ranges[0:15] + msg.ranges[345:360]
            valid_ranges = [r for r in front_ranges if not math.isinf(r) and not math.isnan(r)]
            if valid_ranges:
                self.front_distance = min(valid_ranges)

    def control_loop(self):
        twist = Twist()

        if self.state == 'FORWARD':
            if self.front_distance < 0.6: # 60cm以内に障害物
                self.get_logger().info('Obstacle detected! Avoiding...')
                self.state = 'AVOID'
            else:
                twist.linear.x = 0.2
                twist.angular.z = 0.0

        elif self.state == 'AVOID':
            # 右へ旋回して避ける動作（簡易的）
            twist.linear.x = 0.0
            twist.angular.z = -0.5
            # 一定時間または障害物が遠ざかったら復帰モードへ移行する条件を設定
            if self.front_distance > 1.2:
                self.get_logger().info('Avoided. Returning to start position...')
                self.state = 'RETURN'

        elif self.state == 'RETURN':
            # 元の位置（start_x, start_y）との距離を計算
            dist_to_start = math.sqrt((self.current_x - self.start_x)**2 + (self.current_y - self.start_y)**2)
            if dist_to_start < 0.2:
                self.get_logger().info('Returned to start position!')
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.state = 'FORWARD'
            else:
                # 簡易的に低速で前進・あるいは原点方向への制御
                twist.linear.x = 0.1
                twist.angular.z = 0.1

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
