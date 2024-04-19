#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

class OdomPubNode(Node):
    def __init__(self):
        super().__init__('odom_pub_node')
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/ackodom',
            self.odom_callback,
            30)
        self.subscription_imu = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            30)
        self.publisher_odom = self.create_publisher(
            Odometry,
            '/odom',
            30)
        self.odom_data = None
        self.imu_data = None

    def odom_callback(self, msg):
        self.odom_data = msg

    def imu_callback(self, msg):
        self.imu_data = msg
        self.publish_combined_data()

    def publish_combined_data(self):
        if self.odom_data and self.imu_data:
            new_odom = Odometry()
            new_odom.header.stamp = self.get_clock().now().to_msg()
            new_odom.header.frame_id = self.odom_data.header.frame_id
            new_odom.child_frame_id = self.odom_data.child_frame_id

            # 使用ackodom的线性速度和imu的角速度
            new_odom.twist.twist.linear.x = self.odom_data.twist.twist.linear.x
            new_odom.twist.twist.angular = self.imu_data.angular_velocity

            # 使用imu的方向
            new_odom.pose.pose.orientation = self.imu_data.orientation

            # 发布到/odom
            self.publisher_odom.publish(new_odom)

            # 重置存储的数据
            self.odom_data = None
            self.imu_data = None

def main(args=None):
    rclpy.init(args=args)
    odom_pub_node = OdomPubNode()
    rclpy.spin(odom_pub_node)
    odom_pub_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
