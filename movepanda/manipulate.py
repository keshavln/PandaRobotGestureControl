import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import Point, TwistStamped
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

def velocity(initial_coor, final_coor):
    velocity_unclamped = 50.0*(final_coor-initial_coor)
    if abs(velocity_unclamped) < 1:
        return velocity_unclamped
    else:
        return velocity_unclamped/abs(velocity_unclamped)
    
class Manipulate(Node):
    def __init__(self):
        super().__init__("manipulate")

        self.target_frame_ = 'panda_link0'
        self.source_frame_ = 'panda_link7' 
        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)

        self.coordinates_subscriber = self.create_subscription(Point, 'hand_position', self.move_arm, 10)
        self.velocity_publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)

    def move_arm(self, point_msg):
        now = Time()
        try:
            #First, get the current end-effector position

            trans = self.tf_buffer_.lookup_transform(
                    self.target_frame_,
                    self.source_frame_,
                    now,
                    timeout=Duration(seconds=0.05)
                )
            pos = trans.transform.translation

            #Next, get the desired end-effector position (determined by hand)

            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = "panda_link0"

            twist_msg.twist.angular.x = 0.0
            twist_msg.twist.angular.y = 0.0
            twist_msg.twist.angular.x = 0.0

            twist_msg.twist.linear.x = velocity(pos.x, point_msg.x)
            twist_msg.twist.linear.y = velocity(pos.y, point_msg.y)
            twist_msg.twist.linear.z = velocity(pos.z, point_msg.z)

            self.velocity_publisher.publish(twist_msg)
        except:
            self.get_logger().info("Failed to get transform, will try again")

def main():
    rclpy.init()
    manipulate = Manipulate()
    rclpy.spin(manipulate)
    manipulate.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()