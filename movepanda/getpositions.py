import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class GetPositions(Node):
    def __init__(self):
        super().__init__('getpositions')
        self.target_frame_ = 'panda_link0'
        self.source_frame_ = 'panda_link7' 
        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)
        self.timer_ = self.create_timer(0.1, self.lookup_transform)
        
        self.get_logger().info(
            f"TF Listener started. Looking up transform from "
            f"'{self.target_frame_}' to '{self.source_frame_}'..."
        )

    def lookup_transform(self):
        try:
            now = Time()
            trans = self.tf_buffer_.lookup_transform(
                self.target_frame_,
                self.source_frame_,
                now,
                timeout=Duration(seconds=0.05) # Wait up to 50ms for the transform
            )
            pos = trans.transform.translation
            self.get_logger().info(
                f"End-effector position: [x: {pos.x:.3f}, y: {pos.y:.3f}, z: {pos.z:.3f}]"
            )

        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")


def main():
    rclpy.init()
    node = GetPositions()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

'''
lower z: 0.72
upper z: 0.94
x:0.45
x:0.01
y:-0.6
y:0.6

x: -ve
y: +ve
z: +ve
'''