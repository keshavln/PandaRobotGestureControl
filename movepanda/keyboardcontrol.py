import rclpy
from pynput import keyboard # Changed import
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from time import time
from math import pi, sin, cos

class ServoPublisher(Node):
    def __init__(self):
        super().__init__("servopublisher")
        self.pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.pub2 = self.create_publisher(JointJog, '/servo_node/delta_joint_cmds', 10)
        timerperiod = 0.05
        self.start = time()
        self.timer = self.create_timer(timerperiod, self.publish_twist)

        # --- pynput setup ---
        # Set to store currently pressed keys
        self.pressed_keys = set() 
        # Start the listener thread
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        self.listener.start()
        # --------------------

    # --- pynput callback methods ---
    def on_press(self, key):
        # Add the pressed key to the set
        try:
            self.pressed_keys.add(key.char)
        except AttributeError:
            # For special keys (e.g., space, ctrl)
            self.pressed_keys.add(key)

    def on_release(self, key):
        # Remove the released key from the set
        try:
            self.pressed_keys.remove(key.char)
        except (AttributeError, KeyError):
            # Handle special keys or keys not in set
            if key in self.pressed_keys:
                self.pressed_keys.remove(key)
    # -------------------------------

    def publish_twist(self):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "panda_link0"
        current = time() - self.start
        
        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = 0.0

        # --- Changed checks to use the pressed_keys set ---
        if 'w' in self.pressed_keys:
            twist_msg.twist.linear.x = 1.0
        elif 's' in self.pressed_keys:
            twist_msg.twist.linear.x = -1.0
        else:
            twist_msg.twist.linear.x = 0.0

        if 'a' in self.pressed_keys:
            twist_msg.twist.linear.y = -1.0
        elif 'd' in self.pressed_keys:
            twist_msg.twist.linear.y = 1.0
        else:
            twist_msg.twist.linear.y = 0.0

        if 'e' in self.pressed_keys:
            twist_msg.twist.linear.z = -1.0
        elif 'q' in self.pressed_keys:
            twist_msg.twist.linear.z = 1.0
        else:
            twist_msg.twist.linear.z = 0.0
        # -----------------------------------------------

        self.pub.publish(twist_msg)
        self.get_logger().info(f"Published your messages")

def main():
    rclpy.init()
    servopublisher = ServoPublisher()
    
    # Use try/finally to ensure listener stops on shutdown
    try:
        rclpy.spin(servopublisher)
    except KeyboardInterrupt:
        pass
    finally:
        # --- Cleanly stop the listener thread ---
        servopublisher.listener.stop()
        servopublisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()