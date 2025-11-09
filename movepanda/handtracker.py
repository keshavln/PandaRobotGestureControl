#!/usr/bin/env python3
import os
os.environ['QT_QPA_PLATFORM'] = 'xcb'
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Point
import cv2
import mediapipe as mp
import math
import time
from control_msgs.action import GripperCommand

time.sleep(0.2)

MIN_HAND_SIZE = 0.12  # Normalized distance (very far)
MAX_HAND_SIZE = 0.2  # Normalized distance (very close)
x_upper = 0.45
x_lower = 0.01
y_upper = 0.6
y_lower = -0.6
z_upper = 0.94
z_lower = 0.72

#Helper functions

def lerp(lower, upper, value, multiplier):
    """Performs linear interpolation. Value is expected to be in the range [0,1], and multiplier is either 1 or -1."""
    if multiplier == 1:
        return lower + (upper-lower)*value #= (1-value)*lower + value*upper
    else:
        return upper - (upper-lower)*value #= (1-value)*upper + value*lower
    
def inverse_lerp(a, b, value):
    """Calculates the normalized position of 'value' between 'a' and 'b'."""
    if a == b:
        return 0.0
    return (value - a) / (b - a)

def clamp(n):
    """Clamps a value between 0 and 1."""
    return max(0.0, min(n,1.0))

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5)
mp_drawing = mp.solutions.drawing_utils
x = y = z = 0.0
cap = cv2.VideoCapture(0)

class HandTracker(Node):
    def __init__(self):
        super().__init__('handtracker')
        self.handpublisher = self.create_publisher(Point, "hand_position", 10)
        self.timer = self.create_timer(0.05, self.get_positions)
        self.gripper_state = 'closed'
        self.grip_action = ActionClient(self, GripperCommand, '/panda_hand_controller/gripper_cmd')
        self.grip_action.wait_for_server()

    def get_positions(self):
        global mp_hands, hands, mp_drawing, x, y, z, cap
        success, image = cap.read()
        image = cv2.flip(image, 1)
        image_rgb = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
        results = hands.process(image_rgb)
        
        if results.multi_hand_landmarks:
            hand_landmarks = results.multi_hand_landmarks[0]
            mp_drawing.draw_landmarks(
                image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            #Calculation of calibrated x,y,z coordinates of hand

            #Distance between wrist and base of pinky used to determine x coordinate
            wrist = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST]
            knuckle = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP]
            raw_size = math.hypot(wrist.x - knuckle.x, wrist.y - knuckle.y)

            x_normalized = inverse_lerp(MIN_HAND_SIZE, MAX_HAND_SIZE, raw_size)

            x = 1.0 - clamp(x_normalized)
            y = clamp(wrist.x)
            z = clamp((1.0 - wrist.y) * 1.2)
            
            #Controlling gripper
            #Distance between thumb and index tips used to open and close the gripper

            index_tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
            thumb_tip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP]
            it_ratio = math.hypot(index_tip.x-thumb_tip.x, index_tip.y-thumb_tip.y) / raw_size

            gripper_goal = GripperCommand.Goal()
            gripper_goal.command.max_effort = 20.0

            if it_ratio < 0.5 and self.gripper_state == 'open':
                gripper_goal.command.position = 0.0
                future = self.grip_action.send_goal_async(gripper_goal)
                future.add_done_callback(self.closed)
            elif it_ratio > 0.5 and self.gripper_state == 'closed':
                gripper_goal.command.position = 0.035
                future = self.grip_action.send_goal_async(gripper_goal)
                future.add_done_callback(self.opened)

            self.get_logger().info(f"index thumb distance is {it_ratio:.3f}")

        cv2.putText(image, f"X: {x:.3f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(image, f"Y: {y:.3f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(image, f"Z: {z:.3f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        #Calibrating and publishing coordinates

        msg = Point()
        msg.x = x_new = lerp(x_lower, x_upper, x, -1)
        msg.y = y_new = lerp(y_lower, y_upper, y, 1)
        msg.z = z_new = lerp(z_lower, z_upper, z, 1)
        self.handpublisher.publish(msg)
        
        cv2.imshow('Hand Tracker (Press Q to quit)', image)
    
    def opened(self, future):
        self.get_logger().info("Opened gripper")
        self.gripper_state = 'open'
    
    def closed(self, future):
        self.get_logger().info("Closed gripper")
        self.gripper_state = 'closed'
        

def main():
        rclpy.init()
        handtracker = HandTracker()
        while rclpy.ok():
            rclpy.spin_once(handtracker, timeout_sec=0.001)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        hands.close()
        cap.release()
        cv2.destroyAllWindows()
        handtracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()