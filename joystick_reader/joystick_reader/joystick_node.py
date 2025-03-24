import rclpy
from rclpy.node import Node
from custom_interfaces.msg import ControllerState
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import time
import pygame
import os

class ControllerInterface(Node):
    def __init__(self):
        super().__init__('dualsense_interface')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_joy', 10)
        self.buttons_pub = self.create_publisher(ControllerState, '/controller_state', 10)
        self.dead_man_pub = self.create_publisher(Bool, '/dead_man_switch', 10)
        
        os.environ["SDL_JOYSTICK_DEVICE"] = "/dev/input/js0"
        pygame.init()
        #pygame.joystick.init()
        #self.joystick = pygame.joystick.Joystick(0)
        #self.joystick.init()
        self.joystick = None
        self.joystick_connected = False
        self.connect_joystick()

        self.timer = self.create_timer(0.05, self.check_for_events)

        self.linear_deadband = 0.02 #m/s
        self.angular_deadband = 0.02 #rad/s

    def check_for_events(self):
        events = pygame.event.get()
        self.publish_twist()
        for event in events:
            if event.type == pygame.JOYDEVICEREMOVED:
                self.get_logger().info("Joystick disconnected.")
                self.connect_joystick()
            if event.type == pygame.JOYBUTTONDOWN or event.type == pygame.JOYBUTTONUP or event.type == pygame.JOYHATMOTION:
                self.publish_button_state()
    
    def connect_joystick(self):
        while True:
            pygame.joystick.quit()
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                self.joystick_connected = True
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                self.get_logger().info("Joystick connected.")
                return
            self.joystick_connected = False
            self.get_logger().info("No joystick found, retrying in 1 second...")
            time.sleep(1)

    def publish_button_state(self):

        buttons_msg = ControllerState()
        
        buttons_msg.a_button = bool(self.joystick.get_button(0))
        buttons_msg.b_button = bool(self.joystick.get_button(1))
        buttons_msg.x_button = bool(self.joystick.get_button(2))
        buttons_msg.y_button = bool(self.joystick.get_button(3))
        buttons_msg.l1_button = bool(self.joystick.get_button(4))
        buttons_msg.r1_button = bool(self.joystick.get_button(5))
        buttons_msg.select_button = bool(self.joystick.get_button(6))
        buttons_msg.start_button = bool(self.joystick.get_button(7))
        #buttons_msg.select_button = bool(self.joystick.get_button(8))
        #buttons_msg.start_button = bool(self.joystick.get_button(9))
        
        hat = self.joystick.get_hat(0)
        buttons_msg.right_button = hat[0]==1
        buttons_msg.up_button = hat[1]==1
        buttons_msg.left_button = hat[0]==-1
        buttons_msg.down_button = hat[1]==-1

        self.buttons_pub.publish(buttons_msg)

        dead_man = Bool()
        dead_man.data = bool(self.joystick.get_button(4))

        self.dead_man_pub.publish(dead_man)

    def publish_twist(self):
        twist_msg = Twist()
        twist_msg.linear.x = -self.joystick.get_axis(1) if abs(self.joystick.get_axis(1)) > self.linear_deadband else 0.0
        twist_msg.linear.y = -self.joystick.get_axis(0) if abs(self.joystick.get_axis(0)) > self.linear_deadband else 0.0
        twist_msg.angular.y = -self.joystick.get_axis(4) if abs(self.joystick.get_axis(4)) > self.angular_deadband else 0.0
        twist_msg.angular.z = -self.joystick.get_axis(3) if abs(self.joystick.get_axis(3)) > self.angular_deadband else 0.0
        if twist_msg.linear.x != 0 or twist_msg.linear.y != 0 or twist_msg.angular.z != 0:
            self.cmd_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()