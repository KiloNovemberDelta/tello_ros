import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

from tello_msgs.srv import TelloAction
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library


class ActionManager(Node):
    def __init__(self):
        super().__init__('controller_manager')
        self.cli = self.create_client(TelloAction, '/drone1/tello_action')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = TelloAction.Request()

    def ask_for_takeoff(self):
        self.req.cmd = "takeoff"
        self.future = self.cli.call_async(self.req)

    def ask_for_landing(self):
        self.req.cmd = "land"
        self.future = self.cli.call_async(self.req)


class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        ### CREATE THE PUBLISHER FOR COMMAND VELOCITY
        self.cmd_vel_publisher_ = self.create_publisher(Twist, '/drone1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.cmd_vel_loop)
        # Init desired speed
        self.vx = 0.0
        self.vy = 0.0
        self.v_yaw = 0.0

        ### CREATE THE SUBSCRIBER FOR IMAGE RECEPTION
        self.image_subscription_ = self.create_subscription(Image, '/drone1/image_raw', self.image_callback, 10)
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

    def cmd_vel_loop(self):
        """
        I'm used to publish every 0.5 seconds the velocity command.
        You don't need to modify me, just modify vx, vy and v_yaw when you need it.
        """
        msg = Twist()
        msg.linear.x = self.vx
        msg.linear.y = self.vy
        msg.angular.z = self.v_yaw
        self.cmd_vel_publisher_.publish(msg)

    def image_callback(self, data):
        self.get_logger().info('Receiving video frame')
        current_frame = self.br.imgmsg_to_cv2(data)
        # Display image
        cv2.imshow("camera", current_frame)

        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    # Wait for take off service to be ready
    action_manager = ActionManager()
    action_manager.ask_for_takeoff()
    ready_to_continue_mission = False

    # Try to takeoff, wait for the return of the service
    while rclpy.ok():
        rclpy.spin_once(action_manager)
        if action_manager.future.done():
            try:
                response = action_manager.future.result()
                if response.rc == 1:  # OK
                    print("Take off is a success !")
                    ready_to_continue_mission = True
                else:  # NOT OK
                    print("Something is wrong with the takeoff. LANDING NOW")
            except Exception as e:
                action_manager.get_logger().info("Service call failed %r" % (e,))
            break

    if ready_to_continue_mission:
        controller = Controller()
        try:
            while rclpy.ok():
                rclpy.spin_once(controller)
        except KeyboardInterrupt:
            print("Stopping the control. Ask for landing.")
        controller.destroy_node()

    # ASK FOR LANDING
    action_manager.ask_for_landing()
    while rclpy.ok():
        rclpy.spin_once(action_manager)
        if action_manager.future.done():
            try:
                response = action_manager.future.result()
                if response.rc == 1:  # OK
                    print("Landing is a success !")
                    break  # Only if landing is ok
            except Exception as e:
                action_manager.get_logger().info("Service call failed %r" % (e,))
            break

    rclpy.shutdown()


if __name__ == '__main__':
    main()
