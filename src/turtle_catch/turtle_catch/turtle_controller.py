#!/usr/bin/env python3

import rclpy
import math
import random
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from turtle_interfaces.msg import TurtleArray
from turtle_interfaces.srv import CatchTurtle
from geometry_msgs.msg import Twist

class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.get_logger().info("Turtle Controller Active...")

        # Parameter Declarations
        self.declare_parameter("catch_closest_turtle_first", True)
        self.declare_parameter("k_linear", 1.0)
        self.declare_parameter("k_angular", 4.0)
        self.declare_parameter("catch_radius", 0.5)
        self.declare_parameter("master_pen_off", 0)
        self.declare_parameter("master_pen_width", 3)
        
        # Parameter Value Assignemnt
        self.catch_closest_turtle_first = self.get_parameter("catch_closest_turtle_first").value
        self.k_linear = self.get_parameter("k_linear").value
        self.k_angular = self.get_parameter("k_angular").value
        self.catch_radius = self.get_parameter("catch_radius").value
        self.master_pen_off = bool(self.get_parameter("master_pen_off").value)
        self.master_pen_width = self.get_parameter("master_pen_width").value

        self.current_pose = None
        self.current_target_name = None
        self.is_catching = False  # Flag: true if curently catching turtle
        self.alive_turtles = []
        
        # Pen client
        self.pen_client_ = self.create_client(SetPen, "/turtle1/set_pen")
        while not self.pen_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for /set_pen service...")
            
        self.catch_turtle_client_ = self.create_client(CatchTurtle, "catch_turtle")
        while not self.catch_turtle_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for /catch_turtle service...")
            
        # Disable Pen Logic        
        if self.master_pen_off:
            pen_req = SetPen.Request()
            pen_req.off = 1
            self.pen_client_.call_async(pen_req)
            self.get_logger().info("Pen turned OFF for master turtle.") 
        
        # Get current position of turtle
        # sub: turtle1/pose
        self.master_subscriber_ = self.create_subscription(Pose, "turtle1/pose", self.get_master_pose, 10) 
        
        # sub: alive_turtles
        self.alive_turtles_subscriber_ = self.create_subscription(TurtleArray, "alive_turtles", self.get_alive_turtles, 10)

        # pub: /turtle1/cmd_vel
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        # control loop timer
        self.timer_ = self.create_timer(0.1, self.control_loop)

    def get_master_pose(self, msg: Pose):
        """
        Updates Current position of master turtle.
        
        Args:
            msg (Pose): Pose message containing the x (float), y (float), and theta (float)
        
        Returns:
            None        
        """
        self.current_pose = [msg.x, msg.y, msg.theta]

    def get_alive_turtles(self, msg: TurtleArray):
        """
        Updates the list of alive turtles and resets the target if it has been caught.

        Args:
            msg (TurtleArray): Array of currently active turtles.

        Returns:
            None
        """

        self.alive_turtles = msg.turtles
        
        # If currently catching and target is gone, reset
        if self.is_catching and self.current_target_name not in [t.name for t in self.alive_turtles]:
            self.get_logger().debug(f"Target {self.current_target_name} caught successfully!")
            self.is_catching = False
            self.current_target_name = None

    def control_loop(self):
        """
        Main control loop for the hunter turtle.

        Computes velocities to move towards the current target turtle using P-control.
        Stops and calls the catch service when close enough.

        Returns:
            None
        """

        # Early return if no pose or no turtles
        if self.current_pose is None or not self.alive_turtles:
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmd_vel_publisher_.publish(msg)
            return

        # If catching, wait for completion
        if self.is_catching:
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmd_vel_publisher_.publish(msg)
            return

        # Select target turtle
        if self.current_target_name not in [t.name for t in self.alive_turtles]:
            # Current target is not valid, select new one
            if self.catch_closest_turtle_first:
                target_turtle = self.get_closest_turtle()
            else:
                target_turtle = self.alive_turtles[0]
            self.current_target_name = target_turtle.name
            self.get_logger().debug(f"New target: {self.current_target_name}")
        else:
            # Find current target turtle in the list
            target_turtle = next(t for t in self.alive_turtles if t.name == self.current_target_name)

        # Get target turtle coordinates
        xt, yt = target_turtle.x, target_turtle.y
        target_name = target_turtle.name

        # Compute P-Controller
        xm, ym, theta = self.current_pose
        dx = xt - xm
        dy = yt - ym
        distance = math.sqrt(dx**2 + dy**2)

        # Angle to target
        angle_to_target = math.atan2(dy, dx)
        angle_error = angle_to_target - theta
        
        # Normalize angle error to [-pi, pi]
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        # Create velocity message
        msg = Twist()

        # Check if turtle is close enough to catch
        if distance >= self.catch_radius:
            # Move towards target
            v = max(1.0, self.k_linear * distance)
            omega = self.k_angular * angle_error * min(1.0, distance/self.catch_radius)
            msg.linear.x = v
            msg.angular.z = omega
        else:
            # Close enough - stop and catch
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            
            # Set catching flag to prevent further movement
            self.is_catching = True
            
            # Call catch service
            self.get_logger().debug(f"Attempting to catch {target_name}")
            catch_request = CatchTurtle.Request()
            catch_request.name = target_name
            future = self.catch_turtle_client_.call_async(catch_request)
            future.add_done_callback(self.callback_catch_turtle)
                    
        # Publish velocity command
        self.cmd_vel_publisher_.publish(msg)

    def change_pen_color(self):
        """
        Randomly changes the pen color of the master turtle.

        Uses the SetPen service to update RGB values and pen width.

        Returns:
            None
        """


        request = SetPen.Request()
        request.r = random.randint(0,255)
        request.g = random.randint(0,255)
        request.b = random.randint(0,255)
        request.width = self.master_pen_width
        
        self.pen_client_.call_async(request)
        
        
    def get_closest_turtle(self):
        """
        Finds and returns the closest alive turtle to the master turtle.

        Returns:
            Turtle: The nearest turtle object based on Euclidean distance.
        """

        xm, ym, _ = self.current_pose
        nearest_turtle = min(
            self.alive_turtles,
            key=lambda t: math.sqrt((t.x - xm)**2 + (t.y - ym)**2)
        )
        return nearest_turtle

    def callback_catch_turtle(self, future):
        """
        Callback for the catch service response.

        Handles the result of the CatchTurtle service call. If successful, optionally
        changes the master turtle's pen color. Resets flags if the service call fails.

        Args:
            future (Future): The future object returned by the asynchronous service call.

        Returns:
            None
        """

        try:
            response = future.result()
            if response.success:
                self.get_logger().debug(f"Catch service responded: {response.success}")
                if not self.master_pen_off:
                    self.change_pen_color()
        
        except Exception as e:
            self.get_logger().error(f"Catch service call failed: {e}")
            # If service call failed, reset catching flag
            self.is_catching = False
            self.current_target_name = None


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()