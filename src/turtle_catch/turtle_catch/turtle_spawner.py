#!/usr/bin/env python3

import rclpy
import random
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill, SetPen
from turtle_interfaces.srv import CatchTurtle
from geometry_msgs.msg import Twist
from functools import partial
from std_msgs.msg import String

class TurtleSpawnerNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        self.get_logger().info("Turtle Spawner Active...")

        # Parameter Declarations
        self.declare_parameter("spawn_frequency", 2.0)
        self.declare_parameter("turtle_name_prefix", "turtle")
        self.declare_parameter("turtle_pen_off", 1)
        
        # Parameter Value Assignemnt
        self.spawn_frequency = self.get_parameter("spawn_frequency").value
        self.turtle_name_prefix = self.get_parameter("turtle_name_prefix").value
        self.turtle_pen_off = bool(self.get_parameter("turtle_pen_off").value)
        
        self.turtle_publishers = {}
        self.turtle_suffix_num = 2
        
        # server: catch_turtle
        self.catch_server = self.create_service(
            CatchTurtle, "catch_turtle", self.catch_turtle
        )

        # pub: spawned_turtle
        self.spawned_turtle_publisher= self.create_publisher(String, "spawned_turtle", 10)
        
        # pub: caught_turtle
        self.caught_turtle_publisher_= self.create_publisher(String, "caught_turtle", 10)
        
        # client: /spawn
        self.spawner_client = self.create_client(Spawn, "/spawn")
        while not self.spawner_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Turtlesim...")
            
        # client: /kill
        self.kill_turtle_client = self.create_client(Kill, "/kill")
        while not self.kill_turtle_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Turtlesim...")

        # spawn turtle timer
        self.timer_ = self.create_timer(self.spawn_frequency, self.spawn_turtle)
        
        # move turtle timer
        self.timer_random_move = self.create_timer(0.5, self.move_randomly)

    def spawn_turtle(self):
        """
        Spawns a new turtle at a random position and orientation.

        Creates a publisher for the new turtle's velocity commands and optionally
        turns off its pen if configured.

        Returns:
            None
        """
        x = random.uniform(0.0, 11.0)
        y = random.uniform(0.0, 11.0)
        t = random.uniform(-3.14, 3.14)

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = t
        request.name = f"{self.turtle_name_prefix}{self.turtle_suffix_num}"

        self.turtle_suffix_num += 1

        future = self.spawner_client.call_async(request=request)

        future.add_done_callback(partial(self.callback_turtle_spawned, request=request))

        pub = self.create_publisher(Twist, f"/{request.name}/cmd_vel",10)
        self.turtle_publishers[request.name] = pub
        
        # disable pen logic
        if self.turtle_pen_off:
            self.pen_client_ = self.create_client(SetPen, f"{request.name}/set_pen")
            while not self.pen_client_.wait_for_service(1.0):
                self.get_logger().warn("Waiting for /set_pen service...")
                
            pen_req = SetPen.Request()
            pen_req.off = 1
            self.pen_client_.call_async(pen_req)
            self.get_logger().debug("Pen turned OFF for turtle1")
    
    def move_randomly(self):
        """
        Moves all spawned turtles randomly.

        Publishes random linear and angular velocities to each turtle's `/cmd_vel` topic.

        Returns:
            None
        """

        for name, pub in self.turtle_publishers.items():
            msg = Twist()
            msg.linear.x = random.uniform(0.5,2.0)
            msg.angular.z = random.uniform(-3.14,3.14)
            pub.publish(msg)
            
    def callback_turtle_spawned(self, future, request):
        """
        Handles the response from the spawn service.

        Publishes the name of the newly spawned turtle and logs its creation.

        Args:
            future (Future): Future object from the asynchronous spawn service call.
            request (Spawn.Request): The original spawn request containing turtle details.

        Returns:
            None
        """

        response = future.result()
        name = request.name
        msg = String()
        msg.data = name
        
        self.spawned_turtle_publisher.publish(msg) 
        self.get_logger().debug(f"New Turtle Spawned: {name}")


    def catch_turtle(self, request: CatchTurtle.Request, response: CatchTurtle.Response):
        """
        Service callback to catch (kill) a specific turtle.

        Removes the turtle's publisher, calls the Kill service, and returns success.

        Args:
            request (CatchTurtle.Request): Service request containing the turtle name to catch.
            response (CatchTurtle.Response): Service response to set success flag.

        Returns:
            CatchTurtle.Response: Response indicating if the catch was successful.
        """

        turtle_name = request.name
        kill_request = Kill.Request()
        kill_request.name = turtle_name
        
        if turtle_name in self.turtle_publishers:
            self.destroy_publisher(self.turtle_publishers[turtle_name])
            del self.turtle_publishers[turtle_name]
        
        self.get_logger().debug(f"Catching turtle: {turtle_name}")
        future = self.kill_turtle_client.call_async(kill_request)
        future.add_done_callback(partial(self.callback_kill_turtle, turtle_name=turtle_name))

        response.success = True
        return response

    def callback_kill_turtle(self, future, turtle_name):
        """
        Handles the result of the Kill service call for a turtle.

        Logs success or failure and publishes the name of the turtle that was caught.

        Args:
            future (Future): Future object from the asynchronous kill service call.
            turtle_name (str): Name of the turtle being caught.

        Returns:
            None
        """

        try:
            result = future.result()
            self.get_logger().debug(f"Turtle '{turtle_name}' has been caught.")
            
            # Publish that this turtle was caught
            msg = String()
            msg.data = turtle_name
            self.caught_turtle_publisher_.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Failed to catch turtle '{turtle_name}': {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()