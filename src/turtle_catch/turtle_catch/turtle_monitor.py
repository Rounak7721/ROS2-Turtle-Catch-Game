#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import String
from turtle_interfaces.msg import Turtle, TurtleArray
from turtle_interfaces.srv import CatchTurtle

class TurtleMonitorNode(Node):
    def __init__(self):
        super().__init__("turtle_monitor")
        self.get_logger().info("Turtle Monitor Active...")

        self.alive_turtles = {}
        self.turtle_pose_subscribers = {}

        # sub: spawned_turtle
        self.get_turtles_subscriber = self.create_subscription(String, "spawned_turtle", self.callback_get_new_turtles, 10)

        # sub: caught_turtle
        self.caught_turtle_subscriber_ = self.create_subscription(String, "caught_turtle", self.callback_turtle_caught, 10 )

        # pub: alive_turtles
        self.alive_turtles_publisher = self.create_publisher(TurtleArray, "alive_turtles", 10)

        # Timer to publih alive turtles
        self.timer = self.create_timer(0.1, self.publish_alive_turtles_pose)

    def callback_get_new_turtles(self, msg: String):
        """
        Handles detection of newly spawned turtles.

        Subscribes to the new turtle's Pose topic and initializes its entry in the alive turtles list.

        Args:
            msg (String): Message containing the name of the newly spawned turtle.

        Returns:
            None
        """

        name = msg.data
        if name in self.turtle_pose_subscribers:
            return  # already subscribed

        self.get_logger().debug(f"New Turtle detected: {name}")

        # Dynamically create a sub to  new turtle Pose
        self.turtle_pose_subscribers[name] = self.create_subscription(Pose, f"/{name}/pose",
                                                                      lambda pose, n=name: self.pose_callback(pose, n), 10)

        self.alive_turtles[name] = Pose()

    def callback_turtle_caught(self, msg: String):
        """
        Handles the event of a turtle being caught.

        Removes the turtle from the alive list, logs the removal, and destroys its Pose subscription.

        Args:
            msg (String): Message containing the name of the caught turtle.

        Returns:
            None
        """

        name = msg.data
        if name in self.alive_turtles:
            del self.alive_turtles[name]
            self.get_logger().debug(f"Turtle {name} removed from alive list")
        
        # Destroy sub to save resources
        if name in self.turtle_pose_subscribers:
            self.destroy_subscription(self.turtle_pose_subscribers[name])
            del self.turtle_pose_subscribers[name]

    def pose_callback(self, pose_msg: Pose, name: str):
        """
        Updates the stored pose of a specific turtle.

        Args:
            pose_msg (Pose): Latest pose of the turtle.
            name (str): Name of the turtle.

        Returns:
            None
        """

        # Update turtle pose
        self.alive_turtles[name] = pose_msg

    def publish_alive_turtles_pose(self):
        """
        Publishes the current list of alive turtles as a TurtleArray message.

        Iterates over all tracked turtles and publishes their names and positions.

        Returns:
            None
        """

        msg = TurtleArray()
        for name, pose in self.alive_turtles.items():
            turtle = Turtle()
            turtle.name = name
            turtle.x = pose.x
            turtle.y = pose.y
            turtle.t = pose.theta
            msg.turtles.append(turtle)

        self.alive_turtles_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleMonitorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()