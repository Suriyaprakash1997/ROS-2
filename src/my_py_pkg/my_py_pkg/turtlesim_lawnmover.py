#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Kill
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
import math
import time


class turtlesim_lawnmover(Node):
        def __init__(self):
            super().__init__("turtlesim_lawnmover")

            self.start_point_ = [2.0, 2.0, 0.0]
            self.height_ = 3.0
            self.width_ = 5.0

            # Killing the default turtle
            turtle_kill_ = self.create_client(Kill, "/kill")
            while not turtle_kill_.wait_for_service(1.0):
                self.get_logger().warn("Waiting for service...")
            request = Kill.Request()
            request.name = 'turtle1'
            future = turtle_kill_.call_async(request)
            future.add_done_callback(self.callback_call_Kill_turtle)

            # Spawning the new turtle at the specified position
            turtle_spawn_ = self.create_client(Spawn, "/spawn")
            while not turtle_spawn_.wait_for_service(1.0):
                self.get_logger().warn("Waiting for service...")
            request = Spawn.Request()
            request.x = self.start_point_[0]
            request.y = self.start_point_[1]
            request.theta = self.start_point_[2]
            request.name = 'turtle2'

            future = turtle_spawn_.call_async(request)
            future.add_done_callback(self.callback_call_Spawn_turtle)

            # Creating Velocity Publisher
            self.cmv_vel_pub_ = self.create_publisher(msg_type=Twist,
                                          topic='turtle2/cmd_vel',
                                          qos_profile=10)
            # Creating Pose Subscriber to get the update on the pose of the turtle
            self.pose_subscriber_ = self.create_subscription(Pose, 'turtle2/pose', self.Update_turtle_pose, 10)
            # Initialising message type for velocity commands
            self.vel_msg_ = Twist()

            # Initialising position tracking variables
            self.current_x = 0.0
            self.current_y = 0.0
            self.current_theta = 0.0

            self.msg_rate = self.create_rate(60)
            self.pose_updated = False

            self.Turtle_move()


        def callback_call_Spawn_turtle(self, future):
            try:
                response = future.result()
                self.get_logger().info("Turtle Spawned")
            except Exception as e:
                self.get_logger().error("Service call failed %r" % (e,))

        def callback_call_Kill_turtle(self, future):
            try:
                response = future.result()
                self.get_logger().info("Turtle Killed")
            except Exception as e:
                self.get_logger().error("Service call failed %r" % (e,))

        def Update_turtle_pose(self, current_pose):
            self.get_logger().info('Updating pose')
            self.current_x = current_pose.x
            self.current_y = current_pose.y
            self.current_theta = current_pose.theta
            self.pose_updated = True

        def is_position_updated(self):
            return self.is_position_updated


        def Turtle_forward(self, lin_vel):
            self.get_logger().info("Turtle going forward")
            if lin_vel > 1000:
                lin_vel = 1000
            self.vel_msg_.linear.x = lin_vel
            self.vel_msg_.angular.z = 0.0
            self.cmv_vel_pub_ .publish(self.vel_msg_)

        def Turtle_rotate(self, ang_vel):
            self.get_logger().info("Turtle is turning")
            self.vel_msg_.linear.x = 0.0
            self.vel_msg_.angular.z = ang_vel
            self.cmv_vel_pub_ .publish(self.vel_msg_)


        def Next_point_calculation(self, Next_x, Next_y):
            dx = Next_x - self.current_x
            dy = Next_y - self.current_y
            dtheta = math.atan2(dy, dx) - self.current_theta
            distance = math.sqrt((dx**2)+(dy**2))

            while (distance > 0.01):
                self.Turtle_forward(distance)
                dx = Next_x - self.current_x
                dy = Next_y - self.current_y
                distance = math.sqrt((dx**2)+(dy**2))
                self.msg_rate.sleep()
                self.Turtle_forward(0.0)

            while (dtheta > 0.01):
                self.Turtle_rotate(dtheta)
                dtheta = math.atan2(dy, dx) - self.current_theta
                self.msg_rate.sleep()
                self.Turtle_rotate(0.0)


        def Turtle_move(self):
            self.get_logger().info('Turtle travel started....')
            while not self.is_position_updated():
                self.get_logger().warn("Position not updated")
            self.get_logger().info("Position updated successfully")

            self.Next_point_calculation(self.current_x + self.width_, self.current_y)
            self.Next_point_calculation(self.current_x, self.current_y + self.height_)
            self.Next_point_calculation(self.current_x - self.width_, self.current_y)
            self.Next_point_calculation(self.current_x, self.current_y + self.height_)
            self.Next_point_calculation(self.current_x + self.width_, self.current_y)
            self.Next_point_calculation(self.start_point_[0], self.start_point_[1])

            self.get_logger().info("Turtle finished mowing the path...")


def main(args=None):
    rclpy.init(args=args)
    node = turtlesim_lawnmover()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
