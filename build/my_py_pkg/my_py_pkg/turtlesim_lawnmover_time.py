#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Kill
from turtlesim.srv import Spawn
from datetime import datetime
import math
PI = 3.1415926535897

class turtlesim_lawnmover_time(Node):
    def __init__(self):
        super().__init__("turtlesim_lawnmover_time")
        self.cmv_vel_pub_ = self.create_publisher (msg_type = Twist, 
                                                   topic = "/turtle2/cmd_vel",
                                                   qos_profile = 10)
        self.get_logger().info ("Lawnmover turtle started!")
        self.move()
        self.get_logger().info ("Turtle job completed")

    def move (self):
        self.get_logger().info("Inside the move")
        msg = Twist()
        home_position = Pose()
        Pose.x = 3
        Pose.y = 2
        
        client_kill_ = self.create_client(Kill , "/kill")
        while not client_kill_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service...")
        
        request = Kill.Request()
        request.name = 'turtle1'
        

        future = client_kill_.call_async(request)
        future.add_done_callback(self.callback_call_Kill_turtle)

        #New turtle spawn

        turtle_spawn_ = self.create_client(Spawn , "/spawn")
        while not turtle_spawn_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service...")
        
        request = Spawn.Request()
        request.x = 2.0
        request.y = 2.0
        request.theta = 0.0
        request.name = 'turtle2'
        

        future = turtle_spawn_.call_async(request)
        future.add_done_callback(self.callback_call_Spawn_turtle)

        forward_speed = 1.0
        turning_speed = 30
        self.rate = self.create_rate(30)

        self.forward(msg,7,forward_speed)
        self.left_turn(msg,90, turning_speed)
        self.forward(msg,3,forward_speed)
        self.left_turn(msg,90, turning_speed)
        self.forward(msg,7,forward_speed)
        self.right_turn(msg,90, turning_speed)
        self.forward(msg,3,forward_speed)
        self.right_turn(msg,90, turning_speed)
        self.forward(msg,7,forward_speed)
        self.right_turn(msg,123, turning_speed)
        self.forward(msg,5,forward_speed)
    
    def callback_call_Spawn_turtle(self, future):
        try:
            response = future.result()
            self.get_logger().info("Turtle Spawned")
        except Exception as e:
            self.get_logger().error("Service call failed %r" %(e,))
    
    def callback_call_Kill_turtle(self, future):
        try:
            response = future.result()
            self.get_logger().info("Turtle Killed")
        except Exception as e:
            self.get_logger().error("Service call failed %r" %(e,))
        


    def forward (self,msg, distance,speed):
        #rate = self.create_rate(10)
        self.get_logger().info("Inside the Forward")
        t0 = datetime.now().second
        msg.linear.x = speed
        current_distance = 0
        #Loop to move the turtle in an specified distance
        while(current_distance < distance):
            self.cmv_vel_pub_ .publish(msg)
            #rate.sleep()
            t1=datetime.now().second
            current_distance= speed*(t1-t0)
            
    
        msg.linear.x = 0.0
        self.cmv_vel_pub_ .publish(msg)
    
    def left_turn (self,msg, angle, speed):
        global PI
        #rate = self.create_rate(10)
        self.get_logger().info("Inside the Left Turn")
        angle = angle*2*PI/360.0
        speed = speed*2*PI/360.0
        msg.angular.z = speed
        t0 = datetime.now().second
        current_angle = 0
  
        while(current_angle < angle):
            self.cmv_vel_pub_ .publish(msg)
            t1 = datetime.now().second
            current_angle = speed*(t1-t0)
            #rate.sleep()
   
        #Forcing our robot to stop
        msg.angular.z = 0.0
        self.cmv_vel_pub_ .publish(msg)
    
    def right_turn (self,msg, angle, speed):
        global PI
        #rate = self.create_rate(100)
        self.get_logger().info("Inside the Right Turn")
        angle = angle*2*PI/360
        speed = speed*2*PI/360
        msg.angular.z = -abs(speed)
        t0 = datetime.now().second
        current_angle = 0
  
        while(current_angle < angle):
            self.cmv_vel_pub_ .publish(msg)
            t1 = datetime.now().second
            current_angle = speed*(t1-t0)
            #rate.sleep()
        #Forcing our robot to stop
        msg.angular.z = 0.0
        self.cmv_vel_pub_ .publish(msg)
        


def main(args=None):
    rclpy.init(args=args)
    node = turtlesim_lawnmover_time()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()