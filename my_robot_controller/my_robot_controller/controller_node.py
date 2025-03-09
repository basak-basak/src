#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class VelocityControllerNode(Node):

  def __init__(self):
     super().__init__("controller_node")
     self.cmd_vel_publisher_= self.create_publisher(Twist,"/cmd_vel",10)
     self.pose_subscriber_ = self.create_subscription(Odometry, "/odom", self.pose_callback, 10)
     self.timer_ = self.create_timer(0.5, self.velocity_command)
     self.get_logger().info("Program has been started.")

  def velocity_command(self):
      cmd = Twist()      
      cmd.linear.x = 2.0
      cmd.angular.z = 0.0
      self.cmd_vel_publisher_.publish(cmd) 


  def pose_callback(self, msg: Odometry):
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        cmd = Twist()   

        if x > 9.0 or x < 2.0 or y > 9.0 or y < 2.0:
            cmd.linear.x = 1.0
            cmd.angular.z = 0.5
        else:
            cmd.linear.x = 3.0
            cmd.angular.z = 0.5
            self.cmd_vel_publisher_.publish(cmd) 
        
 
        if 0.0 <= x <= 2.0 and 0.0 <= y <= 2.0:
          cmd.linear.x = 0.0
          cmd.angular.z = 0.0
          self.cmd_vel_publisher_.publish(cmd) 


def main(args=None):
    rclpy.init(args=args)
    node = VelocityControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()
