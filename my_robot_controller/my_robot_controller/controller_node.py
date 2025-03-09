#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class VelocityControllerNode(Node):

  def __init__(self):
     super().__init__("controller_node")
     self.cmd_vel_publisher_= self.create_publisher(Twist,"/turtle1/cmd_vel",10)
     self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
     self.timer_ = self.create_timer(0.5, self.velocity_command)
     self.get_logger().info("Program has been started.")

  def velocity_command(self):
      cmd = Twist()      
      cmd.linear.x = 2.0
      cmd.angular.z = 1.0
      self.cmd_vel_publisher_.publish(cmd) 


  def pose_callback(self, pose: Pose):
        cmd = Twist()   

        if pose.x > 9.0 or pose.x < 2.0 or pose.y > 9.0 or pose.y < 2.0:
            cmd.linear.x = 15.0
            cmd.angular.z = 1.0
        else:
            cmd.linear.x = 20.0
            cmd.angular.z = 0.0
            self.cmd_vel_publisher_.publish(cmd) 
        
 
        if 0.5 <= pose.x <= 2.0 and 0.5 <= pose.y <= 2.0:
          cmd.linear.x = 0.0
          cmd.angular.z = 0.0
          self.cmd_vel_publisher_.publish(cmd) 


def main(args=None):
    rclpy.init(args=args)
    node = VelocityControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()