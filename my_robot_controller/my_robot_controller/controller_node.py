#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class VelocityControllerNode(Node):

  def __init__(self):       #nesne oluşturulması ve ilk özelliklein tanımlanması için oluşturuldu
    super().__init__("controller_node")
    self.cmd_vel_publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
    self.pose_subscriber_ = self.create_subscription(Odometry, "/odom", self.pose_callback, 10)
    self.timer_ = self.create_timer(0.5, self.velocity_command)

    self.destination_x = float(input("determine x spot"))  
    self.destination_y = float(input("determine y spot"))  
    

    self.current_x = 0.0
    self.current_y = 0.0
    self.current_yaw = 0.0

  def position_info(self):     # Hedef ile robot arasındaki mesafe ve yön hesaplaması için oluşturuldu
                                                
    dx = self.destination_x - self.current_x
    dy = self.destination_y - self.current_y       
    distance = math.sqrt(dx ** 2 + dy ** 2)
    angle = math.atan2(dy, dx)

    return distance, angle

  def velocity_command(self):                  #hızı publishlemek için oluşturuludu

    distance, angle = self.position_info()
    cmd = Twist()                                    
    angle_diff = angle - self.current_yaw  # Mevcut yön ile hedef yönü arasındaki fark

    if abs(angle_diff) > 0.1:  
        cmd.angular.z = 0.3 if angle_diff > 0 else -0.3  
    else:
        cmd.angular.z = 0.0 
    
    if distance > 0.3:  # Hedefe ile mesafe uzaksa hız bilgisi gönder
      cmd.linear.x = 0.5       
    else:   
      cmd.linear.x = 0.0               # Hedefe yakınsan dur
      cmd.angular.z = 0.0
      self.get_logger().info("Reached!")

    self.cmd_vel_publisher_.publish(cmd)      # hareket komutunu yayınla 

  def pose_callback(self, msg: Odometry):          # bilgileri güncellemek için oluşturuldu
    self.current_x = msg.pose.pose.position.x
    self.current_y = msg.pose.pose.position.y

        
    q = msg.pose.pose.orientation
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)           # Mevcut yönü hesapla (quaternion'dan yaw açısını bulma) internetten buldum
    self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

                                                 
    distance, _ = self.position_info()

    if distance < 0.3:  

      cmd = Twist()                  # Hedefe yaklaşıldığında dur
      cmd.linear.x = 0.0
      cmd.angular.z = 0.0
      self.get_logger().info("Reached!")  
      self.cmd_vel_publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()
