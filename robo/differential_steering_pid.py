#!/usr/bin/python3

import rospy
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped

from std_msgs.msg import Header

import tf
from tf.transformations import quaternion_from_euler

from std_msgs.msg import Float32

import numpy as np
import math

class DifferentialSteering:
    
    def __init__(self):
        rospy.sleep(2.0)

        self.init_left_wheel_tick = None
        self.init_right_wheel_tick = None
        self.left_wheel_tick = None
        self.right_wheel_tick = None
        self.left_wheel_tick_old = None
        self.right_wheel_tick_old = None
        self.right_wheel_diff = None
        self.left_wheel_diff = None
        self.prev_fehler_rechts = None
        self.prev_fehler_links = None

        rospy.init_node("differential_steering_prj", anonymous=True)


        self.robot_name = "delta"
        left_wheel_encoder_topic = "/" + self.robot_name + "/left_wheel_encoder_node/tick"
        right_wheel_encorder_topic = "/" + self.robot_name + "/right_wheel_encoder_node/tick"

        wheel_command_topic = '/' + self.robot_name + '/wheels_driver_node/wheels_cmd'

        # queue_size ???
        self.wheel_command_publisher = rospy.Publisher(wheel_command_topic, WheelsCmdStamped, queue_size = 10)

        rospy.Subscriber(left_wheel_encoder_topic, WheelEncoderStamped, self.callback_left_wheel_tick)
        rospy.Subscriber(right_wheel_encorder_topic, WheelEncoderStamped, self.callback_right_wheel_tick)

    def rotate_left(self):
        self.command = WheelsCmdStamped()
        self.command.vel_right = 0.1
        self.command.vel_left = 0.0
        self.command.header = Header()
        self.command.header.stamp = rospy.Time.now()
        self.wheel_command_publisher.publish(self.command)

    def rotate_right(self):
        self.command = WheelsCmdStamped()
        self.command.vel_right = 0.0
        self.command.vel_left = 0.2
        self.command.header = Header()
        self.command.header.stamp = rospy.Time.now()
        self.wheel_command_publisher.publish(self.command)

    def move(self, vel_right=0, vel_left=0):
        self.command = WheelsCmdStamped()
        self.command.vel_right = vel_right
        self.command.vel_left = vel_left
        self.command.header = Header()
        self.command.header.stamp = rospy.Time.now()
        self.wheel_command_publisher.publish(self.command)
        
    def stop(self):
        self.command = WheelsCmdStamped()
        self.command.vel_right = 0.0
        self.command.vel_left = 0.0
        self.command.header = Header()
        self.command.header.stamp = rospy.Time.now()
        self.wheel_command_publisher.publish(self.command)


    def publish_transform(self, x, y, theta):
        dt_in_sekunden = 0.1
        fehler_x = x
        fehler_y = y
        current_x = 0
        current_y = 0
        
        while not rospy.is_shutdown():
            current_left_tick = self.left_wheel_tick
            current_right_tick = self.right_wheel_tick

            
            vel_rechts, vel_links = self.pid(fehler_x, fehler_y, dt_hz)
            self.move(vel_rechts, vel_links)
            
            rospy.sleep(dt_in_sekunden)
            
            diff_links_tick = self.current_left_tick - self.left_wheel_tick
            diff_rechts_tick = self.current_right_tick - self.right_wheel_tick
            
        

    def pid(self, fehler_x, fehler_y, dt_in_sekunden):
        # Geschwindigkeitsverstärker damit Wert im Rahmen von den Geschiwindigkeitsgrenzen von Duckie-Robot bleibt
        vel_verstaerker = 0.3
        # Verstärkungsfaktor P-Anteil
        kp = 0.1
        # Verstärkungsfaktor I-Anteil
        ki = 0.1
        # Verstärkungsfaktor D-Anteil
        kd = 0.1
        
        # Ticks, welche es braucht für einen entsprechenden Winkel
        grad_zu_ticks = 0.848
        winkel_ticks = math.atan(fehler_y/fehler_x) * 180 / math.pi * grad_zu_ticks
        distance_von_fehler = math.sqrt(fehler_x**2 + fehler_y**2)
        
        if winkel_ticks > 0:
            fehler_links = winkel_ticks + distance_von_fehler
            fehler_rechts = distance_von_fehler
        else:
            fehler_links = distance_von_fehler
            fehler_rechts = winkel_ticks + distance_von_fehler
        
        # PID Berechnung für rechts
        integral_anteil_rechts += fehler_rechts * dt_in_sekunden
        differential_anteil_rechts = (fehler_rechts - self.prev_fehler_rechts) / dt_in_sekunden if dt_in_sekunden > 0 else 0
        pid_output_rechts = kp * fehler_rechts + ki * integral_anteil_rechts + kd * differential_anteil_rechts
        self.prev_fehler_rechts = fehler_rechts
        
        # PID Berechnung für links
        integral_anteil_links += fehler_links * dt_in_sekunden
        differential_anteil_links = (fehler_links - self.prev_fehler_links) / dt_in_sekunden if dt_in_sekunden > 0 else 0
        pid_output_links = kp * fehler_links + ki * integral_anteil_links + kd * differential_anteil_links
        self.prev_fehler_links = fehler_links
  
        return (pid_output_rechts * vel_verstaerker, pid_output_links * vel_verstaerker) 
        
        
    
    def callback_left_wheel_tick(self, msg):
        if self.init_left_wheel_tick is None:
            self.init_left_wheel_tick = msg.data
            
        self.left_wheel_tick = msg.data
        
    
    def callback_right_wheel_tick(self, msg):
        if self.init_right_wheel_tick is None:
            self.init_right_wheel_tick = msg.data
        
        self.init_right_wheel_tick = msg.data
    
    def run(self):
        rospy.sleep(2)
    
        self.publish_transform(100, 100, 45)


    
if __name__ == '__main__':
    ds = DifferentialSteering()
    ds.run()
