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
        self.left_wheel_tick = 0
        self.right_wheel_tick = 0
        self.left_wheel_tick_old = 0
        self.right_wheel_tick_old = 0
        self.right_wheel_diff = 0
        self.left_wheel_diff = 0
        self.prev_fehler_rechts = 0
        self.prev_fehler_links = 0
        self.integral_anteil_links = 0
        self.integral_anteil_rechts = 0
        self.imu = None

        self.transform_broadcaster = tf.TransformBroadcaster()


        self.grad_zu_ticks = 0.848
        self.strecke_zu_ticks = 6.76
        
        rospy.init_node("labirynth", anonymous=True)


        self.robot_name = "gamma"
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

    def move(self, vel_right=0.2, vel_left=0.2):
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
        graph = [('s'),('l'),('r'),('s'),('l'),('l')]
        distance = 30 * self.strecke_zu_ticks
        rotate_degree = 90 * self.grad_zu_ticks
        while not rospy.is_shutdown():
            current_right_tick = self.right_wheel_tick
            current_left_tick = self.left_wheel_tick
            for g in graph:
                if g == 'r':
                    while self.left_wheel_tick <= current_left_tick + rotate_degree:
                        self.rotate_right()
                    self.stop()
                    rospy.sleep(1)
                    current_left_tick = self.left_wheel_tick
                    while self.left_wheel_tick <= current_left_tick + distance:
                        self.move()
                    self.stop()
                    rospy.sleep(1)
                elif g == 'l':
                    while self.right_wheel_tick <= current_right_tick + rotate_degree:
                        self.rotate_left()
                    self.stop()
                    rospy.sleep(1)
                    current_left_tick = self.left_wheel_tick
                    while self.left_wheel_tick <= current_left_tick + distance:
                        self.move()
                    self.stop()
                    rospy.sleep(1)
                else:
                    while self.left_wheel_tick <= current_left_tick + distance:
                        self.move()
                    self.stop()
                    rospy.sleep(1)
        
        
    
    def callback_left_wheel_tick(self, msg):
        if self.init_left_wheel_tick is None:
            self.init_left_wheel_tick = msg.data
            
        self.left_wheel_tick = msg.data
        
    
    def callback_right_wheel_tick(self, msg):
        if self.init_right_wheel_tick is None:
            self.init_right_wheel_tick = msg.data
        
        self.right_wheel_tick = msg.data
    
    def run(self):
        rospy.sleep(2)
    
        self.publish_transform(100, 0, 0)


    
if __name__ == '__main__':
    ds = DifferentialSteering()
    ds.run()



