#!/usr/bin/python3

import rospy
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped

from std_msgs.msg import Header

import tf
from tf.transformations import quaternion_from_euler

import math

class DifferentialSteering:
    
    def __init__(self):
        rospy.init_node("differential_steering_prj", anonymous=True)
        
        self.robot_name = "delta"
        left_wheel_encoder_topic = "/" + self.robot_name + "/left_wheel_encoder_node/tick"
        right_wheel_encorder_topic = "/" + self.robot_name + "/right_wheel_encoder_node/tick"
        
        wheel_command_topic = '/' + self.robot_name + '/wheels_driver_node/wheels_cmd'
        
        # queue_size ???
        self.wheel_command_publisher = rospy.Publisher(wheel_command_topic, WheelsCmdStamped, queue_size = 10)
        
        rospy.Subscriber(left_wheel_encoder_topic, WheelEncoderStamped, self.callback_left_wheel_tick)
        rospy.Subscriber(right_wheel_encorder_topic, WheelEncoderStamped, self.callback_right_wheel_tick)
        
        self.command = WheelsCmdStamped()
        self.command.header = Header()
        
        self.init_left_wheel_tick = 0
        self.init_right_wheel_tick = 0
        self.left_wheel_tick = 0
        self.right_wheel_tick = 0
        
        
    def callback_left_wheel_tick(self, data):
        
        if self.init_left_wheel_tick == 0:
            self.init_left_wheel_tick = data.data
        else:
            self.left_wheel_tick = data.data
    
    def callback_right_wheel_tick(self, data):
        if self.init_right_wheel_tick == 0:
            self.init_right_wheel_tick = data.data
        else:
            self.right_wheel_tick = data.data
    
        
    
    def run(self, velocity_left_wheel, velocity_right_wheel):
        while not rospy.is_shutdown():
            rospy.loginfo("Tick-Left-Message: " + str(self.left_wheel_tick - self.init_left_wheel_tick))
            
            theta = 100 * math.pi / 180
            
            ninty_degree_turn_ticks = theta * 90
            
            
            if self.right_wheel_tick == ninty_degree_turn_ticks:
                self.command.vel_left = 0
                self.command.vel_right = 0
                self.command.header.stamp = rospy.Time.now()
                self.wheel_command_publisher.publish(self.command)
                
            self.command.vel_left = velocity_left_wheel
            self.command.vel_right = velocity_right_wheel
            self.command.header.stamp = rospy.Time.now()
            self.wheel_command_publisher.publish(self.command)
            
            
            
       

    
if __name__ == '__main__':
    ds = DifferentialSteering()
    ds.run(0, 0)
