#!/usr/bin/python3

import rospy
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped


from std_msgs.msg import Header

import tf

from camera_labirynth import CameraLabirynth
from dijkstra import Dijkstra, Instruction
from labirynth_generator import generateLabirynth

class Labirynth:
    
    def __init__(self):
        rospy.sleep(2.0)

        self.left_wheel_tick = 0
        self.right_wheel_tick = 0

        self.transform_broadcaster = tf.TransformBroadcaster()

        self.grad_zu_ticks = 0.848
        self.strecke_zu_ticks = 6.76
        self.rotate_degree = 90 * self.grad_zu_ticks
        
        rospy.init_node("labirynth", anonymous=True)

        self.robot_name = "delta"
        left_wheel_encoder_topic = "/" + self.robot_name + "/left_wheel_encoder_node/tick"
        right_wheel_encorder_topic = "/" + self.robot_name + "/right_wheel_encoder_node/tick"

        wheel_command_topic = '/' + self.robot_name + '/wheels_driver_node/wheels_cmd'

        self.camera = CameraLabirynth(self.robot_name)
        graph, start, destination = generateLabirynth()
        self.dijstra = Dijkstra(graph, start, destination)

        self.wheel_command_publisher = rospy.Publisher(wheel_command_topic, WheelsCmdStamped, queue_size = 10)

        rospy.Subscriber(left_wheel_encoder_topic, WheelEncoderStamped, self.callback_left_wheel_tick)
        rospy.Subscriber(right_wheel_encorder_topic, WheelEncoderStamped, self.callback_right_wheel_tick)

    def rotate_left(self,vel_left, vel_right):
        self.command = WheelsCmdStamped()
        self.command.vel_right = vel_right * 0.003775 * 1.6
        self.command.vel_left = vel_left * 0.005 * 1.6
        self.command.header = Header()
        self.command.header.stamp = rospy.Time.now()
        self.wheel_command_publisher.publish(self.command)

    def rotate_right(self, vel_left, vel_right):
        self.command = WheelsCmdStamped()
        self.command.vel_right = vel_right * 0.003775 * 1.6
        self.command.vel_left = vel_left * 0.005 * 1.6
        self.command.header = Header()
        self.command.header.stamp = rospy.Time.now()
        self.wheel_command_publisher.publish(self.command)

    def move(self, ausrichtung):
        self.command = WheelsCmdStamped()
        self.command.vel_right = 0.1 + ausrichtung.value
        self.command.vel_left = 0.1 + 0.02
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

    def move_forward(self):
        distance = 30 * self.strecke_zu_ticks
        current_left_tick = self.left_wheel_tick
        while self.left_wheel_tick <= current_left_tick + distance:
            ausrichtung = self.camera.get_aktuelle_ausrichtung()
            self.move(ausrichtung)
        self.stop()
        rospy.sleep(2)

    def turn_right_and_move(self):
        current_left_tick = self.left_wheel_tick
        current_right_tick = self.right_wheel_tick
        while self.left_wheel_tick <= current_left_tick + self.rotate_degree:
            vel_right = self.left_wheel_tick - current_left_tick - self.rotate_degree/2
            vel_left = -(self.right_wheel_tick - current_right_tick - self.rotate_degree/2)
            self.rotate_right(vel_left, vel_right)
        self.stop()
        rospy.sleep(2)
        self.move_forward()

    def turn_left_and_move(self):
        current_left_tick = self.left_wheel_tick
        current_right_tick = self.right_wheel_tick
        while self.left_wheel_tick >= current_left_tick - self.rotate_degree/2:
            vel_right = -(self.right_wheel_tick - current_right_tick - self.rotate_degree/2)
            vel_left = self.left_wheel_tick - current_left_tick - self.rotate_degree/2
            self.rotate_left(vel_left, vel_right)
        self.stop()
        rospy.sleep(2)
        self.move_forward()


    def publish_transform(self):
        instructions = self.dijstra.generate_instructions_for_shortest_path()
        complete = False
        while complete is not True:
            for index, instruction in enumerate(instructions):
                if instruction == Instruction.RIGHT:
                    self.turn_right_and_move()
                elif instruction == Instruction.LEFT:
                    self.turn_left_and_move()
                else:
                    self.move_forward()
                if index == len(instructions)-1:
                    complete = True
                
    def callback_left_wheel_tick(self, msg):
        self.left_wheel_tick = msg.data
    
    def callback_right_wheel_tick(self, msg):
        self.right_wheel_tick = msg.data
    
    def run(self):
        rospy.sleep(2)
        self.publish_transform()

    
if __name__ == '__main__':
    la = Labirynth()
    la.run()