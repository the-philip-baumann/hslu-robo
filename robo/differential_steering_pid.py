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

        self.transform_broadcaster = tf.TransformBroadcaster()


        self.grad_zu_ticks = 0.848
        self.strecke_zu_ticks = 6.76
        
        rospy.init_node("differential_steering_prj", anonymous=True)


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

    def move(self, vel_right, vel_left):
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
        x_global = 0.0 
        y_global = 0.0
        theta_global = 0.0
        
        while not rospy.is_shutdown():
            current_left_tick = self.left_wheel_tick
            current_right_tick = self.right_wheel_tick
            
            vel_rechts, vel_links = self.pid(fehler_x, fehler_y, dt_in_sekunden)

            self.move(vel_rechts, vel_links)
            
            rospy.sleep(dt_in_sekunden)
            
            diff_links_tick = self.left_wheel_tick - current_left_tick
            diff_rechts_tick = self.right_wheel_tick - current_right_tick
            
            if diff_links_tick > diff_rechts_tick:
                ausrichtung = (diff_links_tick - diff_rechts_tick) / self.grad_zu_ticks 
                strecke = diff_rechts_tick / self.strecke_zu_ticks
            else:
                ausrichtung = -(diff_rechts_tick - diff_links_tick) / self.grad_zu_ticks
                strecke = diff_links_tick / self.strecke_zu_ticks
                
            x_neu = math.cos(ausrichtung) * strecke
            y_neu = math.sin(ausrichtung) * strecke
            theta_neu = ausrichtung
            print(f'{theta_neu=}')

            x_global, y_global, theta_global = self.global_position(x_neu, y_neu, theta_neu, x_global, y_global, theta_global)
            
            fehler_x, fehler_y = self.homogeneous_transform(fehler_x, fehler_y, theta_neu, x_neu, y_neu)
            print(f'{fehler_x=}, {fehler_y=}')
            q = quaternion_from_euler(0, 0 , theta_global)
            self.transform_broadcaster.sendTransform((x_global, y_global, 0.0), q, rospy.Time.now(),  self.robot_name + "/base", "map")
            
            
    def global_position(self, x, y, theta, x_global, y_global, theta_global):
        theta_global += theta
        x_global = (x_global + math.cos(theta_global) * x - math.sin(theta_global) * y)/10
        y_global = (y_global + math.sin(theta_global) * x + math.cos(theta_global) * y)/10
        return x_global, y_global, theta_global

            
    def homogeneous_transform(self, x, y, theta, tx, ty):
        
        # Drehwinkel in Radiant umwandeln
        theta = np.radians(theta)
        
        # Transformationsmatrix (Rotation + Translation)
        T = np.array([
            [np.cos(theta)  ,  np.sin(theta), -tx],        
            [-np.sin(theta)  ,  np.cos(theta), -ty],
            [0              , 0             , 1]])
        
        # Punkt in homogenen Koordinaten
        P = np.array([x, y, 1])
        
        # Transformation anwenden
        P_new = T @ P
        
        return P_new[:2]  # Rückgabe nur der (x', y')-Koordinaten
        

    def pid(self, fehler_x, fehler_y, dt_in_sekunden):
        # Geschwindigkeitsverstärker damit Wert im Rahmen von den Geschiwindigkeitsgrenzen von Duckie-Robot bleibt
        vel_verstaerker = 0.004
        # Verstärkungsfaktor P-Anteil
        kp = 0.1
        # Verstärkungsfaktor I-Anteil
        ki = 0.01
        # Verstärkungsfaktor D-Anteil
        kd = 0.03
        #Winkel Gewichtung
        kw = 5
        
        # Ticks, welche es braucht für einen entsprechenden Winkel
        
        winkel_rad = math.atan(fehler_y/ fehler_x)
        winkel_ticks = winkel_rad * 180 / math.pi * self.grad_zu_ticks
        print(f'{winkel_ticks=}')


        distance_von_fehler = math.sqrt(fehler_x**2 + fehler_y**2) * self.strecke_zu_ticks
        
        if winkel_ticks > 0:
            fehler_links = distance_von_fehler
            fehler_rechts = distance_von_fehler + kw * winkel_ticks
        else:
            fehler_links = distance_von_fehler + kw * winkel_ticks
            fehler_rechts = distance_von_fehler
        
        # PID Berechnung für rechts
        self.integral_anteil_rechts += fehler_rechts * dt_in_sekunden
        if self.integral_anteil_rechts > 10:
            self.integral_anteil_rechts = 11

        differential_anteil_rechts = (fehler_rechts - self.prev_fehler_rechts) / dt_in_sekunden
        pid_output_rechts = kp * fehler_rechts + ki * self.integral_anteil_rechts + kd * differential_anteil_rechts
        self.prev_fehler_rechts = fehler_rechts
        
        # PID Berechnung für links
        self.integral_anteil_links += fehler_links * dt_in_sekunden
        if self.integral_anteil_links > 10:
            self.integral_anteil_links = 11
    
        differential_anteil_links = (fehler_links - self.prev_fehler_links) / dt_in_sekunden
        pid_output_links = kp * fehler_links + ki * self.integral_anteil_links + kd * differential_anteil_links
        self.prev_fehler_links = fehler_links
  
        return (pid_output_rechts * vel_verstaerker, pid_output_links * vel_verstaerker) 
        
        
    
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