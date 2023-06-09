#!/usr/bin/python3
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped

import math

flag_move = 0

d = 0.175
l = 0.26*2
steer_threshold = 0.001

def set_throttle_steer(data):

    global flag_move

    pub_vel_left_rear_wheel = rospy.Publisher('/ackermann_chassis/left_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_rear_wheel = rospy.Publisher('/ackermann_chassis/right_rear_wheel_velocity_controller/command', Float64, queue_size=1)

    pub_pos_left_steering_hinge = rospy.Publisher('/ackermann_chassis/left_steering_hinge_position_controller/command', Float64, queue_size=1)
    pub_pos_right_steering_hinge = rospy.Publisher('/ackermann_chassis/right_steering_hinge_position_controller/command', Float64, queue_size=1)

    throttle = data.drive.speed
    steer = data.drive.steering_angle # angular velocity in fact
    
    if abs(steer) < steer_threshold:
        pub_vel_left_rear_wheel.publish(throttle * 31.75)
        pub_vel_right_rear_wheel.publish(throttle * 31.75)
        pub_pos_left_steering_hinge.publish(steer)
        pub_pos_right_steering_hinge.publish(steer)

        rospy.loginfo("throttle(target velocity): %f", throttle* 31.75)
        rospy.loginfo("steer(target angular velocity): %f", steer)
    elif ( abs(throttle) > abs(steer * d)):
        throttle_left = (throttle - steer * d) * 31.75
        throttle_right = (throttle + steer * d) * 31.75
        R = throttle/steer
        if (R < 0):
            steer_left = math.atan2(l, R + d) - math.pi
            steer_right = math.atan2(l, R - d) - math.pi
        else:
            steer_left  = math.atan2(l, R - d)
            steer_right = math.atan2(l, R + d)
        pub_vel_left_rear_wheel.publish(throttle_left)
        pub_vel_right_rear_wheel.publish(throttle_right)
        pub_pos_left_steering_hinge.publish(steer_left)
        pub_pos_right_steering_hinge.publish(steer_right)

        rospy.loginfo("throttle_left: %f", throttle_left)
        rospy.loginfo("throttle_right: %f", throttle_right)
        rospy.loginfo("steer_left: %f", steer_left)
        rospy.loginfo("steer_right: %f", steer_right)
    else:
        pub_vel_left_rear_wheel.publish(throttle * 31.75)
        pub_vel_right_rear_wheel.publish(throttle * 31.75)
        pub_pos_left_steering_hinge.publish(steer)
        pub_pos_right_steering_hinge.publish(steer)

        rospy.loginfo("throttle(target velocity): %f", throttle* 31.75)
        rospy.loginfo("steer(target angular velocity): %f", steer)

    

def servo_commands():

    rospy.init_node('servo_commands', anonymous=True)

    rospy.Subscriber("/ackermann_cmd_mux/output", AckermannDriveStamped, set_throttle_steer)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        servo_commands()
    except rospy.ROSInterruptException:
        pass