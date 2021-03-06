from curses import KEY_MAX
from turtle import speed

import scipy as sp
import rospy
import numpy as np
import tf
from math import cos, sin, fabs

from std_msgs.msg import String
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist




hz = 20
v_max = 0.22
w_max = 2.
cur_coords = np.zeros([3],dtype=float)


def yaw_calculation(orientation):
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([orientation.x,orientation.y, orientation.z, orientation.w])
    return yaw


def odom_callback(data):
    global cur_coords
    cur_coords[0] = data.pose.pose.position.x
    cur_coords[1] = data.pose.pose.position.y
    cur_coords[2] =yaw_calculation(data.pose.pose.orientation)


def calculate_pid_control(robot_state, ref_state, ref_speed):
    err_vec = ref_state - robot_state
    rot_matr = np.array([[cos(robot_state[2]), sin(robot_state[2]), 0],[-sin(robot_state[2]), cos(robot_state[2]), 0],[0, 0, 1]],dtype= float)
    err_vec = np.dot(rot_matr, (ref_state -robot_state))
    print("Err vec:")
    print(err_vec)
    
    speed_vec = np.zeros([2],dtype=float)
    Kx = 0.368
    Ky = 49
    Ktheta = 0.3685
    speed_vec[0] = ref_speed[0] * cos(err_vec[2]) + Kx * err_vec[0]
    #speed_vec[0] = - speed_vec[0]
    speed_vec[1] = ref_speed[1] + Ky * ref_speed[0] * err_vec[1] + Ktheta * ref_speed[0] * sin(err_vec[2])
    if(fabs(speed_vec[0]) > v_max):
        speed_vec[0] = speed_vec[0]/fabs(speed_vec[0]) * v_max
    if(fabs(speed_vec[1]) > w_max):
        speed_vec[1] = speed_vec[1]/fabs(speed_vec[1]) * w_max

    return speed_vec


def get_ref_coords(iter):
    v = 0.1
    ref_coords = np.zeros([3],dtype=float)
    ref_speed = np.zeros([2],dtype= float)
    ref_coords[0] = iter * 1/hz * v
    ref_coords[1] = 1.
    ref_coords[2] = 0.
    ref_speed[0] = v
    ref_speed[1] = 0
    print("REF VEC:")
    print(ref_coords)

    return ref_coords, ref_speed


def pid_controller():
    rospy.init_node('pid_controller', anonymous=True)
    control_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber("/odom",Odometry,odom_callback)
    rate = rospy.Rate(hz)
    iter = 0
    control = Twist()
    while not rospy.is_shutdown():
        ref_coords, ref_speed = get_ref_coords(iter)
        print("CUR_POS:")
        print(cur_coords)
        speed = calculate_pid_control(cur_coords, ref_coords, ref_speed)
        
        control.linear.x = speed[0]
        control.angular.z = speed[1]
        control_pub.publish(control)
        iter+=1

        rate.sleep()


if __name__ == '__main__':
    pid_controller()