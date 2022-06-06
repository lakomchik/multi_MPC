
from traceback import print_tb
from turtle import speed
import scipy as sp
import rospy
import numpy as np
import tf
from math import cos, sin, fabs, sqrt, fmod, pi

from std_msgs.msg import String
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
hz = 10
cur_coords = np.zeros([3],dtype=float)
prev_loc_errs = np.zeros([3],dtype=float)
prev_glob_errs = np.zeros([3],dtype=float)
file = open('omni_pid_4.txt', 'w')
rob_speed = np.zeros([3],dtype=float)


def odom_callback(data):
    global cur_coords
    cur_coords[0] = data.x
    cur_coords[1] = data.y
    cur_coords[2] = data.theta
    return 0

def get_ref_coords(iter):
    dt = float(1)/hz
    cur_phase = pi*iter/200
    d_phase = pi/200
    ref_coords = np.zeros([3],dtype=float)
    ref_speed = np.zeros([3],dtype= float)
    x_cur = cos(cur_phase)
    y_cur = sin(cur_phase)
    theta_cur = 6 * cur_phase
    x_next = cos(cur_phase + d_phase)
    y_next = sin(cur_phase + d_phase)
    theta_next = 6 * (cur_phase + d_phase)
    v_x_cur = (x_next-x_cur)/dt
    v_y_cur = (y_next -y_cur)/dt
    omega_cur = (theta_next-theta_cur)/dt
    theta_cur = fmod(theta_cur, 2*pi)
    ref_coords[0] = x_cur
    ref_coords[1] = y_cur
    ref_coords[2] = theta_cur
    ref_speed[0] = v_x_cur
    ref_speed[1] = v_y_cur
    ref_speed[2] = omega_cur

    return ref_coords, ref_speed


def calculate_pid_control(robot_state, ref_state, ref_speed): 
    k_i = 1.   
    k_p = 0.4 
    dt = 1./hz
    speed_vec = np.zeros([3],dtype=float)
    theta = robot_state[2]
    rot_matr = np.array([[cos(theta), sin(theta),0],[-sin(theta),cos(theta),0],[0,0,1.]],dtype=float)
    glob_err = np.transpose(ref_state-robot_state)
    if(fabs(glob_err[2])>pi):
        glob_err[2] = fmod(glob_err[2], pi)
    loc_errs = np.dot(rot_matr,glob_err)
    loc_speed = np.dot(rot_matr,ref_speed)
    speed_vec = k_i * loc_errs + k_p * loc_speed
    print("SPEED")
    print(speed_vec)
    max_speed = np.asarray([0.5,0.5,3],dtype=float)
    for i in range(3):
        speed_vec[i] = fmod(speed_vec[i],max_speed[i])
    return speed_vec

def calculate_pid_control_v2(robot_state, ref_state, ref_speed): 
    global prev_loc_errs, prev_glob_errs, rob_speed
    k_i = 1.   
    k_p = 2. 
    k_d = 0.5
    dt = 1./hz
    speed_vec = np.zeros([3],dtype=float)
    theta = robot_state[2]
    rot_matr = np.array([[cos(theta), sin(theta),0],[-sin(theta),cos(theta),0],[0,0,1.]],dtype=float)
    glob_err = np.transpose(ref_state-robot_state)
    d_loc_err = np.dot(rot_matr,((glob_err-prev_glob_errs)/dt))
    d_loc_err[2] = fmod(d_loc_err[2],pi/dt)
    if(fabs(glob_err[2])>pi):
        glob_err[2] = fmod(glob_err[2], pi)
    loc_errs = np.dot(rot_matr,glob_err)
    loc_speed_err = np.dot(rot_matr,-ref_speed+rob_speed)
    speed_vec = k_p * loc_errs + k_i * (loc_errs + prev_loc_errs)*dt + k_d * d_loc_err
    #print("SPEED")
    #print(speed_vec)
    #print((loc_errs - prev_loc_errs)/dt)
    prev_loc_errs = np.copy(loc_errs)
    prev_glob_errs = np.copy(glob_err)
    max_speed = np.asarray([0.5,0.5,3],dtype=float)
    for i in range(3):
        speed_vec[i] = fmod(speed_vec[i],max_speed[i])
    rob_speed = np.dot(np.transpose(rot_matr),speed_vec)
    print(rob_speed)
    return speed_vec


def pid_controller():
    rospy.init_node('pid_controller', anonymous=True)
    control_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber("open_base/pose/world",Pose2D,odom_callback)
    rate = rospy.Rate(hz)
    iter = 0
    control = Twist()
    while not rospy.is_shutdown():
        ref_coords, ref_speed = get_ref_coords(iter)
        print("CUR_POS:")
        print(cur_coords)
        speed = calculate_pid_control_v2(cur_coords, ref_coords, ref_speed)
        
        control.linear.x = speed[0]
        control.linear.y = speed[1]
        control.angular.z = speed[2]
        control_pub.publish(control)
        iter+=1
        file.write(str(cur_coords[0])+"\t" + str(cur_coords[1])+"\t"+str(cur_coords[2])+"\n")
        rate.sleep()







if __name__ == '__main__':
    pid_controller()