import scipy as sp
import rospy
import numpy as np
import tf
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates
from math import pow, fabs
import scipy.linalg


cur_coords = np.zeros([4,1],dtype=float)
file = open('lqr_exp_1.txt', 'w')
control = Float64()
def lqr_coeffs():
    A = np.zeros([4,4],dtype=float)
    B = np.zeros([4,1],dtype= float)
    #System parameters
    mb = 10.
    Jb = 1.40833
    r = 0.2
    l = 0.4
    mw = 1.
    Jw = 0.02
    g = 9.81
    #Constants for linearized A and B
    den1 = mb + 2*(1/pow(r,2)*Jw + mw) - pow(mb * l,2)/(Jb + mb* pow(l,2)) 
    C1 = (pow((mb * l),2) * g / (Jb + mb * pow(l,2)))/den1
    C2 = (2/r + 2*mb*l/(Jb + mb * pow(l,2)))/den1
    den2 = Jb + mb*pow(l,2) - pow(mb*l,2)/(mb + 2* (1/pow(r,2) * Jw + mw))
    C3 = mb * g * l / den2
    C4 = (2 * mb * l/(mb*r + 2 * (Jw/r + mw * r))+2)/den2
    A[0,1] = 1
    A[1,2] = - C1
    A[2,3] = 1
    A[3,2] = C3
    B[1,0] = C2
    B[3,0] = -C4
    print(A)
    print(B)
    R = np.diag([0.01])
    Q = np.diag([0., 0.5, 5.5, .4])
    file.write(str(Q[0,0])+"\t" + str(Q[1,1])+"\t"+str(Q[2,2])+str(Q[3,3])+"\t"+str(R[0,0])+"\n")
    P = scipy.linalg.solve_continuous_are(A,B,Q,R)
    print(P)
    R_inv = scipy.linalg.inv(R)
    K = np.dot(R_inv, np.dot(B.transpose(), P))
    print(K)
    print(scipy.linalg.eigvals(A - np.dot(B,K)))
    return K



def pitch_calculation(orientation):
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([orientation.x,orientation.y, orientation.z, orientation.w])
    return pitch

def odom_callback(data):
    cur_coords[0,0] = -data.pose[1].position.x
    cur_coords[1,0] = -data.twist[1].linear.x
    cur_coords[2,0] = -pitch_calculation(data.pose[1].orientation)
    cur_coords[3,0] = -data.twist[1].angular.y
   
    #print(cur_coords.transpose())
    pass

def lqr_controller():
    rospy.init_node('lqr_ip', anonymous=True)

    T1_pub  = rospy.Publisher('/teeterbot/right_torque_cmd', Float64, queue_size= 1)
    T2_pub  = rospy.Publisher('/teeterbot/left_torque_cmd', Float64, queue_size= 1)
    rospy.Subscriber("/gazebo/model_states",ModelStates,odom_callback)
    rate = rospy.Rate(40)
    
    ref_state = np.zeros([4,1],dtype= float)
    K = lqr_coeffs()
    T_max = 5.2
    while not rospy.is_shutdown():
        file.write(str(cur_coords[0,0])+"\t" + str(cur_coords[1,0])+"\t"+str(cur_coords[2,0])+"\t"+str(cur_coords[3,0])+"\t"+str(control.data)+"\n")
        ref_state = np.zeros([4,1],dtype= float)
        torgue = np.dot(K,(ref_state-cur_coords))
        torgue = torgue
        control.data = -torgue[0,0]
        if(fabs(control.data) > T_max):
            control.data = T_max * (control.data)/fabs(control.data)
        print(control.data)
        T1_pub.publish(control)
        T2_pub.publish(control)
        rate.sleep()

    return 0







if __name__ == '__main__':
    lqr_controller()