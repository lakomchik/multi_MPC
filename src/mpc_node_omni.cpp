#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "mpc_omni.cpp"
#include <sstream>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"
#include "tf/tf.h"
#include <fstream>
#include "nav_msgs/Path.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include <tf/transform_datatypes.h>
#include <thread>
#include <math.h>

MPC::MPC mpc;
Eigen::Matrix <double, MPC::n_states, 1> cur_state;
std::ofstream myfile;

bool is_operation;

void odom_callback(const geometry_msgs::Pose2D & input)
{
  if(!is_operation)
  {
    cur_state(0,0) = input.x;
    cur_state(1,0) = input.y;
    cur_state(2,0) = input.theta;
    myfile<<input.x<<"\t"<<input.y<<"\t"<<input.theta<<"\n";
  }
}


void set_ref_path(const int & step, MPC::MPC & mpc_object)
{
  int period_iter = 200;
  double k2 =1;
  double k_theta = 6;
  double cur_phase = k2*(PI*double(step)/period_iter);
  double d_phase = k2*PI/period_iter;
  Eigen::Matrix<autodiff::real, MPC::n_states, MPC::N> ref_path;
  Eigen::Matrix<autodiff::real,  MPC::n_controls,  MPC::N> ref_con;
  double cur_x, cur_y, cur_theta, prev_x, prev_y, prev_theta, v_x, v_y, omega;
  double cout_theta;
  cout_theta = std::fmod((k_theta*cur_phase),2*PI);
  if(cout_theta > PI)
      {
        cout_theta = cout_theta-2*PI;
      }
  std::cout<<"CUR THETA: "<<cout_theta<<"\n";
  for(int i = 0; i <  MPC::N; ++i)
  {
      double k = 1;
      cur_x = k*cos(cur_phase + i*d_phase);
      cur_y = k*sin(cur_phase + i*d_phase);
      
      
      
      prev_x = k*cos(cur_phase + (i-1)*d_phase);
      prev_y = k*sin(cur_phase + (i-1)*d_phase);
      cur_theta = std::fmod(k_theta*(cur_phase+d_phase*i),2*PI);
      prev_theta = std::fmod(k_theta*(cur_phase+d_phase*(i-1)),2*PI);
      omega = (cur_theta - prev_theta)/MPC::dt;
      if(cur_theta > PI)
      {
        cur_theta = cur_theta-2*PI;
      }
      if(prev_theta > PI)
      {
        prev_theta = prev_theta-2*PI;
      }
      v_x = (cur_x-prev_x)/MPC::dt*cos(cur_theta) + (cur_y-prev_y)/MPC::dt*sin(cur_theta);
      v_y = -(cur_x-prev_x)/MPC::dt*sin(cur_theta) + (cur_y-prev_y)/MPC::dt*cos(cur_theta);
      ref_path(0,i) = cur_x;
      ref_path(1,i) = cur_y;
      ref_path(2,i) = cur_theta;
      ref_con(0,i) = v_x;
      ref_con(1,i) = v_y;
      ref_con(2,i) = omega;
  }
  mpc_object.set_reference(ref_path, ref_con);
}






void init_ref_trajectory(std::vector<double> & ref_trajectory)
{
  double cur_x, cur_y, prev_x, prev_y, c, theta, ds;
  c = 2.;
  ds = 0.009;
  double j =0;

  cur_x = 1;
  cur_y = 0;
  theta = 0;
  for (int i = 0; i< ref_trajectory.size(); i=i+3)
  {
    ref_trajectory[i] = cur_x;
    ref_trajectory[i+1] = cur_y;
    ref_trajectory[i+2] = theta;
  }
}




int main(int argc, char **argv)
{
  auto thr_num =  std::thread::hardware_concurrency();
  std::cout<<"thread_num = "<<thr_num<<"\n";
  myfile.open ("omni_mpc_4.txt");
  ros::init(argc, argv, "mpc_node");
  ros::NodeHandle n;
  ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Publisher predicted_trajectory_pub = n.advertise<nav_msgs::Path>("predicted_trajectory", 1);
  ros::Publisher ref_traj_pub = n.advertise<nav_msgs::Path>("reference_trajectory", 1);
  ros::Subscriber position_sub = n.subscribe("/open_base/pose/world", 1, odom_callback);

  
  std::vector<double> ref_traj (MPC::n_states*20, 0);
  init_ref_trajectory(ref_traj);
  std::cout<<"ref traj initiated\n";
  int step = 0;
  mpc.init_nlopt();
  Eigen::Matrix<autodiff::real, MPC::n_states, MPC::N> ref_path;
  Eigen::Matrix<autodiff::real,  MPC::n_controls,  MPC::N> ref_con;
  for(int i = 0; i <  MPC::N; ++i)
  {
      ref_path(0,i) = 0.;
      ref_path(1,i) = 0.0;
      ref_path(2,i) = 0;
      ref_con(0,i) = 0;
      ref_con(1,i) = 0;
      ref_con(2,i) = 0;
  }
  
  mpc.set_reference(ref_path, ref_con);
  cur_state(0,0) =0;
  cur_state(1,0) = 0;
  cur_state(2,0) = 0;
  
  ROS_INFO("MPC_INITIATED");


  is_operation = false;
  bool fl = true;
  int hz = 1/MPC::dt;
  ros::Rate loop_rate(10);
  double minf = 1;
  //myfile <<"X\tY\tTheta\t\n";
  while (ros::ok())
  {
    is_operation = true;
    geometry_msgs::Twist speeds;
    speeds.linear.x = 0;
    speeds.angular.z = 0;
    /*for(int i = 0; i <  MPC::N; ++i)
    {

        ref_path(0,i) = 1;
        ref_path(1,i) = 0;
        ref_path(2,i) = 0;
        ref_con(0,i) = 0;
        ref_con(1,i) = 0;
        ref_con(2,i) = 0; 
    }   
    mpc.set_reference(ref_path, ref_con);*/
    set_ref_path(step,mpc);
    Eigen::Matrix<double,MPC::n_controls, 1> calc_control;
    std::vector<double> predicted_states ((MPC::N+1)*MPC::n_states,0.);
    double func;
    if(fl == true)
    {
      calc_control = mpc.solve(cur_state, func);
      speeds.linear.x = calc_control(0,0);
      speeds.linear.y = calc_control(1,0);
      speeds.angular.z = calc_control(2,0);

    }
    else
    {
      fl = false;
    }  
    is_operation = false;
    velocity_pub.publish(speeds);
    


    ros::spinOnce();
    
    loop_rate.sleep();
    ++step;
  }


  return 0;
}