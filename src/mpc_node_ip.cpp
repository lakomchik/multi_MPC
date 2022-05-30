#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "mpc_ip.cpp"
#include <sstream>
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include <fstream>
#include "nav_msgs/Path.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include <tf/transform_datatypes.h>
#include "gazebo_msgs/ModelStates.h"
#include "std_msgs/Float64.h"
#include <fstream>
MPC::MPC mpc;
Eigen::Matrix <double, MPC::n_states, 1> cur_state;
std::ofstream file;

std_msgs::Float64 torgue;

bool is_operation;

void odom_callback(const gazebo_msgs::ModelStates::ConstPtr & input)
{
  if(true)
  {
    cur_state(0,0) = input->pose[1].position.x;
    cur_state(1,0) = input->twist[1].linear.x;



    //getting orientation
    tf::Quaternion q(
          input->pose[1].orientation.x,
          input->pose[1].orientation.y,
          input->pose[1].orientation.z,
          input->pose[1].orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
    cur_state(2,0) = -pitch;
    cur_state(3,0) = -input->twist[1].angular.y;
    std::cout<<"CUR POS: ";
    for(int i = 0; i< MPC::n_states; ++i)
    {
      std::cout<<cur_state(i,0)<<" ";
      file<<cur_state(i,0)<<"\t";
    }
    std::cout<<"\n";
    file<<torgue.data<<"\n";
  }
}













int main(int argc, char **argv)
{
  mpc.set_q();
  ros::init(argc, argv, "mpc_node_ip");
  ros::NodeHandle n;
  ros::Publisher Torgue1_pub = n.advertise<std_msgs::Float64>("/teeterbot/left_torque_cmd", 1);
  ros::Publisher Torgue2_pub = n.advertise<std_msgs::Float64>("/teeterbot/right_torque_cmd", 1);
  ros::Subscriber position_sub = n.subscribe("/gazebo/model_states", 1, odom_callback);


  double r1;
  Eigen::Matrix<autodiff::real, MPC::n_controls, MPC::n_controls> RR;
  RR = mpc.R;
  r1 = RR(0,0).val();
  std::string file_name = "nonlinear_exp_4.txt"; 
  //std::string file_name = "linear_exp_4.txt"; 
  file.open(file_name);
  for (int i = 0; i < MPC::n_states; ++i)
  {
    file<<mpc.Q(i,i)<<"\t";
  }
  file<<r1<<"\n";
  torgue.data = 0;



  mpc.init_nlopt();
  Eigen::Matrix<autodiff::real, MPC::n_states, MPC::N> ref_path;
  Eigen::Matrix<autodiff::real,  MPC::n_controls,  MPC::N> ref_con;
  for(int i = 0; i <  MPC::N; ++i)
  {
      ref_path(0,i) = 0.0;
      ref_path(1,i) = 0.0;
      ref_path(2,i) = 0.;
      ref_path(3,i) = 0.;
      ref_con(0,i) = 0;
  }
  
  mpc.set_reference(ref_path, ref_con);
  cur_state(0,0) = 0.;
  cur_state(1,0) = 0;
  cur_state(2,0) = 0.;
  cur_state(3,0) = 0;
  
  ROS_INFO("MPC_INITIATED");


  is_operation = false;
  bool fl = true;
  int hz = 1/MPC::dt;
  ros::Rate loop_rate(40);
  double minf = 1;

  while (ros::ok())
  {


   
    mpc.set_reference(ref_path, ref_con);
    Eigen::Matrix<double,MPC::n_controls, 1> calc_control;

    double minf;
    calc_control = mpc.solve(cur_state,minf);
    torgue.data = -calc_control(0,0);
    /*if(fl == true)
    {


    }
    else
    {
      fl = false;
    }*/
    

    
    is_operation = false;
    Torgue1_pub.publish(torgue);
    Torgue2_pub.publish(torgue);
    


    ros::spinOnce();
    
    loop_rate.sleep();
  }


  return 0;
}