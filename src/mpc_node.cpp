#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "mpc_diff.cpp"
#include <sstream>
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include <fstream>


MPC::MPC mpc;
Eigen::Matrix <double, MPC::n_states, 1> cur_state;
std::ofstream myfile;

bool is_operation;

void odom_callback(const nav_msgs::Odometry::ConstPtr & input)
{
  if(!is_operation)
  {
    cur_state(0,0) = input->pose.pose.position.x;
    cur_state(1,0) = input->pose.pose.position.y;
    tf::Quaternion q(
          input->pose.pose.orientation.x,
          input->pose.pose.orientation.y,
          input->pose.pose.orientation.z,
          input->pose.pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
    cur_state(2,0) = yaw;
    myfile<<input->pose.pose.position.x<<"\t"<<input->pose.pose.position.y<<"\t"<<yaw<<"\n";
  }
}





int main(int argc, char **argv)
{
  myfile.open ("output.txt");
  ros::init(argc, argv, "mpc_node");
  ros::NodeHandle n;
  ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Subscriber position_sub = n.subscribe("/odom", 1, odom_callback);
 
  



  double s = 0;
  double ds = 0.007;
  double c =2.;

  mpc.init_nlopt();
  Eigen::Matrix<autodiff::real, MPC::n_states, MPC::N> ref_path;
  Eigen::Matrix<autodiff::real,  MPC::n_controls,  MPC::N> ref_con;
  for(int i = 0; i <  MPC::N; ++i)
  {
      ref_path(0,i) = 0.;
      ref_path(1,i) = 0.0;
      ref_path(2,i) = 0;
      ref_con(0,i) = 0;
      ref_con(0,i) = 0;
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
  myfile <<"X\tY\tTheta\t\n";
  while (ros::ok())
  {
 
    is_operation = true;
    geometry_msgs::Twist speeds;
   // ROS_INFO("IIIII");
    speeds.linear.x = 0;
    speeds.angular.z = 0;
    s+=ds;
    double cur_x, cur_y, prev_x, prev_y, theta, v_ref;
    for(int i = 0; i <  MPC::N; ++i)
    {
        double j;
        j = s + ds*i;
        //circle
        /*ref_path(0,i) = std::cos(j);
        ref_path(1,i) = std::sin(j);
        ref_path(2,i) = 0;
        ref_con(0,i) = ds;
        ref_con(0,i) = ds;*/
        //lemniscate
        /*ref_path(0,i) = c*sqrt(2)*std::cos(j)/(1+pow(sin(j),2));
        ref_path(1,i) = c*sqrt(2)*std::sin(j)*std::cos(j)/(1+pow(sin(j),2));
        ref_path(2,i) = 0;
        ref_con(0,i) = 0;
        ref_con(0,i) = 0;*/
        cur_x = c*sqrt(2)*std::cos(j)/(1+pow(sin(j),2));
        cur_y = c*sqrt(2)*std::sin(j)*std::cos(j)/(1+pow(sin(j),2));
        prev_x = c*sqrt(2)*std::cos(j-ds)/(1+pow(sin(j-ds),2));
        prev_y = c*sqrt(2)*std::sin(j-ds)*std::cos(j-ds)/(1+pow(sin(j-ds),2));
        theta = atan2(cur_y-prev_y,cur_x-prev_x);
        v_ref = sqrt( pow(cur_y-prev_y,2) + pow(cur_x-prev_x,2) )/MPC::dt;

        ref_path(0,i) = cur_x;
        ref_path(1,i) = cur_y;
        ref_path(2,i) = theta;
        ref_con(0,i) = v_ref;
        ref_con(0,i) = 0;
        







    }
    cur_x = c*sqrt(2)*std::cos(s)/(1+pow(sin(s),2));
    cur_y = c*sqrt(2)*std::sin(s)*std::cos(s)/(1+pow(sin(s),2));
    prev_x = c*sqrt(2)*std::cos(s-ds)/(1+pow(sin(s-ds),2));
    prev_y = c*sqrt(2)*std::sin(s-ds)*std::cos(s-ds)/(1+pow(sin(s-ds),2));
    theta = atan2(cur_y-prev_y,cur_x-prev_x);
    v_ref = sqrt( pow(cur_y-prev_y,2) + pow(cur_x-prev_x,2) )/MPC::dt;
    std::cout<<"\n x: "<<cur_x<<"\ty: "<<cur_y<<"\ttheta: "<<theta<<"\tv: "<<v_ref<<"\n";
    mpc.set_reference(ref_path, ref_con);
    Eigen::Matrix<double,MPC::n_controls, 1> calc_control;
    if(/* minf > 0.000000000000000000001&&*/ fl == true)
    {
      calc_control = mpc.solve(cur_state, minf);
      speeds.linear.x = calc_control(0,0);
      speeds.angular.z = calc_control(1,0);
    }
    else
    {
      fl = false;
    }
    

    
    is_operation = false;
    velocity_pub.publish(speeds);
    


    ros::spinOnce();
    
    loop_rate.sleep();
  }


  return 0;
}