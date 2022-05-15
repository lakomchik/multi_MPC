#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "mpc_diff.cpp"
#include <sstream>
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include <fstream>
#include "nav_msgs/Path.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include <tf/transform_datatypes.h>
#include <string>

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


nav_msgs::Path get_predicted_path(const std::vector<double> & predicted_vec)
{
  nav_msgs::Path path;
  geometry_msgs::PoseStamped point;
  for (int i =0; i < predicted_vec.size(); i=i+3)
  {
    point.pose.position.x = predicted_vec[i];
    point.pose.position.y = predicted_vec[i+1];
    tf2::Quaternion rot;
    rot.setRPY(0,0,predicted_vec[i+2]);
    point.pose.orientation.w = rot.getW();
    point.pose.orientation.x = rot.getX();
    point.pose.orientation.y = rot.getY();
    point.pose.orientation.z = rot.getZ();
    path.poses.push_back(point);
  }
  return path;
  //path.ppa
  
}







void init_ref_trajectory(std::vector<double> & ref_trajectory)
{
  double cur_x, cur_y, prev_x, prev_y, c, theta, ds;
  c = 2.;
  ds = 0.009;
  double j =0;

  cur_x = c*sqrt(2)*std::cos(j)/(1+pow(sin(j),2));
  cur_y = c*sqrt(2)*std::sin(j)*std::cos(j)/(1+pow(sin(j),2));
  prev_x = c*sqrt(2)*std::cos(j-ds)/(1+pow(sin(j-ds),2));
  prev_y = c*sqrt(2)*std::sin(j-ds)*std::cos(j-ds)/(1+pow(sin(j-ds),2));
  theta = atan2(cur_y-prev_y,cur_x-prev_x);
  for (int i = 0; i< ref_trajectory.size(); i=i+3)
  {
    ref_trajectory[i] = cur_x;
    ref_trajectory[i+1] = cur_y;
    ref_trajectory[i+2] = theta;
  }
}




int main(int argc, char **argv)
{
  double q1, q2, q3, r1, r2;
  Eigen::Matrix<autodiff::real, MPC::n_states, MPC::n_states> QQ;
  QQ = MPC::Q;
  q1 = QQ(0,0).val();
  q2 = QQ(1,1).val();
  q3 = QQ(2,2).val();
  Eigen::Matrix<autodiff::real, MPC::n_controls, MPC::n_controls> RR;
  RR = MPC::R;
  r1 = RR(0,0).val();
  r2 = RR(1,1).val();
  std::stringstream ss;
  std::string file_name;
  
  ss<< "" << std::fixed << std::setprecision(2) << "coeffs" << " " << q1 << " " << q2<< " " << q3<< " " << r1<< " " << r2<<".txt";
  file_name = ss.str();    
  myfile.open(file_name);
  ros::init(argc, argv, "mpc_node");
  ros::NodeHandle n;
  ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Publisher predicted_trajectory_pub = n.advertise<nav_msgs::Path>("predicted_trajectory", 1);
  ros::Publisher ref_traj_pub = n.advertise<nav_msgs::Path>("reference_trajectory", 1);
  ros::Subscriber position_sub = n.subscribe("/odom", 1, odom_callback);
 
  
  std::vector<double> ref_traj (MPC::n_states*20, 0);
  init_ref_trajectory(ref_traj);


  double s = 0;
  double ds = 0.006;
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
  //myfile <<"X\tY\tTheta\t\n";
  while (ros::ok())
  {
    double theta_prev =0;
    is_operation = true;
    geometry_msgs::Twist speeds;
   // ROS_INFO("IIIII");
    speeds.linear.x = 0;
    speeds.angular.z = 0;
    s+=ds;
    double cur_x, cur_y, prev_x, prev_y, theta, v_ref;
    cur_x = c*sqrt(2)*std::cos(s)/(1+pow(sin(s),2));
    cur_y = c*sqrt(2)*std::sin(s)*std::cos(s)/(1+pow(sin(s),2));
    prev_x = c*sqrt(2)*std::cos(s-ds)/(1+pow(sin(s-ds),2));
    prev_y = c*sqrt(2)*std::sin(s-ds)*std::cos(s-ds)/(1+pow(sin(s-ds),2));
    theta = atan2(cur_y-prev_y,cur_x-prev_x);
    std::rotate(ref_traj.begin(), ref_traj.begin()+3, ref_traj.end());
    
    ref_traj.pop_back();
    ref_traj.pop_back();
    ref_traj.pop_back();
    ref_traj.push_back(cur_x);
    ref_traj.push_back(cur_y);
    ref_traj.push_back(theta);
    /*ref_traj[ref_traj.size()-2] = cur_x;
    ref_traj[ref_traj.size()-1] = cur_y;
    ref_traj[ref_traj.size()] = theta;*/
   /* std::cout<<"ref_traj:\n";
    for(int i = 0; i < ref_traj.size(); i+=3)
    {
      for(int j = 0; j < 3; ++j)
      {
        std::cout<<ref_traj[i*3+j]<<"\t";
      }
      std::cout<<"\n";
    }*/
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
        double next_x, next_y, next_theta;
        cur_x = c*sqrt(2)*std::cos(j)/(1+pow(sin(j),2));
        cur_y = c*sqrt(2)*std::sin(j)*std::cos(j)/(1+pow(sin(j),2));
        prev_x = c*sqrt(2)*std::cos(j-ds)/(1+pow(sin(j-ds),2));
        prev_y = c*sqrt(2)*std::sin(j-ds)*std::cos(j-ds)/(1+pow(sin(j-ds),2));
        next_x=c*sqrt(2)*std::cos(j+ds)/(1+pow(sin(j+ds),2));
        next_y = c*sqrt(2)*std::sin(j+ds)*std::cos(j+ds)/(1+pow(sin(j+ds),2));
        next_theta = atan2(next_y-cur_y,next_x-cur_x);
        theta = atan2(cur_y-prev_y,cur_x-prev_x);
        v_ref = sqrt( pow(cur_y-prev_y,2) + pow(cur_x-prev_x,2) )/MPC::dt;

        ref_path(0,i) = cur_x;
        ref_path(1,i) = cur_y;
        ref_path(2,i) = theta;
        ref_con(0,i) = v_ref;
        
        double w_ref;
        w_ref = (next_theta-theta)/MPC::dt;
        ref_con(1,i) = w_ref; 
        std::cout<<"W_REF "<<w_ref<<"\n";
        theta_prev = theta;
        







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
    std::vector<double> predicted_states ((MPC::N+1)*MPC::n_states,0.);

    if(fl == true)
    {
      calc_control = mpc.solve(cur_state,predicted_states);
      nav_msgs::Path pred_path = get_predicted_path(predicted_states);

      pred_path.header = std_msgs::Header();
      pred_path.header.frame_id = "odom";
      pred_path.header.stamp = ros::Time::now();
      predicted_trajectory_pub.publish(pred_path);

      nav_msgs::Path reference_path = get_predicted_path(ref_traj);
      reference_path.header = std_msgs::Header();
      reference_path.header.frame_id = "odom";
      reference_path.header.stamp = ros::Time::now();
      ref_traj_pub.publish(reference_path);

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