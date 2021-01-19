#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "kdl/chainiksolver.hpp"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/chain.hpp"
#include "kdl/chainiksolverpos_nr.hpp"
#include "kdl/chainiksolvervel_pinv.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl_conversions/kdl_msg.h"
geometry_msgs::PoseStamped pippo_;
KDL::JntArray   q_init(6);
 unsigned int k=0;
KDL::Frame desired_frame;
//#include "kdl/chainiksolverpos_nr.hpp"
void get_omni_pose(const geometry_msgs::PoseStamped& pose_stamped_)
{
  
tf::PoseMsgToKDL(pose_stamped_.pose,desired_frame);
//std::cout<<desired_frame<<std::endl;

}
void get_ur5_states(const sensor_msgs::JointState& pluto){
        for(int i=0;i<6;i++){
        q_init.data[i]=pluto.position[i];
      }
      
}
int main(int argc,char **argv){
    ros::init(argc,argv,"joint_node");
    ros::NodeHandle nh_;
    ros::Subscriber sub_ur5_joint_ =nh_.subscribe("joint_states",1,get_ur5_states);
    ros::Subscriber sub_omni_= nh_.subscribe("omni1_pose", 1, get_omni_pose);
    ros::Publisher joint_trajectory_pub = nh_.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 1000);
    ros::Rate loop_rate(1);
    trajectory_msgs::JointTrajectory msg_trajectory;
    KDL::Tree my_tree;
  
   std::string robot_desc_string;
   
   nh_.param("robot_description", robot_desc_string, std::string());
   std::cout<<robot_desc_string<<std::endl;
   //kdl_parser::treeFromString(robot_desc_string, my_tree);
   if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
      ROS_ERROR("Failed to construct kdl tree");
      return false;
   }
 KDL::Chain chain;
 
  my_tree.getChain("world","tool0",chain);
  KDL::ChainFkSolverPos_recursive fk_solver(chain);
    KDL::ChainIkSolverVel_pinv ik_solver_vel(chain);
    KDL::ChainIkSolverPos_NR ik_solver(chain, fk_solver,
            ik_solver_vel, 1000, 100);
  //KDL::JntArray   q_init(my_tree.getNrOfJoints());
  KDL::JntArray q_out(chain.getNrOfJoints());


 std::cout<<chain.getNrOfJoints()<<std::endl;
  ik_solver.CartToJnt(q_init,desired_frame,q_out);
  msg_trajectory.header.stamp = ros::Time::now();
  msg_trajectory.joint_names.resize(6);
  msg_trajectory.joint_names[0] = "shoulder_pan_joint";
  msg_trajectory.joint_names[1] = "shoulder_lift_joint";
  msg_trajectory.joint_names[2] = "elbow_joint";
  msg_trajectory.joint_names[3] = "wrist_1_joint";
  msg_trajectory.joint_names[4] = "wrist_2_joint";
  msg_trajectory.joint_names[5] = "wrist_3_joint";
  msg_trajectory.points.resize(k+1);
  msg_trajectory.points[k].positions.resize(6);
msg_trajectory.points[k].positions[0]=(q_out.data(0));
msg_trajectory.points[k].positions[1]=(q_out.data(1));
msg_trajectory.points[k].positions[2]=(q_out.data(2));
msg_trajectory.points[k].positions[3]=(q_out.data(3));
msg_trajectory.points[k].positions[4]=(q_out.data(4));
msg_trajectory.points[k].positions[5]=(q_out.data(5));  
 /* trajectory_msgs::JointTrajectoryPoint points_n;
    points_n.positions.resize(0);
  points_n.positions.push_back(q_out.data[0]);
  points_n.positions.resize(1);
  points_n.positions.push_back(q_out.data[1]);
  points_n.positions.resize(2);
  points_n.positions.push_back(q_out.data[2]);
  points_n.positions.resize(3);
  points_n.positions.push_back(q_out.data[3]);
  points_n.positions.resize(4);
  points_n.positions.push_back(q_out.data[4]);
  points_n.positions.resize(5);
  points_n.positions.push_back(q_out.data[5]);
  msg_trajectory.points.push_back(points_n); */
 // msg_trajectory.header.stamp = ros::Tme::now();
  msg_trajectory.points[0].time_from_start = ros::Duration((k+1)*1.0);
  joint_trajectory_pub.publish(msg_trajectory);
  //msg_trajectory.points[0].positions[0] = 2;
  //msg_trajectory.points[0].positions.push_back(2);
  //msg_trajectory.points[0].positions[0]=2;
  while (ros::ok()){
     ik_solver.CartToJnt(q_init,desired_frame,q_out);
      std::cout<<q_out.data<<std::endl;
  msg_trajectory.header.stamp = ros::Time::now();
  msg_trajectory.joint_names.resize(6);
  msg_trajectory.joint_names[0] = "shoulder_pan_joint";
  msg_trajectory.joint_names[1] = "shoulder_lift_joint";
  msg_trajectory.joint_names[2] = "elbow_joint";
  msg_trajectory.joint_names[3] = "wrist_1_joint";
  msg_trajectory.joint_names[4] = "wrist_2_joint";
  msg_trajectory.joint_names[5] = "wrist_3_joint";
    msg_trajectory.points.resize(k+1);
  msg_trajectory.points[k].positions.resize(6);
msg_trajectory.points[k].positions[0]=(q_out.data(0));
msg_trajectory.points[k].positions[1]=(q_out.data(1));
msg_trajectory.points[k].positions[2]=(q_out.data(2));
msg_trajectory.points[k].positions[3]=(q_out.data(3));
msg_trajectory.points[k].positions[4]=(q_out.data(4));
msg_trajectory.points[k].positions[5]=(q_out.data(5));  

  /*trajectory_msgs::JointTrajectoryPoint points_n;
  points_n.positions.resize(0);
  points_n.positions.push_back(q_out.data[0]);
  points_n.positions.resize(1);
  points_n.positions.push_back(q_out.data[1]);
  points_n.positions.resize(2);
  points_n.positions.push_back(q_out.data[2]);
  points_n.positions.resize(3);
  points_n.positions.push_back(q_out.data[3]);
  points_n.positions.resize(4);
  points_n.positions.push_back(q_out.data[4]);
  points_n.positions.resize(5);
  points_n.positions.push_back(q_out.data[5]);
  msg_trajectory.points.push_back(points_n); 
  joint_trajectory_pub.publish(msg_trajectory);*/
  
     
     
    //msg_trajectory.header.stamp = ros::Time::now();
     
     
   msg_trajectory.points[k].time_from_start = ros::Duration((k+1)*1.0);
      joint_trajectory_pub.publish(msg_trajectory);
      k++;
      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;}