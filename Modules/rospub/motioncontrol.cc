#include "rospub.h"
//Cleaned up, general purpose rospub library for lua


void armcontrol(std::vector<double> qArm,double duration){
  // trajectory_msgs::JointTrajectory msg;
  // msg.joint_names.push_back("arm_lift_joint");
	// msg.joint_names.push_back("arm_flex_joint");
	// msg.joint_names.push_back("arm_roll_joint");
	// msg.joint_names.push_back("wrist_flex_joint");
	// msg.joint_names.push_back("wrist_roll_joint");
  // msg.points.resize(1);
  // msg.points[0].positions.resize(5);
  // msg.points[0].positions[0]=qArm[0];
	// msg.points[0].positions[1]=qArm[1];
	// msg.points[0].positions[2]=qArm[2];
	// msg.points[0].positions[3]=qArm[3];
	// msg.points[0].positions[4]=qArm[4];
  //
  // msg.points[0].velocities.resize(5);
  // msg.points[0].velocities[0]=0.0;
  // msg.points[0].velocities[1]=0.0;
  // msg.points[0].velocities[2]=0.0;
  // msg.points[0].velocities[3]=0.0;
  // msg.points[0].velocities[4]=0.0;
  // msg.points[0].time_from_start=ros::Duration(duration);
  // arm_publisher.publish(msg);
}

void headcontrol(double pan, double tilt, double duration){
  // trajectory_msgs::JointTrajectory msg;
  // msg.joint_names.push_back("head_pan_joint");
  // msg.joint_names.push_back("head_tilt_joint");
  // msg.points.resize(1);
  // msg.points[0].positions.resize(2);
  // msg.points[0].positions[0]=pan;
  // msg.points[0].positions[1]=tilt;
  // msg.points[0].velocities.resize(2);
  // msg.points[0].velocities[0]=0.0;
  // msg.points[0].velocities[1]=0.0;
  // msg.points[0].time_from_start=ros::Duration(duration);
  // head_publisher.publish(msg);
}

void grippercontrol(double pos, double effort, double duration){
  // trajectory_msgs::JointTrajectory msg;
  // msg.joint_names.push_back("hand_motor_joint");
  // msg.points.resize(1);
  // msg.points[0].positions.resize(1);
  // msg.points[0].positions[0]=pos;
  // msg.points[0].velocities.resize(1);
  // msg.points[0].velocities[0]=0.0;
  // msg.points[0].effort.resize(1);
  // msg.points[0].effort[0]=effort;
  // msg.points[0].time_from_start=ros::Duration(duration);
  // gripper_publisher.publish(msg);
}

void baseteleop(double x, double y, double a){
  geometry_msgs::Twist msg;
  msg.linear.x=x;
  msg.linear.y=y;
  msg.linear.z=0.0;
  msg.angular.x=0.0;
  msg.angular.y=0.0;
  msg.angular.z=a;
  baseteleop_publisher.publish(msg);
}
