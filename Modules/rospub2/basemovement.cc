#include "rospub.h"

void send_odom(std::vector<double>pose){
  nav_msgs::Odometry msg;
  msg.header.stamp=ros::Time::now();
  msg.header.frame_id="odom";
  msg.child_frame_id="base_link";
  msg.pose.pose.position.x=pose[0];
  msg.pose.pose.position.y=pose[1];
  msg.pose.pose.position.z=0.0;
  msg.pose.pose.orientation.x=0.0;
	msg.pose.pose.orientation.y=0.0;
	msg.pose.pose.orientation.z=1.0*sin(pose[2]/2.0);
  msg.pose.pose.orientation.w=cos(pose[2]/2.0);
  odom_publisher.publish(msg);
}


void basegoal(std::vector<double> pose){
  geometry_msgs::PoseStamped msg;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id="map";
	msg.pose.position.x=pose[0];
	msg.pose.position.y=pose[1];
	msg.pose.position.z=0.0;
  msg.pose.orientation.x=0.0;
	msg.pose.orientation.y=0.0;
	msg.pose.orientation.z=1.0*sin(pose[2]/2.0);
  msg.pose.orientation.w=cos(pose[2]/2.0);
  basegoal_publisher.publish(msg);
}


void posereset(std::vector<double> pose){
  geometry_msgs::PoseWithCovarianceStamped msg;
	msg.header.frame_id="map";
	msg.pose.pose.position.x=pose[0];
	msg.pose.pose.position.y=pose[1];
	msg.pose.pose.position.z=0.0;
	float yaw=pose[2];
	float pitch=0.0;
	double t0=cos(yaw*0.5);double t1=sin(yaw*0.5);
	double t2=1.0;double t3=0.0;
	double t4=cos(pitch  * 0.5); //pitch 90 deg
	double t5=sin(pitch  * 0.5); //pitch 90 deg
	msg.pose.pose.orientation.x=t0*t3*t4-t1*t2*t5;
	msg.pose.pose.orientation.y=t0*t2*t5+t1*t3*t4;
	msg.pose.pose.orientation.z=t1*t2*t4-t0*t3*t5;
	msg.pose.pose.orientation.w=t0*t2*t4+t1*t3*t5;
	for (int i=0;i<36;i++) msg.pose.covariance[i]=0.0;
	msg.pose.covariance[0]=0.01;
	posereset_publisher.publish(msg);
}
