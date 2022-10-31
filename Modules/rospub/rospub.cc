#include "rospub.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
using namespace cv;
//Cleaned up, general purpose rospub library for lua

//Joint movements
ros::Publisher arm_publisher; //standard arm motion
ros::Publisher head_publisher; //linear movement of head
ros::Publisher gripper_publisher; //for xb360 control messages
ros::Publisher gripperforce_publisher; //for xb360 control messages
ros::Publisher baseteleop_publisher; //for various commands

ros::Publisher armjoint7_publisher;
ros::Publisher armpose7_publisher;


//For joint visualization
ros::Publisher jointstate_publisher;
ros::Publisher sensortr_publisher;

//Pathplan and navigation
ros::Publisher basegoal_publisher; //this one skips python api
ros::Publisher posereset_publisher;
ros::Publisher lomap_publisher;
ros::Publisher path_publisher;

//For RVIZ visualization
ros::Publisher markerrectangle_publisher; //main surface (for grab detection and etc)
ros::Publisher markerarray_publisher; //objects (cylinder)
ros::Publisher markerarray2_publisher; //objects (cylinder)

//For logging
ros::Publisher poselog_publisher;
ros::Publisher imagelog_publisher;

std::unique_ptr<tf::TransformBroadcaster> _tf_odom_broadcaster;

//Variable sensors
ros::Publisher lidar_publisher;
ros::Publisher rgbinfo_publisher;
ros::Publisher depthinfo_publisher;
ros::Publisher rgb_publisher;
ros::Publisher depth_publisher;
ros::Publisher depth2_publisher;
ros::Publisher odom_publisher;


ros::Publisher lidar2_publisher;


void init_publishers(){
  _tf_odom_broadcaster.reset(new tf::TransformBroadcaster());

  arm_publisher = rosNode->advertise<trajectory_msgs::JointTrajectory>("/pnu/arm_trajectory", 10, true);
  head_publisher = rosNode->advertise<trajectory_msgs::JointTrajectory>("/pnu/head_trajectory", 1, true);
  gripper_publisher = rosNode->advertise<trajectory_msgs::JointTrajectory>("/pnu/gripper_trajectory", 1, true);
  baseteleop_publisher = rosNode->advertise<geometry_msgs::Twist>("/pnu/command_velocity", 1, true);
  armjoint7_publisher = rosNode->advertise<sensor_msgs::JointState>("/arm_joint7", 1, true);
  armpose7_publisher = rosNode->advertise<geometry_msgs::PoseStamped>("/arm_pose7", 1, true);


  sensortr_publisher= rosNode->advertise<geometry_msgs::PoseStamped>("/pnu/sensor_tr", 1, true);

	basegoal_publisher= rosNode->advertise<geometry_msgs::PoseStamped>("/pnu/goal", 1, true);
	posereset_publisher= rosNode->advertise<geometry_msgs::PoseWithCovarianceStamped>("/laser_2d_correct_pose", 1, true);
  lomap_publisher= rosNode->advertise<nav_msgs::OccupancyGrid>("/pnu/lowres_map", 1, true);
  path_publisher= rosNode->advertise<nav_msgs::Path>("/pnu/path", 1, true);

  markerrectangle_publisher = rosNode->advertise<object_recognition_msgs::TableArray>("/pnu/markerrectangle", 1, true);
  markerarray_publisher= rosNode->advertise<visualization_msgs::MarkerArray>("/pnu/markerarray", 1, true);
  markerarray2_publisher= rosNode->advertise<visualization_msgs::MarkerArray>("/pnu/markerarray2", 1, true);

  lidar_publisher= rosNode->advertise<sensor_msgs::LaserScan>("/pnu/base_scan", 1, true);
  rgbinfo_publisher= rosNode->advertise<sensor_msgs::CameraInfo>("/pnu/head_rgbd_sensor/rgb/camera_info", 1, true);
  depthinfo_publisher= rosNode->advertise<sensor_msgs::CameraInfo>("/pnu/head_rgbd_sensor/depth_registered/camera_info", 1, true);
  rgb_publisher= rosNode->advertise<sensor_msgs::Image>("/pnu/rgb_rect_raw", 1, true);
  depth_publisher= rosNode->advertise<sensor_msgs::Image>("/pnu/depth_rect_raw", 1, true);
  depth2_publisher= rosNode->advertise<sensor_msgs::Image>("/pnu/depth_rect_raw2", 1, true);
  odom_publisher= rosNode->advertise<nav_msgs::Odometry>("/odom", 1, true);

  jointstate_publisher= rosNode->advertise<sensor_msgs::JointState>("/joint_states", 1, true);
  lidar2_publisher= rosNode->advertise<sensor_msgs::LaserScan>("/scan", 1, true);
}



void basegoal(int seq,double x, double y, double a){
  geometry_msgs::PoseStamped msg;
	msg.header.stamp = ros::Time::now();
	msg.header.seq = seq;
	msg.header.frame_id="map";
	msg.pose.position.x=x;
	msg.pose.position.y=y;
	msg.pose.position.z=0.0;
  msg.pose.orientation.x=0.0;
	msg.pose.orientation.y=0.0;
	msg.pose.orientation.z=1.0*sin(a/2.0);
  msg.pose.orientation.w=cos(a/2.0);
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
	double t0=cos(yaw*0.5);
	double t1=sin(yaw*0.5);
	double t2=1.0; //roll 0 deg
	double t3=0.0;
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


void occgrid(float res, int width, int height, float x0, float y0, float z0, const char* data){
  nav_msgs::OccupancyGrid msg;
  msg.header.stamp=ros::Time::now();
  msg.header.frame_id="map";
  msg.info.resolution=res;
  msg.info.width=width;
  msg.info.height=height;
  msg.info.origin.position.x=x0;
  msg.info.origin.position.y=y0;
  msg.info.origin.position.z=z0;
  msg.data.insert(msg.data.end(), &data[0],&data[width*height]);
  lomap_publisher.publish(msg);
}

void path(std::vector<double> posx,std::vector<double> posy,std::vector<double> posa){
  nav_msgs::Path msg;
  msg.header.stamp=ros::Time::now();
  msg.header.frame_id="map";
  msg.poses.resize(posx.size());
  for(int i=0;i<posx.size();i++){
    msg.poses[i].header.stamp=ros::Time::now();
    msg.poses[i].header.frame_id="map";
    msg.poses[i].pose.position.x=posx[i];
    msg.poses[i].pose.position.y=posy[i];
    msg.poses[i].pose.position.z=0.0;
    msg.poses[i].pose.orientation.x=0.0;
    msg.poses[i].pose.orientation.y=0.0;
    msg.poses[i].pose.orientation.z=sin(posa[i]/2);
    msg.poses[i].pose.orientation.w=cos(posa[i]/2);
  }
  path_publisher.publish(msg);
}


void jointstates(std::vector<double> head,std::vector<double> arm){
  sensor_msgs::JointState msg;
  msg.header.stamp=ros::Time::now();
  msg.name.push_back("head_pan_joint");
  msg.name.push_back("head_tilt_joint");

  msg.name.push_back("arm_lift_joint");
  msg.name.push_back("arm_flex_joint");
  msg.name.push_back("arm_roll_joint");
  msg.name.push_back("wrist_flex_joint");
  msg.name.push_back("wrist_roll_joint");

  msg.position.push_back(head[0]);
  msg.position.push_back(head[1]);

  msg.position.push_back(arm[0]);
  msg.position.push_back(arm[1]);
  msg.position.push_back(arm[2]);
  msg.position.push_back(arm[3]);
  msg.position.push_back(arm[4]);
  jointstate_publisher.publish(msg);
}

void send_jointstate(std::vector<std::string> jointnames,std::vector<double>jangles){
  sensor_msgs::JointState msg;
  msg.header.stamp=ros::Time::now();
  msg.name.resize(jointnames.size());
  msg.position.resize(jointnames.size());
  for(int i=0;i<jointnames.size();i++){
    msg.name[i]=jointnames[i];
    msg.position[i]=jangles[i];
  }
  jointstate_publisher.publish(msg);
}

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

void webotslaserscancrop(int seq,  float angle_min, float angle_max,int n,
  float angle_increment,float* ranges, const char* linkname,int ray_num){
  //webots scan data is from left (+yaw) to right (-yaw)
  //for -pi to pi,
  //angle[0]=179.5 deg
  //angle[1]= 178.5 deg
  //angle[179]=0.5 deg
  //angle[180]=-0.5 deg
  //angle[359]=179.5 deg

  sensor_msgs::LaserScan msg;
  msg.header.seq=seq;
  msg.header.stamp=ros::Time::now();
  msg.header.frame_id=linkname;
  msg.angle_min=angle_min+angle_increment*ray_num;
  msg.angle_max=angle_max-angle_increment*ray_num;
  msg.angle_increment=angle_increment;
  msg.range_min = 0.1;
  msg.range_max = 6.0;
  msg.time_increment=0;
  msg.scan_time=0.2;

  msg.ranges.resize(n-2*ray_num);
  for(int i=ray_num;i<n-ray_num;i++) {
    msg.ranges[i-ray_num]=ranges[n-1-i];
  }
  lidar2_publisher.publish(msg);
}


void savergbimage(int width, int height, const char* enc, const char* data, const char* filename){
  sensor_msgs::Image msg;
  msg.header.seq=1;
  msg.header.stamp=ros::Time::now();
  msg.header.frame_id="head_rgbd_sensor_rgb_frame_sync";
  msg.width=width;msg.height=height;
  msg.encoding=enc;
  msg.is_bigendian=0;
  msg.step = width*3; //640*3
  msg.data.insert(msg.data.end(), &data[0],&data[width*height*3]);

  cv_bridge::CvImagePtr cv_ptr;
  try{cv_ptr=cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }catch(cv_bridge::Exception& e){printf("Error %s\n",e.what());}
  printf("Image imported , w%d h%d\n",cv_ptr->image.cols,cv_ptr->image.rows );
  std::vector<int> cparams;
  cparams.push_back(CV_IMWRITE_PNG_COMPRESSION);
  cparams.push_back(9);
  imwrite(filename,cv_ptr->image,cparams);
  printf("Log %s saved\n",filename);
}
