#ifndef ROSPUB_H_
#define ROSPUB_H_

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/LaserScan.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Joy.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "object_recognition_msgs/TableArray.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>



//Arm movements
extern ros::Publisher arm_publisher; //standard arm motion
extern ros::Publisher head_publisher; //linear movement of head
extern ros::Publisher gripper_publisher; //for xb360 control messages
extern ros::Publisher gripperforce_publisher; //for xb360 control messages

//For joint visualization
extern ros::Publisher jointstate_publisher;
extern ros::Publisher sensortr_publisher;

//Base movement
extern ros::Publisher baseteleop_publisher; //for various commands
extern ros::Publisher basegoal_publisher; //this one skips python api
extern ros::Publisher posereset_publisher;

//For RVIZ visualization
extern ros::Publisher markerrectangle_publisher; //main surface (for grab detection and etc)
extern ros::Publisher markerarray_publisher; //objects (cylinder)
extern ros::Publisher markerarray2_publisher; //objects (cylinder)

//For logging
extern ros::Publisher poselog_publisher;
extern ros::Publisher imagelog_publisher;
extern std::unique_ptr<tf::TransformBroadcaster> _tf_odom_broadcaster;

//Variable sensors
extern ros::Publisher lidar_publisher;
extern ros::Publisher rgbinfo_publisher;
extern ros::Publisher depthinfo_publisher;
extern ros::Publisher rgb_publisher;
extern ros::Publisher depth_publisher;
extern ros::Publisher depth2_publisher;

//Path planning
extern ros::Publisher lomap_publisher;
extern ros::Publisher path_publisher;

extern ros::Publisher armjoint7_publisher;
extern ros::Publisher armpose7_publisher;


void init_publishers();

void armcontrol(std::vector<double> qArm,double duration);
void headcontrol(double pan, double tilt, double duration);
void grippercontrol(double pos, double effort, double duration);
void baseteleop(double x, double y, double a);

void jointstates(std::vector<double> head,std::vector<double> arm);
void basegoal(int seq,double x, double y, double a);
void posereset(std::vector<double> pose);
void occgrid(float res, int width, int height, float x0, float y0, float z0, const char* data);
void path(std::vector<double> posx,std::vector<double> posy,std::vector<double> posa);

void marker(int num,
  std::vector<double> types,
  std::vector<double> posx,std::vector<double> posy,std::vector<double> posz,std::vector<double> yaw,
  std::vector<std::string> names,std::vector<double> scales,std::vector<double> colors
);
void marker3d(int num,
  std::vector<double> types,
  std::vector<double> posx,std::vector<double> posy,std::vector<double> posz,
  std::vector<double> orir,std::vector<double> orip,std::vector<double> oriy,
  std::vector<double> scalex,std::vector<double> scaley,std::vector<double> scalez,
  std::vector<std::string> names,std::vector<double> colors, std::vector<double> alpha
);
void marker_rectangle(int seq,float posx, float posy, float yaw,float lw, float rw, float th, float bh);

void laserscan(int seq,  float angle_min, float angle_max,int n,float angle_increment,float* ranges, const char* linkname);
void camerainfo(int seq,int width, int height, double fov);
void rgbimage(int seq,int width, int height, const char* data,int enc);
void jpgimage(int seq,const char* data);
void depthimage(int seq,int width, int height, const char* data,int send_surface,int type);
void send_odom(std::vector<double>pose);


void send_jointstate(std::vector<std::string> jointnames,std::vector<double>jangles);//general purpose!!
void webotslaserscancrop(int seq,  float angle_min, float angle_max,int n,
  float angle_increment,float* ranges, const char* linkname,int ray_num);

void savergbimage(int width, int height, const char* enc, const char* data, const char* filename);

extern std::unique_ptr<ros::NodeHandle> rosNode;
#endif
