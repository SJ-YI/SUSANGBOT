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
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// #include <opencv2/opencv.hpp> //for MAT

#define MAX_LIDAR_NO 5
#define MAX_CAMERA_NO 5
#define MAX_PATH_NO 5
#define MAX_OCCGRID_NO 5
#define MAX_JOINTTRAJECTORY_NO 5

extern std::unique_ptr<ros::NodeHandle> rosNode;
extern std::vector<std::string> camera_names;
extern std::vector<std::string> lidar_names;
extern std::vector<std::string> path_names;
extern std::vector<std::string> occgrid_names;



extern ros::Publisher rgb_publisher;
extern ros::Publisher teleopvel_publisher;
extern ros::Publisher cmdvel_publisher;

extern ros::Publisher jointstate_publisher;

extern ros::Publisher basegoal_publisher;
extern ros::Publisher navigation_status_publisher;

extern ros::Publisher posereset_publisher;
extern ros::Publisher odom_publisher;
extern ros::Publisher mapcmd_publisher;

extern ros::Publisher lidar_publisher[MAX_LIDAR_NO];
extern ros::Publisher image_publisher[MAX_CAMERA_NO];
extern ros::Publisher camerainfo_publisher[MAX_CAMERA_NO];
extern ros::Publisher path_publisher[MAX_PATH_NO];
extern ros::Publisher occgrid_publisher[MAX_OCCGRID_NO];


extern ros::Publisher markerarray_publisher;
extern ros::Publisher markerarray2_publisher; //objects (cylinder)
extern ros::Publisher markermatrix_publisher; //marker matrix


extern ros::Publisher camerainfo0_publisher;
extern ros::Publisher jointactiongoalpublisher;
extern ros::Publisher gripperactiongoalpublisher;

extern ros::Publisher jointtrajectorypublisher;
extern ros::Publisher jointtrajectorypublisher2; //TEMPORARY ONE FOR ATOMICDOG PROJECT...
extern ros::Publisher jointtrajectorypublisher3; //generic one for multiple traj points


extern ros::Publisher displayjpg_publisher; //for displaying JPG to the head display
// extern ros::Publisher pointcloudpublisher;

void init_publishers();

void basegoal(std::vector<double> pose);
void posereset(std::vector<double> pose);

void send_image(int seq,int width, int height, double fov, const char* data,int enc, int ch, const char* frame);

void send_imageonly(int seq,int width, int height, const char* data,int enc, int ch, const char* frame);

void send_imagefrompng(int seq,int width, int height, const char* data,int enc, int ch, const char* frame);

void savergbimage(int width, int height, const char* enc, const char* data, const char* filename);

void jpgimage(int seq,const char* data,int ch, const char* frame);


void send_jointstate(std::vector<std::string> jointnames,std::vector<double>jangles);

void send_odom(std::vector<double>pose);

void laserscan(int seq,  float angle_min, float angle_max,int n,float* ranges, const char* linkname, int ch);
void webotslaserscan(int seq,float angle_min, float angle_max, int n,float* ranges, const char* linkname, int ch);

void send_int32(int msg, ros::Publisher pub);
void send_bool(int msg, ros::Publisher pub);
void send_twist(double x, double y, double a,ros::Publisher pub);




void posereset(std::vector<double> pose);



void send_path(std::vector<double> posx,std::vector<double> posy,std::vector<double> posa, int ch);
void send_occgrid(float res, int width, int height, float x0, float y0, float z0, const char* data, int ch, const char* frameid);

void marker(int num,
  std::vector<double> types,
  std::vector<double> posx,std::vector<double> posy,std::vector<double> posz,std::vector<double> yaw,
  std::vector<std::string> names,std::vector<double> scales,std::vector<double> colors,
  float dur
);

void marker3d(int num,
  std::vector<double> types,
  std::vector<double> posx,std::vector<double> posy,std::vector<double> posz,
  std::vector<double> orir,std::vector<double> orip,std::vector<double> oriy,
  std::vector<double> scalex,std::vector<double> scaley,std::vector<double> scalez,
  std::vector<std::string> names,std::vector<double> colors, std::vector<double> alpha
);

void markermatrix(float x0,float y0,float a0, int mapx, int mapy, float scale, float* heights, float basez, float z0, const char* frame,int flip_y);
void markermatrixcolor(float x0,float y0,float a0, int mapx, int mapy, float scale, float* heights, uint8_t* colors,float basez, float z0,float xoffset,float ymax);

void markerlines(int seq, int num,
      std::vector<double> x0,std::vector<double> y0,std::vector<double> z0,
      std::vector<double> x1,std::vector<double> y1,std::vector<double> z1,
      float linewidth);

//
//
//
//
//
//
//
//
//
//
//
//
//
// //Arm movements
// extern ros::Publisher arm_publisher; //standard arm motion
// extern ros::Publisher head_publisher; //linear movement of head
// extern ros::Publisher gripper_publisher; //for xb360 control messages
// extern ros::Publisher gripperforce_publisher; //for xb360 control messages
//
// //For joint visualization
// extern ros::Publisher jointstate_publisher;
// extern ros::Publisher sensortr_publisher;
//
// //Base movement
// extern ros::Publisher baseteleop_publisher; //for various commands
// extern ros::Publisher basegoal_publisher; //this one skips python api
// extern ros::Publisher posereset_publisher;
//
// //For RVIZ visualization
// extern ros::Publisher markerrectangle_publisher; //main surface (for grab detection and etc)
// extern ros::Publisher markerarray_publisher; //objects (cylinder)
//
// //For logging
// extern ros::Publisher poselog_publisher;
// extern ros::Publisher imagelog_publisher;
// extern std::unique_ptr<tf::TransformBroadcaster> _tf_odom_broadcaster;
//
// //Variable sensors
// extern ros::Publisher lidar_publisher;
// extern ros::Publisher rgbinfo_publisher;
// extern ros::Publisher depthinfo_publisher;
// extern ros::Publisher rgb_publisher;
// extern ros::Publisher depth_publisher;
// extern ros::Publisher depth2_publisher;
//
// //Path planning
// extern ros::Publisher lomap_publisher;
// extern ros::Publisher path_publisher;
//
// extern ros::Publisher armjoint7_publisher;
// extern ros::Publisher armpose7_publisher;
//
//

//
// void armcontrol(std::vector<double> qArm,double duration);
// void headcontrol(double pan, double tilt, double duration);
// void grippercontrol(double pos, double effort, double duration);
// void baseteleop(double x, double y, double a);
//
// void jointstates(std::vector<double> head,std::vector<double> arm);

// void occgrid(float res, int width, int height, float x0, float y0, float z0, const char* data);
// void path(std::vector<double> posx,std::vector<double> posy,std::vector<double> posa);
//

// void marker3d(int num,
//   std::vector<double> types,
//   std::vector<double> posx,std::vector<double> posy,std::vector<double> posz,
//   std::vector<double> orix,std::vector<double> oriy,std::vector<double> oriz,std::vector<double> oriw,
//   std::vector<std::string> names,std::vector<double> scales,std::vector<double> colors
// );
// void marker_rectangle(int seq,float posx, float posy, float yaw,float lw, float rw, float th, float bh);
//
// void laserscan(int seq,  float angle_min, float angle_max,int n,float angle_increment,float* ranges, const char* linkname);
void camerainfo(int seq,int width, int height, double fov);
void camerainfo2(int seq,int width, int height, float fx, float fy, float cx, float cy, const char* frame_id);
// void rgbimage(int seq,int width, int height, const char* data,int enc);
// void jpgimage(int seq,const char* data);
// void depthimage(int seq,int width, int height, const char* data,int send_surface,int type);
// void send_odom(std::vector<double>pose);
//
//
// void send_jointstate(std::vector<std::string> jointnames,std::vector<double>jangles);//general purpose!!
// void webotslaserscancrop(int seq,  float angle_min, float angle_max,int n,
//   float angle_increment,float* ranges, const char* linkname,int ray_num);
//


#endif
