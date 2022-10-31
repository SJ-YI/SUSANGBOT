#ifndef ROSPUB_H_
#define ROSPUB_H_

#include "ros/ros.h"
#include "tf/transform_broadcaster.h"

// #include "sensor_msgs/PointCloud2.h"
// #include "sensor_msgs/LaserScan.h"
// #include "trajectory_msgs/JointTrajectory.h"
// #include "sensor_msgs/Imu.h"
// #include "sensor_msgs/Joy.h"
// #include "nav_msgs/Odometry.h"
// #include "object_recognition_msgs/TableArray.h"
#include "visualization_msgs/MarkerArray.h"
// #include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include <sensor_msgs/NavSatFix.h>




// #include "std_msgs/String.h"
// #include "std_msgs/Float32MultiArray.h"

//
// #include <sensor_msgs/JointState.h>
// #include <sensor_msgs/LaserScan.h>
// #include <sensor_msgs/CameraInfo.h>
// #include <sensor_msgs/Image.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <nav_msgs/OccupancyGrid.h>
// #include <nav_msgs/Path.h>
//
// #include <actionlib/client/simple_action_client.h>
// #include <control_msgs/FollowJointTrajectoryAction.h>
// #include <control_msgs/FollowJointTrajectoryGoal.h>
// #include <geometry_msgs/PoseWithCovarianceStamped.h>
//
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/opencv.hpp> //for MAT

extern std::unique_ptr<ros::NodeHandle> rosNode;


extern ros::Publisher gps_fix_publisher;
extern ros::Publisher markerarray2_publisher; //objects (cylinder)
void init_publishers();
void markermatrix(float x0,float y0,float a0, int mapx, int mapy, float scale, float* heights, float basez, float z0, const char* frame,int flip_y);


#endif
