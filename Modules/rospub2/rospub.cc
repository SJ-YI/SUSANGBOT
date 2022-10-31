#include "rospub.h"

ros::Publisher teleopvel_publisher;
ros::Publisher cmdvel_publisher;

ros::Publisher jointstate_publisher;

ros::Publisher basegoal_publisher;
ros::Publisher navigation_status_publisher;

ros::Publisher posereset_publisher;
ros::Publisher odom_publisher;
ros::Publisher mapcmd_publisher;

ros::Publisher markermatrix_publisher; //3D marker objects
ros::Publisher markerarray2_publisher; //3D marker objects
ros::Publisher markerarray_publisher;
ros::Publisher rgb_publisher;

ros::Publisher lidar_publisher[MAX_LIDAR_NO];
ros::Publisher image_publisher[MAX_CAMERA_NO];
ros::Publisher camerainfo_publisher[MAX_CAMERA_NO];
ros::Publisher path_publisher[MAX_PATH_NO];
ros::Publisher occgrid_publisher[MAX_OCCGRID_NO];



ros::Publisher camerainfo0_publisher; //for ARC
ros::Publisher jointactiongoalpublisher; //Arm jont target publisher
ros::Publisher gripperactiongoalpublisher;


ros::Publisher jointtrajectorypublisher;
ros::Publisher jointtrajectorypublisher2;
ros::Publisher jointtrajectorypublisher3;


ros::Publisher displayjpg_publisher; //for displaying JPG to the head display



void init_publishers(){
  int i;

  rgb_publisher= rosNode->advertise<sensor_msgs::Image>("/pnu/rgb_rect_raw_input", 1, true);
  jointstate_publisher= rosNode->advertise<sensor_msgs::JointState>("/joint_states", 1, true);
  basegoal_publisher= rosNode->advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, true);
  navigation_status_publisher= rosNode->advertise<std_msgs::Int32>("/navigation_status", 1, true);

  teleopvel_publisher = rosNode->advertise<geometry_msgs::Twist>("/teleop_vel", 1, true);
  cmdvel_publisher = rosNode->advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
  odom_publisher= rosNode->advertise<nav_msgs::Odometry>("/odom", 1, true);

  mapcmd_publisher= rosNode->advertise<std_msgs::Int32>("/mapcmd", 1, true);
  posereset_publisher= rosNode->advertise<geometry_msgs::PoseWithCovarianceStamped>("/laser_2d_correct_pose", 1, true);

  markerarray_publisher= rosNode->advertise<visualization_msgs::MarkerArray>("/pnu/markerarray", 1, true);
  markerarray2_publisher= rosNode->advertise<visualization_msgs::MarkerArray>("/pnu/markerarray2", 1, true);
  markermatrix_publisher= rosNode->advertise<visualization_msgs::MarkerArray>("/pnu/markermatrix", 1, true);

  camerainfo0_publisher= rosNode->advertise<sensor_msgs::CameraInfo>("/camerainfo", 1, true);
  jointactiongoalpublisher= rosNode->advertise<control_msgs::FollowJointTrajectoryActionGoal>("/follow_joint_trajectory/goal", 1, true);
  gripperactiongoalpublisher= rosNode->advertise<control_msgs::FollowJointTrajectoryActionGoal>("/grp_follow_joint_trajectory/goal", 1, true);

  jointtrajectorypublisher=rosNode->advertise<trajectory_msgs::JointTrajectory>("/joint_group_position_controller/command", 1, true);
  jointtrajectorypublisher2=rosNode->advertise<trajectory_msgs::JointTrajectory>("/joint_finger_position_controller/command", 1, true);
  jointtrajectorypublisher3=rosNode->advertise<trajectory_msgs::JointTrajectory>("/pnu/jointtrajectory", 1, true);
  // pointcloudpublisher=rosNode->advertise<sensor_msgs/PointCloud2>("/pnu/pointcloud", 1, true);

  for(i=0;i<lidar_names.size();i++) lidar_publisher[i]=rosNode->advertise<sensor_msgs::LaserScan>(lidar_names[i].c_str(), 1, true);
  for(i=0;i<path_names.size();i++) path_publisher[i]= rosNode->advertise<nav_msgs::Path>(path_names[i].c_str(), 1, true);
  for(i=0;i<occgrid_names.size();i++) occgrid_publisher[i]= rosNode->advertise<nav_msgs::OccupancyGrid>(occgrid_names[i].c_str(), 1, true);
  for(i=0;i<camera_names.size();i++){
    char buf[255];sprintf(buf,"%s/camera_info",camera_names[i].c_str());
    image_publisher[i]= rosNode->advertise<sensor_msgs::Image>(camera_names[i], 1, true);
    camerainfo_publisher[i]= rosNode->advertise<sensor_msgs::CameraInfo>(buf, 1, true);
  }

  displayjpg_publisher=rosNode->advertise<std_msgs::String>("/display_jpg", 1, true);

}

void send_int32(int data, ros::Publisher pub){std_msgs::Int32 msg;msg.data=data;pub.publish(msg);}
void send_bool(int data, ros::Publisher pub){std_msgs::Bool msg;msg.data=data;pub.publish(msg);}
void send_twist(double x, double y, double a,ros::Publisher pub){
  geometry_msgs::Twist msg;
  msg.linear.x=x;msg.linear.y=y;msg.linear.z=0.0;
  msg.angular.x=0.0;msg.angular.y=0.0;msg.angular.z=a;
  pub.publish(msg);
}
