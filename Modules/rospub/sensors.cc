#include "rospub.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//Cleaned up, general purpose rospub library for lua

void laserscan(int seq,float angle_min, float angle_max, int n,
  float angle_increment,float* ranges, const char* linkname){
  sensor_msgs::LaserScan msg;
  msg.header.seq=seq;
  msg.header.stamp=ros::Time::now();
  msg.header.frame_id=linkname;
  msg.angle_min=angle_min;
  msg.angle_max=angle_max;
  msg.angle_increment=angle_increment;
  msg.range_min = 0.1;
  msg.range_max = 6.0;
  msg.time_increment=0.0;
  msg.ranges.resize(n);
  for(int i=0;i<n;i++) msg.ranges[i]=ranges[i];
  lidar_publisher.publish(msg);
}


void camerainfo(int seq,int width, int height, double fov){
  sensor_msgs::CameraInfo msg;
  msg.header.seq=seq;
  msg.header.stamp=ros::Time::now();
  msg.header.frame_id="head_rgbd_sensor_rgb_frame";
  msg.width=width;
  msg.height=height;
  msg.distortion_model="plumb_bob";

  // double fx=554.3827128226441; //focal length in pixels, width=640
  double fx = ( (double) width )/2.0 / tan(fov/2.0);
//fov = atan(640/2/554)*2.0 : 60 degree

  double cx=(msg.width+1.0)/2.0;
  double cy=(msg.height+1.0)/2.0;
  double P[12]={fx,0.0,cx,0.0,   0.0,fx,cy,0.0,    0.0,0.0,1.0,0.0};
  double K[9]={fx,0.0,cx,   0.0,fx,cy,    0.0,0.0,1.0};
  double R[9]={1.0,0.0,0.0,   0.0,1.0,0.0,   0.0,0.0,1.0};
  int i;
  msg.D.resize(5);
  for(i=0;i<5;i++) msg.D[i]=0.0;
  for(i=0;i<12;i++) msg.P[i]=P[i];
  for(i=0;i<9;i++) {msg.K[i]=K[i];msg.R[i]=R[i];}

  msg.binning_x=0;
  msg.binning_y=0;
  msg.roi.x_offset=0;
  msg.roi.y_offset=0;
  msg.roi.height=0;
  msg.roi.width=0;
  msg.roi.do_rectify=false;
  rgbinfo_publisher.publish(msg);
  depthinfo_publisher.publish(msg);
}

void rgbimage(int seq,int width, int height, const char* data,int enc){
  sensor_msgs::Image msg;
  msg.header.seq=seq;
  msg.header.stamp=ros::Time::now();
  msg.header.frame_id="head_rgbd_sensor_rgb_frame_sync";
  msg.width=width;
  msg.height=height;
  if (enc==1) {
    msg.encoding="bgra8";//for azure kinect
    msg.step = width*4;
  }
  else{
    if (enc==0){
      msg.encoding="rgb8";//for realsense
      msg.step = width*3; //640*3
    }
    else{
      msg.encoding="bgr8";//for simulated primesense
      msg.step = width*3; //640*3
    }
  }

  msg.is_bigendian=0;
  // msg.data.resize(width*height*3);
  // memcpy(msg.data,data,width*height*3);
  //  msg.data.insert(msg.data.end(), &data[0],&data[width*height*3]);
  msg.data.insert(msg.data.end(), &data[0],&data[msg.step*height]);
  rgb_publisher.publish(msg);
}

void jpgimage(int seq,const char* data){
  // printf("Image read trying : %s\n",data);
  cv::Mat image0=cv::imread(data,CV_LOAD_IMAGE_COLOR);
  // cv::Mat image;
  // cv::resize(image0,image,{608,608});
  // printf("Image read done!");
  sensor_msgs::ImagePtr msg=cv_bridge::CvImage(std_msgs::Header(), "bgr8",image0).toImageMsg();
  msg->header.seq=seq;
  msg->header.stamp=ros::Time::now();
  rgb_publisher.publish(msg);
}

void depthimage(int seq,int width, int height, const char* data,int send_surface,int type){
  sensor_msgs::Image msg;
  msg.header.seq=seq;
  msg.header.stamp=ros::Time::now();
  msg.header.frame_id="head_rgbd_sensor_rgb_frame_sync";
  msg.width=width;
  msg.height=height;
  if (type==0){ //webots
    msg.encoding="32FC1"; //32 bit float encoding for webots
    msg.is_bigendian=0;
    msg.step = width*4;
    msg.data.insert(msg.data.end(), &data[0],&data[width*height*4]);
    depth_publisher.publish(msg);
    if (send_surface>0) depth2_publisher.publish(msg);
  }else{
    msg.encoding="16UC1";//12bit encoding for actual sensor
    msg.is_bigendian=0;
    msg.step = width*2;
    msg.data.insert(msg.data.end(), &data[0],&data[width*height*2]);
    depth2_publisher.publish(msg);
  }
}
