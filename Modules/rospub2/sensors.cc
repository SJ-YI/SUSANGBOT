#include "rospub.h"

void savergbimage(int width, int height, const char* enc, const char* data, const char* filename){
  sensor_msgs::Image msg;
  msg.header.seq=1;
  msg.header.stamp=ros::Time::now();
  msg.header.frame_id="head_rgbd_sensor_rgb_frame_sync";
  msg.width=width;msg.height=height;
  msg.encoding=enc;
  msg.is_bigendian=0;

  cv_bridge::CvImagePtr cv_ptr;
  // printf("Encoding:%s step:%\n",enc);

  if(strcmp(enc,"bgra8")==0) {
    // printf("BGRA8!!!!\n");
    msg.step = width*4;
    msg.data.insert(msg.data.end(), &data[0],&data[width*height*4]);
    try{cv_ptr=cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGRA8);}
    catch(cv_bridge::Exception& e){printf("Error %s\n",e.what());}

  }else{

    msg.step = width*3; //640*3
    msg.data.insert(msg.data.end(), &data[0],&data[width*height*3]);
    try{cv_ptr=cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);}
    catch(cv_bridge::Exception& e){printf("Error %s\n",e.what());}
  }
  printf("Image imported , w%d h%d\n",cv_ptr->image.cols,cv_ptr->image.rows );
  std::vector<int> cparams;
  cparams.push_back(CV_IMWRITE_PNG_COMPRESSION);
  cparams.push_back(9);
  // cparams.push_back(0);
  imwrite(filename,cv_ptr->image,cparams);
  printf("Log %s saved\n",filename);
}


void send_imagefrompng(int seq,int width, int height, const char* filename,int enc, int ch, const char* frame){;
  cv::Mat image=imread(filename, cv::IMREAD_COLOR);
  std_msgs::Header header;
  header.seq=seq;
  header.stamp=ros::Time::now();
  header.frame_id=frame;
  sensor_msgs::Image img_msg;

  //SJ: cannot change format to BGRA8 here

  // cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGRA8, image);
  cv_bridge::CvImage img_bridge;
  if(enc==2) img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, image);
  else img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, image);


  img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
  image_publisher[ch].publish(img_msg);
}


void send_image(int seq,int width, int height, double fov, const char* data,int enc, int ch, const char* frame){
  int i;
  sensor_msgs::Image msg;
  msg.header.seq=seq;
  msg.header.stamp=ros::Time::now();
  msg.header.frame_id=frame;
  msg.width=width;
  msg.height=height;
  msg.is_bigendian=0;
  switch (enc){
    case 0: //Standard RGB
      msg.encoding="rgb8";//for realsense
      msg.step = width*3; //640*3
      break;
    case 1: //Azure kinect RGB
      msg.encoding="bgra8";
      msg.step = width*4;
      break;
    case 2: //Webots 32 bit floating point encoding
      msg.encoding="32FC1";
      msg.step = width*4;
      break;
    case 3: //16 bit encoding for actual sensors
      msg.encoding="16UC1";
      msg.step = width*2;
      break;
    case 4: //16 bit encoding for actual sensors
      msg.encoding="bgr8";
      msg.step = width*3;
      break;
    case 5: //SPECIAL CASE, BGRA8 to BGR8 conversion
      msg.encoding="bgr8";
      msg.step = width*3;
      break;
  }
  if (enc==5){
    msg.data.resize(width*height*3);
    for(int x=0;x<msg.height;x++)
      for(int y=0;y<msg.width;y++){
        int index=x*msg.width+y;
        memcpy(&msg.data[index*3],&data[index*4],3);
      }
  }
  else msg.data.insert(msg.data.end(), &data[0],&data[msg.step*height]);
  image_publisher[ch].publish(msg);

  sensor_msgs::CameraInfo msg2;
  msg2.header.seq=seq;
  msg2.header.stamp=ros::Time::now();
  msg2.header.frame_id=frame;
  msg2.width=width;
  msg2.height=height;
  msg2.distortion_model="plumb_bob";
  double fx = ( (double) width )/2.0 / tan(fov/2.0);
  double cx=(width+1.0)/2.0;
  double cy=(height+1.0)/2.0;
  double P[12]={fx,0.0,cx,0.0,   0.0,fx,cy,0.0,    0.0,0.0,1.0,0.0};
  double K[9]={fx,0.0,cx,   0.0,fx,cy,    0.0,0.0,1.0};
  double R[9]={1.0,0.0,0.0,   0.0,1.0,0.0,   0.0,0.0,1.0};
  msg2.D.resize(5);
  for(i=0;i<5;i++) msg2.D[i]=0.0;
  for(i=0;i<12;i++) msg2.P[i]=P[i];
  for(i=0;i<9;i++) {msg2.K[i]=K[i];msg2.R[i]=R[i];}
  msg2.binning_x=0;msg2.binning_y=0;
  msg2.roi.x_offset=0;msg2.roi.y_offset=0;
  msg2.roi.height=0;msg2.roi.width=0;
  msg2.roi.do_rectify=false;
  camerainfo_publisher[ch].publish(msg2);
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
  camerainfo0_publisher.publish(msg);
}

void camerainfo2(int seq,int width, int height, float fx, float fy, float cx, float cy, const char* frame_id){
  sensor_msgs::CameraInfo msg;
  msg.header.seq=seq;
  msg.header.stamp=ros::Time::now();
  msg.header.frame_id=frame_id;
  msg.width=width;
  msg.height=height;
  msg.distortion_model="plumb_bob";

  double P[12]={fx,0.0,cx,0.0,   0.0,fy,cy,0.0,    0.0,0.0,1.0,0.0};
  double K[9]={fx,0.0,cx,   0.0,fy,cy,    0.0,0.0,1.0};
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
  camerainfo0_publisher.publish(msg);
}


void jpgimage(int seq,const char* data,int ch, const char* frame){
  // printf("JPGIMAGE:frame %s\n",frame);
  cv::Mat image0=cv::imread(data,CV_LOAD_IMAGE_COLOR);
  // sensor_msgs::ImagePtr msg=cv_bridge::CvImage(std_msgs::Header(), "rgb8",image0).toImageMsg();
  sensor_msgs::ImagePtr msg=cv_bridge::CvImage(std_msgs::Header(), "bgr8",image0).toImageMsg();
  msg->header.seq=seq;
  msg->header.stamp=ros::Time::now();
  msg->header.frame_id=frame;
  rgb_publisher.publish(msg);
}



void send_imageonly(int seq,int width, int height, const char* data,int enc, int ch, const char* frame){
  int i;
  sensor_msgs::Image msg;
  msg.header.seq=seq;
  msg.header.stamp=ros::Time::now();
  msg.header.frame_id=frame;
  msg.width=width;
  msg.height=height;
  msg.is_bigendian=0;
  switch (enc){
    case 0: //Standard RGB
      msg.encoding="rgb8";//for realsense
      msg.step = width*3; //640*3
      break;
    case 1: //Azure kinect RGB
      msg.encoding="bgra8";
      msg.step = width*4;
      break;
    case 2: //Webots 32 bit floating point encoding
      msg.encoding="32FC1";
      msg.step = width*4;
      break;
    case 3: //16 bit encoding for actual sensors
      msg.encoding="16UC1";
      msg.step = width*2;
      break;
    case 4: //Bgr8
      msg.encoding="bgr8";
      msg.step = width*3;
      break;
    case 5: //SPECIAL CASE, BGRA8 to BGR8 conversion
      msg.encoding="bgr8";
      msg.step = width*3;
      break;
    case 6: //
        msg.encoding="mono8";
        msg.step = width;
        break;
    }
  if (enc==5){
    msg.data.resize(width*height*3);
    for(int x=0;x<msg.height;x++)
      for(int y=0;y<msg.width;y++){
        int index=x*msg.width+y;
        memcpy(&msg.data[index*3],&data[index*4],3);
      }
  }else{
    msg.data.insert(msg.data.end(), &data[0],&data[msg.step*height]);
  }
  image_publisher[ch].publish(msg);
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


void laserscan(int seq,float angle_min, float angle_max, int n,float* ranges, const char* linkname, int ch){
  float angle_increment=(angle_max-angle_min)/((float) n-1);
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
  lidar_publisher[ch].publish(msg);
}

void webotslaserscan(int seq,float angle_min, float angle_max, int n,float* ranges, const char* linkname, int ch){
  float angle_increment=(angle_max-angle_min)/((float) n-1);
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
  msg.scan_time=0.2;
  msg.ranges.resize(n);
  for(int i=0;i<n;i++) msg.ranges[i]=ranges[n-1-i];
  lidar_publisher[ch].publish(msg);
}



















//Cleaned up, general purpose rospub library for lua
//
// void laserscan(int seq,float angle_min, float angle_max, int n,
//   float angle_increment,float* ranges, const char* linkname){
//   sensor_msgs::LaserScan msg;
//   msg.header.seq=seq;
//   msg.header.stamp=ros::Time::now();
//   msg.header.frame_id=linkname;
//   msg.angle_min=angle_min;
//   msg.angle_max=angle_max;
//   msg.angle_increment=angle_increment;
//   msg.range_min = 0.1;
//   msg.range_max = 6.0;
//   msg.time_increment=0.0;
//   msg.ranges.resize(n);
//   for(int i=0;i<n;i++) msg.ranges[i]=ranges[i];
//   lidar_publisher.publish(msg);
// }


// void jpgimage(int seq,const char* data){
//   // printf("Image read trying : %s\n",data);
//   cv::Mat image0=cv::imread(data,CV_LOAD_IMAGE_COLOR);
//   cv::Mat image;
//   cv::resize(image0,image,{608,608});
//   // printf("Image read done!");
//   sensor_msgs::ImagePtr msg=cv_bridge::CvImage(std_msgs::Header(), "bgr8",image).toImageMsg();
//   msg->header.seq=seq;
//   msg->header.stamp=ros::Time::now();
//   rgb_publisher.publish(msg);
// }
//
