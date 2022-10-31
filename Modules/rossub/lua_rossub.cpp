#include "rossub.h"
#include <tf/transform_listener.h>
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/opencv.hpp>

template <typename Type>
class _Subscriber
{
public:
  _Subscriber(std::shared_ptr<ros::NodeHandle> rosNode, const char *topic_name)
  {
    _rosNode = rosNode;
    so = ros::SubscribeOptions::create<Type>(
        topic_name,
        100,
        boost::bind(&_Subscriber<Type>::OnRosMsg, this, _1),
        ros::VoidPtr(), &this->_rosQueue);
    _rosSub = _rosNode->subscribe(so);
  }
  void start(){
      this->_rosQueueThread = std::thread(std::bind(&_Subscriber<Type>::QueueThread, this));
  }
  bool checkMsg(){return _new;}
  Type getMsg()  {
    mtx_lock.lock();
    Type returnMsg = _newMsg;
    _new = false;
    mtx_lock.unlock();
    return returnMsg;
  }
  void OnRosMsg(const typename Type::ConstPtr &msg)
  {
    mtx_lock.lock();
    _new = true;
    _newMsg = *msg;
    mtx_lock.unlock();
  }
  void QueueThread()
  {
    static const double timeout = 0.01;
    while(_rosNode->ok()){
      this->_rosQueue.callAvailable(ros::WallDuration(timeout));
    }
  }
private:
  std::shared_ptr<ros::NodeHandle> _rosNode;
  ros::SubscribeOptions so;
  ros::Subscriber _rosSub;
  ros::CallbackQueue _rosQueue;
  std::thread _rosQueueThread;
  Type _newMsg;
  bool _new = false;
  std::mutex mtx_lock;
};
std::vector<std::shared_ptr<void>> subList;
std::shared_ptr<ros::NodeHandle> rosNode;
std::shared_ptr<tf::TransformListener> listener;

static void lua_pushvector(lua_State *L, std::vector<double> v, int n) {
	lua_createtable(L, n, 0);
	for (int i = 0; i < n; i++) {
		lua_pushnumber(L, v[i]);
		lua_rawseti(L, -2, i+1);
	}
}
static void lua_pusharray(lua_State *L, float* v, int n) {
	lua_createtable(L, n, 0);
	for (int i = 0; i < n; i++) {
		lua_pushnumber(L, v[i]);
		lua_rawseti(L, -2, i+1);
	}
}

static void lua_pushstringvector(lua_State *L, std::vector<std::string> v, int n) {
	lua_createtable(L, n, 0);
	for (int i = 0; i < n; i++) {
		lua_pushstring(L, v[i].c_str());
		lua_rawseti(L, -2, i+1);
	}
}

static int lua_init(lua_State *L){
  const char *ros_node_name = lua_tostring(L, 1);
  if(!ros::isInitialized())  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, ros_node_name, ros::init_options::NoSigintHandler);

  }
  listener.reset(new tf::TransformListener() );
  rosNode.reset(new ros::NodeHandle(ros_node_name));
  return 0;
}

static int lua_subscribeTwist(lua_State *L){
  const char *topic_name = lua_tostring(L, 1);
  std::shared_ptr<_Subscriber<geometry_msgs::Twist>> sub = std::shared_ptr<_Subscriber<geometry_msgs::Twist>>(new _Subscriber<geometry_msgs::Twist>(rosNode, topic_name));
  sub->start();subList.push_back(sub);
  lua_pushnumber(L, subList.size() - 1);
  return 1;
}

static int lua_checkTwist(lua_State *L){
  int idx = lua_tonumber(L, 1);
  std::shared_ptr<_Subscriber<geometry_msgs::Twist>> sub = std::static_pointer_cast<_Subscriber<geometry_msgs::Twist>>(subList[idx]);
  if (sub->checkMsg()){
    geometry_msgs::Twist ps = sub->getMsg();
    lua_createtable(L,6,0);
    lua_pushnumber(L,(float)ps.linear.x );
    lua_rawseti(L,-2,1);
    lua_pushnumber(L,(float)ps.linear.y );
    lua_rawseti(L,-2,2);
    lua_pushnumber(L,(float)ps.linear.z );
    lua_rawseti(L,-2,3);
    lua_pushnumber(L,(float)ps.angular.x );
    lua_rawseti(L,-2,4);
    lua_pushnumber(L,(float)ps.angular.y );
    lua_rawseti(L,-2,5);
    lua_pushnumber(L,(float)ps.angular.z );
    lua_rawseti(L,-2,6);
    return 1;
  }else return 0;
}

static int lua_subscribeJointTrajectory(lua_State *L){
  const char *topic_name = lua_tostring(L, 1);
  std::shared_ptr<_Subscriber<trajectory_msgs::JointTrajectory>> sub = std::shared_ptr<_Subscriber<trajectory_msgs::JointTrajectory>>(new _Subscriber<trajectory_msgs::JointTrajectory>(rosNode, topic_name));
  sub->start();subList.push_back(sub);
  lua_pushnumber(L, subList.size() - 1);
  return 1;
}

static int lua_checkJointTrajectory(lua_State *L){
  int idx = lua_tonumber(L, 1);
  std::shared_ptr<_Subscriber<trajectory_msgs::JointTrajectory>> sub = std::static_pointer_cast<_Subscriber<trajectory_msgs::JointTrajectory>>(subList[idx]);
  if (sub->checkMsg()){
    trajectory_msgs::JointTrajectory ps = sub->getMsg();

    double arm1=ps.points[0].positions[0];
    double arm2=ps.points[0].positions[1];
    double arm3=ps.points[0].positions[2];
    double arm4=ps.points[0].positions[3];
    double grip=ps.points[0].positions[4];
    lua_pushnumber(L,arm1);
    lua_pushnumber(L,arm2);
    lua_pushnumber(L,arm3);
    lua_pushnumber(L,arm4);
    lua_pushnumber(L,grip);
    return 5;
  }else return 0;
}


static int lua_subscribeJointState(lua_State *L){
  const char *topic_name = lua_tostring(L, 1);
  std::shared_ptr<_Subscriber<sensor_msgs::JointState>> sub = std::shared_ptr<_Subscriber<sensor_msgs::JointState>>(new _Subscriber<sensor_msgs::JointState>(rosNode, topic_name));
  sub->start();subList.push_back(sub);
  lua_pushnumber(L, subList.size() - 1);
  return 1;
}

static int lua_checkJointState(lua_State *L){
  int idx = lua_tonumber(L, 1);
  std::shared_ptr<_Subscriber<sensor_msgs::JointState>> sub = std::static_pointer_cast<_Subscriber<sensor_msgs::JointState>>(subList[idx]);
  if (sub->checkMsg()){
    sensor_msgs::JointState ps = sub->getMsg();
    lua_pushstringvector(L,ps.name,ps.name.size());
    lua_pushvector(L,ps.position,ps.position.size());
    return 2;
  }else return 0;
}


static int lua_subscribeFloat32MultiArray(lua_State *L){
  const char *topic_name = lua_tostring(L, 1);
  std::shared_ptr<_Subscriber<std_msgs::Float32MultiArray>> sub = std::shared_ptr<_Subscriber<std_msgs::Float32MultiArray>>(new _Subscriber<std_msgs::Float32MultiArray>(rosNode, topic_name));
  sub->start();subList.push_back(sub);
  lua_pushnumber(L, subList.size() - 1);
  return 1;
}

static int lua_checkFloat32MultiArray(lua_State *L){
  int idx = lua_tonumber(L, 1);
  float buf[32];
  std::shared_ptr<_Subscriber<std_msgs::Float32MultiArray>> sub = std::static_pointer_cast<_Subscriber<std_msgs::Float32MultiArray>>(subList[idx]);
  if (sub->checkMsg()){
    std_msgs::Float32MultiArray ps = sub->getMsg();
    int size=ps.layout.dim[0].size;
    std::vector<double> data;
    for(int i=0;i<size;i++) data.push_back(ps.data[i]);
    lua_pushvector(L,data,size);
    return 1;
  }else return 0;
}

static int lua_checkTF(lua_State *L){
  const char *mother_frame = lua_tostring(L, 1);
  const char *child_frame = lua_tostring(L, 2);
  tf::StampedTransform transform;
  try{
    listener->lookupTransform(mother_frame,child_frame, ros::Time(0), transform);
  } catch (tf::TransformException ex){
    // printf("%s",ex.what());
    return 0;
  }
  float x=transform.getOrigin().x();
  float y=transform.getOrigin().y();
  float z=transform.getOrigin().z();
  tf::Quaternion q=transform.getRotation();
  tf::Matrix3x3 m(q);
  double roll,pitch,yaw;
  m.getRPY(roll,pitch,yaw);

  lua_createtable(L, 6, 0);
	lua_pushnumber(L, x);
	lua_rawseti(L, -2, 1);
  lua_pushnumber(L, y);
  lua_rawseti(L, -2, 2);
  lua_pushnumber(L, z);
  lua_rawseti(L, -2, 3);
  lua_pushnumber(L, roll);
  lua_rawseti(L, -2, 4);
  lua_pushnumber(L, pitch);
  lua_rawseti(L, -2, 5);
  lua_pushnumber(L, yaw);
  lua_rawseti(L, -2, 6);
  return 1;
}

static int lua_subscribePoseStamped(lua_State *L){
  const char *topic_name = lua_tostring(L, 1);
  std::shared_ptr<_Subscriber<geometry_msgs::PoseStamped>> sub = std::shared_ptr<_Subscriber<geometry_msgs::PoseStamped>>(new _Subscriber<geometry_msgs::PoseStamped>(rosNode, topic_name));
  sub->start();subList.push_back(sub);
  lua_pushnumber(L, subList.size() - 1);
  return 1;
}

static int lua_checkPoseStamped(lua_State *L){
  int idx = lua_tonumber(L, 1);
  int update_tr=luaL_optnumber(L,2,0);
	float pose[3];
  std::shared_ptr<_Subscriber<geometry_msgs::PoseStamped>> sub = std::static_pointer_cast<_Subscriber<geometry_msgs::PoseStamped>>(subList[idx]);
  if (sub->checkMsg()){
    float orientation[4];
    geometry_msgs::PoseStamped ps=sub->getMsg();
    pose[0] = (float)ps.pose.position.x;
    pose[1] = (float)ps.pose.position.y;
    orientation[0] = (float)ps.pose.orientation.x;
    orientation[1] = (float)ps.pose.orientation.y;
    orientation[2] = (float)ps.pose.orientation.z;
    orientation[3] = (float)ps.pose.orientation.w;
    float angle = atan2(orientation[2],orientation[3])*2.0;
    pose[2]=angle;//change to 2D pose (x,y,theta)
		lua_pusharray(L, pose,3);
		return 1;
  }else return 0;
}

static int lua_checkPoseStamped3D(lua_State *L){
  int idx = lua_tonumber(L, 1);
  int update_tr=luaL_optnumber(L,2,0);
	float pose[6];
  std::shared_ptr<_Subscriber<geometry_msgs::PoseStamped>> sub = std::static_pointer_cast<_Subscriber<geometry_msgs::PoseStamped>>(subList[idx]);
  if (sub->checkMsg()){
    float orientation[4];
    geometry_msgs::PoseStamped ps=sub->getMsg();
    pose[0] = (float)ps.pose.position.x;
    pose[1] = (float)ps.pose.position.y;
    pose[2] = (float)ps.pose.position.z;
    orientation[0] = (float)ps.pose.orientation.x;
    orientation[1] = (float)ps.pose.orientation.y;
    orientation[2] = (float)ps.pose.orientation.z;
    orientation[3] = (float)ps.pose.orientation.w;
    float angle = atan2(orientation[2],orientation[3])*2.0;
    pose[4]=0.0;pose[5]=0.0;
    pose[5]=angle;//change to 2D pose (x,y,theta)
		lua_pusharray(L, pose,6);
		return 1;
  }else return 0;
}

static int lua_subscribeWrenchStamped(lua_State *L){
  const char *topic_name = lua_tostring(L, 1);
  std::shared_ptr<_Subscriber<geometry_msgs::WrenchStamped>> sub = std::shared_ptr<_Subscriber<geometry_msgs::WrenchStamped>>(new _Subscriber<geometry_msgs::WrenchStamped>(rosNode, topic_name));
  sub->start();subList.push_back(sub);
  lua_pushnumber(L, subList.size() - 1);
  return 1;
}

static int lua_checkWrenchStamped(lua_State *L){
  int idx = lua_tonumber(L, 1);
  std::shared_ptr<_Subscriber<geometry_msgs::WrenchStamped>> sub = std::static_pointer_cast<_Subscriber<geometry_msgs::WrenchStamped>>(subList[idx]);
  if (sub->checkMsg()){
    geometry_msgs::WrenchStamped ps=sub->getMsg();

    float force[3],torque[3];
    force[0]=(float)ps.wrench.force.x;
    force[1]=(float)ps.wrench.force.y;
    force[2]=(float)ps.wrench.force.z;

    torque[0]=(float)ps.wrench.torque.x;
    torque[1]=(float)ps.wrench.torque.y;
    torque[2]=(float)ps.wrench.torque.z;

		lua_pusharray(L, force,3);
    lua_pusharray(L, torque,3);
		return 2;
    // printf("force:%.2f %.2f %.2f\n",(float)ps.wrench.force.x,(float)ps.wrench.force.y,(float)ps.wrench.force.z);
    // return 0;
  }else return 0;
}


static int lua_subscribeOccupancyGrid(lua_State *L){
  const char *topic_name = lua_tostring(L, 1);
  std::shared_ptr<_Subscriber<nav_msgs::OccupancyGrid>> sub = std::shared_ptr<_Subscriber<nav_msgs::OccupancyGrid>>(new _Subscriber<nav_msgs::OccupancyGrid>(rosNode, topic_name));
  sub->start();subList.push_back(sub);
  lua_pushnumber(L, subList.size() - 1);
  return 1;
}
static int lua_checkOccupancyGrid(lua_State *L){
  int idx = lua_tonumber(L, 1);
  std::shared_ptr<_Subscriber<nav_msgs::OccupancyGrid>> sub = std::static_pointer_cast<_Subscriber<nav_msgs::OccupancyGrid>>(subList[idx]);
  if (sub->checkMsg()){
    nav_msgs::OccupancyGrid ps = sub->getMsg();
    lua_pushnumber(L,ps.info.resolution);
    lua_pushnumber(L,ps.info.width);
    lua_pushnumber(L,ps.info.height);
    lua_pushnumber(L,ps.info.origin.position.x);
    lua_pushnumber(L,ps.info.origin.position.y);
    lua_pushnumber(L,ps.info.origin.position.z);
    lua_pushlstring(L, (char*) &ps.data[0], ps.data.size());
    return 7;
  }
  return 0;
}

static int lua_subscribeOdometry(lua_State *L){
  const char *topic_name = lua_tostring(L, 1);
  std::shared_ptr<_Subscriber<nav_msgs::Odometry>> sub =
    std::shared_ptr<_Subscriber<nav_msgs::Odometry>>(new _Subscriber<nav_msgs::Odometry>(rosNode, topic_name));
  sub->start();subList.push_back(sub);
  lua_pushnumber(L, subList.size() - 1);
  return 1;
}

static int lua_checkOdometry(lua_State *L){
  int idx = lua_tonumber(L, 1);
  std::shared_ptr<_Subscriber<nav_msgs::Odometry>> sub = std::static_pointer_cast<_Subscriber<nav_msgs::Odometry>>(subList[idx]);
  if (sub->checkMsg()){
    nav_msgs::Odometry ps = sub->getMsg();
    float odom[3];
    odom[0]=(float)ps.pose.pose.position.x;
    odom[1]=(float)ps.pose.pose.position.y;
    odom[2]=2*atan2(ps.pose.pose.orientation.z,ps.pose.pose.orientation.w);
    lua_pusharray(L, odom, 3);

    float etc=(float)ps.pose.pose.position.z;
    float etc2=(float)ps.pose.pose.orientation.x;
    float etc3=(float)ps.pose.pose.orientation.y;
    lua_pushnumber(L, etc);
    lua_pushnumber(L, etc2);
    lua_pushnumber(L, etc3);
    return 4;
  }else return 0;
}


static int lua_subscribeInt32(lua_State *L){
  const char *topic_name = lua_tostring(L, 1);
  std::shared_ptr<_Subscriber<std_msgs::Int32>> sub = std::shared_ptr<_Subscriber<std_msgs::Int32>>(new _Subscriber<std_msgs::Int32>(rosNode, topic_name));
  sub->start();subList.push_back(sub);
  lua_pushnumber(L, subList.size() - 1);
  return 1;
}

static int lua_checkInt32(lua_State *L){
  int idx = lua_tonumber(L, 1);
  std::shared_ptr<_Subscriber<std_msgs::Int32>> sub = std::static_pointer_cast<_Subscriber<std_msgs::Int32>>(subList[idx]);
  if (sub->checkMsg()){
    std_msgs::Int32 ps = sub->getMsg();
    lua_pushnumber(L,ps.data);
    return 1;
  }else return 0;
}

static int lua_subscribeFloat64(lua_State *L){
  const char *topic_name = lua_tostring(L, 1);
  std::shared_ptr<_Subscriber<std_msgs::Float64>> sub =
    std::shared_ptr<_Subscriber<std_msgs::Float64>>(new _Subscriber<std_msgs::Float64>(rosNode, topic_name));
  sub->start();subList.push_back(sub);
  lua_pushnumber(L, subList.size() - 1);
  return 1;
}

static int lua_checkFloat64(lua_State *L){
  int idx = lua_tonumber(L, 1);
  std::shared_ptr<_Subscriber<std_msgs::Float64>> sub =
    std::static_pointer_cast<_Subscriber<std_msgs::Float64>>(subList[idx]);
  if (sub->checkMsg()){
    std_msgs::Float64 ps = sub->getMsg();
    lua_pushnumber(L,ps.data);
    return 1;
  }else return 0;
}



// static int lua_subscribeCompressedImage(lua_State *L){
//   const char *topic_name = lua_tostring(L, 1);
//   std::shared_ptr<_Subscriber<sensor_msgs::CompressedImage>> sub = std::shared_ptr<_Subscriber<sensor_msgs::CompressedImage>>(new _Subscriber<sensor_msgs::CompressedImage>(rosNode, topic_name));
//   sub->start();subList.push_back(sub);
//   lua_pushnumber(L, subList.size() - 1);
//   return 1;
// }
//
// static int lua_checkCompressedImage(lua_State *L){
//   int idx = lua_tonumber(L, 1);
//   std::shared_ptr<_Subscriber<sensor_msgs::CompressedImage>> sub = std::static_pointer_cast<_Subscriber<sensor_msgs::CompressedImage>>(subList[idx]);
//   if (sub->checkMsg()){
//     sensor_msgs::CompressedImage ps=sub->getMsg();
//     cv_bridge::CvImagePtr cv_ptr;
//     try{cv_ptr=cv_bridge::toCvCopy(ps, sensor_msgs::image_encodings::RGB8);
//     }catch(cv_bridge::Exception& e){printf("Error %s\n",e.what());}
//     lua_pushnumber(L, cv_ptr->image.cols);
//     lua_pushnumber(L, cv_ptr->image.rows);
//     lua_pushlstring(L, (char*) cv_ptr->image.data, cv_ptr->image.cols*cv_ptr->image.rows*3);
//     return 3;
//   }
//   return 0;
// }

static int lua_subscribeImage(lua_State *L){
  const char *topic_name = lua_tostring(L, 1);
  std::shared_ptr<_Subscriber<sensor_msgs::Image>> sub = std::shared_ptr<_Subscriber<sensor_msgs::Image>>(new _Subscriber<sensor_msgs::Image>(rosNode, topic_name));
  sub->start();subList.push_back(sub);
  lua_pushnumber(L, subList.size() - 1);
  return 1;
}

static int lua_checkImage(lua_State *L){
  int idx = lua_tonumber(L, 1);
  std::shared_ptr<_Subscriber<sensor_msgs::Image>> sub = std::static_pointer_cast<_Subscriber<sensor_msgs::Image>>(subList[idx]);
  if (sub->checkMsg()){
    sensor_msgs::Image ps=sub->getMsg();
    int datasize=ps.width*ps.height*3;
    float tstamp=ps.header.stamp.toSec();

    lua_pushnumber(L, ps.width);
    lua_pushnumber(L, ps.height);
    lua_pushlstring(L, ps.encoding.c_str(),ps.encoding.size());
    lua_pushlstring(L, (char*) &ps.data[0], ps.data.size());
    lua_pushnumber(L, tstamp);
    lua_pushlstring(L, ps.header.frame_id.c_str(), ps.header.frame_id.size());

    return 6;
  }else return 0;
}

static int lua_subscribeBattery(lua_State *L){
  const char *topic_name = lua_tostring(L, 1);
  std::shared_ptr<_Subscriber<sensor_msgs::BatteryState>> sub = std::shared_ptr<_Subscriber<sensor_msgs::BatteryState>>(new _Subscriber<sensor_msgs::BatteryState>(rosNode, topic_name));
  sub->start();subList.push_back(sub);
  lua_pushnumber(L, subList.size() - 1);
  return 1;
}

static int lua_checkBattery(lua_State *L){
  int idx = lua_tonumber(L, 1);
  std::shared_ptr<_Subscriber<sensor_msgs::BatteryState>> sub = std::static_pointer_cast<_Subscriber<sensor_msgs::BatteryState>>(subList[idx]);
  if (sub->checkMsg()){
    sensor_msgs::BatteryState ps = sub->getMsg();
    lua_pushnumber(L, ps.voltage);
    return 1;
  }else return 0;
}

static int lua_subscribeLaserScan(lua_State *L){
  const char *topic_name = lua_tostring(L, 1);
  std::shared_ptr<_Subscriber<sensor_msgs::LaserScan>> sub = std::shared_ptr<_Subscriber<sensor_msgs::LaserScan>>(new _Subscriber<sensor_msgs::LaserScan>(rosNode, topic_name));
  sub->start();subList.push_back(sub);
  lua_pushnumber(L, subList.size() - 1);
  return 1;
}

static int lua_checkLaserScan(lua_State *L){
  int idx = lua_tonumber(L, 1);
  std::shared_ptr<_Subscriber<sensor_msgs::LaserScan>> sub = std::static_pointer_cast<_Subscriber<sensor_msgs::LaserScan>>(subList[idx]);
  if (sub->checkMsg()){
    sensor_msgs::LaserScan ps = sub->getMsg();
    lua_pushnumber(L,ps.angle_min);
    lua_pushnumber(L,ps.angle_max);
    lua_pushnumber(L,ps.angle_increment);
    lua_pushnumber(L,ps.time_increment);
    lua_pushnumber(L,ps.scan_time);
    lua_pushnumber(L,ps.range_min);
    lua_pushnumber(L,ps.range_max);
    lua_pusharray(L, &ps.ranges[0], ps.ranges.size());
    return 8;
  }
  return 0;
}

static int lua_checkLaserScanPField(lua_State *L){
  int idx = lua_tonumber(L, 1);
  int pfield_div=lua_tonumber(L, 2);
  float lidar_x0=lua_tonumber(L, 3);
  float dist_buf[32];
  for(int i=0;i<32;i++) dist_buf[i]=3.0;
  std::shared_ptr<_Subscriber<sensor_msgs::LaserScan>> sub = std::static_pointer_cast<_Subscriber<sensor_msgs::LaserScan>>(subList[idx]);
  if (sub->checkMsg()){
    sensor_msgs::LaserScan ps = sub->getMsg();
    float pfield_div_angle=(2.0*3.14159265/ ( (float) pfield_div) );

    for(int i=0;i<ps.ranges.size();i++){
      float angle=ps.angle_min + i*ps.angle_increment;
      float range=ps.ranges[i];
      float dx=range*cos(angle)+lidar_x0;
      float dy=range*sin(angle);
      float range2=sqrt(dx*dx+dy*dy);
      float angle2=atan2(dy,dx);
      if(angle2<0) angle2+=2.0*3.141592;
      //make it to 0 to pfield_div-1
      int index= (int) (floor(angle2/pfield_div_angle +0.5))%pfield_div;
      // printf("(%.1f %.1f %d) ",angle*180.0/3.141592,angle2*180.0/3.141592,index);
      if (dist_buf[index]>range2) dist_buf[index]=range2;
    }
    lua_pusharray(L, dist_buf, pfield_div);
    return 1;
  }
  return 0;
}

static int lua_subscribeString(lua_State *L){
  const char *topic_name = lua_tostring(L, 1);
  std::shared_ptr<_Subscriber<std_msgs::String>> sub =
    std::shared_ptr<_Subscriber<std_msgs::String>>(new _Subscriber<std_msgs::String>(rosNode, topic_name));
  sub->start();subList.push_back(sub);
  lua_pushnumber(L, subList.size() - 1);
  return 1;
}

static int lua_checkString(lua_State *L){
  int idx = lua_tonumber(L, 1);
  std::shared_ptr<_Subscriber<std_msgs::String>> sub =
    std::static_pointer_cast<_Subscriber<std_msgs::String>>(subList[idx]);
  if (sub->checkMsg()){
    std_msgs::String ps = sub->getMsg();
    lua_pushstring(L,ps.data.c_str());
    return 1;
  }else return 0;
}

static int lua_subscribeImu(lua_State *L){
  const char *topic_name = lua_tostring(L, 1);
  std::shared_ptr<_Subscriber<sensor_msgs::Imu>> sub =
    std::shared_ptr<_Subscriber<sensor_msgs::Imu>>(new _Subscriber<sensor_msgs::Imu>(rosNode, topic_name));
  sub->start();subList.push_back(sub);
  lua_pushnumber(L, subList.size() - 1);
  return 1;
}

static int lua_checkImu(lua_State *L){
  int idx = lua_tonumber(L, 1);
  std::shared_ptr<_Subscriber<sensor_msgs::Imu>> sub =
    std::static_pointer_cast<_Subscriber<sensor_msgs::Imu>>(subList[idx]);
  if (sub->checkMsg()){
    sensor_msgs::Imu ps = sub->getMsg();
    tf::Quaternion quat;
    tf::quaternionMsgToTF(ps.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    float rpy[3];
    rpy[0]=roll;rpy[1]=pitch;rpy[2]=yaw;
    lua_pusharray(L, rpy,3);
    return 1;
  }else return 0;
}


static int lua_subscribeYOLO(lua_State *L)
{
  const char *topic_name = lua_tostring(L, 1);
  std::shared_ptr<_Subscriber<darknet_ros_msgs::BoundingBoxes>> sub = std::shared_ptr<_Subscriber<darknet_ros_msgs::BoundingBoxes>>(new _Subscriber<darknet_ros_msgs::BoundingBoxes>(rosNode, topic_name));
  sub->start();
  subList.push_back(sub);
  lua_pushnumber(L, subList.size() - 1);
  return 1;
}

static int lua_checkYOLO(lua_State *L)
{
  int idx = lua_tonumber(L, 1);
  std::shared_ptr<_Subscriber<darknet_ros_msgs::BoundingBoxes>> sub = std::static_pointer_cast<_Subscriber<darknet_ros_msgs::BoundingBoxes>>(subList[idx]);
  if (sub->checkMsg()){
    darknet_ros_msgs::BoundingBoxes yolo_msg = sub->getMsg();
    float tstamp=yolo_msg.image_header.stamp.toSec();
    int obj_num=yolo_msg.bounding_boxes.size();
  	lua_pushnumber(L,obj_num);
  	lua_pushnumber(L,tstamp);
    if(obj_num>0){
      lua_createtable(L, obj_num, 0);
      for (int i = 0; i < obj_num; i++) {
        lua_pushstring(L,yolo_msg.bounding_boxes[i].Class.c_str() );
        lua_rawseti(L, -2, i+1);    }
      lua_createtable(L, obj_num, 0);
      for (int i = 0; i < obj_num; i++) {
        lua_pushnumber(L,yolo_msg.bounding_boxes[i].xmin);
        lua_rawseti(L, -2, i+1);    }
      lua_createtable(L, obj_num, 0);
      for (int i = 0; i < obj_num; i++) {
        lua_pushnumber(L,yolo_msg.bounding_boxes[i].xmax);
        lua_rawseti(L, -2, i+1);    }
      lua_createtable(L, obj_num, 0);
      for (int i = 0; i < obj_num; i++) {
        lua_pushnumber(L,yolo_msg.bounding_boxes[i].ymin);
        lua_rawseti(L, -2, i+1);    }
      lua_createtable(L, obj_num, 0);
      for (int i = 0; i < obj_num; i++) {
        lua_pushnumber(L,yolo_msg.bounding_boxes[i].ymax);
        lua_rawseti(L, -2, i+1);    }
      return 7;
    }
  	return 2;
  }else return 0;
}


static int lua_subscribeCameraInfo(lua_State *L)
{
  const char *topic_name = lua_tostring(L, 1);
  std::shared_ptr<_Subscriber<sensor_msgs::CameraInfo>> sub = std::shared_ptr<_Subscriber<sensor_msgs::CameraInfo>>(new _Subscriber<sensor_msgs::CameraInfo>(rosNode, topic_name));
  sub->start();
  subList.push_back(sub);
  lua_pushnumber(L, subList.size() - 1);
  return 1;
}

static int lua_checkCameraInfo(lua_State *L)
{
  int idx = lua_tonumber(L, 1);
  std::shared_ptr<_Subscriber<sensor_msgs::CameraInfo>> sub = std::static_pointer_cast<_Subscriber<sensor_msgs::CameraInfo>>(subList[idx]);
  if (sub->checkMsg()){
    sensor_msgs::CameraInfo camera_msg = sub->getMsg();
    int width=camera_msg.width;
    int height=camera_msg.height;
    float fx=camera_msg.P[0];
    float fy=camera_msg.P[5];
    float cx=camera_msg.P[2];
    float cy=camera_msg.P[6];
    lua_createtable(L, 6, 0);
    lua_pushnumber(L, width);lua_rawseti(L, -2, 1);
    lua_pushnumber(L, height);lua_rawseti(L, -2, 2);
    lua_pushnumber(L, fx);lua_rawseti(L, -2, 3);
    lua_pushnumber(L, fy);lua_rawseti(L, -2, 4);
    lua_pushnumber(L, cx);lua_rawseti(L, -2, 5);
    lua_pushnumber(L, cy);lua_rawseti(L, -2, 6);
  	return 1;
  }else return 0;
}

static int lua_subscribeJTCState(lua_State *L)
{
  const char *topic_name = lua_tostring(L, 1);
  std::shared_ptr<_Subscriber<control_msgs::JointTrajectoryControllerState>> sub =
    std::shared_ptr<_Subscriber<control_msgs::JointTrajectoryControllerState>>(new _Subscriber<control_msgs::JointTrajectoryControllerState>(rosNode, topic_name));
  sub->start();
  subList.push_back(sub);
  lua_pushnumber(L, subList.size() - 1);
  return 1;
}

static int lua_checkJTCState(lua_State *L)
{
  int idx = lua_tonumber(L, 1);
  std::shared_ptr<_Subscriber<control_msgs::JointTrajectoryControllerState>> sub =
      std::static_pointer_cast<_Subscriber<control_msgs::JointTrajectoryControllerState>>(subList[idx]);
  if (sub->checkMsg()){
    control_msgs::JointTrajectoryControllerState msg = sub->getMsg();
    lua_createtable(L, 3, 0);
    lua_pushnumber(L, msg.error.positions[0]);lua_rawseti(L, -2, 1);
    lua_pushnumber(L, msg.error.positions[1]);lua_rawseti(L, -2, 2);
    lua_pushnumber(L, msg.error.positions[2]);lua_rawseti(L, -2, 3);
  	return 1;
  }else return 0;
}

static int lua_subscribeOpenPose(lua_State *L)
{
  const char *topic_name = lua_tostring(L, 1);
  std::shared_ptr<_Subscriber<ros_openpose::Frame>> sub =std::shared_ptr<_Subscriber<ros_openpose::Frame>>(new _Subscriber<ros_openpose::Frame>(rosNode, topic_name));
  sub->start();
  subList.push_back(sub);
  lua_pushnumber(L, subList.size() - 1);
  return 1;
}

static int lua_checkOpenPose(lua_State *L)
{
  int idx = lua_tonumber(L, 1);
  std::shared_ptr<_Subscriber<ros_openpose::Frame>> sub = std::static_pointer_cast<_Subscriber<ros_openpose::Frame>>(subList[idx]);
  float xpos[8*32],ypos[8*32],score[8*32]; //do we need more than 32 people?

  if (sub->checkMsg()){
    ros_openpose::Frame msg = sub->getMsg();
    // printf("Openpose|Total humans: %d\n",msg.persons.size());
    int num_persons=msg.persons.size();
    if (num_persons==0){
      lua_pushnumber(L, 0);
      return 1;
    }
    if (num_persons>32) num_persons=32;
    for (int i=0;i<num_persons;i++){
      // https://cmu-perceptual-computing-lab.github.io/openpose/web/html/.github/media/keypoints_pose_25.png
      //0: face
      //1: shoulder center
      //2 3 4: right arm
      //5 6 7: left arm
      //Lets output xy positions for 0-7
      for (int j=0;j<8;j++){
        int index=i*8+j;
        score[index]=msg.persons[i].bodyParts[j].score;
        xpos[index]=msg.persons[i].bodyParts[j].pixel.x;
        ypos[index]=msg.persons[i].bodyParts[j].pixel.y;
      }
    }
    lua_pushnumber(L, num_persons);
    lua_pusharray(L, xpos,8*num_persons);
    lua_pusharray(L, ypos,8*num_persons);
    lua_pusharray(L, score,8*num_persons);
    lua_pushnumber(L, msg.header.stamp.sec);
    lua_pushnumber(L, msg.header.stamp.nsec);
  	return 6;
  }else return 0;
}

static int lua_checkOpenPoseBody(lua_State *L)
{
  int idx = lua_tonumber(L, 1);
  std::shared_ptr<_Subscriber<ros_openpose::Frame>> sub = std::static_pointer_cast<_Subscriber<ros_openpose::Frame>>(subList[idx]);

  // https://cmu-perceptual-computing-lab.github.io/openpose/web/html/.github/media/keypoints_pose_25.png
  //0:nose
  //1: shoulder center
  //2-3-4: right arm 5-6-7: left arm
  //8: pelvis center
  //9-10-11: right leg
  //12-13-14: left leg
  //We will map them to 0-14 (total 15 points)

  float xpos[15*32],ypos[15*32],score[15*32]; //do we need more than 32 people?

  if (sub->checkMsg()){
    ros_openpose::Frame msg = sub->getMsg();
    int num_persons=msg.persons.size();
    if (num_persons==0){
      lua_pushnumber(L, 0);
      return 1;
    }
    if (num_persons>32) num_persons=32;
    for (int i=0;i<num_persons;i++){
      for (int j=0;j<15;j++){
        int index=i*15+j;
        score[index]=msg.persons[i].bodyParts[j].score;
        xpos[index]=msg.persons[i].bodyParts[j].pixel.x;
        ypos[index]=msg.persons[i].bodyParts[j].pixel.y;
      }
    }
    lua_pushnumber(L, num_persons);
    lua_pusharray(L, xpos,15*num_persons);
    lua_pusharray(L, ypos,15*num_persons);
    lua_pusharray(L, score,15*num_persons);
    lua_pushnumber(L, msg.header.stamp.sec);
    lua_pushnumber(L, msg.header.stamp.nsec);
  	return 6;
  }else return 0;
}

static int lua_checkOpenPoseFace(lua_State *L)
{
  int idx = lua_tonumber(L, 1);
  std::shared_ptr<_Subscriber<ros_openpose::Frame>> sub = std::static_pointer_cast<_Subscriber<ros_openpose::Frame>>(subList[idx]);


  float x0[32],x1[32],y0[32],y1[32];
  float hand[32];


  if (sub->checkMsg()){
    ros_openpose::Frame msg = sub->getMsg();
    int num_persons=msg.persons.size();
    if (num_persons==0){
      lua_pushnumber(L, 0);
      return 1;
    }
    if (num_persons>32) num_persons=32;

    int num_faces=0;

    for (int i=0;i<num_persons;i++){
      float nose_score=msg.persons[i].bodyParts[0].score;
      float lear_score=msg.persons[i].bodyParts[17].score;
      float rear_score=msg.persons[i].bodyParts[18].score;
      float leye_score=msg.persons[i].bodyParts[15].score;
      float reye_score=msg.persons[i].bodyParts[16].score;
      float neck_score=msg.persons[i].bodyParts[1].score;
      float lhand_score=msg.persons[i].bodyParts[4].score;
      float rhand_score=msg.persons[i].bodyParts[7].score;
      float chest_score=msg.persons[i].bodyParts[1].score;

      //essential parts: nose, both eye, neck
      if ( (nose_score>0) && (neck_score>0) && (leye_score>0) && (reye_score>0) ){
        if (num_faces<32){
          float tx0=0.0,tx1=0.0,ty0=0.0,ty1=0.0;//tight boundingbox around the face

          float cx=msg.persons[i].bodyParts[0].pixel.x;
          float cy=msg.persons[i].bodyParts[0].pixel.y;
          ty1=(cy+msg.persons[i].bodyParts[1].pixel.y)*0.5; //middlepoint between nose and neck
          ty0= cy - (ty1-cy)*1.2;

          if(lear_score>0) tx0=msg.persons[i].bodyParts[17].pixel.x; //left year X
          else{
            float l_eye_dx= cx-msg.persons[i].bodyParts[15].pixel.x; //left eye X
            tx0=cx - l_eye_dx * 4.0;
          }
          if(rear_score>0) tx1=msg.persons[i].bodyParts[18].pixel.x; //right year X
          else{
            float r_eye_dx= msg.persons[i].bodyParts[16].pixel.x-cx; //right eye X
            tx1=cx + r_eye_dx * 4.0;
          }
          float x_scale=1.5;
          float y_scale=1.5;
          x0[num_faces]=cx- (cx-tx0)*x_scale;
          x1[num_faces]=cx+ (tx1-cx)*x_scale;
          y0[num_faces]=cy- (cy-ty0)*y_scale;
          y1[num_faces]=cy+ (ty1-cy)*y_scale;


          hand[num_faces]=0;
          if ( (lhand_score>0)&&(chest_score>0)&&  (   msg.persons[i].bodyParts[4].pixel.y <  msg.persons[i].bodyParts[1].pixel.y  )) hand[num_faces]+=1;
          if ( (rhand_score>0)&&(chest_score>0)&&  (   msg.persons[i].bodyParts[7].pixel.y <  msg.persons[i].bodyParts[1].pixel.y  )) hand[num_faces]+=2;
          num_faces++;
        }
      }
    }
    lua_pushnumber(L, num_faces);
    lua_pusharray(L, x0,num_faces);
    lua_pusharray(L, x1,num_faces);
    lua_pusharray(L, y0,num_faces);
    lua_pusharray(L, y1,num_faces);
    lua_pusharray(L, hand,num_faces);
    return 6;
  }else return 0;
}



static int lua_rostime(lua_State *L){
  double time1 = ros::Time::now().toSec();
  lua_pushnumber(L,time1);
  return 1;
}

static const struct luaL_Reg rossub_lib[] = {
  {"init", lua_init},

  {"checkTF", lua_checkTF},

  {"subscribeTwist", lua_subscribeTwist},
  {"checkTwist", lua_checkTwist},

  {"subscribeJointState", lua_subscribeJointState},
  {"checkJointState", lua_checkJointState},

  {"subscribeJointTrajectory", lua_subscribeJointTrajectory},
  {"checkJointTrajectory", lua_checkJointTrajectory},

  {"subscribeFloat32MultiArray", lua_subscribeFloat32MultiArray},
  {"checkFloat32MultiArray", lua_checkFloat32MultiArray},

  {"subscribeOccupancyGrid", lua_subscribeOccupancyGrid},
  {"checkOccupancyGrid", lua_checkOccupancyGrid},

  {"subscribePoseStamped", lua_subscribePoseStamped},
  {"checkPoseStamped", lua_checkPoseStamped},
  {"checkPoseStamped3D", lua_checkPoseStamped3D},

  {"subscribeOdometry", lua_subscribeOdometry},
  {"checkOdometry", lua_checkOdometry},

  {"subscribeInt32", lua_subscribeInt32},
  {"checkInt32", lua_checkInt32},

  {"subscribeFloat64", lua_subscribeFloat64},
  {"checkFloat64", lua_checkFloat64},

  // {"subscribeCompressedImage", lua_subscribeCompressedImage},
  // {"checkCompressedImage", lua_checkCompressedImage},

  {"subscribeImage", lua_subscribeImage},
  {"checkImage", lua_checkImage},

  {"subscribeCameraInfo", lua_subscribeCameraInfo},
  {"checkCameraInfo", lua_checkCameraInfo},

  {"subscribeLaserScan", lua_subscribeLaserScan},
  {"checkLaserScan", lua_checkLaserScan},
  {"checkLaserScanPField", lua_checkLaserScanPField},

  {"subscribeBattery", lua_subscribeBattery},
  {"checkBattery", lua_checkBattery},

  {"subscribeString", lua_subscribeString},
  {"checkString", lua_checkString},

  {"subscribeImu", lua_subscribeImu},
  {"checkImu", lua_checkImu},

  {"subscribeYOLO", lua_subscribeYOLO},
	{"checkYOLO", lua_checkYOLO},

  {"subscribeWrenchStamped", lua_subscribeWrenchStamped},
  {"checkWrenchStamped", lua_checkWrenchStamped},


  {"subscribeJTCState", lua_subscribeJTCState},
  {"checkJTCState", lua_checkJTCState},


  {"subscribeOpenPose", lua_subscribeOpenPose},
  {"checkOpenPose", lua_checkOpenPose},
  {"checkOpenPoseFace", lua_checkOpenPoseFace},
  {"checkOpenPoseBody", lua_checkOpenPoseBody},


  {"rostime", lua_rostime},

  {NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
int luaopen_rossub(lua_State *L)
{
  #if LUA_VERSION_NUM == 502
  	luaL_newlib(L, rossub_lib);
  #else
  	luaL_register(L, "rossub", rossub_lib);
  #endif

  return 1;
}
