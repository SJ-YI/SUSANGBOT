#include <lua.hpp>
#include <cstdio>
#include <vector>
#include <algorithm>
#include <sys/time.h>
#include "rospub.h"

std::unique_ptr<ros::NodeHandle> rosNode;

static void lua_pushvector(lua_State *L, float* v, int n) {
	lua_createtable(L, n, 0);
	for (int i = 0; i < n; i++) {
		lua_pushnumber(L, v[i]);
		lua_rawseti(L, -2, i+1);
	}
}

static std::vector<double> lua_checkvector(lua_State *L, int narg) {
	if ( !lua_istable(L, narg) )	{
    printf("ERROR!!!!\n");
    luaL_argerror(L, narg, "vector");
  }
#if LUA_VERSION_NUM == 502
	int n = lua_rawlen(L, narg);
#else
	int n = lua_objlen(L, narg);
#endif
	std::vector<double> v(n);
	for (int i = 0; i < n; i++) {
		lua_rawgeti(L, narg, i+1);
		v[i] = lua_tonumber(L, -1);
		lua_pop(L, 1);
	}
  for (int i = 0; i < n; i++)
	return v;
}

static std::vector<std::string> lua_checkstringvector(lua_State *L, int narg) {
	if ( !lua_istable(L, narg) )	{
    printf("ERROR!!!!\n");
    luaL_argerror(L, narg, "vector");
  }
#if LUA_VERSION_NUM == 502
	int n = lua_rawlen(L, narg);
#else
	int n = lua_objlen(L, narg);
#endif
	std::vector<std::string> v(n);
	for (int i = 0; i < n; i++) {
		lua_rawgeti(L, narg, i+1);
		v[i] = lua_tostring(L, -1);
		lua_pop(L, 1);
	}
  for (int i = 0; i < n; i++)
	return v;
}

static int lua_init(lua_State *L)
{
  const char *ros_node_name = lua_tostring(L, 1);
  if(!ros::isInitialized())
  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, ros_node_name, ros::init_options::NoSigintHandler);
  }
  rosNode.reset(new ros::NodeHandle(ros_node_name));
  init_publishers();
  ros::Time::init();
  ros::Time::useSystemTime();
  return 0;
}

static int lua_armcontrol(lua_State *L){
	std::vector<double> qArm=lua_checkvector(L,1);
  double duration=lua_tonumber(L, 2);
  armcontrol(qArm, duration);
  return 0;
}
static int lua_headcontrol(lua_State *L) {// frame_id, child_frame_id, pose x, y, z, quaternion x, y, z, w, twist linear x, y, z, angular x, y, z
  headcontrol(lua_tonumber(L, 1),lua_tonumber(L, 2),lua_tonumber(L, 3));
  return 0;
}
static int lua_grippercontrol(lua_State *L){ // frame_id, child_frame_id, pose x, y, z, quaternion x, y, z, w, twist linear x, y, z, angular x, y, z
  grippercontrol(lua_tonumber(L, 1),lua_tonumber(L, 2),lua_tonumber(L, 3));
  return 0;
}
static int lua_baseteleop(lua_State *L){ // frame_id, child_frame_id, pose x, y, z, quaternion x, y, z, w, twist linear x, y, z, angular x, y, z
  baseteleop(lua_tonumber(L, 1),lua_tonumber(L, 2),lua_tonumber(L, 3));
  return 0;
}


// static int lua_jointstates(lua_State *L){
//     std::vector<double> head=lua_checkvector(L,1);
//     std::vector<double> arm=lua_checkvector(L,2);
//     jointstates(head,arm);
//     return 0;
// }

static int lua_jointstates(lua_State *L){
	int seq=lua_tonumber(L, 1);
	std::vector<std::string> jointnames=lua_checkstringvector(L,2);
  std::vector<double>jangles=lua_checkvector(L,3);
	send_jointstate(jointnames,jangles);
  return 0;
}

static int lua_sensor_tr(lua_State *L){
  std::vector<double>p=lua_checkvector(L,1);
  geometry_msgs::PoseStamped msg;tf::Quaternion q;
  q.setRPY(p[3],p[4],p[5]);
  msg.header.stamp = ros::Time::now();
  msg.pose.position.x=p[0];msg.pose.position.y=p[1];msg.pose.position.z=p[2];
  msg.pose.orientation.x=q.x();msg.pose.orientation.y=q.y();msg.pose.orientation.z=q.z();msg.pose.orientation.w=q.w();
  sensortr_publisher.publish(msg);
  return 0;
}
static int lua_tf(lua_State *L){
  std::vector<double>xyz=lua_checkvector(L,1);
  std::vector<double>rpy=lua_checkvector(L,2);
  const char *mother_link = lua_tostring(L, 3);
  const char *child_link = lua_tostring(L, 4);
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(xyz[0],xyz[1],xyz[2]));
  tf::Quaternion q; q.setRPY(rpy[0],rpy[1],rpy[2]);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),mother_link, child_link));
  return 0;
}



static int lua_basegoal(lua_State *L){
  basegoal(lua_tonumber(L, 1),lua_tonumber(L, 2),lua_tonumber(L, 3),lua_tonumber(L, 4));
  return 0;
}
static int lua_posereset(lua_State *L){ // frame_id, child_frame_id, pose x, y, z, quaternion x, y, z, w, twist linear x, y, z, angular x, y, z
	std::vector<double> pose=lua_checkvector(L,1);
  posereset(pose);
  return 0;
}
static int lua_occgrid(lua_State *L){
  occgrid(lua_tonumber(L, 1),lua_tonumber(L, 2),lua_tonumber(L, 3),lua_tonumber(L, 4),lua_tonumber(L, 5),lua_tonumber(L, 6),lua_tostring(L, 7));
  return 0;
}
static int lua_path(lua_State *L){
  std::vector<double>posx=lua_checkvector(L,1);
  std::vector<double>posy=lua_checkvector(L,2);
  std::vector<double>posa=lua_checkvector(L,3);
  path(posx,posy,posa);
  return 0;
}


static int lua_markerrectangle(lua_State *L) // frame_id, child_frame_id, pose x, y, z, quaternion x, y, z, w, twist linear x, y, z, angular x, y, z
{
  int seq=lua_tonumber(L,1);
  float posx = lua_tonumber(L,2);
  float posy = lua_tonumber(L,3);
  float yaw = lua_tonumber(L,4);
	float lw = lua_tonumber(L,5);
  float rw = lua_tonumber(L,6);
  float th = lua_tonumber(L,7);
  float bh = lua_tonumber(L,8);
  marker_rectangle(seq,posx,posy,yaw,lw,rw,th,bh);
  return 0;
}
static int lua_marker(lua_State *L)
{
  int marker_num=lua_tonumber(L, 1);
  std::vector<double> types=lua_checkvector(L,2);
	std::vector<double> posx=lua_checkvector(L,3);
	std::vector<double> posy=lua_checkvector(L,4);
	std::vector<double> posz=lua_checkvector(L,5);
  std::vector<double> yaw=lua_checkvector(L,6);
	std::vector<std::string> names=lua_checkstringvector(L,7);
  std::vector<double> scales=lua_checkvector(L,8);
  std::vector<double> colors=lua_checkvector(L,9);
  marker(marker_num, types, posx, posy, posz, yaw,names, scales, colors);
  return 0;
}

static int lua_marker3d(lua_State *L)
{
  int marker_num=lua_tonumber(L, 1);
  std::vector<double> types=lua_checkvector(L,2);
	std::vector<double> posx=lua_checkvector(L,3);
	std::vector<double> posy=lua_checkvector(L,4);
	std::vector<double> posz=lua_checkvector(L,5);

	std::vector<double> orir=lua_checkvector(L,6);
	std::vector<double> orip=lua_checkvector(L,7);
	std::vector<double> oriy=lua_checkvector(L,8);

	std::vector<double> scalex=lua_checkvector(L,9);
	std::vector<double> scaley=lua_checkvector(L,10);
	std::vector<double> scalez=lua_checkvector(L,11);

	std::vector<std::string> names=lua_checkstringvector(L,12);
  std::vector<double> colors=lua_checkvector(L,13);
	std::vector<double> alpha=lua_checkvector(L,14);

  marker3d(marker_num, types, posx, posy, posz, orir,orip,oriy, scalex, scaley, scalez, names, colors,alpha);
  return 0;
}

static int lua_laserscan(lua_State *L){ // frame_id, child_frame_id, pose x, y, z, quaternion x, y, z, w, twist linear x, y, z, angular x, y, z
  int seq=lua_tonumber(L, 1);
  float angle_min=lua_tonumber(L, 2);
  float angle_max=lua_tonumber(L, 3);
  int n=lua_tonumber(L, 4);
  float angle_increment=(angle_max-angle_min)/((float) n-1);
  float* ranges=(float*) luaL_checkstring(L, 5);
  const char *linkname = lua_tostring(L, 6);
  laserscan(seq,angle_min,angle_max,n,angle_increment,ranges,linkname);
  return 0;
}
static int lua_camerainfo(lua_State *L){
  camerainfo(lua_tonumber(L, 1),lua_tonumber(L, 2),lua_tonumber(L, 3),lua_tonumber(L, 4));
  return 0;
}
static int lua_rgbimage(lua_State *L){
  int enc=luaL_optnumber(L,5,0);
  rgbimage(lua_tonumber(L, 1),lua_tonumber(L, 2),lua_tonumber(L, 3), lua_tostring(L, 4),enc);
  return 0;
}
static int lua_jpgimage(lua_State *L){
  jpgimage(lua_tonumber(L, 1),lua_tostring(L, 2));
  return 0;
}
static int lua_depthimage(lua_State *L){
  int send_surface=luaL_optnumber(L, 5,0);
	int type=luaL_optnumber(L, 6,0);
  depthimage(lua_tonumber(L, 1),lua_tonumber(L, 2),lua_tonumber(L, 3), lua_tostring(L, 4),send_surface,type);
  return 0;
}

static int lua_armjoint7(lua_State *L) // frame_id, child_frame_id, pose x, y, z, quaternion x, y, z, w, twist linear x, y, z, angular x, y, z
{
  std::vector<double> arm=lua_checkvector(L,1);
  sensor_msgs::JointState msg;

	msg.header.stamp=ros::Time::now();
	msg.name.push_back("panda_joint1");
	msg.name.push_back("panda_joint2");
	msg.name.push_back("panda_joint3");
	msg.name.push_back("panda_joint4");
	msg.name.push_back("panda_joint5");
	msg.name.push_back("panda_joint6");
	msg.name.push_back("panda_joint7");
  msg.position.push_back(arm[0]);
  msg.position.push_back(arm[1]);
  msg.position.push_back(arm[2]);
  msg.position.push_back(arm[3]);
  msg.position.push_back(arm[4]);
  msg.position.push_back(arm[5]);
	msg.position.push_back(arm[6]);

  armjoint7_publisher.publish(msg);
  return 0;
}

static int lua_tf_qt(lua_State *L){
  std::vector<double>xyz=lua_checkvector(L,1);
  std::vector<double>qt=lua_checkvector(L,2);
  const char *mother_link = lua_tostring(L, 3);
  const char *child_link = lua_tostring(L, 4);
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(xyz[0],xyz[1],xyz[2]));
  tf::Quaternion q(qt[0],qt[1],qt[2],qt[3]);//X-Y-Z-W
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),mother_link, child_link));
  return 0;
}

static int lua_armpose7qt(lua_State *L){
  std::vector<double>p=lua_checkvector(L,1);
	std::vector<double>q=lua_checkvector(L,2); //w-x-y-z index
  geometry_msgs::PoseStamped msg;

  msg.header.stamp = ros::Time::now();
  msg.pose.position.x=p[0];
  msg.pose.position.y=p[1];
  msg.pose.position.z=p[2];
	msg.pose.orientation.w=q[0];
  msg.pose.orientation.x=q[1];
  msg.pose.orientation.y=q[2];
  msg.pose.orientation.z=q[3];
  armpose7_publisher.publish(msg);
  return 0;
}

static int lua_curtime(lua_State *L){
	double time1 = ros::Time::now().toSec();
	lua_pushnumber(L, time1);
	return 1;
}

static int lua_odom(lua_State *L){
  std::vector<double>pose=lua_checkvector(L,1);
	send_odom(pose);
  return 0;
}

static int lua_jointstate(lua_State *L){
	int seq=lua_tonumber(L, 1);
	std::vector<std::string> jointnames=lua_checkstringvector(L,2);
  std::vector<double>jangles=lua_checkvector(L,3);
	send_jointstate(jointnames,jangles);
  return 0;
}

static int lua_webotslaserscancrop(lua_State *L) // frame_id, child_frame_id, pose x, y, z, quaternion x, y, z, w, twist linear x, y, z, angular x, y, z
{
  int seq=lua_tonumber(L, 1);
  float angle_min=lua_tonumber(L, 2);
  float angle_max=lua_tonumber(L, 3);
	int n=lua_tonumber(L, 4);
	float angle_increment=(angle_max-angle_min)/((float) n-1);
  float* ranges=(float*) luaL_checkstring(L, 5);
  const char *linkname = lua_tostring(L, 6);
	int crop_ray_num=lua_tonumber(L, 7);
  webotslaserscancrop(seq,angle_min,angle_max,n,angle_increment,ranges,linkname,crop_ray_num);
  return 0;
}


static int lua_savergbimage(lua_State *L){ //W H ENC DATA LOGNAME
  savergbimage(lua_tonumber(L, 1),lua_tonumber(L, 2),lua_tostring(L, 3), lua_tostring(L, 4), lua_tostring(L, 5));
  return 0;
}

static const struct luaL_Reg rospub_lib[] = {
  {"init", lua_init},
  {"tf", lua_tf},


	{"armcontrol",lua_armcontrol},
  {"headcontrol",lua_headcontrol},
  {"grippercontrol",lua_grippercontrol},
  {"baseteleop", lua_baseteleop},
	{"armjoint7", lua_armjoint7},
	// {"armpose7", lua_armpose7},
	{"armpose7qt", lua_armpose7qt},

	{"jointstates",lua_jointstates},
  {"sensor_tr",lua_sensor_tr},
	{"tf_qt", lua_tf_qt},
	{"basegoal", lua_basegoal},
  {"posereset", lua_posereset},
	{"occgrid",lua_occgrid},
	{"path",lua_path},


  {"marker",lua_marker},
	{"marker3d",lua_marker3d},
  {"markerrectangle",lua_markerrectangle},

  {"laserscan", lua_laserscan},
  {"camerainfo",lua_camerainfo},
  {"rgbimage",lua_rgbimage},
	{"jpgimage",lua_jpgimage},
	{"depthimage",lua_depthimage},


	{"curtime",lua_curtime},
	{"odom",lua_odom},

	{"jointstate",lua_jointstate},
	{"webotslaserscancrop", lua_webotslaserscancrop},

	{"savergbimage",lua_savergbimage},

  {NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
int luaopen_rospub(lua_State *L)
{
  #if LUA_VERSION_NUM == 502
  	luaL_newlib(L, rospub_lib);
  #else
  	luaL_register(L, "rospub", rospub_lib);
  #endif

  return 1;
}
