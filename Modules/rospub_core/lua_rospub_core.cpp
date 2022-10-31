#include <lua.hpp>
#include <cstdio>
#include <vector>
#include <algorithm>
#include <sys/time.h>
#include "rospub.h"

std::unique_ptr<ros::NodeHandle> rosNode;
// std::vector<std::string> camera_names;
// std::vector<std::string> lidar_names;
// std::vector<std::string> path_names;
// std::vector<std::string> occgrid_names;

static void lua_pushvector(lua_State *L, float* v, int n) {
	lua_createtable(L, n, 0);
	for (int i = 0; i < n; i++) {
		lua_pushnumber(L, v[i]);
		lua_rawseti(L, -2, i+1);
	}
}

static std::vector<double> lua_checkvector(lua_State *L, int narg) {
	if ( !lua_istable(L, narg) )	{printf("ERROR!!!!\n");luaL_argerror(L, narg, "vector");}
	int n = lua_objlen(L, narg);
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
	if ( !lua_istable(L, narg) )	{printf("ERROR!!!!\n");luaL_argerror(L, narg, "vector");}
	int n = lua_objlen(L, narg);
	std::vector<std::string> v(n);
	for (int i = 0; i < n; i++) {
		lua_rawgeti(L, narg, i+1);
		v[i] = lua_tostring(L, -1);
		lua_pop(L, 1);
	}
  for (int i = 0; i < n; i++)
	return v;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
static int lua_init(lua_State *L){
  const char *ros_node_name = lua_tostring(L, 1);
  if(!ros::isInitialized())  {
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

static int lua_gps_fix(lua_State *L){
  const char *frame_id = lua_tostring(L, 1);
	sensor_msgs::NavSatFix msg;
	msg.header.seq=1;
	msg.header.frame_id=frame_id;
	msg.header.stamp=ros::Time::now();
	msg.status.service=1;
	msg.latitude=lua_tonumber(L,2);
	msg.longitude=lua_tonumber(L,3);
	msg.position_covariance={4.0,0.0,0.0, 0.0,4.0,0.0, 0.0,0.0,8.0};
	msg.position_covariance_type=2;
	gps_fix_publisher.publish(msg);
  return 0;
}

static int lua_markermatrix(lua_State *L)
{
	float x0=lua_tonumber(L, 1);
	float y0=lua_tonumber(L, 2);
	float a0=lua_tonumber(L, 3);
	int hmapx=lua_tonumber(L, 4);
	int hmapy=lua_tonumber(L, 5);
	float scale=lua_tonumber(L, 6);
	float basez=lua_tonumber(L, 8);
	float z0=luaL_optnumber(L,9,0.0);
	int use_int8=luaL_optnumber(L,10,0);
	const char *frame=luaL_optstring(L,11,"map");
	int flip_y=luaL_optnumber(L,12,0);

	if (use_int8==0){
		float* heights=(float*) luaL_checkstring(L, 7);
  	markermatrix(x0,y0,a0, hmapx, hmapy, scale, heights,basez,z0,frame,flip_y);
	}else{
		// printf("xy:%d %d\n",hmapx,hmapy);
		int8_t* heights_int=(int8_t*) luaL_checkstring(L, 7);
		float heights[100*50];
		for (int i=0;i<hmapx*hmapy;i++)  heights[i]= ( (float) heights_int[i] ) * 0.01;
		markermatrix(x0,y0,a0, hmapx, hmapy, scale, heights,basez,z0,frame,flip_y);
	}
  return 0;
}

static const struct luaL_Reg rospub_core_lib[] = {
  {"init", lua_init},
  {"tf", lua_tf},
	{"gpsfix", lua_gps_fix},
	{"markermatrix",lua_markermatrix},
  {NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
int luaopen_rospub_core(lua_State *L){
	luaL_register(L, "rospub_core", rospub_core_lib);
  return 1;
}
