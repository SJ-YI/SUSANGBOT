#include <lua.hpp>
#include <cstdio>
#include <vector>
#include <algorithm>
#include <sys/time.h>
#include "rospub.h"

std::unique_ptr<ros::NodeHandle> rosNode;
std::vector<std::string> camera_names;
std::vector<std::string> lidar_names;
std::vector<std::string> path_names;
std::vector<std::string> occgrid_names;

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
	camera_names.push_back("/camera");
	lidar_names.push_back("/lidar");
	path_names.push_back("/path");
	occgrid_names.push_back("/map");
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

static int lua_init_custom(lua_State *L){
  const char *ros_node_name = lua_tostring(L, 1);
	camera_names=lua_checkstringvector(L,2);
	lidar_names=lua_checkstringvector(L,3);
	path_names=lua_checkstringvector(L,4);
	occgrid_names=lua_checkstringvector(L,5);
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

static int lua_image(lua_State *L){ //seq w h fov data encoding channel framename
	int enc=luaL_optnumber(L,6,0);int ch=luaL_optnumber(L,7,0);
	const char *frame=luaL_optstring(L,8,"head_rgbd_sensor_rgb_frame_sync");
  send_image(lua_tonumber(L, 1),lua_tonumber(L, 2),lua_tonumber(L, 3),lua_tonumber(L, 4), lua_tostring(L, 5),enc,ch,frame);
  return 0;
}

static int lua_imageonly(lua_State *L){ //seq w h data encoding channel framename
	int enc=luaL_optnumber(L,5,0);int ch=luaL_optnumber(L,6,0);
	const char *frame=luaL_optstring(L,7,"head_rgbd_sensor_rgb_frame_sync");
  send_imageonly(lua_tonumber(L, 1),lua_tonumber(L, 2),lua_tonumber(L, 3),lua_tostring(L, 4), enc,ch,frame);
  return 0;
}


static int lua_savergbimage(lua_State *L){ //W H ENC DATA LOGNAME
  savergbimage(lua_tonumber(L, 1),lua_tonumber(L, 2),lua_tostring(L, 3), lua_tostring(L, 4), lua_tostring(L, 5));
  return 0;
}

static int lua_imagefrompng(lua_State *L){ //read png data and send as img topic
	//seq w h data encoding channel framename
	int enc=luaL_optnumber(L,5,0);int ch=luaL_optnumber(L,6,0);
	const char *frame=luaL_optstring(L,7,"head_rgbd_sensor_rgb_frame_sync");
  send_imagefrompng(lua_tonumber(L, 1),lua_tonumber(L, 2),lua_tonumber(L, 3),lua_tostring(L, 4), enc,ch,frame);
  return 0;
}


static int lua_jpgimage(lua_State *L){
	const char *frame=luaL_optstring(L,4,"head_rgbd_sensor_rgb_frame_sync");
  jpgimage(lua_tonumber(L, 1),lua_tostring(L, 2),lua_tonumber(L,3),frame);
  return 0;
}

static int lua_camerainfo(lua_State *L){
  camerainfo(lua_tonumber(L, 1),lua_tonumber(L, 2),lua_tonumber(L, 3),lua_tonumber(L, 4));
  return 0;
}

static int lua_camerainfo2(lua_State *L){
  camerainfo2(lua_tonumber(L, 1), //seq
				lua_tonumber(L, 2),lua_tonumber(L, 3), //W H
				lua_tonumber(L, 4),lua_tonumber(L, 5),lua_tonumber(L, 6),lua_tonumber(L, 7),
				lua_tostring(L, 8)
			);
  return 0;
}

void camerainfo2(int seq,int width, int height, float fx, float fy, float cx, float cy, char* frame_id);



static int lua_jointstate(lua_State *L){
	int seq=lua_tonumber(L, 1);
	send_jointstate(lua_checkstringvector(L,2),lua_checkvector(L,3));
  return 0;
}

static int lua_odom(lua_State *L){
	send_odom(lua_checkvector(L,1));
  return 0;
}

static int lua_laserscan(lua_State *L){
	//seq angle_min angle_max n data_str linkname channel is_webots
  int seq=lua_tonumber(L, 1);
  float angle_min=lua_tonumber(L, 2);float angle_max=lua_tonumber(L, 3);
  int n=lua_tonumber(L, 4);float* ranges=(float*) luaL_checkstring(L, 5);
  const char *linkname = lua_tostring(L, 6);int ch=luaL_optnumber(L,7,0);
	int is_webots=luaL_optnumber(L,8,0);
	if (is_webots>0) webotslaserscan(seq,angle_min,angle_max,n,ranges,linkname,ch);
	else laserscan(seq,angle_min,angle_max,n,ranges,linkname,ch);
  return 0;
}

static int lua_mapcmd(lua_State *L){
	send_int32( lua_tonumber(L, 1), mapcmd_publisher);
  return 0;
}

static int lua_navigation_status(lua_State *L){
	send_int32( lua_tonumber(L, 1), navigation_status_publisher);
  return 0;
}

static int lua_cmd_vel(lua_State *L){
	send_twist(lua_tonumber(L,1),lua_tonumber(L,2),lua_tonumber(L,3),cmdvel_publisher);
  return 0;
}

static int lua_teleop_vel(lua_State *L){
	send_twist(lua_tonumber(L,1),lua_tonumber(L,2),lua_tonumber(L,3),teleopvel_publisher);
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
	float dur=luaL_optnumber(L,10,1.0);
  marker(marker_num, types, posx, posy, posz, yaw,names, scales, colors,dur);
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

static int lua_markermatrixcolor(lua_State *L)
{
	std::vector<double> limits=lua_checkvector(L,1);
	// float x0=lua_tonumber(L, 1);

	float xoffset=limits[0];
	float ymax=limits[3];

	float x0=lua_tonumber(L, 2);
	float y0=lua_tonumber(L, 3);
	float a0=lua_tonumber(L, 4);
	int hmapx=lua_tonumber(L, 5);
	int hmapy=lua_tonumber(L, 6);
	float scale=lua_tonumber(L, 7);
	float* heights=(float*) luaL_checkstring(L, 8);
	uint8_t* colors=(uint8_t*) luaL_checkstring(L, 9);
	float basez=lua_tonumber(L, 10);
	float z0=luaL_optnumber(L,11,0.0);
  markermatrixcolor(x0,y0,a0, hmapx, hmapy, scale, heights,colors,basez,z0,xoffset,ymax);
  return 0;
}

static int lua_markerlines(lua_State *L)
{
	int seq=lua_tonumber(L, 1);
	float num=lua_tonumber(L, 2);
	float wid=lua_tonumber(L, 3);

	std::vector<double> x0=lua_checkvector(L,4);
	std::vector<double> y0=lua_checkvector(L,5);
	std::vector<double> z0=lua_checkvector(L,6);
	std::vector<double> x1=lua_checkvector(L,7);
	std::vector<double> y1=lua_checkvector(L,8);
	std::vector<double> z1=lua_checkvector(L,9);
	markerlines(seq,num,x0,y0,z0, x1,y1,z1, wid);
  return 0;
}

static int lua_posereset(lua_State *L){
	posereset(lua_checkvector(L,1));
  return 0;
}

static int lua_basegoal(lua_State *L){
	basegoal(lua_checkvector(L,1));
  return 0;
}

static int lua_occgrid(lua_State *L){
	int ch=luaL_optnumber(L,8,0);
	const char *frame=luaL_optstring(L,9,"map");
  send_occgrid(lua_tonumber(L, 1),lua_tonumber(L, 2),lua_tonumber(L, 3),lua_tonumber(L, 4),
			lua_tonumber(L, 5),lua_tonumber(L, 6),lua_tostring(L, 7),ch,frame);
  return 0;
}


static int lua_path(lua_State *L){
	int ch=luaL_optnumber(L,4,0);
  send_path(lua_checkvector(L,1),lua_checkvector(L,2),lua_checkvector(L,3),ch);
  return 0;
}



static int lua_jointactiongoal(lua_State *L){
	std::vector<std::string> names=lua_checkstringvector(L,1);
	const char* id =luaL_checkstring(L, 2);
	std::vector<double>traj_points=lua_checkvector(L,3);
	std::vector<double>durations=lua_checkvector(L,4);
	int ch=lua_tonumber(L, 5);
	int num_joints=names.size();
	int num=durations.size();

	control_msgs::FollowJointTrajectoryActionGoal msg;
	msg.header.seq=1;
	msg.header.stamp=ros::Time::now();
	// msg.goal_id.id=id;
	printf("num_joint #:%d points num:%d set:%d\n",num_joints, num, traj_points.size());
	// msg.goal.trajectory.header.frame_id="world";
	msg.goal.trajectory.joint_names.resize(0);
	for(int i=0;i<num_joints;i++) msg.goal.trajectory.joint_names.push_back(names[i]);
	msg.goal.trajectory.points.resize(num);

	bool all_correct=true;

	int count=0;
	for(int i=0;i<num;i++){
		int sec=floor(durations[i]);
		int nsec= (durations[i]-sec)*1E9;
		msg.goal.trajectory.points[i].positions.resize(num_joints);
		msg.goal.trajectory.points[i].velocities.resize(num_joints);
		msg.goal.trajectory.points[i].accelerations.resize(num_joints);
		msg.goal.trajectory.points[i].time_from_start=ros::Duration(sec,nsec);
		for(int j=0;j<num_joints;j++){
			if (traj_points[count]!=traj_points[count]) all_correct=false; //NAN CHECK
			msg.goal.trajectory.points[i].positions[j]=traj_points[count];
			msg.goal.trajectory.points[i].velocities[j]=1.0;
			msg.goal.trajectory.points[i].accelerations[j]=1.0;
			count++;
		}
	}
	if (all_correct){
		printf("msg ready, sending\n");
		if (ch==0) jointactiongoalpublisher.publish(msg);
		else gripperactiongoalpublisher.publish(msg);
		printf("msg sent ok\n");
	}else{
		printf("NAN AT JOINTANGLES!!!\n");
	}
	// jointactiongoalpublisher.publish(msg);
	return 0;
}


static int lua_jointtrajectory(lua_State *L){
	std::vector<std::string> names=lua_checkstringvector(L,1);
	std::vector<double>traj_points=lua_checkvector(L,2);
	int ch=luaL_optnumber(L,3,0);

	int num_joints=names.size();
	trajectory_msgs::JointTrajectory msg;
	msg.header.seq=1;
	msg.points.resize(1);
	msg.points[0].positions.resize(num_joints);
	for(int i=0;i<num_joints;i++){
    msg.joint_names.push_back(names[i]);
		msg.points[0].positions[i]=traj_points[i];
		msg.points[0].time_from_start=ros::Duration(1.0/60.0);
  }
	msg.header.stamp=ros::Time::now();
	if(ch==0)	jointtrajectorypublisher.publish(msg);
	else jointtrajectorypublisher2.publish(msg);
	return 0;
}



static int lua_jointtrajectories(lua_State *L){
	std::vector<std::string> names=lua_checkstringvector(L,1);
	std::vector<double>trajlist=lua_checkvector(L,2);
	int ch=luaL_optnumber(L,3,2);
	int num_joints=names.size();
	int num_traj=trajlist.size()/num_joints;

	trajectory_msgs::JointTrajectory msg;
	msg.header.stamp=ros::Time::now();
	msg.header.seq=1;
	msg.points.resize(num_traj);
	for(int i=0;i<num_joints;i++) msg.joint_names.push_back(names[i]);
	int count=0;
	for(int i=0;i<num_traj;i++){
		msg.points[i].positions.resize(num_joints);
		msg.points[i].time_from_start=ros::Duration(1.0/60.0);
		for(int j=0;j<num_joints;j++) msg.points[i].positions[j]=trajlist[count++];
  }
	if(ch==0)	jointtrajectorypublisher.publish(msg);
	else{
		if(ch==1)	jointtrajectorypublisher2.publish(msg);
		else jointtrajectorypublisher3.publish(msg);
	}
	printf("DONE!\n");
	return 0;
}









static int lua_displayjpg(lua_State *L){
	const char* jpgname=luaL_checkstring(L,1);
	char buf[255];
	int len_str=strlen(jpgname);
	// printf("String length:%d\n",len_str);
	// memcpy(buf,jpgname,len_str+1);
	std_msgs::String msg;
	msg.data=jpgname;
	// msg.data=buf;
	// printf("here");
	displayjpg_publisher.publish(msg);
	return 0;
}

static int lua_imagecrop(lua_State *L){ //seq w h data encoding channel framename
	int seq=lua_tonumber(L, 1);
	int width=lua_tonumber(L, 2);
	int height=lua_tonumber(L, 3);
	const char* data=lua_tostring(L, 4);
	int x0=lua_tonumber(L, 5);
	int x1=lua_tonumber(L, 6);
	int y0=lua_tonumber(L, 7);
	int y1=lua_tonumber(L, 8);
	int ch=luaL_optnumber(L,9,0);
	const char *frame=luaL_optstring(L,10,"head_rgbd_sensor_rgb_frame_sync");

	sensor_msgs::Image msg;
  msg.header.seq=seq;
  msg.header.stamp=ros::Time::now();
  msg.header.frame_id=frame;
  msg.width=x1-x0+1;
  msg.height=y1-y0+1;
  msg.is_bigendian=0;
  msg.encoding="rgb8";
  msg.step = width*3;
  msg.data.resize(msg.width*msg.height*3);
	for(int y=y0;y<=y1;y++){ //height
		for(int x=x0;x<=x1;x++){ //width
			int index1=y*width+x;
			int index2=(y-y0)*msg.width + (x-x0);
			memcpy(&msg.data[index2*3],&data[index1*3],3);
		}
	}
  image_publisher[ch].publish(msg);

	lua_pushnumber(L, msg.width);
	lua_pushnumber(L, msg.height);
	lua_pushlstring(L, (char*) &msg.data[0], msg.data.size());
	return 3;
}




static const struct luaL_Reg rospub_lib[] = {
  {"init", lua_init},
	{"init_custom", lua_init_custom},
  {"tf", lua_tf},

	{"image",lua_image},
	{"imageonly",lua_imageonly},
	{"imagecrop",lua_imagecrop},
	{"savergbimage",lua_savergbimage},
	{"jpgimage",lua_jpgimage},
	{"imagefrompng",lua_imagefrompng},

	{"jointstate",lua_jointstate},

	{"mapcmd",lua_mapcmd},
	{"posereset", lua_posereset},
	{"basegoal", lua_basegoal},
	{"navigation_status",lua_navigation_status},

	{"odom", lua_odom},
	{"cmd_vel", lua_cmd_vel},
	{"teleop_vel", lua_teleop_vel},

	// {"rostime",lua_rostime},

	// {"arm_vel6D",lua_arm_vel6D},	//Twist, 6D direct velocity control
	// {"arm_move6D",lua_arm_move6D},	//relative 6D cartesian movement control
	// {"arm_vellimit",lua_arm_vellimit},	//Set joint angle velocity limits
	// {"arm_jointtargets",lua_arm_jointtargets},//joint angle trajectory control
	// {"arm_stop",lua_arm_stop},//Stop arm immediately
	//
	// //Sensor feeds (from webots or robot)

	{"laserscan",lua_laserscan},

	// //mapping and navigation
	{"occgrid",lua_occgrid},	//publish occgrid
	{"path",lua_path},				//publish path
	{"marker",lua_marker},
	{"marker3d",lua_marker3d},

	{"markermatrix",lua_markermatrix},
	{"markermatrixcolor",lua_markermatrixcolor},
	{"markerlines",lua_markerlines},

	{"camerainfo",lua_camerainfo},
	{"camerainfo2",lua_camerainfo2},
	{"jointactiongoal",lua_jointactiongoal},
	{"jointtrajectory",lua_jointtrajectory},
	{"jointtrajectories",lua_jointtrajectories},

	{"displayjpg",lua_displayjpg},

  {NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
int luaopen_rospub2(lua_State *L){
	luaL_register(L, "rospub", rospub_lib);
  return 1;
}
