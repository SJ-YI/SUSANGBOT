#include <lua.hpp>
#ifdef __cplusplus
extern "C"
#endif

#include "navigation.h"
// #include "Transform.h"


static std::vector<double> lua_checkvector(lua_State *L, int narg) {
	if ( !lua_istable(L, narg) )	luaL_argerror(L, narg, "vector");
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
	return v;
}
static void lua_pusharray(lua_State *L, float* v, int n) {
	lua_createtable(L, n, 0);
	for (int i = 0; i < n; i++) {
		lua_pushnumber(L, v[i]);
		lua_rawseti(L, -2, i+1);
	}
}


int lua_init(lua_State *L){
  init_lomap();
	return 0;
}



int lua_process_smap(lua_State *L){
  smap_res = luaL_checknumber(L, 1);
  smap_x = luaL_checknumber(L, 2);
  smap_y = luaL_checknumber(L, 3);
  smap_x0 = luaL_checknumber(L, 4);
  smap_y0 = luaL_checknumber(L, 5);
  smap_z0 = luaL_checknumber(L, 6);
  const char* data=luaL_checkstring(L, 7);
	int downsample_factor=luaL_optnumber(L,8,1);
	int increase_factor=luaL_optnumber(L,9,3);

  memcpy(smap,data,smap_x*smap_y);
  printf("Static map load completee %.3f x%d y%d zero (%.2f,%.2f,%.2f)\n",smap_res, smap_x,smap_y, smap_x0,smap_y0,smap_z0);
  //0: free space 100: obstacle 255: unknown
  // for(int i=0;i<smap_x*smap_y;i++){if (smap[i]!=255) printf("[%d]",smap[i]);}
  downsample_smap(downsample_factor,increase_factor);
  return 0;
}


int lua_process_smap2(lua_State *L){ //FOR TURTLEBOT
  smap_res = luaL_checknumber(L, 1);
  smap_x = luaL_checknumber(L, 2);
  smap_y = luaL_checknumber(L, 3);
  smap_x0 = luaL_checknumber(L, 4);
  smap_y0 = luaL_checknumber(L, 5);
  smap_z0 = luaL_checknumber(L, 6);

  lomap_res=smap_res*3.0;
  lomap_x0=-lomap_res*((float)lomap_size-1)/2;
  lomap_y0=lomap_x0;
  const char* data=luaL_checkstring(L, 7);
  memcpy(smap,data,smap_x*smap_y);
  //0: free space 100: obstacle 255: unknown
  // for(int i=0;i<smap_x*smap_y;i++){if (smap[i]!=255) printf("[%d]",smap[i]);}

  // for(int i=0;i<smap_y;i+=10){
  //   for(int j=0;j<smap_x;j+=10) printf("%3d",smap[i*smap_x+j]);
  //   printf("\n");
  // }
  // for(int i=0;i<smap_y;i++){
  //   for(int j=0;j<smap_x;j++)
  //     if (smap[i*smap_x+j]==-1) smap[i*smap_x+j]==0;
  // }
  downsample_smap2();
  return 0;
}



int lua_pathplan(lua_State *L){
  std::vector<double> pose=lua_checkvector(L,1);
  std::vector<double> target=lua_checkvector(L,2);
	float max_c=luaL_checknumber(L, 3);
  update_valuemap(pose[0],pose[1],pose[2],max_c);
  int ret=pathplan(pose[0],pose[1], target[0],target[1],0);
  if (ret>0){
    lua_pusharray(L,xpath,path_num);
    lua_pusharray(L,ypath,path_num);
    lua_pusharray(L,xpaths,path_num_s);
    lua_pusharray(L,ypaths,path_num_s);
    return 4;
  }else return 0;
}

int lua_get_lomap(lua_State *L){
  lua_pushnumber(L,lomap_res);
  lua_pushnumber(L,lomap_size);
  lua_pushnumber(L,lomap_size);
  lua_pushnumber(L,lomap_x0-lomap_res/2.0);
  lua_pushnumber(L,lomap_y0-lomap_res/2.0);
  lua_pushnumber(L,0);
  lua_pushlstring(L, (char*) lomap, lomap_size*lomap_size);
  return 7;
}

int lua_reset_lomap(lua_State *L){
	const char* data=luaL_checkstring(L, 1);
  memcpy(lomap, data, lomap_size*lomap_size);
	return 0;
}

int lua_add_obstacle(lua_State *L){
	float obs_x = luaL_checknumber(L, 1);
	float obs_y = luaL_checknumber(L, 2);
	add_obstacle(obs_x,obs_y);
	return 0;
}


int lua_check_empty_space(lua_State *L){
  std::vector<double> target=lua_checkvector(L,1);
	float radius=luaL_checknumber(L, 2);
  int ret=check_empty_space(target[0],target[1],radius);
	lua_pushnumber(L,ret);
	return 1;
}


int lua_init_blank_smap(lua_State *L){
  smap_res = luaL_checknumber(L, 1);
  smap_x = luaL_checknumber(L, 2);
  smap_y = luaL_checknumber(L, 3);
  smap_x0 = luaL_checknumber(L, 4);
  smap_y0 = luaL_checknumber(L, 5);
  smap_z0 = luaL_checknumber(L, 6);
	int downsample_factor=luaL_optnumber(L,7,1);
	int increase_factor=luaL_optnumber(L,8,3);
  // memset(smap,-1,smap_x*smap_y);
	memset(smap,0,smap_x*smap_y);
  downsample_smap(downsample_factor,increase_factor);
  return 0;
}


int lua_update_rgbd(lua_State *L){
	int num_pos=luaL_checknumber(L, 1);
	float* xpos = (float*) lua_tostring(L,2);
	float* ypos = (float*) lua_tostring(L,3);
	std::vector<double> pose=lua_checkvector(L,4);
	float radius = smap_z0 = luaL_checknumber(L, 5);

	for(int i=0;i<num_pos;i++){
		float dx,dy;
		dx=pose[0] + cos(pose[2])*xpos[i] -sin(pose[2])*ypos[i];
		dy=pose[1] + sin(pose[2])*xpos[i] +cos(pose[2])*ypos[i];
		add_obstacle_direct(dx,dy,radius);
	}

	return 0;
}





static const struct luaL_Reg lib [] = {
	{"init", lua_init},
  {"process_smap", lua_process_smap},
  {"process_smap2", lua_process_smap2}, //for turtlebot
  {"get_lomap", lua_get_lomap},
  {"pathplan", lua_pathplan},

	{"reset_lomap", lua_reset_lomap},
	{"add_obstacle", lua_add_obstacle},

	{"check_empty_space", lua_check_empty_space},
  // {"process_dmap", lua_process_dmap},

	{"init_blank_smap", lua_init_blank_smap},
	{"update_rgbd", lua_update_rgbd},

	{NULL, NULL}
};


#ifdef __cplusplus
extern "C"
#endif


int luaopen_navigation (lua_State *L) {
#if LUA_VERSION_NUM == 502
	luaL_newlib(L, lib);
#else
	luaL_register(L, "navigation", lib);
#endif
	return 1;
}
