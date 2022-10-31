/*
Lua file to send and receive UDP messages.
Daniel D. Lee copyright 2009 <ddlee@seas.upenn.edu>
Stephen G. McGill copyright 2013 <smcgill3@seas.upenn.edu>
Yida Zhang copyright 2013 <yida@seas.upenn.edu>
*/


// Based on codes in https://www.geeksforgeeks.org/socket-programming-cc/

#include <string>
#include <cstring>
#include <deque>
#include <map>
#include <vector>
#include <algorithm>

#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <errno.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <assert.h>

#include <lua.hpp>


#define MT_NAME "tcp_mt"

using namespace std;


std::vector<double> lua_checkvector(lua_State *L, int narg) {
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

static void lua_pusharray(lua_State *L, float* v, int n) {
	lua_createtable(L, n, 0);
	for (int i = 0; i < n; i++) {
		lua_pushnumber(L, v[i]);
		lua_rawseti(L, -2, i+1);
	}
}


static int lua_tcp_connect(lua_State *L) {
  const char *ipaddr = lua_tostring(L, 1);
  int port=lua_tonumber(L, 2);
  int sock=0;
  struct sockaddr_in serv_addr;
  if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0){
    printf("\n Socket creation error \n");
    return 0;
  }
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(port);
    // Convert IPv4 and IPv6 addresses from text to binary form
  if(inet_pton(AF_INET, ipaddr, &serv_addr.sin_addr)<=0){
    printf("\nInvalid address/ Address not supported \n");
    return 0;
  }
  if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0){
    printf("\nConnection Failed \n");
    return 0;
  }
  lua_pushnumber(L,sock);
  return 1;
}


static int lua_tcp_client_send(lua_State *L) {
  int sock=lua_tonumber(L, 1);
  const char* data=lua_tostring(L, 2);
  int datalen=lua_tonumber(L,3);
  send(sock , data , datalen , 0 );

  char buffer[1024] = {0};
  int valread = read( sock , buffer, 1024);
  printf("%s\n",buffer );
  lua_pushnumber(L,1);
	return 1;
}


static int lua_hyuhand_send(lua_State *L) {
  int sock=lua_tonumber(L, 1);
  //duty rate, -1 to 1
  std::vector<double> finger1=lua_checkvector(L,2);
  std::vector<double> finger2=lua_checkvector(L,3);
  std::vector<double> finger3=lua_checkvector(L,4);
  std::vector<double> finger4=lua_checkvector(L,5);

	//direction correction
	finger2[3]=-finger2[3];
	finger3[3]=-finger3[3];
	finger4[3]=-finger4[3];

  uint8_t data[33];
  data[0]=0;
  for (int i=0;i<4;i++){
    uint16_t duty1=finger1[i]*32767+0x8000;
    uint16_t duty2=finger2[i]*32767+0x8000;
    uint16_t duty3=finger3[i]*32767+0x8000;
    uint16_t duty4=finger4[i]*32767+0x8000;

    data[2*i+1] = (duty1>>8)&0x00ff;
    data[2*i+2] = duty1&0x00ff;

    data[2*i+1+8] = (duty2>>8)&0x00ff;
    data[2*i+2+8] = duty2&0x00ff;

    data[2*i+1+16] = (duty3>>8)&0x00ff;
    data[2*i+2+16] = duty3&0x00ff;

    data[2*i+1+24] = (duty4>>8)&0x00ff;
    data[2*i+2+24] = duty4&0x00ff;
  }

  send(sock , data , 33 , 0 );
  uint8_t buffer[1024] = {0};
  int valread = read( sock , (char*) buffer, 1024);

  // for (int i=0;i<4;i++){
  for (int i=0;i<4;i++){
		float ret[4];
    ret[0]= ( (buffer[8*i]<<8) |  buffer[8*i+1])/298.0;
    ret[1]=( (buffer[8*i+2]<<8) |  buffer[8*i+3])/298.0;
    ret[2]=( (buffer[8*i+4]<<8) |  buffer[8*i+5])/298.0;
    ret[3]=( (buffer[8*i+6]<<8) |  buffer[8*i+7])/298.0;
		lua_pusharray(L,ret,4);
		//
    // printf("%.1f %.1f %.1f %.1f\n",ret1,ret2,ret3,ret4);

  }
  // printf("RET:%d\n",valread );
  // lua_pushnumber(L,1);
  return 4;
}


static const struct luaL_Reg tcp_lib [] = {
  {"connect", lua_tcp_connect},
	{"client_send", lua_tcp_client_send},
  {"hyuhand_send", lua_hyuhand_send},
	{NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
int luaopen_tcp (lua_State *L) {
  luaL_newmetatable(L, MT_NAME);
	luaL_register(L, "tcp", tcp_lib);
	return 1;
}
