/*
  Lua module to provide keyboard input
*/
#include <lua.hpp>
#ifdef __cplusplus
extern "C"
#endif

#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
#include <unistd.h>
#include "dynamixel_sdk.h"

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel
#define BAUDRATE                        1000000

//CORRECT X SERIES VALUE!!!
#define ADDR_X_CONTROL_MODE   11//0 for current, 1 for velocity, 3 for pos, 4 for multi-turn pos, 5 for curret-based pos
#define ADDR_X_TORQUE_ENABLE  64                 // Control table address is different in Dynamixel model
#define ADDR_X_LED 65
#define ADDR_X_GOAL_CURRENT          102
#define ADDR_X_GOAL_VELOCITY          104
#define ADDR_X_GOAL_POSITION          116
#define ADDR_X_PRESENT_POSITION       132


#define ADDR_X_P_GAIN       84

// Data Byte Length
#define LEN_X_LED_RED                 1
#define LEN_X_GOAL_POSITION           4
#define LEN_X_GOAL_VELOCITY           4
#define LEN_X_GOAL_CURRENT            2
#define LEN_X_PRESENT_POSITION        4
#define LEN_X_P_GAIN            2


#define ADDR_PRO_ACC_LIMIT 26 //4 byte acc limit, 0 to 2147483647
#define ADDR_PRO_TORQUE_LIMIT 30 //2 byte, 0 to 1860
#define ADDR_PRO_TORQUE_ENABLE 512 //1 byte
#define ADDR_PRO_GOAL_VELOCITY 552 //4 bytes
#define ADDR_PRO_GOAL_POSITION 564 //4 bytes
#define ADDR_PRO_GOAL_TORQUE 550 //2 bytes
#define ADDR_PRO_PRESENT_POSITION 580 //4 bytes
#define ADDR_PRO_PRESENT_VOLTAGE 623 //2 bytes
#define ADDR_PRO_PRESENT_TEMPERATURE 625 //1 bytes


int port_num=0;
int groupwrite_num=0;
int groupread_num=0;


int dxl_comm_result = COMM_TX_FAIL;
std::vector<uint8_t> vec;                       // Dynamixel data storages


static dynamixel::PortHandler *portHandler;
static dynamixel::PacketHandler *packetHandler;
static dynamixel::GroupBulkWrite *gBW;
static dynamixel::GroupBulkRead *gBR;

static dynamixel::GroupSyncWrite *gSW;
static dynamixel::GroupSyncWrite *gSW_vel;
static dynamixel::GroupSyncWrite *gSW_current;
static dynamixel::GroupSyncWrite *gSW_pgain;
static dynamixel::GroupSyncRead *gSR;

static dynamixel::GroupSyncWrite *gSW_pro_pos;
static dynamixel::GroupSyncWrite *gSW_pro_vel;
static dynamixel::GroupSyncRead *gSR_pro_pos;


uint8_t dxl_error = 0;
bool dxl_addparam_result = false;
bool dxl_getdata_result = false;


static void lua_pusharray(lua_State *L, float* v, int n) {
	lua_createtable(L, n, 0);
	for (int i = 0; i < n; i++) {
		lua_pushnumber(L, v[i]);
		lua_rawseti(L, -2, i+1);
	}
}

static std::vector<double> lua_checkvector(lua_State *L, int narg) {
	if ( !lua_istable(L, narg) )	luaL_argerror(L, narg, "vector");
	int n = lua_objlen(L, narg);
	std::vector<double> v(n);
	for (int i = 0; i < n; i++) {
		lua_rawgeti(L, narg, i+1);
		v[i] = lua_tonumber(L, -1);
		lua_pop(L, 1);
	}
	return v;
}


static int lua_openport(lua_State *L){

  const char* devname=lua_tostring(L,1);
  int baudrate = lua_tonumber(L, 2);

  portHandler = dynamixel::PortHandler::getPortHandler(devname);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  if (portHandler->openPort())  printf("Succeeded to open the port %s\n",devname);
  else{printf("Failed to open the port!\n");return 0;}
  if (portHandler->setBaudRate(baudrate))  printf("Succeeded to connect with baudrate %d\n",baudrate);
  else{printf("Failed to change the baudrate!\n");return 0;}
  gBR =new dynamixel::GroupBulkRead(portHandler, packetHandler);
  gBW =new dynamixel::GroupBulkWrite(portHandler, packetHandler);

  gSW = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_X_GOAL_POSITION, LEN_X_GOAL_POSITION);
  gSW_vel = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_X_GOAL_VELOCITY, LEN_X_GOAL_VELOCITY);
  gSW_current = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_X_GOAL_CURRENT, LEN_X_GOAL_CURRENT);
  gSW_pgain = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_X_P_GAIN, LEN_X_P_GAIN);

  gSR = new dynamixel::GroupSyncRead(portHandler, packetHandler, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);

	gSW_pro_pos = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_X_GOAL_POSITION);
	gSW_pro_vel = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_VELOCITY, LEN_X_GOAL_VELOCITY);
  gSR_pro_pos = new dynamixel::GroupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_X_PRESENT_POSITION);

  lua_pushnumber(L,1);
  return 1;
}
//
static int lua_pingprobe(lua_State *L){
  dxl_comm_result = packetHandler->broadcastPing(portHandler, vec);
  if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  printf("Detected Dynamixel : \n");
  for (int i = 0; i < (int)vec.size(); i++) printf("[ID:%03d]\n", vec.at(i));
  return 0;
}

static int lua_control_mode(lua_State *L){
  std::vector<double> dxl_ids=lua_checkvector(L,1);
  std::vector<double> control_modes=lua_checkvector(L,2);
  for (int i=0;i<dxl_ids.size();i++){
    int dxl_id = (int) dxl_ids[i];
    int control_mode=(int) control_modes[i];
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_X_CONTROL_MODE , control_mode, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)  printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    else if (dxl_error != 0)   printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    else printf("Dynamixel#%d control mode set to %d \n", dxl_id,control_mode);
		usleep(1E6*0.02);
  }
  return 0;
}

static int lua_torque_enable(lua_State *L){
  std::vector<double> dxl_ids=lua_checkvector(L,1);
  std::vector<double> torque_enables=lua_checkvector(L,2);
  for (int i=0;i<dxl_ids.size();i++){
    int dxl_id = (int) dxl_ids[i];
    int torque_enable=(int) torque_enables[i];
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_X_TORQUE_ENABLE, torque_enable, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)  printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    else if (dxl_error != 0)   printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    else printf("Dynamixel#%d torque has been successfully set to %d \n", dxl_id,torque_enable);
		usleep(1E6*0.02);
  }
  return 0;
}
//
//
static int lua_read_position(lua_State *L){
  static std::vector<double> dxl_ids=lua_checkvector(L,1);
  int i;
  gBR->clearParam();
  for (i=0;i<dxl_ids.size();i++){
    int dxl_id=(int) dxl_ids[i];
    dxl_addparam_result = gBR->addParam(dxl_id, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
    if (dxl_addparam_result != true)  {
      fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed", dxl_id);
      return 0;
    }
  }
  dxl_comm_result = gBR->txRxPacket(); //send bulk read packet
  if (dxl_comm_result != COMM_SUCCESS)  printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  std::vector<double> ret;
  for (i=0;i<dxl_ids.size();i++){
    int dxl_id=(int) dxl_ids[i];
    dxl_getdata_result = gBR->isAvailable(dxl_id, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
    if (dxl_getdata_result != true){fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed", dxl_id);
      return 0;
    }else{
      int32_t present_position = gBR->getData(dxl_id, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
      ret.push_back(present_position);
    }
  }
  //return the array
  lua_createtable(L, ret.size(), 0);
	for (int i = 0; i < ret.size(); i++) {
		lua_pushnumber(L, ret[i]);
		lua_rawseti(L, -2, i+1);
	}
  return 1;
}

static int lua_read_position2(lua_State *L){

  static std::vector<double> dxl_ids=lua_checkvector(L,1);
  int i;
  gSR->clearParam();
  for (i=0;i<dxl_ids.size();i++){
    int dxl_id=(int) dxl_ids[i];
    dxl_addparam_result = gSR->addParam(dxl_id);
    if (dxl_addparam_result != true)  {fprintf(stderr, "[ID:%03d] grouSyncRead addparam failed", dxl_id);return 0;}
  }
  dxl_comm_result = gSR->txRxPacket(); //send bulk read packet
  if (dxl_comm_result != COMM_SUCCESS)  printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  std::vector<double> ret;
  for (i=0;i<dxl_ids.size();i++){
    int dxl_id=(int) dxl_ids[i];
    dxl_getdata_result = gSR->isAvailable(dxl_id, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
    if (dxl_getdata_result != true){fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", dxl_id);
      return 0;
    }else{
      int32_t present_position = gSR->getData(dxl_id, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
      ret.push_back(present_position);
    }
  }
  lua_createtable(L, ret.size(), 0); //return the array
	for (int i = 0; i < ret.size(); i++) {
		lua_pushnumber(L, ret[i]);
		lua_rawseti(L, -2, i+1);
	}
  return 1;
}



static int lua_set_goal_positions(lua_State *L){
  std::vector<double> dxl_ids=lua_checkvector(L,1);
  std::vector<double> dxl_goal_positions=lua_checkvector(L,2);
  int i;

  for (i=0;i<dxl_ids.size();i++){
    int dxl_id=(int) dxl_ids[i];
    double goal_position=dxl_goal_positions[i];
//    int dxl_goal_position = 2048 + (int) (goal_position/3.141592/2.0 * 4096.0) ;
    int dxl_goal_position =  (int) goal_position;

    uint8_t param_goal_position[4];
    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position));
    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position));
    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position));
    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position));
    dxl_addparam_result = gBW->addParam(dxl_id, ADDR_X_GOAL_POSITION, LEN_X_GOAL_POSITION, param_goal_position);
    if (dxl_addparam_result != true) {
     fprintf(stderr, "[ID:%03d] groupBulkWrite addparam failed", dxl_id);
     return 0;
    }
  }
  dxl_comm_result = gBW->txPacket();
  if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  gBW->clearParam();
  return 0;
}

static int lua_set_goal_positions2(lua_State *L){
  std::vector<double> dxl_ids=lua_checkvector(L,1);
  std::vector<double> dxl_goal_positions=lua_checkvector(L,2);
  for (int i=0;i<dxl_ids.size();i++){
    int dxl_id=(int) dxl_ids[i];
    uint8_t param_goal_position[4];
    double goal_position=dxl_goal_positions[i];
//    int dxl_goal_position = 2048 + (int) (goal_position/3.141592/2.0 * 4096.0) ;
    int dxl_goal_position = (int) goal_position;
    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position));
    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position));
    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position));
    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position));
    dxl_addparam_result = gSW->addParam(dxl_id, param_goal_position);
    if (dxl_addparam_result != true) {
     fprintf(stderr, "[ID:%03d] groupBulkWrite addparam failed", dxl_id);
     return 0;
    }
  }
  dxl_comm_result = gSW->txPacket();
  if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  gSW->clearParam();
  return 0;
}

static int lua_set_goal_velocities(lua_State *L){
  std::vector<double> dxl_ids=lua_checkvector(L,1);
  std::vector<double> dxl_goal_velocities=lua_checkvector(L,2);
  for (int i=0;i<dxl_ids.size();i++){
    int dxl_id=(int) dxl_ids[i];
    uint8_t param_goal_velocity[4];
    int dxl_goal_velocity = (int) dxl_goal_velocities[i];
    param_goal_velocity[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_velocity));
    param_goal_velocity[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_velocity));
    param_goal_velocity[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_velocity));
    param_goal_velocity[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_velocity));
    dxl_addparam_result = gSW_vel->addParam(dxl_id, param_goal_velocity);
    if (dxl_addparam_result != true) {
     fprintf(stderr, "[ID:%03d] groupBulkWrite addparam failed", dxl_id);
     return 0;
    }
  }
  dxl_comm_result = gSW_vel->txPacket();
  if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  gSW_vel->clearParam();
  return 0;
}

static int lua_set_goal_currents(lua_State *L){
  std::vector<double> dxl_ids=lua_checkvector(L,1);
  std::vector<double> dxl_goal_currents=lua_checkvector(L,2);
  for (int i=0;i<dxl_ids.size();i++){
    int dxl_id=(int) dxl_ids[i];
    uint8_t param_goal_current[2];
    int dxl_goal_current = (int) dxl_goal_currents[i];
    param_goal_current[0] = DXL_LOBYTE(dxl_goal_current);
    param_goal_current[1] = DXL_HIBYTE(dxl_goal_current);
    dxl_addparam_result = gSW_current->addParam(dxl_id, param_goal_current);
    if (dxl_addparam_result != true) {
     fprintf(stderr, "[ID:%03d] sync_write_current addparam failed", dxl_id);
     return 0;
    }
  }
  dxl_comm_result = gSW_current->txPacket();
  if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  gSW_current->clearParam();
  return 0;
}


static int lua_set_p_gain(lua_State *L){
  std::vector<double> dxl_ids=lua_checkvector(L,1);
  std::vector<double> dxl_goal_currents=lua_checkvector(L,2);
  for (int i=0;i<dxl_ids.size();i++){
    int dxl_id=(int) dxl_ids[i];
    uint8_t param_goal_current[2];
    int dxl_goal_current = (int) dxl_goal_currents[i];
    param_goal_current[0] = DXL_LOBYTE(dxl_goal_current);
    param_goal_current[1] = DXL_HIBYTE(dxl_goal_current);
    dxl_addparam_result = gSW_pgain->addParam(dxl_id, param_goal_current);
    if (dxl_addparam_result != true) {
     fprintf(stderr, "[ID:%03d] sync_write_current addparam failed", dxl_id);
     return 0;
    }
  }
  dxl_comm_result = gSW_pgain->txPacket();
  if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  gSW_pgain->clearParam();
  return 0;
}


static int lua_closeport(lua_State *L){
  portHandler->closePort();
  return 0;
}



static int lua_pro_torque_enable(lua_State *L){
  std::vector<double> dxl_ids=lua_checkvector(L,1);
  std::vector<double> torque_enables=lua_checkvector(L,2);
  for (int i=0;i<dxl_ids.size();i++){
    int dxl_id = (int) dxl_ids[i];
    int torque_enable=(int) torque_enables[i];
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_PRO_TORQUE_ENABLE, torque_enable, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)  printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    else if (dxl_error != 0)   printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    else printf("Dynamixel Pro #%d torque has been successfully set to %d \n", dxl_id,torque_enable);
		usleep(1E6*0.02);
  }
  return 0;
}

static int lua_pro_read_position(lua_State *L){
  static std::vector<double> dxl_ids=lua_checkvector(L,1);
  int i;
  gSR_pro_pos->clearParam();
  for (i=0;i<dxl_ids.size();i++){
    int dxl_id=(int) dxl_ids[i];
    dxl_addparam_result = gSR_pro_pos->addParam(dxl_id);
    if (dxl_addparam_result != true)  {fprintf(stderr, "[ID:%03d] grouSyncRead addparam failed", dxl_id);return 0;}
  }
  dxl_comm_result = gSR_pro_pos->txRxPacket(); //send bulk read packet
  if (dxl_comm_result != COMM_SUCCESS)  printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  std::vector<double> ret;
  for (i=0;i<dxl_ids.size();i++){
    int dxl_id=(int) dxl_ids[i];
    dxl_getdata_result = gSR_pro_pos->isAvailable(dxl_id, ADDR_PRO_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
    if (dxl_getdata_result != true){fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", dxl_id);
      return 0;
    }else{
      int32_t present_position = gSR_pro_pos->getData(dxl_id, ADDR_PRO_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
      ret.push_back(present_position);
    }
  }
  lua_createtable(L, ret.size(), 0); //return the array
	for (int i = 0; i < ret.size(); i++) {
		lua_pushnumber(L, ret[i]);
		lua_rawseti(L, -2, i+1);
	}
  return 1;
}


static int lua_pro_set_goal_velocities(lua_State *L){
  std::vector<double> dxl_ids=lua_checkvector(L,1);
  std::vector<double> dxl_goal_velocities=lua_checkvector(L,2);
	gSW_pro_vel->clearParam();
  for (int i=0;i<dxl_ids.size();i++){
    int dxl_id=(int) dxl_ids[i];
    uint8_t param_goal_velocity[4];
    int dxl_goal_velocity = (int) dxl_goal_velocities[i];
    param_goal_velocity[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_velocity));
    param_goal_velocity[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_velocity));
    param_goal_velocity[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_velocity));
    param_goal_velocity[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_velocity));
    dxl_addparam_result = gSW_pro_vel->addParam(dxl_id, param_goal_velocity);
    if (dxl_addparam_result != true) {
     fprintf(stderr, "[ID:%03d] groupBulkWrite addparam failed", dxl_id);
     return 0;
    }
  }
  dxl_comm_result = gSW_pro_vel->txPacket();
  if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  gSW_vel->clearParam();
  return 0;
}

static int lua_pro_set_goal_positions(lua_State *L){
  std::vector<double> dxl_ids=lua_checkvector(L,1);
  std::vector<double> dxl_goal_positions=lua_checkvector(L,2);
  for (int i=0;i<dxl_ids.size();i++){
    int dxl_id=(int) dxl_ids[i];
    uint8_t param_goal_position[4];
    double goal_position=dxl_goal_positions[i];
    int dxl_goal_position = (int) goal_position;
    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position));
    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position));
    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position));
    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position));
    dxl_addparam_result = gSW_pro_pos->addParam(dxl_id, param_goal_position);
    if (dxl_addparam_result != true) {
     fprintf(stderr, "[ID:%03d] groupBulkWrite addparam failed", dxl_id);
     return 0;
    }
  }
  dxl_comm_result = gSW_pro_pos->txPacket();
  if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  gSW_pro_pos->clearParam();
  return 0;
}




static const struct luaL_Reg DynamixelNew_lib [] = {
  {"openport", lua_openport},
  {"ping_probe", lua_pingprobe},
  {"torque_enable", lua_torque_enable},
  {"control_mode", lua_control_mode},

  {"read_position", lua_read_position},
  {"read_position2", lua_read_position2},
  {"set_goal_positions", lua_set_goal_positions},
  {"set_goal_positions2", lua_set_goal_positions2},

  {"set_goal_velocities", lua_set_goal_velocities},
  {"set_goal_currents", lua_set_goal_currents},

  {"set_p_gain", lua_set_p_gain},
  {"closeport", lua_closeport},


	{"pro_torque_enable", lua_pro_torque_enable},
	{"pro_read_position", lua_pro_read_position},
	{"pro_set_goal_velocities", lua_pro_set_goal_velocities},
	{"pro_set_goal_positions", lua_pro_set_goal_positions},


  {NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
int luaopen_DynamixelNew (lua_State *L) {
#if LUA_VERSION_NUM == 502
	  luaL_newlib(L, DynamixelNew_lib);
#else
	luaL_register(L, "DynamixelNew", DynamixelNew_lib);
#endif
  return 1;
}
