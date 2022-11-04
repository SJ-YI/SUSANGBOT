#!/usr/bin/env luajit
local pwd = os.getenv'PWD'
if string.find(pwd,"Run") then	dofile('../include.lua')
else	dofile('../../include.lua') end
local unix=require'unix'
local vector = require'vector'
local DXL = require 'DynamixelNew'
local util=require'util'


local rossub = require 'rossub'
rossub.init('lua_armdriver_sub')
local sub_idx_joint=rossub.subscribeJoint('/open_manipulator/goal_joint_position')


-------------------------------------------------------------------------------
local dev_name, baudrate="/dev/usb2dxl",1000000
local all_servo_ids={10,11,12,13,14}
local position_servo_ids={10,11,12,13}
local velocity_servo_ids={}
local current_servo_ids={10,11,12,13,14}
local max_servo_velocity = 60*DEG_TO_RAD --60 deg per sec max speed
-------------------------------------------------------------------------------

local position_servo_ids={10,12,13}



local all_servo_ids={10}




local position=vector.zeros(#all_servo_ids)
local command_position=vector.zeros(#all_servo_ids)
local target_position=vector.new({0,0,0,0,0})*DEG_TO_RAD
--control mode
--0 for current
--1 for velocity
--3 for pos
--4 for multi-turn pos
--5 for curret-based pos

DXL.openport(dev_name, baudrate)
DXL.ping_probe()
DXL.torque_enable(all_servo_ids,vector.zeros(#all_servo_ids))
DXL.control_mode(all_servo_ids,{5,0,5,5,0})

--DXL.torque_enable(all_servo_ids,vector.zeros(#all_servo_ids))
--DXL.control_mode(all_servo_ids,{3,3,3,3,0})

DXL.torque_enable(all_servo_ids,vector.ones(#all_servo_ids))
command_position = vector.new(DXL.read_position2(all_servo_ids))
target_position = vector.new(DXL.read_position2(all_servo_ids))



DXL.set_p_gain(position_servo_ids,{800,800,800,800})
DXL.set_goal_currents( current_servo_ids,{100, 10,800,800,0})

-- unix.usleep(1E6*1)
-- DXL.set_goal_currents( current_servo_ids,{-100}) --open
-- unix.usleep(1E6*1)
-- DXL.set_goal_currents( current_servo_ids,{0}) --open

local count=0
t0 = unix.time()
t_last=unix.time()
local old_ret=DXL.read_position2({command_positions})
while 1 do
  local pos,vel,eff= rossub.checkJointPos(sub_idx_joint)
  if pos then
    print("GOAL POSITION INPUT!!!")
    target_position=pos
  end
  local t = unix.time()
  local dt=(t-t_last+0.0000001)
  t_last=t
  local ret=DXL.read_position2({command_positions})

  --------------------------------------------------
  local d_pos = target_position[2]-ret[2]
  local v_pos = (ret[2]-old_ret[2])/dt
  old_ret = ret

  local ff_current = 100

  local p_gain = 10
  local v_gain = -0.2
  local p_current = util.procFunc(d_pos/DEG_TO_RAD*p_gain, 0, 100)
  local d_current = util.procFunc(v_pos/DEG_TO_RAD*v_gain, 0, 400)


  print(string.format("Current P:%d d:%d ff:%d",p_current, d_current, ff_current))
  DXL.set_goal_currents({11},{p_current+d_current+ff_current})
  ---------------------------------------------------

  for i=1,#position_servo_ids do
    local d_movement = target_position[i]-command_position[i]
    command_position[i]= command_position[i]+util.procFunc(d_movement, 0, dt * max_servo_velocity)
  end
  DXL.set_goal_positions2(position_servo_ids,command_position)

  count=count+1
  if count%100==0 then
    local t1=unix.time()
    print(unpack(vector.new(ret)/DEG_TO_RAD))
    print("FPS:",count/(t1-t0))
    t0,count=t1,0
  end
end
--
-- DXL.closeport()
