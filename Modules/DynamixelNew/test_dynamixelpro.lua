#!/usr/bin/env luajit
local pwd = os.getenv'PWD'
if string.find(pwd,"Run") then	dofile('../include.lua')
else	dofile('../../include.lua') end
local si = require'simple_ipc'
local ffi=require'ffi'
local unix=require'unix'
local vector = require'vector'
local DXL = require 'DynamixelNew'

local dev_name, baudrate="/dev/usb2dxl",1000000
local all_servo_ids={11,12,13,14}
local position_servo_ids={}
local velocity_servo_ids={11,12,13,14}
local current_servo_ids={}


local all_servo_ids={11,12,13,14}

--control mode
--0 for current
--1 for velocity
--3 for pos
--4 for multi-turn pos
--5 for curret-based pos

DXL.openport(dev_name, baudrate)
DXL.ping_probe()


DXL.pro_torque_enable(all_servo_ids,{0,0,0,0})
  unix.usleep(1E6*0.02)

DXL.control_mode(all_servo_ids,{1,1,1,1})
  unix.usleep(1E6*0.02)

-- DXL.control_mode(all_servo_ids,{3})
DXL.pro_torque_enable(all_servo_ids,{1,1,1,1})
  unix.usleep(1E6*0.02)

ret=DXL.pro_read_position(all_servo_ids)
if ret then print(unpack(ret)) end

-- ret=DXL.read_position(all_servo_ids)
-- print(unpack(ret))
--
-- ret=DXL.read_position2(all_servo_ids)
-- print(unpack(ret))

print("MOVE!")
DXL.pro_set_goal_velocities(all_servo_ids,{1000,1000,1000,1000})

print("MOVE!")
for i=1,40 do
  ret=DXL.pro_read_position(all_servo_ids)
if ret then   print(unpack(ret)) end
  unix.usleep(1E6*0.05)
end



DXL.pro_set_goal_velocities(all_servo_ids,{0})
unix.usleep(1E6*1)
DXL.pro_torque_enable(all_servo_ids,{0})
DXL.closeport()


-- DXL.set_goal_velocities(velocity_servo_ids,{10000})


-- DXL.torque_enable(all_servo_ids,{0,0,0,0,0})
-- DXL.control_mode(all_servo_ids,{3,3,3,3,1})
-- DXL.torque_enable(all_servo_ids,{1,1,1,1,1})

--[[
DXL.torque_enable(all_servo_ids,{0,0,0})
DXL.control_mode(all_servo_ids,{0,3,1})
DXL.torque_enable(all_servo_ids,{1,1,1})


--goal pos:0 to 4096
goal_pos=vector.new({2048,2048,2048})
goal_vel={100}
move_dir=1


DXL.set_goal_velocities(velocity_servo_ids,{100})
DXL.set_goal_currents(current_servo_ids,{5})

-- --
local count=0
t0 = unix.time()
t_last=unix.time()
while 1 do
  local t = unix.time()
  local dt=t-t_last
  t_last=t

  DXL.set_goal_positions(position_servo_ids,goal_pos)
  -- unix.usleep(1E6*0.01)
  goal_pos=goal_pos+vector.new({1000,1000,1000})*move_dir*dt
  local ret=DXL.read_position2({1,3,4})
  count=count+1
  if count%100==0 then
    move_dir=-move_dir
    local t1=unix.time()
    local t_elapsed=t1-t0
    -- print(unpack(ret))
    print("FPS:",count/t_elapsed)
    t0=t1
    count=0
  end
end

DXL.closeport()
--]]
