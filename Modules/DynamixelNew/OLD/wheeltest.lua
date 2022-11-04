#!/usr/bin/env luajit
local pwd = os.getenv'PWD'
if string.find(pwd,"Run") then	dofile('../include.lua')
else	dofile('../../include.lua') end
local util = require'util'
local unix=require'unix'
local vector = require'vector'
local DXL = require 'DynamixelNew'

local xbox360 = require 'xbox360'
local product_id = 0x0719
xbox360.open(product_id)

local dev_name, baudrate="/dev/usb2dxl",1000000
-- local all_servo_ids={1,2}
-- local velocity_servo_ids={1,2}

local all_servo_ids={1}
local velocity_servo_ids={1}

--control mode
--0 for current, 1 for velocity, 3 for pos, 4 for multi-turn pos, 5 for curret-based pos

DXL.openport(dev_name, baudrate)
DXL.ping_probe()
DXL.torque_enable(all_servo_ids,vector.zeros(#all_servo_ids))
DXL.control_mode(all_servo_ids,vector.ones(#all_servo_ids))
DXL.torque_enable(all_servo_ids,vector.ones(#all_servo_ids))


local function pad_update(ret)
  local x_db,x_max = 10,255
  local y_db,y_max = 8000,32768
  local lt =( util.procFunc(ret.trigger[1],x_db,x_max )/ (x_max-x_db)) --left trigger
  local rt =( util.procFunc(ret.trigger[2],x_db,x_max )/ (x_max-x_db)) --right trigger
  local lax =util.procFunc(ret.lstick[1],y_db,y_max )/(y_max-y_db) --left analog x
  local lay =util.procFunc(ret.lstick[2],y_db,y_max )/(y_max-y_db) --left analog y
  local rax =util.procFunc(ret.rstick[1],y_db,y_max )/(y_max-y_db) --right analog x
  local ray =util.procFunc(ret.rstick[2],y_db,y_max )/(y_max-y_db) --right analog y

print(lt,rt)


  local targetvel={(rt-lt) ,ray  ,lay } --X, Y, A speed

  local dxl_velocity={
    targetvel[1]*100,
    targetvel[1]*100,
    targetvel[1]*100,
    targetvel[1]*100
  }
  DXL.set_goal_velocities(velocity_servo_ids,dxl_velocity)
end

-- --
local count=0
t0 = unix.time()
t_last=unix.time()
while 1 do
  local t = unix.time()
  local dt=t-t_last
  t_last=t
  local padinput = xbox360.read()
  pad_update(padinput)
  local ret=DXL.read_position2(all_servo_ids)

  count=count+1
  if count%100==0 then
    local t1=unix.time()
    local t_elapsed=t1-t0
    -- print(unpack(ret))
    print("FPS:",count/t_elapsed)
    t0=t1
    count=0
  end
end

DXL.closeport()
