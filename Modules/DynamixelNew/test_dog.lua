#!/usr/bin/env luajit
local pwd = os.getenv'PWD'
if string.find(pwd,"Run") then	dofile('../include.lua')
else	dofile('../../include.lua') end
local util = require'util'
local unix=require'unix'
local vector = require'vector'

-- local xbox360 = require 'xbox360'
-- local product_id = 0x0719
-- xbox360.open(product_id)

require 'dcm'
require'mcm'
local Body=require 'Body'



Body.set_leg_torque_enable(0)
dcm.set_actuator_torque_enable_changed(1)
local cur_servo_pos=Body.get_leg_position()
print(unpack(cur_servo_pos))
Body.set_leg_command_position(cur_servo_pos)
unix.usleep(1E6*0.1)
-- Body.set_leg_torque_enable({1,0,0, 0,0,0, 0,0,0, 0,0,0})
Body.set_leg_torque_enable(1)
dcm.set_actuator_torque_enable_changed(1)


t0 = unix.time()
local dur=1.0

while 1 do
  local t = unix.time()
  local dt=t-t0
  local ph=dt%dur

  local k =math.sin(ph*math.pi)


  sr= 30*DEG_TO_RAD*k;
  sp=30*DEG_TO_RAD*k;
  kp=45*DEG_TO_RAD*k;


  Body.set_leg_command_position({sr,sp,kp,    -sr,sp,kp,   sr,sp,kp,  -sr,sp,kp})
  unix.usleep(1E6*0.01)
end


--
--
-- local function pad_update(ret)
--   local x_db,x_max = 10,255
--   local y_db,y_max = 8000,32768
--   local lt =( util.procFunc(ret.trigger[1],x_db,x_max )/ (x_max-x_db)) --left trigger
--   local rt =( util.procFunc(ret.trigger[2],x_db,x_max )/ (x_max-x_db)) --right trigger
--   local lax =util.procFunc(ret.lstick[1],y_db,y_max )/(y_max-y_db) --left analog x
--   local lay =util.procFunc(ret.lstick[2],y_db,y_max )/(y_max-y_db) --left analog y
--   local rax =util.procFunc(ret.rstick[1],y_db,y_max )/(y_max-y_db) --right analog x
--   local ray =util.procFunc(ret.rstick[2],y_db,y_max )/(y_max-y_db) --right analog y
--   local targetvel={
--     (rt-lt)*0.1,
--     ray*0.1,
--     lay*0.4
--    } --X, Y, A speed
--
--   local wheel_r = Config.wheels.r_fw or 0.075
--   local rot_factor =Config.wheels.rot_fw or 0.4
--
--   local vel_forward=vector.new({1,1,1,1})*targetvel[1]   --FL FR RL RR
--   local vel_sideways=vector.new({1,-1,-1,1})*targetvel[2] * math.sqrt(2)
--   local vel_rot = vector.new({-1,1,-1,1})*targetvel[3]*rot_factor
--   local target_vel=(vel_forward + vel_sideways + vel_rot)/wheel_r
--
--   local dxl_velocity={
--     target_vel[1]*0.1,
--     target_vel[2]*0.1,
--     target_vel[3]*0.1,
--     target_vel[4]*0.1
--   }
--   Body.set_wheel_command_velocity(dxl_velocity)
--   unix.usleep(1E6*0.01)
--
--   --BACK START LB RB XBOX X Y A B
--   if ret.buttons[6]>0 then --x button
--     Body.set_arm_command_position({0,-45*DEG_TO_RAD,0*DEG_TO_RAD,0,45*DEG_TO_RAD,-54*DEG_TO_RAD})
--
--   elseif ret.buttons[7]>0 then
--     Body.set_arm_command_position({0,-88*DEG_TO_RAD,88*DEG_TO_RAD,0,0,-54*DEG_TO_RAD})
--
--   end
--
--
--
--   unix.usleep(1E6*0.01)
-- end
--
-- -- --
-- local count=0
-- t0 = unix.time()
-- t_last=unix.time()
-- while 1 do
--   local t = unix.time()
--   local dt=t-t_last
--   t_last=t
--   local padinput = xbox360.read()
--   pad_update(padinput)
--   --
--   -- count=count+1
--   -- if count%100==0 then
--   --   local t1=unix.time()
--   --   local t_elapsed=t1-t0
--   --   print("FPS:",count/t_elapsed)
--   --   t0=t1
--   --   count=0
--   -- end
-- end

--
-- Body.set_wheel_torque_enable(0)
-- dcm.set_actuator_torque_enable_changed(1)
