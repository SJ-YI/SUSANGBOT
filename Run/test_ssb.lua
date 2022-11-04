#!/usr/bin/env luajit
-- (c) 2014 Team THORwIn
local ok = pcall(dofile,'../fiddle.lua')
if not ok then dofile'fiddle.lua' end
local PI=3.14159265

local T=require'Transform'
local Body=require 'Body'
require'hcm'


Body.set_torque_enable(Config.servo.torque_enable)
Body.set_arm_torque_enable({1,1,1,1, 1,1,1,1})
Body.set_wheel_torque_enable({1,1,1})
dcm.set_actuator_torque_enable_changed(1)


local armangle0=vector.new({5,5,0,-30,   5,-5,0,-30,})*DEG_TO_RAD --POSE 0
--FOR SEPARATE ARM 2
-- local armangles={
--   vector.new({5,95,0,0,   -45,-45,60,-90})*DEG_TO_RAD, --POSE 1, left arm extend
--   vector.new({5,95,0,0,   5,-95,0,0})*DEG_TO_RAD, --POSE 2, both arm spread
--   vector.new({-45,45,-60,-90,   5,-95,0,0})*DEG_TO_RAD, --POSE 3, right arm extend
--
--   vector.new({5,45,0,0,   -170,-45,0,-60})*DEG_TO_RAD, --POSE 4, left down / right up
--   vector.new({-90,95,0,-90,   -90,-95,0,-90})*DEG_TO_RAD, --POSE 5, both arm up
--   vector.new({-170,45,0,-60,   5,-45,0,0})*DEG_TO_RAD, --POSE 6, left up/right down
--
--   vector.new({-170,45,0,-60,   -170,0,60,-90})*DEG_TO_RAD, --POSE 7, left up/right head
--   vector.new({-180,0,-60,-90,   -180,0,60,-90})*DEG_TO_RAD, --POSE 8, left head/right head
--   vector.new({-170,0,-60,-90,   -170,-45,0,-60})*DEG_TO_RAD, --POSE 9, left head/right up
--
--   vector.new({5,45,0,0,   5,-45,0,0,})*DEG_TO_RAD, --POSE 10, both low spread
-- }

--FOR SSB
local armangles={
  vector.new({5,95,0,-30,   -90,-30,75,-90})*DEG_TO_RAD, --POSE 1, left arm extend
  vector.new({5,95,0,0,   5,-95,0,0})*DEG_TO_RAD, --POSE 2, both arm spread
  vector.new({-90,30,-75,-90,   5,-95,0,-30})*DEG_TO_RAD, --POSE 3, right arm extend

  vector.new({5,45,0,0,   -145,-45,0,-30})*DEG_TO_RAD, --POSE 4, left down / right up
  vector.new({-90,90,0,-90,   -90,-90,0,-90})*DEG_TO_RAD, --POSE 5, both arm up
  vector.new({-145,45,0,-30,   5,-45,0,0})*DEG_TO_RAD, --POSE 6, left up/right down

  vector.new({-145,45,0,-0,   -145,-30,75,-90})*DEG_TO_RAD, --POSE 7, left up/right head
  vector.new({-145,30,-60,-90,   -145,-30,60,-90})*DEG_TO_RAD, --POSE 8, left head/right head
  vector.new({-145,30,-60,-90,   -145,-45,0,0})*DEG_TO_RAD, --POSE 9, left head/right up

  vector.new({5,45,0,0,   5,-45,0,0,})*DEG_TO_RAD, --POSE 10, both low spread
}


local t_wait=2



local t_last_pressed=unix.time()
local function update(key_code)
  local t=unix.time()
	if type(key_code)~='number' or key_code==0 then return end

  if t-t_last_pressed<0.2 then return end
  t_last_pressed=t
	local key_char = string.char(key_code)
	local key_char_lower = string.lower(key_char)
  local targetvel_new=hcm.get_base_velocity()
  local targetvel=hcm.get_base_velocity()

  local cmd_vel=false
  local cmd_motor=false

	if key_char_lower==("1") then
    Body.set_arm_command_position(armangles[1])
    unix.usleep(1E6*t_wait)
    Body.set_arm_command_position(armangle0)
  elseif key_char_lower==("2") then
    Body.set_arm_command_position(armangles[2])
    unix.usleep(1E6*t_wait)
    Body.set_arm_command_position(armangle0)
  elseif key_char_lower==("3") then
    Body.set_arm_command_position(armangles[3])
    unix.usleep(1E6*t_wait)
    Body.set_arm_command_position(armangle0)
  elseif key_char_lower==("4") then
    Body.set_arm_command_position(armangles[4])
    unix.usleep(1E6*t_wait)
    Body.set_arm_command_position(armangle0)
  elseif key_char_lower==("5") then
    Body.set_arm_command_position(armangles[5])
    unix.usleep(1E6*t_wait)
    Body.set_arm_command_position(armangle0)
  elseif key_char_lower==("6") then
    Body.set_arm_command_position(armangles[6])
    unix.usleep(1E6*t_wait)
    Body.set_arm_command_position(armangle0)
  elseif key_char_lower==("7") then
    Body.set_arm_command_position(armangles[7])
    unix.usleep(1E6*t_wait)
    Body.set_arm_command_position(armangle0)
  elseif key_char_lower==("8") then
    Body.set_arm_command_position(armangles[8])
    unix.usleep(1E6*t_wait)
    Body.set_arm_command_position(armangle0)
  elseif key_char_lower==("9") then
    Body.set_arm_command_position(armangles[9])
    unix.usleep(1E6*t_wait)
    Body.set_arm_command_position(armangle0)
  elseif key_char_lower==("0") then
    Body.set_arm_command_position(armangles[10])
    unix.usleep(1E6*t_wait)
    Body.set_arm_command_position(armangle0)
  end
end

local function update_wheel()
  local wheel_r, body_r=0.05, 0.213
  local xcomp=vector.new( {-2,0, 2} )/wheel_r
  local ycomp=vector.new( {-0.5,1,-0.5} )/wheel_r

  local acomp=vector.new( {-body_r, -body_r, -body_r} )/wheel_r
  local t= unix.time()
  local t_cmd=hcm.get_base_teleop_t()
  if t-t_cmd<0.5 then
    local teleop_vel=hcm.get_base_teleop_velocity()
    local v1,v2,v3=0,0,0 --right back left
    -- local wheel_vel = xcomp*teleop_vel[1] + ycomp*teleop_vel[2] + acomp*teleop_vel[3] --rad per sec


    local wheel_vel = xcomp*teleop_vel[1]*0.3  + ycomp*teleop_vel[2]*0.3 + acomp*teleop_vel[3]*0.3 --rad per sec


    Body.set_wheel_command_velocity(wheel_vel)

    -- print("vel:",unpack(teleop_vel))
  else
    hcm.set_base_teleop_velocity({0,0,0})
    Body.set_wheel_command_velocity({0,0,0})
  end
end

Body.set_arm_command_position(armangle0)

local getch = require'getch'
local running = true
local key_code
while running do
  key_code = getch.nonblock()
  update(key_code)
  update_wheel()
  unix.usleep(1E6*0.02)
end

Body.set_arm_torque_enable({0,0,0,0, 0,0,0,0})
dcm.set_actuator_torque_enable_changed(1)
