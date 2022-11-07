#!/usr/bin/env luajit
-- (c) 2014 Team THORwIn
local ok = pcall(dofile,'../fiddle.lua')
if not ok then dofile'fiddle.lua' end
local PI=3.14159265

unix.usleep(1E6*1.5)

local T=require'Transform'
local Body=require 'Body'
require'hcm'

local rossub=require'rossub'
local rospub = require 'tb3_rospub'
rospub.init('ssb_rospub')
rossub.init('ssb_rossub')
local sub_idx_mapcmd=rossub.subscribeInt32('/mapcmd')
--
-- Body.set_torque_enable(Config.servo.torque_enable)
-- Body.set_arm_torque_enable({1,1,1,1, 1,1,1,1})
-- Body.set_wheel_torque_enable({1,1,1})
-- dcm.set_actuator_torque_enable_changed(1)


local armangle0=vector.new({5,5,0,-30,   5,-5,0,-30,})*DEG_TO_RAD --POSE 0
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


local t_start=unix.time()

local function check_map_command()
	local mapcmd=rossub.checkInt32(sub_idx_mapcmd)
	if mapcmd and unix.time()>t_start+1.0 then
		print("MAP COMMAND:",mapcmd)

		if mapcmd==1 then
			if not is_mapping then
				print"STARTING SLAM!!!!!"
				os.execute('rosnode kill /amcl')
				os.execute('rosnode kill /map_server')
				unix.usleep(1E6*1)
				os.execute('roslaunch pnu_tb3_launch ssb_slam.launch &')
				is_mapping=true
			else
				local robot_pose=rossub.checkTF("map","base_footprint")
				if not robot_pose then print("No tf, returning")
        else
  				local pose={robot_pose[1],robot_pose[2],robot_pose[6]}
  				print"ENDING SLAM!!!"
  				os.execute('rosrun map_server map_saver -f ~/Desktop/SUSANGBOT/Data/map')
  				unix.usleep(1E6*1)
  				print"RESTARTING AMCL!!!"
  				os.execute('rosnode kill /hector_mapping')
  				os.execute('roslaunch pnu_tb3_launch ssb_amcl.launch &')
  				unix.usleep(1E6*3)
  				rospub.posereset(pose)
  				print("POSE RESETTED TO",unpack(pose))
  				is_mapping=false
        end
			end
    elseif mapcmd==99 then
      print("POSE RESET!!!")
      rospub.posereset({0,0,0})
		elseif mapcmd==3 then
      print("MARKER ADDED")
			local robot_pose=rossub.checkTF("map","base_footprint")
			local pose={robot_pose[1],robot_pose[2],robot_pose[6]}
--			marker_pose_list[#marker_pose_list+1]=pose
--			save_markers()
		elseif mapcmd==4 then
      print("MARKER REMOVE")
--			if #marker_pose_list>1 then
  --		  marker_pose_list[#marker_pose_list]=nil
--			  save_markers()
		--	end
		elseif mapcmd==9 then
      print("MOVE START")
		--	rospub.posereset({0,0,0})
		--	unix.usleep(1E6*1)
		--	body_ch:send'start'
		end
	end
end

Body.set_arm_command_position(armangle0)
local getch = require'getch'
local running = true
local key_code
while running do
  key_code = getch.nonblock()
  update(key_code)
  check_map_command()
  unix.usleep(1E6*0.01)
end

-- Body.set_arm_torque_enable({0,0,0,0, 0,0,0,0})
-- dcm.set_actuator_torque_enable_changed(1)
