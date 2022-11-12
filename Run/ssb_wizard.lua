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
local is_mapping=false
local t_start=unix.time()

local function save_markers()
	local fp = io.open("/home/sj/markers.txt","w")
	print("===MARKER LIST===")
	for i=1,#marker_pose_list do
		local pose=marker_pose_list[i]
		fp:write(string.format("%.2f %.2f %.2f\n",pose[1],pose[2],pose[3]))
		print(string.format("%d: %.2f %.2f (%.1f)\n",i,pose[1],pose[2],pose[3]/DEG_TO_RAD))
	end
	fp:close()
end


local function check_map_command()
	local mapcmd=rossub.checkInt32(sub_idx_mapcmd)
	if mapcmd and unix.time()>t_entry+1.0 then
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




local getch = require'getch'
local running = true
local key_code
while running do
  -- key_code = getch.nonblock()
  -- update(key_code)
  check_map_command()
  unix.usleep(1E6*0.01)
end

-- Body.set_arm_torque_enable({0,0,0,0, 0,0,0,0})
-- dcm.set_actuator_torque_enable_changed(1)
