#!/usr/bin/env luajit
local pwd = os.getenv'PWD'
local ok = pcall(dofile,'../fiddle.lua')
if not ok then ok = pcall(dofile,'/fiddle.lua') end

local si = require'simple_ipc'
local ffi=require'ffi'
local unix=require'unix'
local rossub=require'rossub'
local rospub=require 'rospub2'
rospub.init('rosio_granny')
rossub.init('rosio_granny')

local sub_idx_teleopvel=rossub.subscribeTwist('/teleop_vel') --teleop command from xb360 controller
local sub_idx_mapcmd=rossub.subscribeInt32('/mapcmd') --mapping command from xb360 controller
local sub_idx_batt=rossub.subscribeBattery('/battery_state')

local t_entry=unix.time()
local t_last_debug=t_entry
local t_next_debug=t_entry+0.1
local t_last_pub=t_entry
local pub_interval=1/30
local debug_interval=5.0

local function check_teleop_command(t) --receive teleop_vel topic and update hcm
	local ret = rossub.checkTwist(sub_idx_teleopvel)
	if ret and t-t_entry>1 then
		hcm.set_path_execute(0)	--stop pathplanning
		hcm.set_base_teleop_t(t)
		hcm.set_base_velocity({ret[1],ret[2],ret[6]})
	end
end

local function update_cmd_vel(t)
	if t>t_last_pub+pub_interval then
		if t-hcm.get_base_teleop_t()<0.5 then --teleop mode
			-- print("TELEOP MODE")
	--			print("TELEOP:",unpack(hcm.get_base_velocity() ))
			local vel=hcm.get_base_velocity()
			rospub.cmd_vel(vel[1],vel[2],vel[3])
		else
			local pathplan=hcm.get_path_execute() --pathplan mode
			rospub.navigation_status(pathplan)
			if pathplan>0 then
				-- print("PATHPLAN MODE")
				local walkvel=mcm.get_walk_vel()
				rospub.cmd_vel(walkvel[1],walkvel[2],walkvel[3])
			end
		end
		t_last_pub=t
	end
end

local function update_pose(t)
	if Config.use_fast_pose then return end
	local robot_pose=rossub.checkTF("map","base_footprint")
	if robot_pose then
		local pose={robot_pose[1],robot_pose[2],robot_pose[6]}
		wcm.set_robot_pose(pose)
	end
end

local function update_ar_markers(t)
	local arposex=wcm.get_landmark_posx()
	local arposey=wcm.get_landmark_posy()
	local arposea=wcm.get_landmark_posa()
	local arposet=wcm.get_landmark_t()
	for i=1,8 do
		local arpose=rossub.checkTF("base_link","/ar_marker_"..(i-1))
		if arpose then
			arposex[i],arposey[i],arposea[i],arposet[i]=arpose[1],arpose[2],arpose[6]+math.pi/2,t
		end
	end
	wcm.set_landmark_posx(arposex)
	wcm.set_landmark_posy(arposey)
	wcm.set_landmark_posa(arposea)
	wcm.set_landmark_t(arposet)
end

local function update_markers()
	if not Config.pose_targets then print("NO POSE"); return end
	local types,posx,posy,posz,yaw,names,scales,colors={},{},{},{},{},{},{},{}
	local scale, color1, color2, arrowlen = 1,2,4,0.20
	local mc=0

	for i=1,#Config.pose_targets do
	  mc=mc+1        --LOCATION NAMES (string)
	  types[mc],posx[mc],posy[mc],posz[mc],yaw[mc],names[mc],scales[mc],colors[mc]=
	    2,     Config.pose_targets[i][2][1]-0.1,Config.pose_targets[i][2][2],0,
			0, Config.pose_targets[i][1],1, 2

	  mc=mc+1  --arrow
	  types[mc],posx[mc],posy[mc],posz[mc],yaw[mc],names[mc],scales[mc],colors[mc]=
	    5, 	Config.pose_targets[i][2][1],Config.pose_targets[i][2][2],0.01,
	    Config.pose_targets[i][2][3],"x", arrowlen, color1

	  mc=mc+1 --flat cylinder
	  types[mc],posx[mc],posy[mc],posz[mc],yaw[mc],names[mc],scales[mc],colors[mc]=
	    7,   Config.pose_targets[i][2][1],Config.pose_targets[i][2][2],0,
	    Config.pose_targets[i][2][3],"x", 1, color2
	end
  rospub.marker(mc,types,posx,posy,posz,yaw,names,scales,colors)
end




running=true
while running do
	local t=unix.time()
	check_teleop_command(t)
	update_cmd_vel(t)
	update_pose(t)
	update_ar_markers()

	t=unix.time()
	if t>t_next_debug then
		update_markers()
		local t_elapsed=t-t_last_debug
		print(string.format("RosIO| %.1f sec",t-t_entry))
		t_next_debug=t+debug_interval
		t_last_debug=t
	end
	unix.usleep(1E6*0.01)
end
