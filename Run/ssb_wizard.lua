#!/usr/bin/env luajit
-- (c) 2014 Team THORwIn
local ok = pcall(dofile,'../fiddle.lua')
if not ok then dofile'fiddle.lua' end
local PI=3.14159265

unix.usleep(1E6*1.5)

local T=require'Transform'
local Body=require 'Body'
require'hcm'
require'mcm'

local rossub=require'rossub'
local rospub = require 'tb3_rospub'
rospub.init('ssb_rospub')
rossub.init('ssb_rossub')
local sub_idx_mapcmd=rossub.subscribeInt32('/mapcmd')
local sub_idx_bluetoothmsg=rossub.subscribeString('/bluetoothmsg')






local is_mapping=false
local t_start=unix.time()
local t_entry=unix.time()
local marker_pose_list={{0,0,0}}

local function load_markers()
	marker_pose_list={}
	local fp = io.open("/home/sj/markers.txt","r")
	local lineno=1
	if not fp then return end
	while true do
  	local line=fp:read()
		if line==nil then break end
		local t = {}
		for n in line:gmatch("%S+") do   	table.insert(t, tonumber(n))         end
		local marker_index=#marker_pose_list+1
		marker_pose_list[marker_index]=t
		lineno=lineno+1
	end
	fp:close()
	for i=1,#marker_pose_list do
		local pose=marker_pose_list[i]
		print(string.format("%d: %.2f %.2f (%.1f)\n",i,pose[1],pose[2],pose[3]/DEG_TO_RAD))
	end
end

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

local function update_markers()
	local types,posx,posy,posz,yaw,names,scales,colors={},{},{},{},{},{},{},{}
	local scale, color1, color2, arrowlen = 1,2,4,0.20
	local mc=0

--	for i=1,#Config.pose_targets do
	for i=1,#marker_pose_list do
		local marker_name="origin"
		if i>1 then	marker_name=string.format("Pose%02d",i-1)		end
	  local x,y,a=
		marker_pose_list[i][1],
		marker_pose_list[i][2],
		marker_pose_list[i][3]
	  mc=mc+1        --LOCATION NAMES (string)
	  types[mc],posx[mc],posy[mc],posz[mc],yaw[mc],names[mc],scales[mc],colors[mc]=
	    2,     x-0.1,y,0,0,		marker_name,		1, 2
	  mc=mc+1  --arrow
	  types[mc],posx[mc],posy[mc],posz[mc],yaw[mc],names[mc],scales[mc],colors[mc]=
	    5, 	x,y,0.01,   a,"x", arrowlen, color1
	  mc=mc+1 --flat cylinder
	  types[mc],posx[mc],posy[mc],posz[mc],yaw[mc],names[mc],scales[mc],colors[mc]=
	    7,  x,y,0,	    a,"x", 0.5, color2
	end
  rospub.marker(mc,types,posx,posy,posz,yaw,names,scales,colors)
end

local function check_map_command()
	local mapcmd=rossub.checkInt32(sub_idx_mapcmd)

	local bluetoothcmd=rossub.checkString(sub_idx_bluetoothmsg)
	if bluetoothcmd and unix.time()>t_entry+1.0 then
		print("Bluetooth command:",bluetoothcmd)
		os.execute('espeak '..bluetoothcmd)

		if bluetoothcmd=="dance1" then
			body_ch:send'wait'
			arm_ch:send'dance1'
		elseif bluetoothcmd=="dance2" then
				body_ch:send'wait'
				arm_ch:send'dance2'
		elseif bluetoothcmd=="dance3" then
				body_ch:send'wait'
				arm_ch:send'dance3'
		elseif bluetoothcmd=="dance4" then
				body_ch:send'wait'
				arm_ch:send'dance4'
		else


		end


	end

	if mapcmd and unix.time()>t_entry+1.0 then
		print("MAP COMMAND:",mapcmd)
		if mapcmd==1 then
			if not is_mapping then
				print"STARTING SLAM!!!!!"
				os.execute("espeak 'slam'")
				os.execute('rosnode kill /amcl')
				os.execute('rosnode kill /map_server')
				unix.usleep(1E6*1)
				os.execute('roslaunch pnu_tb3_launch ssb_slam.launch &')
				os.execute("espeak 'started'")
				is_mapping=true
        marker_pose_list={{0,0,0}}
			else
				local robot_pose=rossub.checkTF("map","base_footprint")
				if not robot_pose then print("No tf, returning")
        else
  				local pose={robot_pose[1],robot_pose[2],robot_pose[6]}
					os.execute("espeak 'mapping complete'")
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
			marker_pose_list[#marker_pose_list+1]=pose
			save_markers()
		elseif mapcmd==4 then
      print("MARKER REMOVE")
			if #marker_pose_list>1 then
  		  marker_pose_list[#marker_pose_list]=nil
			  save_markers()
			end
		elseif mapcmd==9 then
      print("MOVE START")
      wcm.set_task_index(1) --go from origin to pose #1
			-- rospub.posereset({0,0,0})
		 	unix.usleep(1E6*1)
		  body_ch:send'start'
		end
	end
end


load_markers()
local running = true
rospub.posereset({0,0,0})

local t_next_marker=unix.time()
local t_next_debug=unix.time()
while running do
  t=unix.time()
  if t>t_next_marker then
    update_markers()
    t_next_marker=t+1
  end
  if t>t_next_debug then
    local battery_level=mcm.get_status_battery()
    print(string.format("ssb| %d sec Battery:%.1f",t-t_start, battery_level))
    t_next_debug=t+5
  end

  -- key_code = getch.nonblock()
  -- update(key_code)
  check_map_command()
  unix.usleep(1E6*0.01)
end

-- Body.set_arm_torque_enable({0,0,0,0, 0,0,0,0})
-- dcm.set_actuator_torque_enable_changed(1)
