#!/usr/bin/env luajit
-------------------------------------------
--This codes receives zmq messages from webots and publish them as ros topics
--This code receives ros topics and control webots model based on them

local pwd = os.getenv'PWD'
local ok = pcall(dofile,'../fiddle.lua')
if not ok then ok = pcall(dofile,'/fiddle.lua') end

local si = require'simple_ipc'
local ffi=require'ffi'
local unix=require'unix'
local sr = require'serialization'
local rossub=require'rossub'

rossub.init('webots_rossub')
local rospub = require 'rospub2'
--nodename image_topics lidar_topics path_topics occgrid_topics
rospub.init_custom(
	'webots_rospub',{"/image_raw","/depth_image_raw"},{"/scan"},
	{},--path names
	{}--map names
)


local t_entry=unix.time()
local t_last_debug,t_next_debug=t_entry, t_entry+0.1

--Incoming ros topics
local sub_idx_cmdvel=rossub.subscribeTwist('/cmd_vel')
local sub_idx_joint=rossub.subscribeJointTrajectory('/arm_pos')

--Incoming ZMQ messages
local rgb_ch = si.new_subscriber("rgb")
local depth_ch = si.new_subscriber("depth")
local lidar_ch = si.new_subscriber("lidar")
local pose_ch = si.new_subscriber("pose")
local jointstate_ch = si.new_subscriber("jointstate")
local poller
local lidar_count,rgb_count,pose_count,depth_count,jointstate_count,seq=0,0,0,0,0,0

local function cb_lidar(skt)
	local datastr = unpack(skt:recv_all())
	local n=#datastr/ffi.sizeof("float")
	-- seq angle_min angle_max n data_str linkname channel is_webots
	rospub.laserscan(seq,-math.pi, math.pi, n,datastr,"base_scan",0,1)
	lidar_count,seq=lidar_count+1,seq+1
end

local function cb_rgb(skt)
	local datastr = unpack(skt:recv_all())
	local buf_ffi= ffi.new("uint8_t[?]",370000);  ffi.copy(buf_ffi,datastr,#datastr)
  local whfov_ffi=ffi.new("float[3]");   ffi.copy(whfov_ffi,datastr,3*ffi.sizeof("float"))
  local w,h,fov = whfov_ffi[0],whfov_ffi[1],whfov_ffi[2]
	rospub.image(seq,w,h,fov, ffi.string(buf_ffi+3*ffi.sizeof("float"),w*h*3),  0,0) --encoding 0 (rgb8), 0 ch
	rgb_count,seq=rgb_count+1,seq+1
end

local function cb_depth(skt)
	local datastr = unpack(skt:recv_all())
	local buf_ffi= ffi.new("uint8_t[?]",640000);   ffi.copy(buf_ffi,datastr,#datastr)
  local whfov_ffi=ffi.new("float[3]");   ffi.copy(whfov_ffi,datastr,3*ffi.sizeof("float"))
  local w,h,fov = whfov_ffi[0],whfov_ffi[1],whfov_ffi[2]
	local fl_sz=ffi.sizeof("float")
	rospub.image(seq,w,h,fov, ffi.string(buf_ffi+3*ffi.sizeof("float"),w*h*fl_sz),  2,1) --encoding 2 (webots depth), 1ch
	depth_count,seq=depth_count+1,seq+1
end

local function cb_pose(skt)
	local datastr = unpack(skt:recv_all())
  local pose_ffi=ffi.new("float[3]")
	ffi.copy(pose_ffi,datastr,ffi.sizeof("float")*3)
	rospub.tf({pose_ffi[0], pose_ffi[1],0},{0,0,pose_ffi[2]}, "odom","base_footprint")
	rospub.tf({pose_ffi[0], pose_ffi[1],0},{0,0,pose_ffi[2]}, "map","base_footprint_ground_truth")
  rospub.odom({pose_ffi[0],pose_ffi[1],pose_ffi[2]})
	pose_count,seq=pose_count+1,seq+1
end

local function cb_jointstate(skt)
	local datastr = unpack(skt:recv_all())
	local data_ffi=ffi.new("char[?]",#datastr)
	ffi.copy(data_ffi,datastr,#datastr)
	local datasizes_ffi=ffi.new("int[2]")
	ffi.copy(datasizes_ffi,data_ffi,2*ffi.sizeof("int"))
	local jointnames_size, jointangles_size=datasizes_ffi[0],datasizes_ffi[1]
	jointnames=sr.deserialize(ffi.string(data_ffi+2*ffi.sizeof("int"),jointnames_size))
	jointangles=sr.deserialize(ffi.string(data_ffi+2*ffi.sizeof("int")+jointnames_size,jointangles_size))
  rospub.jointstate(seq,jointnames, jointangles)
	jointstate_count,seq=jointstate_count+1,seq+1
end

local function check_arm_command(t)
	local a1,a2,a3,a4, grip=rossub.checkJointTrajectory(sub_idx_joint)
	if a1 and t-t_entry>1 then
		-- print("Arm Command!")
		hcm.set_base_armtarget({a1,a2,a3,a4})
		hcm.set_base_grippertarget(grip)
	end
end



rgb_ch.callback,lidar_ch.callback,pose_ch.callback,jointstate_ch.callback,depth_ch.callback  =
	cb_rgb,cb_lidar,cb_pose,cb_jointstate,cb_depth
poller = si.wait_on_channels({rgb_ch,lidar_ch,pose_ch,jointstate_ch,depth_ch})


local debug_interval=5.0
running=true
local TIMEOUT = 1e3 / 500 -- Timeout in milliseconds (100 Hz)

while running do
	local t=unix.time()
	npoll = poller:poll(TIMEOUT)
	check_arm_command(t)

	t=unix.time()
	if t>t_next_debug then
		local t_elapsed=t-t_last_debug
		print(string.format("Rospub| RGB %.1f hz Depth: %.1f hz Lidar %.1f hz Pose %.1f hz Joint %.1f hz",
			rgb_count/t_elapsed,depth_count/t_elapsed,
			lidar_count/t_elapsed,pose_count/t_elapsed,jointstate_count/t_elapsed
		))
		depth_count,lidar_count,rgb_count,pose_count,jointstate_count=0,0,0,0,0
		t_next_debug=t+debug_interval
		t_last_debug=t
	end
	unix.usleep(1E6*0.01)
end
