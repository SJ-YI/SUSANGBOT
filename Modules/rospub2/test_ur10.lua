#!/usr/bin/env luajit
-- (c) 2014 Team THORwIn
local ok = pcall(dofile,'../fiddle.lua')
if not ok then ok=pcall(dofile,'./fiddle.lua') end
if not ok then ok=dofile'../../fiddle.lua' end

local util = require'util'
local signal = require'signal'.signal
local gettime = require'unix'.time
local rospub = require 'rospub2'


local tDelay = 0.005*1E6
local running = true
-- Cleanly exit on Ctrl-C
local function shutdown()
	io.write('\n Soft shutdown!\n')
	running=false
end
signal("SIGINT", shutdown)
signal("SIGTERM", shutdown)
rospub.init('test_pub')


local jointnames={
	"shoulder_pan_joint",
	"shoulder_lift_joint",
	"elbow_joint",
	"wrist_1_joint",
	"wrist_2_joint",
	"wrist_3_joint"
}

-- local jangle0=vector.new({0.18, -2.18, 1.57, -0.35, 0.18, 0.0})
-- local jangle1=vector.new({0.18, -2.18, 0, -0.35, 0.18, 0.0})


local jangle0=vector.new({0, -45, 60, -15+90,90,0})*DEG_TO_RAD
local jangle1=vector.new({0, -120, -135, 0,90,0})*DEG_TO_RAD



local function jangle_interpolate(j0, j1, totaltime, div)
	local jangle2,dur2={},{}
	local count=0
	for i=1,div do
		local ph=i/div
		local jangle_intp=ph*j1 + (1-ph)*j0
		-- print(unpack(jangle_intp))
		jangle2[count*6+1],jangle2[count*6+2],jangle2[count*6+3],jangle2[count*6+4],jangle2[count*6+5],jangle2[count*6+6]=
			jangle_intp[1],jangle_intp[2],jangle_intp[3],jangle_intp[4],jangle_intp[5],jangle_intp[6]
		dur2[count+1]=totaltime/div
		count=count+1
	end
	-- print(#jangle2, #dur2)
	return jangle2,dur2
end


-- local jtraj1, dur1=jangle_interpolate(jangle0, jangle1, 2, 10)
-- local jtraj2, dur2=jangle_interpolate(jangle1, jangle0, 1, 10)


local jtraj1, dur1=jangle_interpolate(jangle0, jangle1, 0.5, 4)
local jtraj2, dur2=jangle_interpolate(jangle1, jangle0, 0.5, 4)





local gripperjointnames={
	"finger_1_joint_1",
	"finger_2_joint_1",
	"finger_middle_joint_1"
}

--
while running do
  print("Sending...")
	-- rospub.jointactiongoal(
	-- 	jointnames,
	-- 	"test1",
	-- 	1,
	-- 	{0,-90*DEG_TO_RAD,30*DEG_TO_RAD,90*DEG_TO_RAD,0,0},
	-- 	0
	-- )

	-- rospub.jointactiongoal(gripperjointnames,"test1",1,{0.30,0.30,0.0},1)



	-- rospub.jointactiongoal(gripperjointnames,"test1",1,{0.80,0.80,0.0},1)
	rospub.jointactiongoal(jointnames, "test1",jtraj1, dur1,0)
	unix.usleep(1e6*2)

	rospub.jointactiongoal(jointnames, "test1",jtraj2, dur2,0)
	unix.usleep(1e6*2)
end
