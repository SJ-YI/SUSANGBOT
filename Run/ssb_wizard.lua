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

Body.set_torque_enable(Config.servo.torque_enable)
Body.set_arm_torque_enable({1,1,1,1, 1,1,1,1})
Body.set_wheel_torque_enable({1,1,1})
dcm.set_actuator_torque_enable_changed(1)


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


local last_wheel_pos=nil
--
-- local function update_wheel()
--   local wheel_r, body_r=0.05, 0.14
--  --v1= -0.86602 vx - 0.5 vy - r*va
--  --v2=   vy                 - r*va
--  --v3=  0.86602 vx - 0.5 vy - r*va
--   local xcomp=vector.new( {-0.86602,0, 0.86602} )/wheel_r
--   local ycomp=vector.new( {-0.5, 1, -0.5 } )/wheel_r
--   local acomp=vector.new( {-body_r, -body_r, -body_r} )/wheel_r
--
--   local t= unix.time()
--   local t_cmd=hcm.get_base_teleop_t()
--   if t-t_cmd<0.5 then
--     local teleop_vel=hcm.get_base_teleop_velocity()
--     local v1,v2,v3=0,0,0 --right back left
--     local wheel_vel = xcomp*teleop_vel[1] + ycomp*teleop_vel[2] + acomp*teleop_vel[3] --rad per sec
--     local v1mag,v2mag,v3mag=math.abs(wheel_vel[1]),math.abs(wheel_vel[2]),math.abs(wheel_vel[3])
--     local vmagmax=math.max(math.max(v1mag,v2mag),v3mag)
--     if vmagmax>2*PI then
--       local adj_factor = (vmagmax/(2*PI))
--       wheel_vel[1],wheel_vel[2],wheel_vel[3]=	wheel_vel[1]/adj_factor,wheel_vel[2]/adj_factor,wheel_vel[3]/adj_factor
--     end
--     Body.set_wheel_command_velocity(wheel_vel)
--   else
--     hcm.set_base_teleop_velocity({0,0,0})
--     Body.set_wheel_command_velocity({0,0,0})
--   end
--
--   local wheel_pos=Body.get_wheel_position()
--   if not last_wheel_pos then last_wheel_pos=wheel_pos end
--   local d1,d2,d3=
--     util.mod_angle(wheel_pos[1]-last_wheel_pos[1])*wheel_r,
--     util.mod_angle(wheel_pos[2]-last_wheel_pos[2])*wheel_r,
--     util.mod_angle(wheel_pos[3]-last_wheel_pos[3])*wheel_r
--   last_wheel_pos=wheel_pos
--   --vx = [-0.57735 0 0.57735] [v1 v2 v3]'
--   --vy = [-0.333333 0.666666 -0.33333333] [v1 v2 v3]'
--   --va = [2.38095 2.38095 2.38095] [v1 v2 v3]'
--   local dx=-0.57735*d1 + 0.57735*d3
--   local dy=(-d1+2*d2-d3)/3
--   local da=-(d1+d2+d3)/3/body_r
--   local curpos=wcm.get_robot_pose_odom()
--   local newpos=util.pose_global({dx,dy,da},curpos)
--   wcm.set_robot_pose_odom(newpos)
--   rospub.tf({newpos[1], newpos[2],0},{0,0,newpos[3]}, "odom","base_footprint")
-- --  print(string.format("Pose: %.2f %.2f %.1f",newpos[1],newpos[2],newpos[3]/DEG_TO_RAD ))
-- end

local function update_wheel()
  local wheel_r, body_r=Config.wheels.wheel_r, Config.wheels.body_r
  local xcomp,ycomp,acomp=Config.wheels.xcomp,Config.wheels.ycomp,Config.wheels.acomp
  local velxcomp,velycomp,velacomp=Config.wheels.velxcomp,Config.wheels.velycomp,Config.wheels.velacomp

  local t= unix.time()
  local t_cmd=hcm.get_base_teleop_t()
  if t-t_cmd<0.5 then
    local teleop_vel=hcm.get_base_teleop_velocity()
    local v1,v2,v3=0,0,0 --right back left
    local wheel_vel = xcomp*teleop_vel[1] + ycomp*teleop_vel[2] + acomp*teleop_vel[3] --rad per sec
    local v1mag,v2mag,v3mag=math.abs(wheel_vel[1]),math.abs(wheel_vel[2]),math.abs(wheel_vel[3])
    local vmagmax=math.max(math.max(v1mag,v2mag),v3mag)
    if vmagmax>2*PI then
      local adj_factor = (vmagmax/(2*PI))
      wheel_vel[1],wheel_vel[2],wheel_vel[3]=	wheel_vel[1]/adj_factor,wheel_vel[2]/adj_factor,wheel_vel[3]/adj_factor
    end
    Body.set_wheel_command_velocity(wheel_vel)
  else
    hcm.set_base_teleop_velocity({0,0,0})
    Body.set_wheel_command_velocity({0,0,0})
  end

  local wheel_pos=Body.get_wheel_position()
  if not last_wheel_pos then last_wheel_pos=wheel_pos end
  local d1,d2,d3=
    util.mod_angle(wheel_pos[1]-last_wheel_pos[1])*wheel_r,
    util.mod_angle(wheel_pos[2]-last_wheel_pos[2])*wheel_r,
    util.mod_angle(wheel_pos[3]-last_wheel_pos[3])*wheel_r
  last_wheel_pos=wheel_pos

  local dx=velxcomp[1]*d1 + velxcomp[3]*d3
  local dy=velycomp[1]*d1+velycomp[2]*d2+velycomp[3]*d3
  local da=velacomp[1]*d1+velacomp[2]*d2+velacomp[3]*d3

  local curpos=wcm.get_robot_pose_odom()
  local newpos=util.pose_global({dx,dy,da},curpos)
  wcm.set_robot_pose_odom(newpos)
  rospub.tf({newpos[1], newpos[2],0},{0,0,newpos[3]}, "odom","base_footprint")
--  print(string.format("Pose: %.2f %.2f %.1f",newpos[1],newpos[2],newpos[3]/DEG_TO_RAD ))
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
				if not robot_pose then
                                  print("No tf, returning")
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














local function save_map()
  os.execute('rosrun map_server map_saver -f ~/Desktop/SUSANGBOT/Data/map')
  unix.usleep(1E6)
  os.execute('rosnode kill /hector_mapping')
  os.execute('roslaunch pnu_tb3_launch ssb_amcl.launch &')
  unix.usleep(1E6*3)
end

Body.set_arm_command_position(armangle0)
wcm.set_robot_pose_odom({0,0,0})

local getch = require'getch'
local running = true
local key_code
while running do
  key_code = getch.nonblock()
  update(key_code)
  update_wheel()
  check_map_command()
  unix.usleep(1E6*0.01)
end

Body.set_arm_torque_enable({0,0,0,0, 0,0,0,0})
dcm.set_actuator_torque_enable_changed(1)
