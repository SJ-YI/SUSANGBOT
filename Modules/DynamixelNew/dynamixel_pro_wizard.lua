#!/usr/bin/env luajit
local pwd = os.getenv'PWD'
if string.find(pwd,"Run") then	dofile('../include.lua')
else	dofile('../../include.lua') end
local util = require'util'
local unix=require'unix'
local vector = require'vector'
local DXL = require 'DynamixelNew'
local Body = require'Body'
local T=require'Transform'
local signal = require'signal'.signal
require'dcm'
require'mcm'
require'hcm'
require'gcm'

--XM series
local MAX_STEPS = 4096
local POS_STEPS = 1/4096 --1/4096 rotation per one step
-- local VEL_STEPS = 0.229 ---0.229 rpm per one unit
local VEL_STEPS = 0.229*0.10472 --rad per sec

local CUR_STEPS = 2.69 ---2.69 mA per one unit

--Dynamixel pro 20W
--1/303750/2, 0.0329218, 16.11328
--PH54-200-S500-R, 501923*2 steps per rotation, 0.01 rpm per step, 1mA per unit, max 33RPM




local dev_name, baudrate=Config.servo.dynamixel_dev or "/dev/usb2dxl",1000000
local should_wait_dxl = true
local pro_servo_ids, pro_velocity_servo_ids, pro_current_servo_ids, pro_position_servo_ids={},{},{},{}
local xm_servo_ids, xm_position_servo_ids, xm_current_servo_ids,xm_velocity_servo_ids={},{},{},{}
local pro_servo_index,xm_servo_index={},{}


local pos_offsets, pos_steps,vel_steps,cur_steps={},{},{},{}

local actuator_inited=false
local last_p,current_p=nil,{0,0,0,0,0,0}

--make servo lists by its control mode
for i=1,#Config.servo.dynamixel_ids do
  if Config.servo.dynamixel_type[i]==1 then --XM series
    pos_offsets[i],pos_steps[i],vel_steps[i],cur_steps[i]=2048, 1/4096, 0.229*0.10472,2.69
    xm_servo_ids[#xm_servo_ids+1]=Config.servo.dynamixel_ids[i]
    xm_servo_index[#xm_servo_index+1]=i

    if Config.servo.dynamixel_mode[i]==0 then --current mode
      xm_current_servo_ids[#xm_current_servo_ids+1]=Config.servo.dynamixel_ids[i]
    elseif Config.servo.dynamixel_mode[i]==3 or Config.servo.dynamixel_mode[i]==4 then --position mode
      xm_position_servo_ids[#xm_position_servo_ids+1]=Config.servo.dynamixel_ids[i]
    elseif Config.servo.dynamixel_mode[i]==1 then --velocity mode
      xm_velocity_servo_ids[#xm_velocity_servo_ids+1]=Config.servo.dynamixel_ids[i]
    end

  else --Pro series
    if Config.servo.dynamixel_type[i]==2 then --Ph42-020-S300-R
      pos_offsets[i],pos_steps[i
      ],vel_steps[i],cur_steps[i]=0, 1/303750/2, 0.01/60,1
    else --PH54-100-S500-R, PH54-200-S500-R
      pos_offsets[i],pos_steps[i],vel_steps[i],cur_steps[i]=0, 1/501923/2, 0.01/60, 1
    end
    pro_servo_ids[#pro_servo_ids+1]=Config.servo.dynamixel_ids[i]
    pro_servo_index[#pro_servo_index+1]=i

    if Config.servo.dynamixel_mode[i]==0 then --current mode
      pro_current_servo_ids[#pro_current_servo_ids+1]=Config.servo.dynamixel_ids[i]
    elseif Config.servo.dynamixel_mode[i]==1 then --velocity
      pro_velocity_servo_ids[#pro_velocity_servo_ids+1]=Config.servo.dynamixel_ids[i]
    elseif Config.servo.dynamixel_mode[i]==3 or Config.servo.dynamixel_mode[i]==4 then --position
      pro_position_servo_ids[#pro_position_servo_ids+1]=Config.servo.dynamixel_ids[i]
    elseif Config.servo.dynamixel_mode[i]==5 then --position+current
      pro_current_servo_ids[#pro_current_servo_ids+1]=Config.servo.dynamixel_ids[i]
      pro_position_servo_ids[#pro_position_servo_ids+1]=Config.servo.dynamixel_ids[i]
    end
  end
end






local running=true
local function shutdown()
	io.write('\n Soft shutdown!\n') -- Cleanly exit on Ctrl-C
	running=false
  if #pro_servo_ids>0 then
    DXL.pro_torque_enable(pro_servo_ids,vector.zeros(#pro_servo_ids))
  end
  if #xm_current_servo_ids>0 then
    DXL.torque_enable(xm_servo_ids,vector.zeros(#xm_servo_ids))
  end
  dcm.set_actuator_torque_enable(vector.zeros(Config.nJoint))--TODO
  dcm.set_actuator_torque_enable_changed(1)
  gcm.set_processes_dynamixel({0,0,0})
end
signal("SIGINT", shutdown)
signal("SIGTERM", shutdown)



local function init_servos()
  if #pro_servo_ids>0 then DXL.pro_torque_enable(pro_servo_ids,vector.zeros(#pro_servo_ids)) end
  if #xm_current_servo_ids>0 then DXL.torque_enable(xm_current_servo_ids,vector.zeros(#xm_current_servo_ids))   end
  DXL.control_mode(Config.servo.dynamixel_ids,Config.servo.dynamixel_mode) --this is shared??
  dcm.set_actuator_torque_enable(vector.zeros(Config.nJoint))
  dcm.set_actuator_torque_enable_changed(1)
end


local function read_xm_position()
  if #xm_servo_ids==0 then return 1 end
  local servo_pos=DXL.read_position(xm_servo_ids)
  if not servo_pos then return 0 end
  local cur_position=dcm.get_sensor_position()
  local cur_command_position2=dcm.get_actuator_command_position2()
  local torque_enable=dcm.get_actuator_torque_enable()
  for i=1,#xm_servo_index do
    local idx=xm_servo_index[i]
    local dcm_idx=Config.servo.dynamixel_map[idx]
    -- print(i,idx,dcm_idx)
    -- print(#servo_pos, #pos_offsets, #pos_steps, #Config.servo.direction,#Config.servo.rad_offset)
    cur_position[dcm_idx]=(servo_pos[i]-pos_offsets[idx])*pos_steps[idx]*2*math.pi*Config.servo.direction[dcm_idx]-Config.servo.rad_offset[dcm_idx]
    if torque_enable[dcm_idx]==0
    then cur_command_position2[dcm_idx]=cur_position[dcm_idx] end
  end
  dcm.set_sensor_position(cur_position)
  dcm.set_actuator_command_position2(cur_command_position2)
  return 1
end

local function read_xm_voltage()
  if #xm_servo_ids==0 then return 1 end
  local servo_voltage=DXL.read_voltage(xm_servo_ids)
  return servo_voltage
end

local function read_pro_position()
  if #pro_servo_ids==0 then return 1 end
  local servo_pos=DXL.pro_read_position(pro_servo_ids)
  if not servo_pos then return 0 end
  local cur_position=dcm.get_sensor_position()
  local cur_command_position2=dcm.get_actuator_command_position2()
  local torque_enable=dcm.get_actuator_torque_enable()
  for i=1,#pro_servo_index do
    local index=pro_servo_index[i]
    local dcm_index=Config.servo.dynamixel_map[index]

    -- cur_position[Config.servo.dynamixel_map[i]]=
    --   (servo_pos[i]-MAX_STEPS/2)*POS_STEPS*2*math.pi*Config.servo.direction[Config.servo.dynamixel_map[i]]
    cur_position[dcm_index]=
      (servo_pos[index])*pos_steps[index]*2*math.pi*Config.servo.direction[dcm_index]-Config.servo.rad_offset[dcm_index]
    if torque_enable[dcm_index]==0 then cur_command_position2[dcm_index]=cur_position[dcm_index] end
  end
  dcm.set_sensor_position(cur_position)
  dcm.set_actuator_command_position2(cur_command_position2)
  return 1
end


local function write_pro_position(dt)
  if #pro_position_servo_ids>0 then
    if not actuator_inited then
      print"RESET"
      local cur_position=dcm.get_sensor_position()
      dcm.set_actuator_command_position2(cur_position)
      actuator_inited=true
    end
    local cmd_position=dcm.get_actuator_command_position()
    local cmd_position2=dcm.get_actuator_command_position2() --current command position (velocity limited)
    for i=1,#pro_servo_index do
      local index=pro_servo_index[i]
      local dcm_index=Config.servo.dynamixel_map[index]
      local max_servo_velocity=Config.servo.max_vel[dcm_index]
      local d_movement = cmd_position[dcm_index]-cmd_position2[dcm_index]
      cmd_position2[dcm_index]= cmd_position2[dcm_index]+util.procFunc(d_movement, 0, dt * max_servo_velocity)
    end
    dcm.set_actuator_command_position2(cmd_position2)

    local servo_position={}
    local count=1
    for i=1,#pro_servo_index do
      local index=pro_servo_index[i]
      local dcm_index=Config.servo.dynamixel_map[index]
      if Config.servo.dynamixel_mode[index]>=3 then
        servo_position[#servo_position+1]=
          (cmd_position2[dcm_index]+Config.servo.rad_offset[dcm_index])/
          pos_steps[index]*Config.servo.direction[dcm_index]/2/math.pi
      end
    end
    DXL.pro_set_goal_positions(pro_position_servo_ids,servo_position)
    unix.usleep(1E6*0.002)
  end
end

local function write_xm_position(dt)
  if #xm_position_servo_ids>0 then
    if not actuator_inited then
      print"RESET"
      local cur_position=dcm.get_sensor_position()
      dcm.set_actuator_command_position2(cur_position)
      actuator_inited=true
    end
    local cmd_position=dcm.get_actuator_command_position()
    local cmd_position2=dcm.get_actuator_command_position2() --current command position (velocity limited)
    local servo_position={}
    for i=1,#xm_servo_index do
      local idx=xm_servo_index[i]

      local dcm_idx=Config.servo.dynamixel_map[idx]
      local max_servo_velocity=Config.servo.max_vel[dcm_idx]
      local d_movement = cmd_position[dcm_idx]-cmd_position2[dcm_idx]
      if Config.servo.dynamixel_mode[idx]>=3 then
        cmd_position2[dcm_idx]= cmd_position2[dcm_idx]+util.procFunc(d_movement, 0, dt * max_servo_velocity)
        servo_position[#servo_position+1]=(cmd_position2[dcm_idx]+Config.servo.rad_offset[dcm_idx])/
          pos_steps[idx]*Config.servo.direction[dcm_idx]/2/math.pi+pos_offsets[idx]
      end
    end
    dcm.set_actuator_command_position2(cmd_position2)
    DXL.set_goal_positions(xm_position_servo_ids,servo_position)
    unix.usleep(1E6*0.002)
  end
end

-- local function write_velocity(dt)
--   if #velocity_servo_ids>0 then
--     local cmd_velocity=dcm.get_actuator_command_velocity()
--     local servo_velocity={}
--     local count=1
--     for i=1,#pro_servo_ids do

--       if Config.servo.dynamixel_mode[i]==1 then --velocity
--         -- print(i,velocity_servo_ids[i],Config.servo.dynamixel_map[i],Config.servo.direction[i])
-- 	       servo_velocity[count]=cmd_velocity[Config.servo.dynamixel_map[i]]/vel_steps[i]*Config.servo.direction[i]
--         count=count+1
--       end
--     end
--     DXL.pro_set_goal_velocities(velocity_servo_ids,servo_velocity)
--     unix.usleep(1E6*0.002)
--   end
-- end

local function write_xm_current(dt)
  if #xm_current_servo_ids>0 then
    local cmd_current=dcm.get_actuator_command_current()
    local servo_current={}

    for i=1,#xm_servo_index do
      local idx=xm_servo_index[i]
      local dcm_idx=Config.servo.dynamixel_map[idx]
      if Config.servo.dynamixel_mode[idx]==0 or Config.servo.dynamixel_mode[idx]==5 then
        servo_current[#servo_current+1]=cmd_current[dcm_idx]/cur_steps[idx]*Config.servo.direction[dcm_idx]
      end
    end
    DXL.set_goal_currents(xm_current_servo_ids,servo_current)
    unix.usleep(1E6*0.002)
  end
end

local function write_xm_velocity(dt)
  if #xm_velocity_servo_ids>0 then
    local cmd_velocity=dcm.get_actuator_command_velocity()
    local servo_velocity={}

    for i=1,#xm_servo_index do
      local idx=xm_servo_index[i]
      local dcm_idx=Config.servo.dynamixel_map[idx]
      if Config.servo.dynamixel_mode[idx]==1 then
        servo_velocity[#servo_velocity+1]=cmd_velocity[dcm_idx]/vel_steps[idx]*Config.servo.direction[dcm_idx]
      end
    end
    DXL.set_goal_velocities(xm_velocity_servo_ids,servo_velocity)
    unix.usleep(1E6*0.002)
  end
end

local function write_torque_enable()
  if dcm.get_actuator_torque_enable_changed()==1 then
    dcm.set_actuator_torque_enable_changed(0)
    print("TORQUE ENABLED CHANGED!!!")
    local torque_enable=dcm.get_actuator_torque_enable()
    if #pro_servo_ids>0 then
      local t_var={}
      for i=1,#Config.servo.dynamixel_map do
        if Config.servo.dynamixel_type[i]>1 then
          t_var[#t_var+1]=torque_enable[Config.servo.dynamixel_map[i]]
        end
      end
      DXL.pro_torque_enable(pro_servo_ids,t_var)
      unix.usleep(1E6*0.005)
    end
    if #xm_servo_ids>0 then --enable xm servo
      local t_var={}
      for i=1,#Config.servo.dynamixel_map do
        if Config.servo.dynamixel_type[i]==1 then
          t_var[#t_var+1]=torque_enable[Config.servo.dynamixel_map[i]]
        end
      end
      -- print("XM TORQUE ON!")
      -- print(unpack(xm_current_servo_ids))
      -- print(unpack(t_var))
      DXL.torque_enable(xm_servo_ids,t_var)
      unix.usleep(1E6*0.005)
    end
  end
end



-- --
local count=0
t0 = unix.time()
t_last=unix.time()
t_last_debug=t0

local ret=DXL.openport(dev_name, baudrate)
init_servos()

while running do
  local t = unix.time()
  local dt=t-t_last
  t_last=t
  if should_wait_dxl then
    local rp=read_pro_position()
    local rx=read_xm_position()
    if rx+rp==2 then
      print "DXL connected!!!!"
      --update current velocity limited target positions (to prevent big jump)
      dcm.set_actuator_command_position2(dcm.get_sensor_position()) --current command position (velocity limited)
      dcm.set_actuator_torque_enable_changed(1)
      write_torque_enable()
      should_wait_dxl=false
    else
      print("\nDXL turned off, waiting....")
      unix.usleep(1E6*1)
      mcm.set_servo_offline(1)
      local gccount = gcm.get_processes_dynamixel()
      gcm.set_processes_dynamixel({1,gccount[2]+1,gccount[3]})
    end


    -- if rx+rp>0 then
    --
    --
    --
    -- end
    --
    -- if read_position() then
    --   print "DXL connected!!!!"
    --   --update current velocity limited target positions (to prevent big jump)
    --   dcm.set_actuator_command_position2(dcm.get_sensor_position()) --current command position (velocity limited)
    --   dcm.set_actuator_torque_enable_changed(1)
    --   write_torque_enable()
    --   should_wait_dxl=false
    -- else
    --   print("\nDXL turned off, waiting....")
    --   unix.usleep(1E6*1)
    --   mcm.set_servo_offline(1)
    --   local gccount = gcm.get_processes_dynamixel()
    --   gcm.set_processes_dynamixel({1,gccount[2]+1,gccount[3]})
    -- end
  else
    -- mcm.set_servo_offline(0)
    local rp=read_pro_position()
    local rx=read_xm_position()

    if rp+rx<2 then
      should_wait_dxl = true
      actuator_inited=false
      local gccount = gcm.get_processes_dynamixel()
      gcm.set_processes_dynamixel({1,gccount[2]+1,gccount[3]})
    else
      write_torque_enable()
      write_pro_position(dt)
      write_xm_position(dt)
      write_xm_current(dt)
      write_xm_velocity(dt)
    end

    --write_velocity()
    -- write_xm_current()
    count=count+1
    local gccount = gcm.get_processes_dynamixel()
    gcm.set_processes_dynamixel({2,gccount[2]+1,gccount[3]})
  end

  if t>t_last_debug + 0.5 then
  -- if t>t_last_debug + 0.2 then
    local batt=read_xm_voltage()
    local t1=unix.time()
    local t_elapsed=t1-t0
    t_last_debug=t
    if not should_wait_dxl then
      local cur_pos=dcm.get_sensor_position()
      local cur_cpos=dcm.get_actuator_command_position()
      local torque_enabled=dcm.get_actuator_torque_enable()
      local str=""
      for i=1,#Config.servo.dynamixel_map do
        local dcm_idx=Config.servo.dynamixel_map[i]
        local pos, cmd_pos, tenable=cur_pos[dcm_idx],cur_cpos[dcm_idx],torque_enabled[dcm_idx]
        -- str=str..string.format("[ID%d pos%.1f cpos%.1f]",Config.servo.dynamixel_ids[i],pos/DEG_TO_RAD, cmd_pos/DEG_TO_RAD)

        local pstr=string.format("#%d:%.1f",Config.servo.dynamixel_ids[i],pos/DEG_TO_RAD)
        if tenable>0 then
          str=str.."[".. util.color(pstr,'green').."]"
          -- str=str..string.format("[ID%d pos%.1f cpos%d]",Config.servo.dynamixel_ids[i],pos/DEG_TO_RAD, cmd_pos/DEG_TO_RAD)
        else
          str=str.."[".. util.color(pstr,'yellow').."]"
          -- str=str..string.format("<ID%d pos%.1f cpos%d>",Config.servo.dynamixel_ids[i],pos/DEG_TO_RAD, cmd_pos/DEG_TO_RAD)
        end
      end
      local hz=count/t_elapsed
      local dxl_str=string.format("DXL: %.1fHz / %.1f V" , count/t_elapsed,math.max(unpack(batt))/10 )
      if hz<10 then print(util.color(dxl_str,'red')..str)
      else print(util.color(dxl_str,'green')..str) end
    end
    t0=t1
    count=0
  end
end

DXL.closeport()
