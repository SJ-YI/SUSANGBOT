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
require'hcm'
require'gcm'

--THEY ARE ALL THE SAME FOR XM SERIES
local MAX_STEPS = 4096
local POS_STEPS = 1/4096 --1/4096 rotation per one step
local VEL_STEPS = 0.229 ---0.229 rpm per one unit
local CUR_STEPS = 2.69 ---2.69 mA per one unit

local dev_name, baudrate="/dev/usb2dxl",1000000
local should_wait_dxl = true
local all_servo_ids, velocity_servo_ids, current_servo_ids, position_servo_ids={},{},{},{}
local actuator_inited=false
local last_p,current_p=nil,{0,0,0,0,0,0}

--make servo lists by its control mode
for i=1,#Config.servo.dynamixel_ids do
  all_servo_ids[#all_servo_ids+1]=Config.servo.dynamixel_ids[i]
  if Config.servo.dynamixel_mode[i]==0 then --current
    current_servo_ids[#current_servo_ids+1]=Config.servo.dynamixel_ids[i]
  elseif Config.servo.dynamixel_mode[i]==1 then --velocity
    velocity_servo_ids[#velocity_servo_ids+1]=Config.servo.dynamixel_ids[i]
  elseif Config.servo.dynamixel_mode[i]==3 then --position
    position_servo_ids[#position_servo_ids+1]=Config.servo.dynamixel_ids[i]
  elseif Config.servo.dynamixel_mode[i]==5 then --position+current
    current_servo_ids[#current_servo_ids+1]=Config.servo.dynamixel_ids[i]
    position_servo_ids[#position_servo_ids+1]=Config.servo.dynamixel_ids[i]
  end
end

local running=true
local function shutdown()
	io.write('\n Soft shutdown!\n') -- Cleanly exit on Ctrl-C
	running=false
  gcm.set_processes_dynamixel({0,0,0})
end
signal("SIGINT", shutdown)
signal("SIGTERM", shutdown)

local function init_servos()
  DXL.torque_enable(Config.servo.dynamixel_ids,vector.zeros(#all_servo_ids))
  unix.usleep(1E6*0.005)
  DXL.control_mode(all_servo_ids,Config.servo.dynamixel_mode)
  unix.usleep(1E6*0.005)
  -- dcm.set_actuator_torque_enable_changed(1)
end


-- -------------------------------------------------------------------------------------
-- local function velocity_update(ret)
--   --TODO.. this should be handled by Body.set_wheel_velocity()
--   local vel = hcm.get_xb360_vel()
--   local targetvel={vel[1]*0.20 ,vel[2]*0.10  ,vel[3]*0.5 } --X, Y, A speed in meter per sec
--
--   --rpm conversion
--   local wc = Config.wheels.r*2*math.pi
--   local fwd_vel=vector.new({
--     targetvel[1]/wc*60,
--     targetvel[1]/wc*60,
--     targetvel[1]/wc*60,
--     targetvel[1]/wc*60
--   })
--   local rotate_vel=vector.new({
--     targetvel[3]/wc*60*Config.wheels.af,
--     -targetvel[3]/wc*60*Config.wheels.af,
--     targetvel[3]/wc*60*Config.wheels.af,
--     -targetvel[3]/wc*60*Config.wheels.af,
--   })
--
--   local side_vel=vector.new({
--     targetvel[2]/wc*60*Config.wheels.yf,
--    -targetvel[2]/wc*60*Config.wheels.yf,
--     -targetvel[2]/wc*60*Config.wheels.yf,
--     targetvel[2]/wc*60*Config.wheels.yf,
--   })
--   local dxl_velocity=(fwd_vel+rotate_vel+side_vel)
--   local cur_vel = dcm.get_actuator_command_velocity()
--   for i=1,#dxl_velocity do cur_vel[i]=dxl_velocity[i] end
--   dcm.set_actuator_command_velocity(cur_vel)
-- end
-- -------------------------------------------------------------------------------------




local function read_position()
  unix.usleep(1E6*0.0025)
  local servo_pos=DXL.read_position2(all_servo_ids)
  -- local servo_pos=DXL.read_position(all_servo_ids)
  if not servo_pos then
    should_wait_dxl = true
    actuator_inited=false
    local gccount = gcm.get_processes_dynamixel()
    gcm.set_processes_dynamixel({1,gccount[2]+1,gccount[3]})
    return
  end
  local cur_position=dcm.get_sensor_position()
  for i=1,#all_servo_ids do
    cur_position[Config.servo.dynamixel_map[i]]=
      (servo_pos[i]-MAX_STEPS/2)*POS_STEPS*2*math.pi*Config.servo.direction[Config.servo.dynamixel_map[i]]
  end
  dcm.set_sensor_position(cur_position)
end

local function write_position(dt)
  if not actuator_inited then
    print"RESET"
    local cur_position=dcm.get_sensor_position()
    dcm.set_actuator_command_position2(cur_position)
    -- dcm.set_actuator_command_position(cur_position)
    actuator_inited=true
  end

  if #position_servo_ids>0 then
    local max_servo_velocity = 60*DEG_TO_RAD
    local cmd_position=dcm.get_actuator_command_position() --target command position
    local cmd_position2=dcm.get_actuator_command_position2() --current command position
    for i=1,#all_servo_ids do
      local d_movement = cmd_position[Config.servo.dynamixel_map[i]]-cmd_position2[Config.servo.dynamixel_map[i]]
      cmd_position2[Config.servo.dynamixel_map[i]]= cmd_position2[Config.servo.dynamixel_map[i]]+util.procFunc(d_movement, 0, dt * max_servo_velocity)
    end
    dcm.set_actuator_command_position2(cmd_position2)

    local servo_position={}
    local count=1
    for i=1,#all_servo_ids do
      if Config.servo.dynamixel_mode[i]==3 or Config.servo.dynamixel_mode[i]==5 then --position--0 for current, 1 for velocity, 3 for pos, 4 for multi-turn pos, 5 for curret-limited pos
        servo_position[count]=cmd_position2[Config.servo.dynamixel_map[i]]/POS_STEPS*Config.servo.direction[Config.servo.dynamixel_map[i]]/2/math.pi + (MAX_STEPS/2)
        count=count+1
      end
    end
    DXL.set_goal_positions2(position_servo_ids,servo_position)
    unix.usleep(1E6*0.005)
  end
end
local function write_velocity(dt)
  if #velocity_servo_ids>0 then
    local cmd_velocity=dcm.get_actuator_command_velocity()
    local servo_velocity={}
    local count=1
    for i=1,#all_servo_ids do
      if Config.servo.dynamixel_mode[i]==1 then --velocity
	       servo_velocity[count]=cmd_velocity[Config.servo.dynamixel_map[i]]/VEL_STEPS*Config.servo.direction[Config.servo.dynamixel_map[i]]
        count=count+1
      end
    end
    DXL.set_goal_velocities(velocity_servo_ids,servo_velocity)
    unix.usleep(1E6*0.005)
  end
end

local function write_current(dt)
  if #current_servo_ids>0 then
    local cmd_current= dcm.get_actuator_command_current()
    local servo_current={}
    local count=1
    for i=1,#all_servo_ids do
      servo_current[count]=cmd_current[Config.servo.dynamixel_map[i]]/CUR_STEPS*Config.servo.direction[i]
      count=count+1
    end
    DXL.set_goal_currents(current_servo_ids,servo_current)
    unix.usleep(1E6*0.005)
  end
end

local function write_torque_enable()
  if dcm.get_actuator_torque_enable_changed()==1 then
    print("TORQUE SET!!")
    dcm.set_actuator_torque_enable_changed(0)
    local torque_enable=dcm.get_actuator_torque_enable()
    local t_var={}
    for i=1,#Config.servo.dynamixel_map do
      t_var[i]=torque_enable[Config.servo.dynamixel_map[i]]
    end
    DXL.torque_enable(all_servo_ids,t_var)
    unix.usleep(1E6*0.005)
  end
end



-- --
local count=0
t0 = unix.time()
t_last=unix.time()
t_last_debug=t0


local ret=DXL.openport(dev_name, baudrate)
DXL.ping_probe()
init_servos()


while running do
  local t = unix.time()
  local dt=t-t_last
  t_last=t
  if should_wait_dxl then
    local ret=DXL.read_position2(all_servo_ids)
    if ret then
      print "DXL connected!!!!"
      dcm.set_actuator_torque_enable_changed(1)
      write_torque_enable()
      should_wait_dxl=false
    else
      print("\nDXL turned off, waiting....")
      unix.usleep(1E6*1)
      local gccount = gcm.get_processes_dynamixel()
      gcm.set_processes_dynamixel({1,gccount[2]+1,gccount[3]})
    end
  else
    read_position()
    write_torque_enable()
    write_position(dt)
    write_current()
    -- write_velocity()
    count=count+1
    local gccount = gcm.get_processes_dynamixel()
    gcm.set_processes_dynamixel({2,gccount[2]+1,gccount[3]})
  end

  if t>t_last_debug + 0.5 then
  -- if t>t_last_debug + 0.2 then
    local t1=unix.time()
    local t_elapsed=t1-t0
    t_last_debug=t
    if not should_wait_dxl then
      local cur_pos=dcm.get_sensor_position()
      local dxl_pos={}
      for i=1,#Config.servo.dynamixel_map do dxl_pos[i]=cur_pos[Config.servo.dynamixel_map[i]] end
      local str=''
      for i=1,#dxl_pos do str=str..string.format("%.1f ",dxl_pos[i]*180/math.pi) end

      local cur_cpos=dcm.get_actuator_command_position()
      local dxl_cpos={}
      for i=1,#Config.servo.dynamixel_map do dxl_pos[i]=cur_cpos[Config.servo.dynamixel_map[i]] end
      local str2=''
      for i=1,#dxl_cpos do str2=str2..string.format("%.1f ",dxl_cpos[i]*180/math.pi) end

      print(string.format("Dynamixel: %.1f fps pos: %s" , count/t_elapsed,str))
      print(str2)
    end
    t0=t1
    count=0
  end
end

DXL.closeport()
