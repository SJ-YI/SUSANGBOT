#!/usr/bin/env luajit
local pwd = os.getenv'PWD'
if string.find(pwd,"Run") then	dofile('../include.lua')
else	dofile('../../include.lua') end
local si = require'simple_ipc'
local ffi=require'ffi'
local unix=require'unix'
local vector = require'vector'
require 'dcm'
require'mcm'
local Body=require 'Body'


--torque on wheels
Body.set_wheel_torque_enable(1)
Body.set_arm_torque_enable(0)
dcm.set_actuator_torque_enable_changed(1)


Body.set_wheel_command_velocity({0.1,0.1,0.1,0.1})



--SAFELY torque on arm
local cur_servo_pos=Body.get_arm_position()
print(unpack(cur_servo_pos))
Body.set_arm_command_position(cur_servo_pos)
unix.usleep(1E6*0.1)
Body.set_arm_torque_enable(1)
dcm.set_actuator_torque_enable_changed(1)



Body.set_arm_command_position({0,-45*DEG_TO_RAD,0*DEG_TO_RAD,0,45*DEG_TO_RAD,0})
unix.usleep(1E6*6)

Body.set_arm_command_position({0,-88*DEG_TO_RAD,88*DEG_TO_RAD,0,0,0})
unix.usleep(1E6*4)

unix.usleep(1E6*2)
Body.set_wheel_command_velocity({0,0,0,0})

Body.set_arm_torque_enable(0)
Body.set_wheel_torque_enable(0)

dcm.set_actuator_torque_enable_changed(1)

--
-- unix.usleep(1E6*8)
--
-- Body.set_arm_command_position({0,0,0,0,0,0})

-- count=0
-- target=90*DEG_TO_RAD
-- target=0.1 -- 0.11 rotation per second
--
-- while true do
--   if mcm.get_servo_offline()==1 then
--     print("OFFOFFOFF")
--   else
--     -- count=count+1
--     -- -- if count%20==0 then target=-target end
--     -- Body.set_wheel_command_velocity({target,0,0,0})
--
--     unix.usleep(1E6*0.1)
--   end
-- end
