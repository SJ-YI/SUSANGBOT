#!/usr/bin/env luajit
dofile('../include.lua')
require'dcm'
local vector = require'vector'
local util=require'util'
local Body=require'Body'
require'hcm'




-- Cleanly exit on Ctrl-C
local signal = require'signal'.signal
local running = true
local function shutdown()
	print("SHUTDOWN!!!")
	running = false
end
signal("SIGINT", shutdown)
signal("SIGTERM", shutdown)




print("press any key for torque on")
io.flush()
io.read()


local armdata1=require'armmotion1'
local armdata2=require'armmotion2'

dcm.set_mitmotor_operation_mode({
    1,1,1, --motors, velocity mode
    3,3,3,3, --left arm, pd mode
    3,3,3,3, --right arm, pd mode (slave)
    })


--local maxtorque,pgain=80,20
local maxtorque,pgain=160,20 --need more torque for item grab
dcm.set_mitmotor_max_torque(vector.ones(11)*maxtorque)
dcm.set_mitmotor_pgain(vector.ones(Config.nJoint)*pgain)
dcm.set_mitmotor_dgain(vector.ones(Config.nJoint)*-1)
Body.set_arm_command_torque({0,0,0,0, 0,0,0,0})

Body.set_arm_torque_enable({1,1,1,1, 1,1,1,1})
dcm.set_actuator_torque_enable_changed(1)


for i=1,#armdata1 do
  local armframe=armdata1[i]
  local armtarget={
		armframe[2]*DEG_TO_RAD,armframe[3]*DEG_TO_RAD,armframe[4]*DEG_TO_RAD,armframe[5]*DEG_TO_RAD,
		armframe[6]*DEG_TO_RAD,armframe[7]*DEG_TO_RAD,armframe[8]*DEG_TO_RAD,armframe[9]*DEG_TO_RAD,
  }
  Body.set_arm_command_position(armtarget)
  unix.usleep(1E6/100)
	print(i,#armdata1)
end


print("press any key for torque on")
io.flush()
io.read()

for i=1,#armdata2 do
  local armframe=armdata2[i]
  local armtarget={
		armframe[2]*DEG_TO_RAD,armframe[3]*DEG_TO_RAD,armframe[4]*DEG_TO_RAD,armframe[5]*DEG_TO_RAD,
		armframe[6]*DEG_TO_RAD,armframe[7]*DEG_TO_RAD,armframe[8]*DEG_TO_RAD,armframe[9]*DEG_TO_RAD,
  }
  Body.set_arm_command_position(armtarget)
  unix.usleep(1E6/100)
end

print("press any key for torque on")
io.flush()
io.read()

Body.set_arm_torque_enable({0,0,0,0, 0,0,0,0})
dcm.set_actuator_torque_enable_changed(1)
dcm.set_mitmotor_operation_mode({
    1,1,1, --motors, velocity mode
    2,2,2,2,
    2,2,2,2
    })
