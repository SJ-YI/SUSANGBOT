#!/usr/bin/env luajit
local pwd = os.getenv'PWD'
if string.find(pwd,"Run") then	dofile('../include.lua')
else	dofile('../../include.lua') end

local Body=require 'Body'


if #arg==0 then print("Enter motion number")
else log_index=arg[1]
local motionname=string.format("motionlog%04d",log_index)

--motiondata=require'motionlog0026'
motiondata=require(motionname)

print(#motiondata)

Body.set_arm_torque_enable({1,1,1,1,1,1,1,1})
dcm.set_actuator_torque_enable_changed(1)


for i=1,#motiondata do
  print(i)
  local dat=motiondata[i]
  local armc={dat[4],dat[5],dat[6],dat[7],   dat[8],dat[9],dat[10],dat[11]}
  Body.set_arm_command_position(armc)
  unix.usleep(1E6*0.01)
end

end
