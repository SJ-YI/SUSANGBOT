#!/usr/bin/env luajit
local pwd = os.getenv'PWD'
if string.find(pwd,"Run") then	dofile('../include.lua')
else	dofile('../../include.lua') end

local Body=require 'Body'
motiondata=require'motion0011'
print(#motiondata)

for i=1,#motiondata do
  local dat=motiondata[i]
  local armc={dat[4],dat[5],dat[6],dat[7],   dat[8],dat[9],dat[10],dat[11]}
  Body.set_arm_command_position(armc)
  unix.usleep(1E6*0.01)
end
