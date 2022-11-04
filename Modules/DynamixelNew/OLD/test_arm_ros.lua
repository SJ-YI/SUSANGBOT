#!/usr/bin/env luajit
local pwd = os.getenv'PWD'
if string.find(pwd,"Run") then	dofile('../include.lua')
else	dofile('../../include.lua') end
local unix=require'unix'
local vector = require'vector'


local rospub = require 'rospub'
rospub.init('lua_interface_pub')



rospub.jointgoal({0,0,0,0},{0,0,0,0},{0,0,0,0})
unix.usleep(1E6*1)


rospub.jointgoal({0,0,0,30*DEG_TO_RAD},{0,0,0,0},{0,0,0,0})
unix.usleep(1E6*1)


rospub.jointgoal({0,0,0,-30*DEG_TO_RAD},{0,0,0,0},{0,0,0,0})
unix.usleep(1E6*1)


rospub.jointgoal({0,0,0,0},{0,0,0,0},{0,0,0,0})
unix.usleep(1E6*1)



unix.usleep(1E6*5)
