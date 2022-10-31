#!/usr/bin/env luajit
-- (c) 2014 Team THORwIn
local ok = pcall(dofile,'../fiddle.lua')
if not ok then ok=pcall(dofile,'./fiddle.lua') end
if not ok then ok=dofile'../../fiddle.lua' end

local util = require'util'
local signal = require'signal'.signal
local gettime = require'unix'.time
local rospub = require 'rospub_core'

local tDelay = 0.005*1E6
local running = true
-- Cleanly exit on Ctrl-C
local function shutdown()
	io.write('\n Soft shutdown!\n')
	running=false
end
signal("SIGINT", shutdown)
signal("SIGTERM", shutdown)
-- rospub.init('test_pub')
print("INIT")
rospub.init("test_pub")


lat0=35.2330062
long0=129.0827480

d_lat=0
d_long=0
local count=0

while running do
  print("Sending...")


	count=count+1

	d_lat=d_lat+0.00001



	local r_earth=6371000
	local dx= r_earth*d_long*math.cos(d_lat*DEG_TO_RAD)*DEG_TO_RAD
	local dy= r_earth*d_lat*DEG_TO_RAD

	-- rospub.tf({dx,dy,0},{0,0,0}, "map","base_link")
	rospub.tf({dx,dy,0},{0,0,0}, "map","base_link")
	-- rospub.tf({0,0,0},{0,0,0}, "map","base_link")
	rospub.gpsfix("base_link",lat0+d_lat, long0+d_long)

  unix.usleep(1e6*0.05) --20fps
end
