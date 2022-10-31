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

while running do
  print("Sending...")
  rospub.tf({0,0,0},{0,0,0}, "map","base_link")

	rospub.gpsfix("base_link",35.2330062, 129.0827480)

  unix.usleep(1e6*0.05) --20fps
end
