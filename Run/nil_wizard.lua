#!/usr/bin/env luajit
local ok = pcall(dofile,'../fiddle.lua')
if not ok then dofile'fiddle.lua' end

local last_t=unix.time()

-- while true do
-- --	os.execute('clear')
-- --	print("=======================================================")
-- 	local t = unix.time()
-- 	local dt = t-last_t
-- 	last_t = t
-- 	unix.usleep(0.5*1E6)
-- end

local function entry() end
local function update() end
local function exit() end



if ... and type(...)=='string' then --when called by webots process
  return {entry=entry, update=update, exit=exit}
end
