#!/usr/bin/env luajit
local pwd = os.getenv'PWD'
local ok = pcall(dofile,'../fiddle.lua')
if not ok then dofile'fiddle.lua' end
require'wcm'
local unix=require'unix'
local vector=require'vector'
local rossub=require'rossub'

local sub_idx_voice
local t0

local function entry()
  t0=unix.time()
  rossub.init('tb3_voice')
  sub_idx_voice=rossub.subscribeString('/voice')
end

local function check_ros_msg()
  local t=unix.time()
  local cvoice=rossub.checkString(sub_idx_voice)
  if cvoice and t-t0>2.0 then
    print(cvoice)
    os.execute('mpg321 ~/Desktop/ARAICodes/Media/'..cvoice)
  end
end

local function update()
  check_ros_msg()
end

if ... and type(...)=='string' then --webots handling
  WAS_REQUIRED = true
  return {entry=entry, update=update, exit=nil}
end

local count,tpassed,t_last_debug,max_t_passed=0,0,unix.time(),0
local running = true
local key_code
entry()
while running do
  local t0=unix.time()
  update()
  local t1=unix.time()
  tpassed=tpassed+t1-t0
  max_t_passed=math.max(t1-t0,max_t_passed)
  count=count+1
  unix.usleep(1E6*0.01)
  if count>500 then
    local t=unix.time()
    local t_total=t-t_last_debug
    print(string.format("Voice wizard, average %.1f fps / %.2f ms, peak %.2f ms",
      count/t_total,tpassed/count*1000,max_t_passed*1000))
    tpassed,count,t_last_debug,max_t_passed=0,0,t,0
  end
end
