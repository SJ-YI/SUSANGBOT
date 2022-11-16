#!/usr/bin/env luajit
-- (c) 2014 Team THORwIn
local ok = pcall(dofile,'../fiddle.lua')
if not ok then ok=pcall(dofile,'./fiddle.lua') end
if not ok then ok=dofile'../../fiddle.lua' end
require'mcm'
require'hcm'
local si = require'simple_ipc'

local use_ros,rospub=false,nil
local send_cmd_vel=false
local cmd_type=0
local not_tb3=true
local is_ur10=false

local util = require'util'
local xbox360 = require 'xbox360'
local signal = require'signal'.signal
local gettime = require'unix'.time
rospub=require'tb3_rospub'
rospub.init('xb360_pub')

use_ros=true
-- rospub=require'tb3_rospub'
-- rospub.init('xb360_pub')
-- not_tb3=false



local tDelay = 0.005*1E6
local running = true

-- Cleanly exit on Ctrl-C
local function shutdown()
	io.write('\n Soft shutdown!\n')
	running=false
  gcm.set_processes_xb360({0,0,0})
end
signal("SIGINT", shutdown)
signal("SIGTERM", shutdown)


--local product_id = 0x0291
--local product_id = 0x028e
local product_id1 = 0x0719 --standard black receiver
--local product_id = 0x02a9
local product_id2 = 0x0291 --white receiver
local product_id3 = 0x028e --short white receiver
local product_id4 = 0x02a9 --new black receiver
local dongle_not_found=true



local use_white_dongle=true


gcm.set_processes_xb360({1,0,0})

while dongle_not_found and running do
	ret=xbox360.check({product_id1,product_id2,product_id3,product_id4})
	if ret then
		print("donglefound:",ret)
		dongle_not_found=false;
	else
		print("XB360: Dongle not found, waiting...")
		unix.usleep(1e6*1) --20fps
	end
end


local seq=0
local head_pitch=0
local t_last_head_toggle=0
local t_last_sent=Body.get_time()
local t_last_update=Body.get_time()
local t_last = gettime()
local t_last_debug = gettime()
local t_debug_interval = 1
local t_last_button= gettime()
local slam_started=false
local slam_ended=false
local targetvel={0,0,0}
local curvel={0,0,0}
local last_control_t = gettime()

local gtarget=0


local function update(ret)
  local t = gettime()
  local dt = t-t_last
  t_last=t

--BACK START LB RB XBOX X Y A B
  local x_db,x_max = 10,255
  local y_db,y_max = 8000,32768
  local y_db_rot = 14000

  if use_white_dongle then
    y_db, y_max, y_db_rot = 20, 128, 60
  end

  --Left/right trigger: forward and backward
  --Left stick L/R: rotate
  --Left stick U/D: look up/down
  --Right stick L/R: strafe

	local lt =( util.procFunc(ret.trigger[1],x_db,x_max )/ (x_max-x_db))
	local rt =( util.procFunc(ret.trigger[2],x_db,x_max )/ (x_max-x_db))
	local lax =util.procFunc(ret.lstick[1],y_db,y_max )/(y_max-y_db)
	local lay =util.procFunc(ret.lstick[2],y_db_rot,y_max )/(y_max-y_db)
	local rax =util.procFunc(ret.rstick[1],y_db,y_max )/(y_max-y_db)
	local ray =util.procFunc(ret.rstick[2],y_db,y_max )/(y_max-y_db)

	local maxvel=Config.max_vel or {0.50,0.50,1.0}
	targetvel={(rt-lt) * maxvel[1],ray*maxvel[2],lay*maxvel[3]}

   -- mcm.set_walk_vel(targetvel)


  if ret.buttons[1]==1 and ret.buttons[2]==1 then
    gcm.set_game_selfdestruct(1)
    running=false
  end

  if t-t_last_debug>t_debug_interval then
    print(string.format(
      "Dpad:%d %d LStick:%d %d RStick:%d %d LT%d RT%d",
      ret.dpad[1],ret.dpad[2],  ret.lstick[1],ret.lstick[2],  ret.rstick[1],ret.rstick[2],  ret.trigger[1],ret.trigger[2])
      .. string.format("Buttons:%d%d%d%d%d%d%d%d%d",  unpack(ret.buttons))
    )
    t_last_debug=t
  end

	hcm.set_xb360_dpad({ret.dpad[1],ret.dpad[2]})
	hcm.set_xb360_buttons(ret.buttons)
	hcm.set_xb360_lstick(ret.lstick)
	hcm.set_xb360_rstick(ret.rstick)
	hcm.set_xb360_trigger(ret.trigger)


	if use_ros then
		if t-t_last_button>0.5 then
			if ret.buttons[1]==1 then --START and STOP MAPPING
				print("MAPPING START!!!!!!!!!")

				rospub.mapcmd(1)
				t_last_button=t
			end
			if ret.buttons[2]==1 then --START and STOP MAPPING
				print("GAME START!!!!!!!!!")
				os.execute("espeak 'start'")
				rospub.mapcmd(9)
				t_last_button=t
			end
			if ret.buttons[4]==1 then --ADD MARKER
				os.execute("espeak 'add'")
				rospub.mapcmd(3)
				t_last_button=t
			end
			if ret.buttons[3]==1 then --REMOVE MARKER
				os.execute("espeak 'remove'")
				rospub.mapcmd(4)
				t_last_button=t
			end
			if ret.buttons[2]==8 then --START and STOP MAPPING
				os.execute("espeak 'reset'")
				rospub.mapcmd(99)
				t_last_button=t
			end
		end

	end
end


local function sendcmd()
  local t=gettime()
  local dt=t-t_last_sent

  local acc={Config.max_acc[1],Config.max_acc[2],8.0}
  local maxvel=Config.max_vel or {0.50,0.50,2.0}

  curvel[1]=util.procFunc(targetvel[1]-curvel[1],0,dt*acc[1])+curvel[1]
  curvel[2]=util.procFunc(targetvel[2]-curvel[2],0,dt*acc[2])+curvel[2]
  curvel[3]=util.procFunc(targetvel[3]-curvel[3],0,dt*acc[3])+curvel[3]

  curvel[1]=util.procFunc(curvel[1],0,maxvel[1])
  curvel[2]=util.procFunc(curvel[2],0,maxvel[2])
  curvel[3]=util.procFunc(curvel[3],0,maxvel[3])
  local d1=math.sqrt(curvel[1]*curvel[1]+curvel[2]*curvel[2])
  local d2=math.sqrt(targetvel[1]*targetvel[1]+targetvel[2]*targetvel[2])

  if not (targetvel[1]==0 and targetvel[2]==0 and targetvel[3]==0) then last_control_t = t end
--print(unpack(curvel))

  if t-last_control_t<1.5 then

--		if use_ros then
		if nil then
			if send_cmd_vel then
				print("Vel:",curvel[1],curvel[3])
				rospub.cmdvel(curvel[1],curvel[2],curvel[3])
			else
				rospub.baseteleop(curvel[1],curvel[2],curvel[3])
			end
		else
			hcm.set_base_teleop_t(t)
			hcm.set_base_teleop_velocity({curvel[1],curvel[2],curvel[3]})
		end
  end
  t_last_sent=t
end

while running do
  local ret = xbox360.read()
  update(ret)
  sendcmd()
  unix.usleep(1e6*0.05) --20fps
  local gccount = gcm.get_processes_xb360()
  -- gcm.set_processes_xb360({2,gccount[2]+1,gccount[3]})
  if gcm.get_game_selfdestruct()==1 then
  	os.execute("mpg321 ~/Desktop/ARAICodes/Media/selfdestruct.mp3")
  	os.execute('sync')
  	os.execute('systemctl poweroff -i')
  end
end
