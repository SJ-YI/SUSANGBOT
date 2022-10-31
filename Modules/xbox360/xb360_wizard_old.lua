#!/usr/bin/env luajit
-- (c) 2014 Team THORwIn
local ok = pcall(dofile,'../fiddle.lua')
if not ok then dofile'fiddle.lua' end
local util = require'util'
local xbox360 = require 'xbox360'
local signal = require'signal'.signal
local gettime = require'unix'.time
local rospub = require 'rospub'
require'gcm'
require'hcm'

--local product_id = 0x0291
--local product_id = 0x028e
local product_id = 0x0719
--local product_id = 0x02a9

xbox360.open(product_id)
gcm.set_processes_xb360({1,0,0})
local si = require'simple_ipc'

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
local last_control_t = gettime()

local gtarget=0
rospub.init('xb360_pub')

local function update(ret)
  local t = gettime()
  local dt = t-t_last
  t_last=t

--BACK START LB RB XBOX X Y A B
  local x_db,x_max = 10,255
  local y_db,y_max = 8000,32768
	local y_db_rot = 14000

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
	targetvel={(rt-lt) * 0.50,ray*0.50,lay*1.0}

  head_pitch=head_pitch + lax * 20 * dt
  if head_pitch>50 then head_pitch=50 end
  if head_pitch<-30 then head_pitch=-30 end
  if math.abs(lax)>0 then
    hcm.set_head_target({0,head_pitch,0.3})
    if wcm.get_robot_webots()==0 then
      hcm.set_head_execute(-1)
    else
      hcm.set_head_execute(1)
    end
  end

  if t-t_last_button>0.3 then
    if ret.buttons[1]>0 then --BACK button: start/stop mapping process
			if slam_started==false then
	      rospub.voice("mapping started")
	      print("MAPPING START!!!!!!")
				rospub.hsrbcommand(1) --send topic to robot's datasend.py
				slam_started=true
			else
				rospub.voice("mapping end")
				print("MAPPING END!!!!!!")
				slam_ended=true
				slam_started=false
				rospub.hsrbcommand(2)  --send topic to robot's datasend.py
			end
      t_last_button=t + 1.0 --don't allow re-press for 1 second
    end

    if ret.buttons[3]>0 then --Left shoulder button: add current position to the pose list
      rospub.voice("Pose")
      rospub.hsrbcommand(3) --pose log command: 3
      print("Pose recorded")
      t_last_button=t+0.7 --1 sec delay
    end

    if ret.buttons[4]>0 then --Right shoulder button: Image grab
      rospub.hsrbcommand(4) --image log command: 3
      t_last_button=t+0.2 --0.5 sec delay
    end

		if ret.buttons[6]>0 then  --X button, toggle gripper
			gtarget=1-gtarget
			if gtarget==1 then rospub.gripperforce(-0.05) --gripper close for actual robot
			elseif gtarget==0 then rospub.grippercontrol(1.5,0.1,1.5) --gripper full open
			end
			t_last_button=t+0.5 --0.5 sec delay
		end

		if ret.buttons[7]>0 then  --Y button
			wcm.set_robot_proceed(1)
			t_last_button=t+0.5 --0.5 sec delay
		end

		if ret.buttons[8]>0 then  --A button, reset arm
			print("arm reset")
			rospub.armcontrol({0.01, 0*DEG_TO_RAD, -90*DEG_TO_RAD,-90*DEG_TO_RAD,0*DEG_TO_RAD},2)
			t_last_button=t+0.5 --0.5 sec delay
		end

		if ret.buttons[9]>0 then  --B button, arm forward
			print("arm forward")
			rospub.armcontrol({0.15, -15*DEG_TO_RAD, 0*DEG_TO_RAD,-75*DEG_TO_RAD,0*DEG_TO_RAD},2)
			t_last_button=t+0.5 --0.5 sec delay
		end


    if ret.buttons[1]==1 and ret.buttons[2]==1 then
      gcm.set_game_selfdestruct(1)
      running=false
    end
  end

  if t-t_last_debug>t_debug_interval then
    print(
      string.format(
      "Dpad:%d %d LStick:%d %d RStick:%d %d LT%d RT%d",
      ret.dpad[1],ret.dpad[2],
      ret.lstick[1],ret.lstick[2],
      ret.rstick[1],ret.rstick[2],
      ret.trigger[1],ret.trigger[2])
      ..
      string.format("Buttons:%d%d%d%d%d%d%d%d%d",
        unpack(ret.buttons))
    )
    t_last_debug=t
  end
end


local function sendcmd()
  local t=gettime()
  local dt=t-t_last_sent
  local vel=hcm.get_base_velocity()

  local acc={0.40,0.40,1.60}
  vel[1]=util.procFunc(targetvel[1]-vel[1],0,dt*acc[1])+vel[1]
  vel[2]=util.procFunc(targetvel[2]-vel[2],0,dt*acc[2])+vel[2]
  vel[3]=util.procFunc(targetvel[3]-vel[3],0,dt*acc[3])+vel[3]
  vel[1]=util.procFunc(vel[1],0,0.8)
  vel[2]=util.procFunc(vel[2],0,0.8)
  vel[3]=util.procFunc(vel[3],0,1.00)
  local d1=math.sqrt(vel[1]*vel[1]+vel[2]*vel[2])
  local d2=math.sqrt(targetvel[1]*targetvel[1]+targetvel[2]*targetvel[2])

  if not (targetvel[1]==0 and targetvel[2]==0 and targetvel[3]==0) then
    last_control_t = t
    -- print"UPD"
  end

  if t-last_control_t<1.5 then
    hcm.set_base_teleop_t(t)
    hcm.set_base_velocity({vel[1],vel[2],vel[3]})
    rospub.baseteleop(vel[1],vel[2],vel[3])
  end



  if hcm.get_head_execute()==-1 then
    local headTarget=hcm.get_head_target()
    rospub.headcontrol(headTarget[1]*math.pi/180,-headTarget[2]*math.pi/180,headTarget[3])
    -- hcm.set_head_execute(0)
  end
  t_last_sent=t
end


while running do
  local ret = xbox360.read()
  update(ret)
  sendcmd()
  unix.usleep(1e6*0.05) --20fps
  -- local gccount = gcm.get_processes_xb360()
  -- gcm.set_processes_xb360({2,gccount[2]+1,gccount[3]})
  -- if gcm.get_game_selfdestruct()==1 then
  -- 	os.execute("mpg321 ~/Desktop/ARAICodes/Media/selfdestruct.mp3")
  -- 	os.execute('sync')
  -- 	os.execute('systemctl poweroff -i')
  -- end
end
