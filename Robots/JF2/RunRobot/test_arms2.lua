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



local buttons=hcm.get_xb360_buttons()

print("press any key for torque on")
io.flush()
io.read()

dcm.set_mitmotor_operation_mode({
    1,1,1, --motors, velocity mode
    2,2,2,2, --left arm, torque mode (master)
    3,3,3,3, --right arm, pd mode (slave)
    })


--dcm.set_mitmotor_max_torque(vector.ones(11)*80)
dcm.set_mitmotor_max_torque(vector.ones(11)*150)
dcm.set_mitmotor_pgain(vector.ones(Config.nJoint)*20)
dcm.set_mitmotor_dgain(vector.ones(Config.nJoint)*-1)
Body.set_arm_command_torque({0,0,0,0, 0,0,0,0})


local recording=false
local file=nil
local t_start=nil

local motion_index=1

while running do
  local curarm=Body.get_arm_position()
  curarm[5],curarm[6],curarm[7],curarm[8]=curarm[1],-curarm[2],-curarm[3],curarm[4]
  Body.set_arm_command_position(curarm)
  local buttons=hcm.get_xb360_buttons()

    if buttons[7]==1 then
     if not recording then 
       print("RECORDING "..motion_index.." START!!!!");recording=true 
       file=io.open("armmotion"..motion_index..".lua","w")
       file:write("return {\n")
       t_start=unix.time()
     end
    elseif buttons[8]==1 then
     if recording then 
	print("RECORDING STOP!!!!");recording=false;
        file:write("}\n")
        file:close();motion_index=motion_index+1; end
    end

  if recording then
    local t_elapsed=unix.time()-t_start
    file:write(string.format("{%.3f, %.1f, %.1f, %.1f, %.1f,    %.1f,%.1f,%.1f,%.1f},\n", 
	t_elapsed,
	curarm[1]/DEG_TO_RAD,curarm[2]/DEG_TO_RAD,curarm[3]/DEG_TO_RAD,curarm[4]/DEG_TO_RAD,
	curarm[5]/DEG_TO_RAD,curarm[6]/DEG_TO_RAD,curarm[7]/DEG_TO_RAD,curarm[8]/DEG_TO_RAD
    ))
  end
  unix.usleep(1E6/100)
end
