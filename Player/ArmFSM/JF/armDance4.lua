local state = {}
state._NAME = ...
local Body  = require'Body'
local t_entry, t_update, t_exit
local timeout = 10.0
local q0
local max_angular_vel=vector.ones(8)*45*DEG_TO_RAD
local arm0
local t_next_motion
local motion_index=1

local dance_index=4
local motionname=string.format("motionlog%04d",Config.dancemotion[dance_index])
local motiondata=require(motionname)

function state.entry()
  print(state._NAME..' Entry' )
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry
  t_next_motion=t_entry
  motion_index=Config.startframe[dance_index] or 1
end

function state.update()
  local t  = Body.get_time()
  local dt = t - t_update
  t_update = t
  if t>t_next_motion then
    local dat=motiondata[motion_index]
    local armc={dat[4],dat[5],dat[6],dat[7],   dat[8],dat[9],dat[10],dat[11]}
    Body.set_arm_command_position(armc)
    motion_index=motion_index+1
    t_next_motion=t_next_motion+0.01
  end
  if motion_index>#motiondata then return "done" end
end

function state.exit()
  hcm.set_arm_state(2) --initialized
  print(state._NAME..' Exit' )
end

return state
