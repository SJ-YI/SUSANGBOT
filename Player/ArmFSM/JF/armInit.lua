local state = {}
state._NAME = ...
local Body  = require'Body'
local t_entry, t_update, t_exit
local timeout = 10.0
local q0
local max_angular_vel=vector.ones(8)*45*DEG_TO_RAD
require'dcm'

local arm0
local armTarget=vector.new(
  {10,5,0,-25,
   10,-5,-0,-25
  }
)*DEG_TO_RAD

function state.entry()
  print(state._NAME..' Entry' )
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry
  q0=Body.get_arm_position()
  arm0=Body.get_arm_position()
  hcm.set_arm_state(1)--initializing
end

function state.update()
  local t  = Body.get_time()
  local dt = t - t_update
  t_update = t
  q,w_tol=util.approachTol(arm0,armTarget,max_angular_vel, dt)
  Body.set_arm_command_position(q)

  if w_tol then return "done" end
end

function state.exit()
  hcm.set_arm_state(2) --initialized
  print(state._NAME..' Exit' )
end

return state
