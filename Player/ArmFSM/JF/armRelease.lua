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
  {25,15,25,-25,
   25,-15,-25,-25
  }
)*DEG_TO_RAD

local armmotion=require'armmotion2'
local t_start=0
local motion_index=1

function state.entry()
  print(state._NAME..' Entry' )
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry
  q0=Body.get_arm_position()
  arm0=Body.get_arm_position()
  hcm.set_arm_state(1)--initializing

  t_start=Body.get_time()
  motion_index=1
  hcm.set_head_target({4,0,0})

end

function state.update()
  local t  = Body.get_time()
  local dt = t - t_update
  t_passed=t-t_start


  if motion_index>#armmotion-100 then
    hcm.set_head_target({2,0,0})
  end


  if motion_index>#armmotion then
    print("MOTION END END END")
    hcm.set_head_target({0,0,0})

    return "done"
  else
    local cur_frame=armmotion[motion_index]
    if t_passed>cur_frame[1] then motion_index=motion_index+1 end
    if motion_index<=#armmotion then
      armframe=armmotion[motion_index]
      Body.set_arm_command_position({
        armframe[2]*DEG_TO_RAD,armframe[3]*DEG_TO_RAD,armframe[4]*DEG_TO_RAD,armframe[5]*DEG_TO_RAD,
        armframe[6]*DEG_TO_RAD,armframe[7]*DEG_TO_RAD,armframe[8]*DEG_TO_RAD,armframe[9]*DEG_TO_RAD,
      })
    end
  end
end

function state.exit()
  hcm.set_arm_state(2) --initialized
  print(state._NAME..' Exit' )
end

return state
