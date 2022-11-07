local state = {}
state._NAME = ...
require'mcm'
-- IDLE IS A RELAX STATE
-- NOTHING SHOULD BE TORQUED ON HERE!
local Body = require'Body'
local t_entry, t_update, t_move
local t_wait= 0.5
local stage

function state.entry()
  print(state._NAME..' Entry' )
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  stage=0
  local current_task=Config.task_list[wcm.get_task_index()]
  if current_task[1]=="pickup" then
    arm_ch:send'pickup'
  else
    arm_ch:send'release'
  end
end

function state.update()
  local t = Body.get_time()
  local t_diff = t - t_update
  t_update = t
  if t-t_entry>t_wait and hcm.get_arm_state()==2 then
    wcm.set_task_index(wcm.get_task_index()+1)
    return "done"
  end
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
