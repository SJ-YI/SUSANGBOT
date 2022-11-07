local state = {}
state._NAME = ...
require'mcm'
require'wcm'
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
end

function state.update()
  local t = Body.get_time()
  local t_diff = t - t_update
  t_update = t

  if wcm.get_task_index()>#Config.task_list then
    --all set. do nothing

  else
    local current_task=Config.task_list[wcm.get_task_index()]
    if hcm.get_path_execute()==0 and stage==0 then
      print("Navigating to "..Config.pose_targets[current_task[2]][1])
      hcm.set_path_targetpose(Config.pose_targets[current_task[2]][2])
      hcm.set_path_execute(1)
      t_move=t
      stage=1
    end
    if hcm.get_path_execute()==0 and stage==1 and t>t_move+t_wait then
      print("Navigation complete!!!")
      return "done"
    end
  end
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
