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
local marker_pose_list={}

local function load_markers()
  marker_pose_list={}
  local fp = io.open("/home/sj/markers.txt","r")
  local lineno=1
  if not fp then return end
  while true do
    local line=fp:read()
    if line==nil then break end
    local t = {}
    for n in line:gmatch("%S+") do   	table.insert(t, tonumber(n))         end
    local marker_index=#marker_pose_list+1
    marker_pose_list[marker_index]=t
    lineno=lineno+1
  end
  fp:close()
  marker_pose_list[#marker_pose_list+1]=util.shallow_copy(marker_pose_list[1]) --return to the origin

  for i=1,#marker_pose_list do
    local pose=marker_pose_list[i]
    print(string.format("%d: %.2f %.2f (%.1f)\n",i,pose[1],pose[2],pose[3]/DEG_TO_RAD))
  end
end

function state.entry()
  print(state._NAME..' Entry' )
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  load_markers()
  stage=0
end

function state.update()
  local t = Body.get_time()
  local t_diff = t - t_update
  t_update = t

  if wcm.get_task_index()>#marker_pose_list then
    print("FINISH!!!!")
    return "done"
  else
    if hcm.get_path_execute()==0 and stage==0 then
      local target=marker_pose_list[wcm.get_task_index()]
      print("Navigating to Pose #"..wcm.get_task_index())
      hcm.set_path_targetpose(target)
      hcm.set_path_execute(1)
      t_move=t
      stage=1
    end
    if hcm.get_path_execute()==0 and stage==1 and t>t_move+t_wait then
      print("Navigation complete!!!")
      wcm.set_task_index(wcm.get_task_index()+1)
      return "arrived"
    end
  end
end

function state.exit()  
  print(state._NAME..' Exit' )
end

return state
