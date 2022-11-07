local state = {}
state._NAME = ...
require'mcm'
-- IDLE IS A RELAX STATE
-- NOTHING SHOULD BE TORQUED ON HERE!
local Body = require'Body'
local t_entry, t_update

function state.entry()
  print(state._NAME..' Entry' )
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
end

function state.update()
  local t = Body.get_time()
  local t_diff = t - t_update
  t_update = t
  local current_task=Config.task_list[wcm.get_task_index()]
  local ar_id=current_task[3]

  if ar_id==0 then return "complete" end

  local arposex=wcm.get_landmark_posx()
  local arposey=wcm.get_landmark_posy()
  local arposea=wcm.get_landmark_posa()
  local arposet=wcm.get_landmark_t()

  local arpose={arposex[ar_id],arposey[ar_id],arposea[ar_id]}
  local arpose_t=arposet[ar_id]

  if t-arpose_t<0.1 then
    local pose=wcm.get_robot_pose()
    local approach_diff
    if Config.approach_yaw then
      local abs_pose=util.pose_global(arpose, pose)
      approach_target=util.pose_global({-Config.approach_dist,0,0},abs_pose)
      approach_diff=util.pose_relative(approach_target, pose)
    else
      approach_diff={arpose[1]-Config.approach_dist,arpose[2],0}
    end

    local cmd_vel={0,0,0}
    --hack


    local approach_cost=math.sqrt(approach_diff[1]*approach_diff[1] + approach_diff[2]*approach_diff[2]*Config.y_cost_factor)
    if approach_cost<Config.approach_threshold then
      print("Approaching done")
      return "done"
    end
    local approach_dist=math.sqrt(approach_diff[1]*approach_diff[1] + approach_diff[2]*approach_diff[2])

    local approach_vel= math.min(0.01+approach_dist, 0.40)
    cmd_vel[1]=approach_diff[1]/approach_dist*approach_vel
    cmd_vel[2]=approach_diff[2]/approach_dist*approach_vel
    cmd_vel[3]=util.procFunc(approach_diff[3], 0, 0.10)
    hcm.set_base_velocity(cmd_vel)
    hcm.set_base_velocity_t(unix.time())

  else
    hcm.set_base_velocity({0,0,0})
    hcm.set_base_velocity_t(unix.time())
  end
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
