local state = {}
state._NAME = ...
require'mcm'
require'wcm'
local PI=3.14159265

local Body = require'Body'
local t_entry, t_update, t_debug,t_command
local cmd_vel={0,0,0}
local rossub = require'rossub'
local rospub = require 'tb3_rospub'
local sub_idx_cmdvel

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  t_debug=t_entry
  t_last=t_entry
  t_command=t_entry
  Body.set_wheel_torque_enable(3) --velocity mode
  poseLast=wcm.get_robot_pose()
  rossub.init('motioncontrol_sub')
  rospub.init('motioncontrol_pub')
  sub_idx_cmdvel=rossub.subscribeTwist('/cmd_vel')
  hcm.set_base_teleop_t(0)
  hcm.set_base_velocity({0,0,0})
  wcm.set_robot_pose_odom({0,0,0})
end


local last_wheel_pos=nil
local function update_wheel()
  local wheel_r, body_r=Config.wheels.wheel_r, Config.wheels.body_r
  local xcomp,ycomp,acomp=Config.wheels.xcomp,Config.wheels.ycomp,Config.wheels.acomp
  local velxcomp,velycomp,velacomp=Config.wheels.velxcomp,Config.wheels.velycomp,Config.wheels.velacomp

  local t= unix.time()
  local t_cmd=hcm.get_base_teleop_t()
  if t-t_cmd<0.5 then
    local teleop_vel=hcm.get_base_teleop_velocity()
    local v1,v2,v3=0,0,0 --right back left
    local wheel_vel = xcomp*teleop_vel[1] + ycomp*teleop_vel[2] + acomp*teleop_vel[3] --rad per sec
    local v1mag,v2mag,v3mag=math.abs(wheel_vel[1]),math.abs(wheel_vel[2]),math.abs(wheel_vel[3])
    local vmagmax=math.max(math.max(v1mag,v2mag),v3mag)
    if vmagmax>2*PI then
      local adj_factor = (vmagmax/(2*PI))
      wheel_vel[1],wheel_vel[2],wheel_vel[3]=	wheel_vel[1]/adj_factor,wheel_vel[2]/adj_factor,wheel_vel[3]/adj_factor
    end
    Body.set_wheel_command_velocity(wheel_vel)
  else
    hcm.set_base_teleop_velocity({0,0,0})
    Body.set_wheel_command_velocity({0,0,0})
  end

  local wheel_pos=Body.get_wheel_position()
  if not last_wheel_pos then last_wheel_pos=wheel_pos end
  local d1,d2,d3=
    util.mod_angle(wheel_pos[1]-last_wheel_pos[1])*wheel_r,
    util.mod_angle(wheel_pos[2]-last_wheel_pos[2])*wheel_r,
    util.mod_angle(wheel_pos[3]-last_wheel_pos[3])*wheel_r
  last_wheel_pos=wheel_pos

  local odomvel=velxcomp*d1 + velycomp*d2 + velacomp*d3
  local curpos=wcm.get_robot_pose_odom()
  local newpos=util.pose_global(odomvel,curpos)
  wcm.set_robot_pose_odom(newpos)
  rospub.tf({newpos[1], newpos[2],0},{0,0,newpos[3]}, "odom","base_footprint")
--  print(string.format("Pose: %.2f %.2f %.1f",newpos[1],newpos[2],newpos[3]/DEG_TO_RAD ))
end


--Set actuator commands to resting position, as gotten from joint encoders.
function state.update()
  local t = Body.get_time()
  local dt = t - t_update -- Save this at the last update time
  t_update = t
  local ret = rossub.checkTwist(sub_idx_cmdvel)
  if ret and t-t_entry>1 then
    hcm.set_base_velocity({ret[1],ret[2],ret[6]})
    hcm.set_base_teleop_t(t)
  end
  update_wheel()
end

function state.exit()
  print(state._NAME..' Exit' )
  Body.set_wheel_command_velocity({0,0,0,0})
end

return state
