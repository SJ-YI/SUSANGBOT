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
local sub_idx_cmdvel, sub_idx_laserscan
local poseMoveStart
local pfield=vector.ones(8)*100


function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  t_debug=t_entry
  t_last=t_entry
  t_command=t_entry


  wcm.set_robot_pose_odom({0,0,0})

  poseLast=wcm.get_robot_pose()
  poseMoveStart=wcm.get_robot_pose()
  rossub.init('motioncontrol_sub')
  rospub.init('motioncontrol_pub')
  sub_idx_cmdvel=rossub.subscribeTwist('/cmd_vel')
  sub_idx_laserscan=rossub.subscribeLaserScan('/scan')
  hcm.set_base_teleop_t(0)
  hcm.set_base_velocity({0,0,0})


  -- Body.set_wheel_torque_enable(3) --velocity mode
  Body.set_torque_enable(Config.servo.torque_enable)
  Body.set_arm_torque_enable({1,1,1,1, 1,1,1,1})
  Body.set_wheel_torque_enable({1,1,1})
  dcm.set_actuator_torque_enable_changed(1)
end



local function move_robot(vel)

  local rfdist=pfield[4]
  local fdist=pfield[5]
  local lfdist=pfield[6]

  local pfield_hard_th=Config.pathplan.pfield_hard_th
  local pfield_soft_th=Config.pathplan.pfield_soft_th
  local pfield_soft_th2 = Config.pathplan.pfield_soft_th2
  local pfield_hard_th2 = Config.pathplan.pfield_hard_th2

  local vel_mag0=math.sqrt(vel[1]*vel[1] + vel[2]*vel[2])
  if fdist<pfield_hard_th then vel[1]=math.min(0,vel[1])
  elseif fdist<pfield_soft_th and vel[1]>0 then
    local max_allowed_velx = (fdist-pfield_hard_th)/(pfield_soft_th-pfield_hard_th)*Config.pathplan.max_vel_webots
    vel[1]=math.min(vel[1],max_allowed_velx)
  end

  if lfdist<pfield_soft_th2 and rfdist>pfield_soft_th2 then
    local repul_factor = (lfdist-pfield_hard_th2)/(pfield_soft_th2-pfield_hard_th2)
     local lf_approach_vel = (vel[1]+vel[2])/1.414 * repul_factor
    if lf_approach_vel>0 then
      vel[1],vel[2]=vel[1]-lf_approach_vel,vel[2]-lf_approach_vel
    end
  elseif rfdist<pfield_soft_th2 and lfdist>pfield_soft_th2 then
    local repul_factor = (rfdist-pfield_hard_th2)/(pfield_soft_th2-pfield_hard_th2)

    local rf_approach_vel = (vel[1]-vel[2])/1.414*repul_factor
    if rf_approach_vel>0 then
      vel[1],vel[2]=vel[1]-rf_approach_vel,vel[2]+rf_approach_vel
    end
  end

  local vel_mag2=math.sqrt(vel[1]*vel[1] + vel[2]*vel[2])




  local xcomp,ycomp,acomp=Config.wheels.xcomp,Config.wheels.ycomp,Config.wheels.acomp
  local v1,v2,v3=0,0,0 --right back left
  local wheel_vel = xcomp*vel[1] + ycomp*vel[2] + acomp*vel[3] --rad per sec
  local v1mag,v2mag,v3mag=math.abs(wheel_vel[1]),math.abs(wheel_vel[2]),math.abs(wheel_vel[3])
  local vmagmax=math.max(math.max(v1mag,v2mag),v3mag)
  if vmagmax>2*PI then
    local adj_factor = (vmagmax/(2*PI))
    wheel_vel[1],wheel_vel[2],wheel_vel[3]=	wheel_vel[1]/adj_factor,wheel_vel[2]/adj_factor,wheel_vel[3]/adj_factor
  end
  mcm.set_walk_vel(vel)
  Body.set_wheel_command_velocity(wheel_vel)
end






local t_last_debug=unix.time()
local t_debug_interval = 1

local function follow_path(t,dt)
  local pose=wcm.get_robot_pose()
  if not pose then return end
  local max_vel,max_avel=Config.pathplan.max_vel_webots,Config.pathplan.max_avel
  local targetpose=hcm.get_path_targetpose()
  if not targetpose then return end
  local pathnum,pathindex,pathx,pathy=hcm.get_path_num(),hcm.get_path_index(),hcm.get_path_x(),hcm.get_path_y()
  local dx,dy,dz
  local distleft=0
  if pathindex<pathnum then
    for i=pathindex, pathnum-1 do
      local curdist=math.sqrt((pathx[i]-pathx[i+1])*(pathx[i]-pathx[i+1])+(pathy[i]-pathy[i+1])*(pathy[i]-pathy[i+1]))
      distleft=distleft+curdist
    end
  end
  distleft=distleft+math.sqrt((pathx[pathindex]-pose[1])*(pathx[pathindex]-pose[1])+(pathy[pathindex]-pose[2])*(pathy[pathindex]-pose[2]))

  if pathindex>pathnum then
     print("PATHPLAN ERROR, INDEX>NUM")
      move_robot({0,0,0})
      hcm.set_path_execute(0)
      return true

  elseif pathindex==pathnum then --final waypoint
    local relpose,reloldpose=util.pose_relative(targetpose,pose),util.pose_relative(pose,poseMoveStart)
    local drelpose=math.sqrt(relpose[1]*relpose[1]+relpose[2]*relpose[2])
    local arelpose = util.mod_angle(relpose[3])
    if t-t_last_debug>t_debug_interval then
      print(string.format("Path :%d/%d dist:%.2f ang dist:%.1f",pathindex,pathnum,drelpose,arelpose))
      t_last_debug=t
    end

    local dreloldpose=math.sqrt(reloldpose[1]*reloldpose[1]+reloldpose[2]*reloldpose[2])
    if drelpose<Config.pathplan.targetpose_th[1] and
      math.abs(util.mod_angle(relpose[3]))<Config.pathplan.targetpose_th[2] then
      print(string.format("Target Reached, poserr %.3f angerr %.1f",drelpose,util.mod_angle(relpose[3])/DEG_TO_RAD))
      print(string.format("Target: %.3f %.3f (%.1f) Pose:%.3f %.3f (%.1f)",targetpose[1],targetpose[2],targetpose[3]/DEG_TO_RAD,pose[1],pose[2],pose[3]/DEG_TO_RAD))
      hcm.set_path_execute(0)
      move_robot({0,0,0})
      poseMoveStart=wcm.get_robot_pose()
      return true
    end

    local vellimitStart =math.min(1.0,math.max(Config.pathplan.navigate_minvf1,dreloldpose/Config.pathplan.navigate_r1 ))
    local vellimitEnd =math.min(1.0,math.max(Config.pathplan.navigate_minvffin,drelpose/Config.pathplan.navigate_rfin ))
    local max_vel_acclim=max_vel*math.min(vellimitStart, vellimitEnd)

    local da1=util.procFunc(util.mod_angle(relpose[3])/(20*DEG_TO_RAD),0, 1)*max_avel
    local da2=util.procFunc(util.mod_angle(math.atan2(relpose[2],relpose[1]))/(20*DEG_TO_RAD),0, 1)*max_avel
    local rotfactor=math.min(1,math.max(0,(drelpose-Config.pathplan.rotate_th2)/Config.pathplan.rotate_th1))

    da,dx,dy=rotfactor*da2 + (1-rotfactor)*da1,relpose[1]/drelpose*max_vel_acclim,relpose[2]/drelpose*max_vel_acclim



  else --Not final waypoint
    local curtarget={pathx[pathindex],pathy[pathindex],0}
    local relpose,reloldpose=util.pose_relative(curtarget,pose),util.pose_relative(pose,poseMoveStart)
    local drelpose=math.sqrt(relpose[1]*relpose[1]+relpose[2]*relpose[2])
    local dreloldpose=math.sqrt(reloldpose[1]*reloldpose[1]+reloldpose[2]*reloldpose[2])


    local debug_print=false
    if t-t_last_debug>t_debug_interval then
      print(string.format("Start: (%.2f %.2f %.1f) Pose:(%.2f %.2f %.1f) Target: (%.2f %.2f %.1f",
  	     poseMoveStart[1],	poseMoveStart[2],	poseMoveStart[3],pose[1],pose[2],pose[3],targetpose[1],targetpose[2],targetpose[3]))
      print(string.format("Path :%d/%d dist:%.2f",pathindex,pathnum,drelpose))
      t_last_debug=t
      debug_print=true
    end

    if drelpose<Config.pathplan.waypoint_th then
      print("Path: ",pathindex)
      pathindex=pathindex+1
      hcm.set_path_index(pathindex)
      poseMoveStart=pose; --reset start pose
      return true
    end

    local relposetarget=util.pose_relative(targetpose,pose)
    local drelposetarget=math.sqrt(relposetarget[1]*relposetarget[1]+relposetarget[2]*relposetarget[2])

    local dnx,dny=pathx[pathindex+1]-pathx[pathindex],pathy[pathindex+1]-pathy[pathindex]
    local nangle = math.atan2(dny,dnx)
    local da1=util.procFunc( util.mod_angle(nangle-pose[3]) / (20*DEG_TO_RAD)  , 0, 1)*max_avel
    local da2=util.procFunc( util.mod_angle(math.atan2(relpose[2],relpose[1])) / (20*DEG_TO_RAD)  , 0, 1)*max_avel
    local rotfactor=math.min(1,math.max(0,(drelpose-Config.pathplan.rotate_th2)/Config.pathplan.rotate_th1))
    da=rotfactor*da2 + (1-rotfactor)*da1

    if distleft<Config.pathplan.direct_angle_th then
      local relpose=util.pose_relative(targetpose,pose)
      local drelpose=distleft
      local arelpose = util.mod_angle(relpose[3])
      local da1=util.procFunc(util.mod_angle(relpose[3])/(20*DEG_TO_RAD),0, 1)*max_avel
      local da2=util.procFunc(util.mod_angle(math.atan2(relpose[2],relpose[1]))/(20*DEG_TO_RAD),0, 1)*max_avel
      local rotfactor=math.min(1,math.max(0,(drelpose-Config.pathplan.rotate_th2)/Config.pathplan.rotate_th1))
      da=rotfactor*da2 + (1-rotfactor)*da1
    end

    local vellimitStart =math.min(1.0,math.max(Config.pathplan.navigate_minvf1,dreloldpose/Config.pathplan.navigate_r1 ))
    local vellimitEnd =math.min(1.0,math.max(Config.pathplan.navigate_minvf2,drelpose/Config.pathplan.navigate_r2 ))
    local max_vel_acclim=max_vel*math.min(vellimitStart, vellimitEnd)
    local dirangle = math.atan2(relpose[2],relpose[1])



    local mvel=max_vel_acclim

    if debug_print then
--      print("Start dist:",dreloldpose)
--      print("End dist:",drelpose)
--      print("Max vel:",mvel)
    end


    dx,dy=relpose[1]/drelpose*mvel,relpose[2]/drelpose*mvel

    local dir_angle=math.abs(math.atan2(dy,dx))
    if dir_angle>45*DEG_TO_RAD then
      local vel_slow_factor=math.min(1,  (dir_angle-45*DEG_TO_RAD)/(90*DEG_TO_RAD-45*DEG_TO_RAD) ) --0 to 1
      dx=dx*(1-vel_slow_factor)
      dy=dy*(1-vel_slow_factor)

    end

    local old_vel=mcm.get_walk_vel()
    local ddx=util.procFunc(dx-old_vel[1], 0, dt*Config.pathplan.max_acceleration)
    local ddy=util.procFunc(dy-old_vel[2], 0, dt*Config.pathplan.max_acceleration)
    dx=old_vel[1]+ddx
    dy=old_vel[2]+ddy
  end
  -- print("Vel:",dx,dy,da/DEG_TO_RAD)
  move_robot({dx,dy,da})
  return true
end

function move_base(t,dt)
  local t_real=unix.time()
  local t_last=hcm.get_base_teleop_t()

  if(t_real-t_last<1.0) then
    move_robot(hcm.get_base_teleop_velocity())
    hcm.set_path_execute(0)
    poseMoveStart=wcm.get_robot_pose()
  else
    local t_vel=hcm.get_base_velocity_t()
    if(t_real-t_vel<0.2) then
      move_robot(hcm.get_base_velocity())
    else
      local path_execute=hcm.get_path_execute()
      if path_execute==0 then

        local cur_vel=mcm.get_walk_vel()
        local vel_mag0=math.sqrt(cur_vel[1]*cur_vel[1] + cur_vel[2]*cur_vel[2])

        vel_mag=math.max(0,vel_mag0 -dt*1.0)
        if vel_mag<0.05 then
          mcm.set_walk_vel({0,0,0})
          move_robot({0,0,0})
        else
          local new_vel={cur_vel[1]/vel_mag0*vel_mag,cur_vel[2]/vel_mag0*vel_mag,0}
          mcm.set_walk_vel(new_vel)
          move_robot(new_vel)
        end

        poseMoveStart=wcm.get_robot_pose()
        return
      elseif path_execute==2 then
        follow_path(t,dt)
      end
    end
  end
end

local last_wheel_pos=nil
local function update_odom()
  local wheel_r, body_r=Config.wheels.wheel_r, Config.wheels.body_r
  local velxcomp,velycomp,velacomp=Config.wheels.velxcomp,Config.wheels.velycomp,Config.wheels.velacomp
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

  local robot_pose=rossub.checkTF("map","base_footprint")
	if robot_pose then
		local pose={robot_pose[1],robot_pose[2],robot_pose[6]}
		wcm.set_robot_pose(pose)
	end
end

--Set actuator commands to resting position, as gotten from joint encoders.
function state.update()
  local t = Body.get_time()
  local dt = t - t_update -- Save this at the last update time
  t_update = t

  local ret= rossub.checkLaserScanPField(sub_idx_laserscan, 8, {0.10,0,0})
  if ret then pfield=ret end

  -- local ret = rossub.checkTwist(sub_idx_cmdvel)
  -- if ret and t-t_entry>1 then
  --   hcm.set_base_velocity({ret[1],ret[2],ret[6]})
  --   hcm.set_base_teleop_t(t)
  -- end

  move_base(t,dt)
  update_odom()
end

function state.exit()
  print(state._NAME..' Exit' )
  Body.set_wheel_command_velocity({0,0,0,0})
  Body.set_arm_torque_enable({0,0,0,0, 0,0,0,0})
  dcm.set_actuator_torque_enable_changed(1)
end

return state
