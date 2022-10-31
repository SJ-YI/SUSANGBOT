#!/usr/bin/env luajit
-- (c) 2014 Team THORwIn
local ok = pcall(dofile,'../fiddle.lua')
if not ok then dofile'fiddle.lua' end
local PI=3.14159265

local T=require'Transform'
local rospub=require'rospub'
rospub.init('armtest_pub')
local Body=require 'Body'

--SJ: dunno why, but they just got resetted with body reference
dcm.set_mitmotor_operation_mode({
    1,1,1, --motors, velocity mode
    3,3,3,3, --arms, pd control mode
    3,3,3,3,
    })
dcm.set_mitmotor_max_torque(vector.ones(11)*80)
dcm.set_mitmotor_pgain(vector.ones(Config.nJoint)*20)
dcm.set_mitmotor_dgain(vector.ones(Config.nJoint)*-3)

dcm.set_mitmotor_max_torque(vector.ones(11)*120)
dcm.set_mitmotor_pgain(vector.ones(Config.nJoint)*30)
dcm.set_mitmotor_dgain(vector.ones(Config.nJoint)*-1)



-- local file=io.open("dance.csv","r")
local lines={}
local line_count=0
local header_lines=7



-- local filename="/home/sj/Downloads/dance.csv"
-- local read_lines=3000
-- local joint_map={{7,8}, {9,10}, {11,12},  {5,6},{3,4},{1,2} }  --Lshoulder LElbow LWrist   RShoulder RElbow RWrist

local filename="/home/sj/Downloads/dance2.csv"
local read_lines=12000
local joint_map={  {5,6},{3,4},{1,2}, {7,8}, {9,10}, {11,12}  }  --Lshoulder LElbow LWrist   RShoulder RElbow RWrist



local slowdownfactor=1
-- local slowdownfactor=1

-- local header_lines=1000
-- local read_lines=1300

local xpos,ypos,zpos={},{},{}
local data_num=0;

--DATA FORMAT

--LWRIST1 LWRIST2   LELBOW1 LELVOW2   LSHOLDER1 LSHOULDER2   RSHOULDER1 RSHOULDER2   RELBOW1 RELBOW2 RWRIST1 RWRIST2

local scalefactor=0.001 --in mm
for line in io.lines(filename) do
  line_count=line_count+1
  if line_count>header_lines then
    print(line_count)
    local count,pos=1,1
    line=line.."," --add a comma to the end
    data_num=data_num+1
    while true do
      local s_index=line:find(',',pos,true)
      if not s_index then break; end
      local item=line:sub(pos,s_index-1)
      if count>2 then
        local val=nil
        local xindex=count-2
        if #item>1 then val=tonumber(item)*scalefactor
          -- if #item>0 then val=tonumber(item)*scalefactor
        else -- print("Line:"..line_count.." /"..count..">>   : NIL")
        end
        if xindex%3==1 then xpos[12*(data_num-1) + (xindex-1)/3 +1]= val
        elseif xindex%3==2 then ypos[12*(data_num-1) + (xindex-2)/3 +1]= val
        else zpos[12*(data_num-1) + (xindex-3)/3 +1]= val
        end
      end
      pos=s_index+1
      count=count+1
    end
    -- print("Line "..line_count)
    -- print(line)
  end
  if line_count>=read_lines then break end
end

print(data_num)
print(#xpos, #ypos, #zpos)


local function check_xyzpos(index, jmap)
  local i1, i2=jmap[1],jmap[2]
  local x1,x2=xpos[index+i1],xpos[index+i2]
  local y1,y2=ypos[index+i1],ypos[index+i2]
  local z1,z2=zpos[index+i1],zpos[index+i2]
  if not x1 then --first xyz is NAN
    if not x2 then
      print("Index "..index.."BOTH NAN!!!!")
      return {0,0,0}
    else
      return {x2,y2,z2}
    end
  elseif not x2 then
    return {x1,y1,z1}
  else
    return{0.5*(x1+x2), 0.5*(y1+y2), 0.5*(z1+z2)}
  end
end


local pose0=nil
local odom={0,0,0}



Body.set_torque_enable(Config.servo.torque_enable)
Body.set_arm_torque_enable({1,1,1,1, 1,1,1,1})
dcm.set_actuator_torque_enable_changed(1)


  local i,i_max=1,data_num
  while i<data_num do
    print(i.."/"..data_num)
    local index=(i-1)*12

    local xyz_lshoulder=check_xyzpos(index,joint_map[1])
    local xyz_lelbow=check_xyzpos(index,joint_map[2])
    local xyz_lwrist=check_xyzpos(index,joint_map[3])
    local xyz_rshoulder=check_xyzpos(index,joint_map[4])
    local xyz_relbow=check_xyzpos(index,joint_map[5])
    local xyz_rwrist=check_xyzpos(index,joint_map[6])

    local shoulderxy={xyz_rshoulder[1]-xyz_lshoulder[1], xyz_rshoulder[2]-xyz_lshoulder[2], xyz_rshoulder[3]-xyz_lshoulder[3]}
    local shouldercenter={0.5* (xyz_rshoulder[1]+xyz_lshoulder[1]), 0.5*(xyz_rshoulder[2]+xyz_lshoulder[2]), 0.5*(xyz_rshoulder[3]+xyz_lshoulder[3])}
    local yaw=math.atan2(shoulderxy[1],shoulderxy[2]) + 180*DEG_TO_RAD


    if not pose0 then pose0={shouldercenter[1],shouldercenter[2],yaw} end
    local pose={shouldercenter[1],shouldercenter[2],yaw}
    local rel_pose=util.pose_relative(pose,pose0)


    -- local wheel_r, body_r=0.152,0.213
    -- local xcomp=vector.new({1/1.732,-1/1.732,0})/wheel_r
    -- local ycomp=vector.new({-0.5,-0.5,1})/wheel_r
    -- local acomp=vector.new({-body_r, -body_r, -body_r})/wheel_r
    -- local rel_target_pose=util.pose_relative(rel_pose,odom)
    -- local dt=0.008*slowdownfactor
    -- local velocity_factor=1.5
    -- local velx, vely, vela=rel_target_pose[1]/dt,rel_target_pose[2]/dt, util.mod_angle(rel_target_pose[3])/dt
    -- Body.set_wheel_command_velocity( (xcomp*velx+ ycomp*vely+ acomp*vela) * velocity_factor )
    -- odom=util.pose_global({velx*dt,vely*dt,vela*dt},odom)


      -- Body.set_wheel_command_velocity( {0,0,0} )

    local shoulderroll=math.atan2(shoulderxy[3],math.sqrt(shoulderxy[1]*shoulderxy[1]+shoulderxy[2]*shoulderxy[2]))

    local tr_lwrist=  T.trans(xyz_lwrist[1],xyz_lwrist[2],xyz_lwrist[3])*T.trans(-shouldercenter[1],-shouldercenter[2],-shouldercenter[3])
    local tr_lelbow= T.trans(xyz_lelbow[1],xyz_lelbow[2],xyz_lelbow[3])*T.trans(-shouldercenter[1],-shouldercenter[2],-shouldercenter[3])
    local tr_lshoulder=T.trans(xyz_lshoulder[1],xyz_lshoulder[2],xyz_lshoulder[3])*T.trans(-shouldercenter[1],-shouldercenter[2],-shouldercenter[3])
    local tr_rwrist= T.trans(xyz_rwrist[1],xyz_rwrist[2],xyz_rwrist[3])*T.trans(-shouldercenter[1],-shouldercenter[2],-shouldercenter[3])
    local tr_relbow= T.trans(xyz_relbow[1],xyz_relbow[2],xyz_relbow[3])*T.trans(-shouldercenter[1],-shouldercenter[2],-shouldercenter[3])
    local tr_rshoulder=T.trans(xyz_rshoulder[1],xyz_rshoulder[2],xyz_rshoulder[3])*T.trans(-shouldercenter[1],-shouldercenter[2],-shouldercenter[3])

    local tr_shoulderfix=T.rotX(shoulderroll)*T.rotZ(yaw)

    xyz_lwrist=T.position(tr_shoulderfix*tr_lwrist)
    xyz_lelbow=T.position(tr_shoulderfix*tr_lelbow)
    xyz_lshoulder=T.position(tr_shoulderfix*tr_lshoulder)
    xyz_rwrist=T.position(tr_shoulderfix*tr_rwrist)
    xyz_relbow=T.position(tr_shoulderfix*tr_relbow)
    xyz_rshoulder=T.position(tr_shoulderfix*tr_rshoulder)



    --hack to fix the shoulder hyperextension
    local elbow_x_limit = 0.08
    if xyz_lelbow[1]-xyz_lshoulder[1]<elbow_x_limit then
      local x_offset=elbow_x_limit-(xyz_lelbow[1]-xyz_lshoulder[1])
      xyz_lelbow[1]=xyz_lelbow[1]+x_offset
      xyz_lwrist[1]=xyz_lwrist[1]+x_offset
    end
    if xyz_relbow[1]-xyz_rshoulder[1]<elbow_x_limit then
      local x_offset=elbow_x_limit-(xyz_relbow[1]-xyz_rshoulder[1])
      xyz_relbow[1]=xyz_relbow[1]+x_offset
      xyz_rwrist[1]=xyz_rwrist[1]+x_offset
    end









    local rel_l_elbow={xyz_lelbow[1]-xyz_lshoulder[1], xyz_lelbow[2]-xyz_lshoulder[2], xyz_lelbow[3]-xyz_lshoulder[3]}
    local rel_r_elbow={xyz_relbow[1]-xyz_rshoulder[1], xyz_relbow[2]-xyz_rshoulder[2], xyz_relbow[3]-xyz_rshoulder[3]}

    local rel_l_wrist={xyz_lwrist[1]-xyz_lelbow[1], xyz_lwrist[2]-xyz_lelbow[2],xyz_lwrist[3]-xyz_lelbow[3]}
    local rel_r_wrist={xyz_rwrist[1]-xyz_relbow[1], xyz_rwrist[2]-xyz_relbow[2],xyz_rwrist[3]-xyz_relbow[3]}

    local rel_l_shoulderwrist={xyz_lwrist[1]-xyz_lshoulder[1],xyz_lwrist[2]-xyz_lshoulder[2],xyz_lwrist[3]-xyz_lshoulder[3]}
    local rel_r_shoulderwrist={xyz_rwrist[1]-xyz_rshoulder[1],xyz_rwrist[2]-xyz_rshoulder[2],xyz_rwrist[3]-xyz_rshoulder[3]}



    local l_uarm_dist=math.sqrt(rel_l_elbow[1]*rel_l_elbow[1]+rel_l_elbow[2]*rel_l_elbow[2]+rel_l_elbow[3]*rel_l_elbow[3])
    local r_uarm_dist=math.sqrt(rel_r_elbow[1]*rel_r_elbow[1]+rel_r_elbow[2]*rel_r_elbow[2]+rel_r_elbow[3]*rel_r_elbow[3])

    local l_larm_dist=math.sqrt(rel_l_wrist[1]*rel_l_wrist[1]+rel_l_wrist[2]*rel_l_wrist[2]+rel_l_wrist[3]*rel_l_wrist[3])
    local r_larm_dist=math.sqrt(rel_r_wrist[1]*rel_r_wrist[1]+rel_r_wrist[2]*rel_r_wrist[2]+rel_r_wrist[3]*rel_r_wrist[3])

    local l_shoulderwrist_dist=math.sqrt(rel_l_shoulderwrist[1]*rel_l_shoulderwrist[1]+rel_l_shoulderwrist[2]*rel_l_shoulderwrist[2]+rel_l_shoulderwrist[3]*rel_l_shoulderwrist[3])
    local r_shoulderwrist_dist=math.sqrt(rel_r_shoulderwrist[1]*rel_r_shoulderwrist[1]+rel_r_shoulderwrist[2]*rel_r_shoulderwrist[2]+rel_r_shoulderwrist[3]*rel_r_shoulderwrist[3])


    local LShoulderPitch=math.atan2(-rel_l_elbow[1],-rel_l_elbow[3])
    local RShoulderPitch=math.atan2(-rel_r_elbow[1],-rel_r_elbow[3])

    local LShoulderRoll=math.asin(rel_l_elbow[2]/l_uarm_dist)
    local RShoulderRoll=math.asin(rel_r_elbow[2]/r_uarm_dist)


    local LElbowPitch=math.acos(  (l_uarm_dist*l_uarm_dist + l_larm_dist*l_larm_dist- l_shoulderwrist_dist*l_shoulderwrist_dist)/(2*l_uarm_dist*l_larm_dist) ) - 180*DEG_TO_RAD
    local RElbowPitch=math.acos(  (r_uarm_dist*r_uarm_dist + r_larm_dist*r_larm_dist- r_shoulderwrist_dist*r_shoulderwrist_dist)/(2*r_uarm_dist*r_larm_dist) )- 180*DEG_TO_RAD


    local armc=Body.get_arm_command_position()

    -- LShoulderYaw=0
    -- RShoulderYaw=0
    if math.sin(-LElbowPitch)>0.1 then
       local l_shoulder_yaw_to_wrist=T.position(   T.rotX(-LShoulderRoll)*T.rotY(-LShoulderPitch)*T.trans(rel_l_shoulderwrist[1],rel_l_shoulderwrist[2],rel_l_shoulderwrist[3]) )
       local asf=-l_shoulder_yaw_to_wrist[2]/(math.sin(LElbowPitch)*l_larm_dist)
       -- print(string.format("LSY: %.2f %.2f %.2f  val:%.2f",l_shoulder_yaw_to_wrist[1],l_shoulder_yaw_to_wrist[2],l_shoulder_yaw_to_wrist[3],asf))
       if asf>1.0 then asf=1.0 elseif asf<-1.0 then asf=-1.0 end
       LShoulderYaw=math.asin( asf )
     else
       LShoulderYaw=armc[3] or 0
     end

    if math.sin(-RElbowPitch)>0.1 then
      local r_shoulder_yaw_to_wrist=T.position(   T.rotX(-RShoulderRoll)*T.rotY(-RShoulderPitch)*T.trans(rel_r_shoulderwrist[1],rel_r_shoulderwrist[2],rel_r_shoulderwrist[3]) )
      local asf=-r_shoulder_yaw_to_wrist[2]/(math.sin(RElbowPitch)*r_larm_dist)
      if asf>1.0 then asf=1.0 elseif asf<-1.0 then asf=-1.0 end
      RShoulderYaw=math.asin( asf )
    else
      RShoulderYaw=armc[7] or 0
    end

    local angle_min, angle_max=Config.mit_arms.angle_min,Config.mit_arms.angle_max
    armc[1]=math.max(angle_min[1], math.min(angle_max[1],LShoulderPitch))
    armc[2]=math.max(angle_min[2], math.min(angle_max[2], LShoulderRoll ))
    armc[3]=math.max(angle_min[3], math.min(angle_max[3], LShoulderYaw ))
    armc[4]=math.max(angle_min[4], math.min(angle_max[4], LElbowPitch ))
    armc[5]=math.max(angle_min[5], math.min(angle_max[5],RShoulderPitch))
    armc[6]=math.max(angle_min[6], math.min(angle_max[6],RShoulderRoll))
    armc[7]=math.max(angle_min[7], math.min(angle_max[7],RShoulderYaw))
    armc[8]=math.max(angle_min[8], math.min(angle_max[8],RElbowPitch))
    Body.set_arm_command_position(armc)

    rospub.tf({rel_pose[1],rel_pose[2],0.83},{0,0,rel_pose[3]}, "map","shoulder_center")

    -- rospub.tf({0,0,0.83},{0,0,0}, "map","shoulder_center")
    rospub.tf(xyz_lshoulder,{0,0,0}, "shoulder_center","lshoulder")
    rospub.tf(rel_l_elbow,{0,0,0}, "lshoulder","lelbow")
    rospub.tf(rel_l_wrist,{0,0,0}, "lelbow","lwrist")
    rospub.tf(xyz_rshoulder,{0,0,0}, "shoulder_center","rshoulder")
    rospub.tf(rel_r_elbow,{0,0,0}, "rshoulder","relbow")
    rospub.tf(rel_r_wrist,{0,0,0}, "relbow","rwrist")
    local pos_check_lwrist = T.position(
        T.rotY(LShoulderPitch)*T.rotX(LShoulderRoll)*T.rotZ(LShoulderYaw)
        *T.trans(0,0,-l_uarm_dist)*T.rotY(LElbowPitch)*T.trans(0,0,-l_larm_dist)
      )
    local pos_check_rwrist = T.position(
        T.rotY(RShoulderPitch)*T.rotX(RShoulderRoll)*T.rotZ(RShoulderYaw)
        *T.trans(0,0,-r_uarm_dist)*T.rotY(RElbowPitch)*T.trans(0,0,-r_larm_dist)
      )

    local rel_poscheck_lwrist={pos_check_lwrist[1]-rel_l_elbow[1],pos_check_lwrist[2]-rel_l_elbow[2],pos_check_lwrist[3]-rel_l_elbow[3]}
    local rel_poscheck_rwrist={pos_check_rwrist[1]-rel_r_elbow[1],pos_check_rwrist[2]-rel_r_elbow[2],pos_check_rwrist[3]-rel_r_elbow[3]}

    rospub.tf(rel_poscheck_lwrist,{0,0,0}, "lelbow","lwrist2")
    rospub.tf(rel_poscheck_rwrist,{0,0,0}, "relbow","rwrist2")

    -- rospub.tf(pos_check_lwrist,{0,0,0}, "lshoulder","lwrist3")
    -- rospub.tf(pos_check_rwrist,{0,0,0}, "rshoulder","rwrist3")


    unix.usleep(1E6*0.008*slowdownfactor)
    i=i+1

    -- if i>=740 then print("freeze");i=740 end --FREEZE
  end

  Body.set_arm_torque_enable({0,0,0,0, 0,0,0,0})
  dcm.set_actuator_torque_enable_changed(1)
