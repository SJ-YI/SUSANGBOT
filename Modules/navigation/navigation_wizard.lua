#!/usr/bin/env luajit
local pwd = os.getenv'PWD'
dofile'../fiddle.lua'

require'wcm'
require'hcm'

local unix=require'unix'
local vector=require'vector'
local rossub=require'rossub'
local rospub=require'tb3_rospub'
local navigation=require'navigation'
local util=require'util'
local math=require'math'

local sub_idx_smap,sub_idx_goal
local t_path_last_update
local path_update_interval = 2.0

local function entry()
  navigation.init()
  rossub.init('navigation')
  rospub.init('navigation')
  sub_idx_smap = rossub.subscribeOccupancyGrid('/map')
  sub_idx_goal = rossub.subscribePoseStamped('/move_base_simple/goal')
  t_path_last_update=unix.time()
end

local function rosio()
  res,wid,hei,posx,posy,posz,dat=rossub.checkOccupancyGrid(sub_idx_smap)
  if res then
    print("SMAP LOADED!!!!")
    -- navigation.process_smap(res,wid,hei,posx,posy,posz,dat,3,1)
    navigation.process_smap(res,wid,hei,posx,posy,posz,dat,3,2)
    lres, lw, lh, lpx, lpy, lpz, data=navigation.get_lomap()
    rospub.occgrid(lres,lw,lh,lpx,lpy,lpz,data)
  end

  posetarget=rossub.checkPoseStamped(sub_idx_goal)
  if posetarget then
    -- print("posetarget:",unpack(posetarget))
    hcm.set_path_targetpose(posetarget)
    hcm.set_path_execute(1)

    local pose=wcm.get_robot_pose()
    print(string.format("Pathplan: (%.2f %.2f %.1f)   to   (%.2f %.2f %.1f)",
    pose[1],pose[2],pose[3]/DEG_TO_RAD, posetarget[1],posetarget[2],posetarget[3]/DEG_TO_RAD
    ))

  end
end


local function update_pathplan()
  print("PATHPLAN!!!!!")
  t_path_last_update=t
  local posetarget=hcm.get_path_targetpose()
  local t0=unix.time()


  local pose=wcm.get_robot_pose()
  local rel=util.pose_relative(posetarget,pose)
  local d_target=math.sqrt(rel[1]*rel[1]+rel[2]*rel[2])
  if d_target<0.30 then
    print("TOO NEAR, DIRECT")
    xpaths={pose[1],posetarget[1]}
    ypaths={pose[2],posetarget[2]}
   
    rospub.path(xpaths,ypaths,vector.zeros(#xpaths))
    local x,y=hcm.get_path_x(),hcm.get_path_y()
    for i=1,#xpaths do x[i],y[i]=xpaths[i],ypaths[i] end
    hcm.set_path_num(#xpaths);hcm.set_path_x(x); hcm.set_path_y(y)
    local pose=wcm.get_robot_pose()
    for i=1,#xpaths do
      local pathpose={xpaths[i],ypaths[i],0}
      local relpathpose=util.pose_relative(pathpose,pose)
      print(string.format("#%d: %.2f,%.2f",i,relpathpose[1],relpathpose[2]))
    end
    hcm.set_path_index(2) --waypoint #1 is start pose,
    hcm.set_path_execute(2) --oneshot for test
    return
  end


  xpath,ypath,xpaths,ypaths=navigation.pathplan(wcm.get_robot_pose(), posetarget,5000)

  if not xpath then
print("PATHPLAN FAIL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
print("PATHPLAN FAIL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
print("PATHPLAN FAIL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
print("PATHPLAN FAIL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
print("PATHPLAN FAIL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
print("PATHPLAN FAIL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
print("PATHPLAN FAIL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
print("PATHPLAN FAIL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    local poseback=util.pose_global({-0.30,0,0},wcm.get_robot_pose() )
    xpath,ypath,xpaths,ypaths=navigation.pathplan(poseback, posetarget,5000)
  end


  if xpaths then
    print("MULTIPLE PATH")
    rospub.path(xpaths,ypaths,vector.zeros(#xpaths))
    local x,y=hcm.get_path_x(),hcm.get_path_y()
    for i=1,#xpaths do x[i],y[i]=xpaths[i],ypaths[i] end
    hcm.set_path_num(#xpaths);hcm.set_path_x(x); hcm.set_path_y(y)
    -- hcm.set_path_index(1)
    local pose=wcm.get_robot_pose()
    for i=1,#xpaths do
      local pathpose={xpaths[i],ypaths[i],0}
      local relpathpose=util.pose_relative(pathpose,pose)
      print(string.format("#%d: %.2f,%.2f",i,relpathpose[1],relpathpose[2]))
    end
    hcm.set_path_index(2) --waypoint #1 is start pose,
    hcm.set_path_execute(2) --oneshot for test
  else
    print("DIRECT PATH")
    local pose=wcm.get_robot_pose()
    local posetarget=hcm.get_path_targetpose()
    local relposetarget=util.pose_relative(posetarget,pose)
    local dreltargetpose=math.sqrt(relposetarget[1]*relposetarget[1]+relposetarget[2]*relposetarget[2])

    print("distance:",dreltargetpose)
    if dreltargetpose<0.30 then
      local x,y=hcm.get_path_x(),hcm.get_path_x()
      x[1],y[1]=pose[1],pose[2]
      x[2],y[2]=posetarget[1],posetarget[2]
      hcm.set_path_num(2);hcm.set_path_x(x);hcm.set_path_y(y);hcm.set_path_index(2) --waypoint #1 is start pose,
      hcm.set_path_execute(2) --oneshot for test
      rospub.path({},{},{})
    else
      print("NO PATH!!!!")
      hcm.set_path_num(0)
      hcm.set_path_execute(0)
      rospub.path({},{},{})
    end
  end
  local t1=unix.time()
  print("pathplan time: "..((t1-t0)*1000).."ms")
end


local function update()
  rosio()
  local t=unix.time()
  local path_execute=hcm.get_path_execute()
  if path_execute==1 then update_pathplan() end
end

if ... and type(...)=='string' then --webots handling
  WAS_REQUIRED = true
  return {entry=entry, update=update, exit=nil}
end

local running = true
local key_code
entry()
while running do
  update()
  unix.usleep(1E6*0.01)
end
