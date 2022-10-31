#!/usr/bin/env luajit
local pwd = os.getenv'PWD'
local ok = pcall(dofile,'../fiddle.lua')
	print(ok)
	if not ok then
		ok = pcall(dofile,'./fiddle.lua')
		print(ok)
end

require'wcm'
require'hcm'

local unix=require'unix'
local vector=require'vector'
local rossub=require'rossub'
local rospub=require'rospub2'
local navigation=require'navigation'
local util=require'util'
local math=require'math'
local yolo2=require'yolo2'

local sub_idx_smap,sub_idx_goal, sub_idx_D, sub_idx_D_info
local t_path_last_update
local path_update_interval = 2.0
local lomap_org
local should_load_map=true

local p_camera,depth_camera_info,depthdata=nil,nil,nil
local w_d,h_d,enc_d,depthdata,tstamp
local d_seq=1
local t_depth

local function entry()
  navigation.init()
  rossub.init('navigation')
  -- rospub.init('navigation')
	--nodename image_topics lidar_topics path_topics occgrid_topics
	rospub.init_custom('navigation',{},{},{"/path"},{"/lowres_map"})
  -- sub_idx_smap = rossub.subscribeOccupancyGrid('/map')
	sub_idx_smap = rossub.subscribeOccupancyGrid(Config.map_topic or '/static_obstacle_map_ref')
  sub_idx_goal = rossub.subscribePoseStamped('/pnunavigation/goal') --changed!!!

	sub_idx_D=rossub.subscribeImage('/pnu/depth_rect_raw') --from datarecv
	sub_idx_D_info=rossub.subscribeCameraInfo('/hsrb/head_rgbd_sensor/depth_registered/camera_info')
  t_path_last_update=unix.time()
end

local function rosio()
	if should_load_map then
		res,wid,hei,posx,posy,posz,dat=rossub.checkOccupancyGrid(sub_idx_smap)
		if res then
			print(string.format("smap loaded, resolution:%.3f wid:%d hei:%d posx:%.4f posz:%.4f",
				res,wid,hei,posx,posy,posz))
			navigation.process_smap(res,wid,hei,posx,posy,posz,dat,3,2)
			lres, lw, lh, lpx, lpy, lpz, lomap_org=navigation.get_lomap()
			rospub.occgrid(lres,lw,lh,lpx,lpy,lpz,lomap_org)
			should_load_map=false
		end
	end


	local temp_info=rossub.checkCameraInfo(sub_idx_D_info)
	while temp_info do depth_camera_info=temp_info;temp_info=rossub.checkCameraInfo(sub_idx_D_info) end

  local w,h,enc,data,ts=rossub.checkImage(sub_idx_D) --get the latest depth image
	while w do w_d,h_d,enc_d,depthdata,tstamp=w,h,enc,data,ts; w,h,enc,data,ts=rossub.checkImage(sub_idx_D) end --get the latest depth
	local camera_pose=rossub.checkTF("base_footprint","head_rgbd_sensor_rgb_frame_corrected")
	if camera_pose then p_camera=camera_pose end

	local t=unix.time()
	if wcm.get_objects_update_map()==1 then
		if depth_camera_info and depthdata and p_camera then
			local pose=wcm.get_robot_pose()
			local t0=unix.time()
			local num,posx,posy=yolo2.getobstaclematrix(depth_camera_info, depthdata, p_camera,5, 0.2, 2.0) --div minz maxz
			navigation.update_rgbd(num,posx,posy,pose,2.0)
			lres, lw, lh, lpx, lpy, lpz, data=navigation.get_lomap()
			rospub.occgrid(lres,lw,lh,lpx,lpy,lpz,data)
			local t1=unix.time()
			print(string.format("Map update, %d points, %.1f ms!!!!",num,(t1-t0)*1000))
		end
		wcm.set_objects_update_map(0) --single use
	end

	if wcm.get_objects_update_map()==-1 then
		print("MAP CLEAR!!")
		navigation.init_blank_smap(0.05, 2048, 2048, -51.22, -51.22,0.0, 3,2)
		lres, lw, lh, lpx, lpy, lpz, data=navigation.get_lomap()
		rospub.occgrid(lres,lw,lh,lpx,lpy,lpz,data)
		wcm.set_objects_update_map(0) --single use
	end

  posetarget=rossub.checkPoseStamped(sub_idx_goal)
  if posetarget then
    -- print("posetarget:",unpack(posetarget))
    hcm.set_path_targetpose(posetarget)
    hcm.set_path_execute(1)

    local pose=wcm.get_robot_pose()
		print("============================================")
    print(string.format("Pathplan:(%.2f %.2f %.1f) to (%.2f %.2f %.1f)",
    pose[1],pose[2],pose[3]/DEG_TO_RAD, posetarget[1],posetarget[2],posetarget[3]/DEG_TO_RAD
    ))
		print("============================================")

  end
end


local function update_pathplan(t)
  local t0=unix.time()
  print("PATHPLAN!!!!!")
  t_path_last_update=t
  local pose=wcm.get_robot_pose()
  local posetarget=hcm.get_path_targetpose()


  local rel=util.pose_relative(posetarget,pose)
  local d_target=math.sqrt(rel[1]*rel[1]+rel[2]*rel[2])
  if d_target<0.30 then
    print("TOO NEAR, DIRECT MOVE")
    xpaths,ypaths={pose[1],posetarget[1]},{pose[2],posetarget[2]}
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



local function try_pathplan()
	local pose=wcm.get_robot_pose()
	local posetarget=hcm.get_path_targetpose()
	hcm.set_path_pathfound(0)

	local targetangle=math.atan2(posetarget[2]-pose[2],posetarget[1]-pose[1])
	print("\n\n\nPATHPLAN TRYING!!!!!!"..unix.time())
	-- for i=1, angle_div do

	-- local dists={-0.60, -0.90, -1.20, -1.50, -1.80,-2.10, -2.40}
	local dists={-0.60, -0.90, -1.20, -1.50,-1.80,-2.10,2.40}
	local angles_div=16
	local angles={}
	for i=1,angles_div do angles[i]=360*DEG_TO_RAD/angles_div*i end




	local max_dist_ratio=2.5
	local max_abs_dist=5
	local ddx,ddy=pose[1]-posetarget[1],pose[2]-posetarget[2]
	local min_dist_max=math.min(max_abs_dist,max_dist_ratio * math.sqrt(ddx*ddx+ddy*ddy))


	local min_dist, min_dist_target=min_dist_max,{}
	for i=1,#dists do
		for j=1,#angles do
			print(string.format("Checking target: dist %.2f angle %.1f",dists[i],angles[j]/DEG_TO_RAD))
			local targetpose=util.pose_global(
				{dists[i],0,0}, {posetarget[1],posetarget[2],angles[j] })
			local ret=0
			if targetpose[1]>1.0 and targetpose[2]>-3.6 then  --wall 325
			  ret=navigation.check_empty_space(targetpose,0.45)
		  end
			if ret>0 then
				local t0=unix.time()
				xpath,ypath,xpaths,ypaths=navigation.pathplan(pose, targetpose,5000)
				if xpaths and #xpaths>=1 then
					local dist=0
					for k=1,#xpaths-1 do
						dx,dy=xpaths[k+1]-xpaths[k], ypaths[k+1]-ypaths[k]
						dist=dist+math.sqrt(dx*dx+dy*dy)
					end
					if dist<min_dist then min_dist,min_dist_target=dist,targetpose end
					print("Target found, distance:"..dist)
				else
					print("Target empty but no path found!")
				end
				local t1=unix.time()
				print("Pathplan check:".. ((t1-t0)*1000).." ms")
			end --end ret>0
		end --end angles
		if min_dist<min_dist_max then break end --don't go to larget dist if already found
	end

	if min_dist<min_dist_max then --path found
		print("TARGET FOUND, dist:",min_dist)
		hcm.set_path_targetpose(min_dist_target)
		hcm.set_path_execute(1) --start navigate
		hcm.set_path_pathfound(1)
	else
		hcm.set_path_execute(0)
	end
end








local function update()
  rosio()
  local t=unix.time()
  local path_execute=hcm.get_path_execute()
  if path_execute==1 then update_pathplan(t) end

	-- for person approaching
	-- we sample a number of target positions around and check if they are approachable
	if path_execute==20 then try_pathplan() end

	-- if wcm.get_objects_update_map()>0 then
	-- 	if wcm.get_objects_update_map()==1 then navigation.reset_lomap(lomap_org) end
	-- 	wcm.set_objects_update_map(0)
	-- 	print("UPDATING OBSTACLE MAP!!!")
	-- 	local obj_no=wcm.get_objects_num()
	-- 	local obj_x=wcm.get_objects_xpos()
	-- 	local obj_y=wcm.get_objects_ypos()
	-- 	for i=1,obj_no do navigation.add_obstacle(obj_x[i],obj_y[i]) end
	-- 	lres, lw, lh, lpx, lpy, lpz, data=navigation.get_lomap()
  --   rospub.occgrid(lres,lw,lh,lpx,lpy,lpz,data)
	-- end
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
