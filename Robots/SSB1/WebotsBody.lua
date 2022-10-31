--HSR interface (with main RGBD camera)



local WebotsBody = {}
local ww, cw, mw, kw, sw, fw, rw, vw, kb

local navigation_w, perception_w, rosio_w

local ffi = require'ffi'
require'wcm'
local util = require'util'
local T = require'Transform'

local si = require'simple_ipc'

local msg_kinect,msg_lidar
local get_time = webots.wb_robot_get_time
local nJoint = Config.nJoint
local jointNames = Config.jointNames
local servo = Config.servo

-- Added to Config rather than hard-coded
local ENABLE_POSE = Config.sensors.pose
local ENABLE_IMU = Config.sensors.imu

local ENABLE_CAMERA, NEXT_CAMERA = Config.sensors.front_camera, 0

local ENABLE_CHEST_LIDAR, NEXT_CHEST_LIDAR  = Config.sensors.chest_lidar, 0
local ENABLE_HEAD_LIDAR, NEXT_HEAD_LIDAR = Config.sensors.head_lidar, 0
local ENABLE_KINECT, NEXT_KINECT = Config.sensors.kinect, 0
local ENABLE_VELODYNE1, NEXT_VELODYNE1 = Config.sensors.velodyne1,0
local NEXT_VELODYNE_BROADCAST = 0
local broadcast_count=0
local set_pos, get_pos = webots.wb_motor_set_position, webots.wb_position_sensor_get_value

local PID_P = 32 * vector.ones(nJoint)

-- Acquire the timesteps
local timeStep = webots.wb_robot_get_basic_time_step()
local camera_timeStep = math.max(Config.camera_timestep or 33, timeStep)
local lidar_timeStep = math.max(Config.lidar_timestep or 25, timeStep)
local velodyne_timeStep = math.max(Config.velodyne_timestep or 50, timeStep) --max 20fps
local kinect_timeStep = math.max(Config.kinect_timestep or 30, timeStep)
local navigation_timeStep= math.max(Config.navigation_timestep or 500, timeStep)

local NEXT_NAVIGATION=get_time()


-- Setup the webots tags
local tags = {}
local t_last_error = -math.huge

tags.receiver = webots.wb_robot_get_device("receiver")
if tags.receiver>0 then
	webots.wb_receiver_enable(tags.receiver, timeStep)
	webots.wb_receiver_set_channel(tags.receiver, 13)
end

-- Ability to turn on/off items
local t_last_keypress = get_time()
--webots.wb_robot_keyboard_enable(100)
webots.wb_keyboard_enable(100) --8.5.0 change

local gps_pose0 = nil



local key_action = {
		h = function(override)
			if override~=nil then en=override else en=ENABLE_HEAD_LIDAR==false end
      if en==false and tags.head_lidar then
        print(util.color('HEAD_LIDAR disabled!','yellow'))
--        webots.wb_camera_disable(tags.head_lidar)
        webots.wb_lidar_disable(tags.head_lidar)
        ENABLE_HEAD_LIDAR = false
      elseif tags.head_lidar then
        print(util.color('HEAD_LIDAR enabled!','green'))
--        webots.wb_camera_enable(tags.head_lidar,lidar_timeStep)
        webots.wb_lidar_enable(tags.head_lidar,lidar_timeStep)
				NEXT_HEAD_LIDAR = get_time() + lidar_timeStep / 1000
        ENABLE_HEAD_LIDAR = true
      end
    end,
    l = function(override)
			if override~=nil then en=override else en=ENABLE_CHEST_LIDAR==false end
      if en==false and tags.chest_lidar then
        print(util.color('CHEST_LIDAR disabled!','yellow'))
--        webots.wb_camera_disable(tags.chest_lidar)
        webots.wb_lidar_disable(tags.chest_lidar)
        ENABLE_CHEST_LIDAR = false
      elseif tags.chest_lidar then
        print(util.color('CHEST_LIDAR enabled!','green'))
--        webots.wb_camera_enable(tags.chest_lidar,lidar_timeStep)
        webots.wb_lidar_enable(tags.chest_lidar,lidar_timeStep)
				NEXT_CHEST_LIDAR = get_time() + lidar_timeStep / 1000
        ENABLE_CHEST_LIDAR = true
      end
    end,
    y = function(override)
			if override~=nil then en=override else en=ENABLE_VELODYNE1==false end
      if en==false and tags.velodyne1 then
        print(util.color('VELODYNE #1 disabled!','yellow'))
        webots.wb_lidar_disable(tags.velodyne1,velodyne_timeStep)
        ENABLE_VELODYNE1 = false
      elseif tags.velodyne1 then
        print(util.color('VELODYNE #1 enabled!','green'))
        webots.wb_lidar_enable(tags.velodyne1,velodyne_timeStep)
				NEXT_VELODYNE1 = get_time() + lidar_timeStep / 1000
        ENABLE_VELODYNE1 = true
      end
    end,
    k = function(override)
      if override~=nil then en=override else en=ENABLE_KINECT==false end
      if en==false and tags.kinect then
        print(util.color('KINECT disabled!','yellow'))
        webots.wb_camera_disable(tags.kinectRGB)
        webots.wb_range_finder_disable(tags.kinectD)

        ENABLE_KINECT = false
      elseif tags.kinectRGB then
        print(util.color('KINECT enabled!','green'))
        webots.wb_camera_enable(tags.kinectRGB, kinect_timeStep)
        webots.wb_range_finder_enable(tags.kinectD,kinect_timeStep)
				NEXT_KINECT = get_time() + kinect_timeStep / 1000
        ENABLE_KINECT = true
      end
    end,
    c = function(override)
      if override~=nil then en=override else en=ENABLE_CAMERA==false end
      if en==false then
        print(util.color('CAMERA disabled!','yellow'))
        webots.wb_camera_disable(tags.front_camera)
				webots.wb_camera_disable(tags.back_camera)
        ENABLE_CAMERA = false
      else
        print(util.color('CAMERA enabled!','green'))
        webots.wb_camera_enable(tags.front_camera,camera_timeStep)
				-- webots.wb_camera_enable(tags.back_camera,camera_timeStep)
				NEXT_CAMERA = get_time() + camera_timeStep / 1000
        ENABLE_CAMERA = true
      end
    end,
		p = function(override)
			if override~=nil then en=override else en=ENABLE_POSE==false end
      if tags.gps>0 then
        if en==false then
          print(util.color('POSE disabled!','yellow'))
          webots.wb_gps_disable(tags.gps)
  	  		webots.wb_compass_disable(tags.compass)
  				webots.wb_inertial_unit_disable(tags.inertialunit)
          ENABLE_POSE = false
        else
          print(util.color('POSE enabled!','green'))
          webots.wb_gps_enable(tags.gps, timeStep)
  	  		webots.wb_compass_enable(tags.compass, timeStep)
  				webots.wb_inertial_unit_enable(tags.inertialunit, timeStep)
          ENABLE_POSE = true
        end
      end
    end,
		i = function(override)
			if override~=nil then en=override else en=ENABLE_IMU==false end
      if tags.accelerometer>0 then
        if en==false then
          print(util.color('IMU disabled!','yellow'))
          webots.wb_accelerometer_disable(tags.accelerometer)
    			webots.wb_gyro_disable(tags.gyro)
          ENABLE_IMU = false
        else
          print(util.color('IMU enabled!','green'))
          webots.wb_accelerometer_enable(tags.accelerometer, timeStep)
    			webots.wb_gyro_enable(tags.gyro, timeStep)
          ENABLE_IMU = true
        end
      end
    end,
  }

function WebotsBody.entry(Body)
  -- Request @ t=0 to always be earlier than position reads
	-- Grab the tags from the joint names
	tags.joints, tags.jointsByName = {}, {}
  tags.jointsensors={} --for webots 8

	for i,v in ipairs(jointNames) do
    local tag=0
    if v~="null" then
      tag = webots.wb_robot_get_device(v)
      tag_s = webots.wb_robot_get_device(v..'Sensor')
    end
		tags.joints[i] = tag
    tags.jointsensors[i]=tag_s
    tags.jointsByName[v] = tag
		if tag > 0 then
      --Do nothing for motor(default position control mode)
      webots.wb_position_sensor_enable(tag_s,timeStep)
		end
	end

	-- Add Sensor Tags
	tags.accelerometer = webots.wb_robot_get_device("Accelerometer")
	tags.gyro = webots.wb_robot_get_device("Gyro")
	tags.gps = webots.wb_robot_get_device("GPS")
	tags.compass = webots.wb_robot_get_device("Compass")
	tags.inertialunit = webots.wb_robot_get_device("InertialUnit")

  if Config.sensors.front_camera then
    tags.front_camera = webots.wb_robot_get_device("FrontCamera")
    print("Front camera:",tags.front_camera)
  end
	if Config.sensors.back_camera then tags.back_camera = webots.wb_robot_get_device("BackCamera") end
  if Config.sensors.chest_lidar then tags.chest_lidar = webots.wb_robot_get_device("ChestLidar") end
  if Config.sensors.head_lidar then tags.head_lidar = webots.wb_robot_get_device("HeadLidar") end
  if Config.sensors.velodyne1 then tags.velodyne1 = webots.wb_robot_get_device("Velodyne1") end
  if Config.sensors.kinect then
    tags.kinectRGB = webots.wb_robot_get_device(Config.kinect.RGBname or "kinect2RGB")
    tags.kinectD = webots.wb_robot_get_device(Config.kinect.Dname or"kinect2D")
    -- print("kinect:",tags.kinectRGB,tags.kinectD)
  end
	-- Enable or disable the sensors
	key_action.i(ENABLE_IMU)
	key_action.p(ENABLE_POSE)
	if ENABLE_CAMERA then key_action.c(ENABLE_CAMERA) end
  if ENABLE_CHEST_LIDAR then key_action.l(ENABLE_CHEST_LIDAR) end
	if ENABLE_HEAD_LIDAR then key_action.h(ENABLE_HEAD_LIDAR) end
	if ENABLE_KINECT then key_action.k(ENABLE_KINECT) end
  if ENABLE_VELODYNE1 then key_action.y(ENABLE_VELODYNE1) end

	Body.set_torque_enable(1)-- Ensure torqued on
	webots.wb_robot_step(timeStep)-- Take a step to get some values
  Body.set_position_p(PID_P)  -- PID setting

	local rad, val
	local positions = vector.zeros(nJoint)

  for idx, jtag in ipairs(tags.joints) do
    if jtag>0 then
      if webots.wb_motor_set_control_pid then -- Update the PID if necessary
        webots.wb_motor_set_control_pid(jtag, PID_P[idx], 0, 0)
      end
    end
  end

  local read_devices = tags.jointsensors
    for idx, jtag in ipairs(read_devices) do
    if jtag>0 then
   		val = get_pos(jtag)
      rad = servo.direction[idx] * val - servo.rad_offset[idx]
			rad = rad==rad and rad or 0
			positions[idx] = rad
    end
  end

	dcm.set_sensor_position(positions)
	dcm.set_actuator_command_position(positions)
	Body.tags = tags

	cw = Config.sensors.head_camera and require(Config.sensors.head_camera)
	vw = Config.sensors.vision and require(Config.sensors.vision)
  kw = Config.sensors.kinect and require(Config.sensors.kinect)
	fw = Config.sensors.feedback and require(Config.sensors.feedback)
  ww = Config.sensors.world and require(Config.sensors.world)
	kb = Config.webots_startup.test_file and require(Config.webots_startup.test_file)

  navigation_w=Config.sensors.navigation and require(Config.sensors.navigation)
  perception_w=Config.sensors.perception and require(Config.sensors.perception)
  rosio_w=Config.sensors.rosio and require(Config.sensors.rosio)

	obsw = Config.sensors.obstacle and require(Config.sensors.obstacle)
	s3w = Config.sensors.slam3d and require(Config.sensors.slam3d)
	WebotsBody.USING_KB = type(kb)=='table' and type(kb.update)=='function'
	if ww then ww.entry() end
  if fw then fw.entry() end
  if rw then rw.entry() end
	if obsw then obsw.entry() end
	if s3w then s3w.entry() end
	if vw and vw.entry then vw.entry() end
  if kw and kw.entry then kw.entry() end
  if navigation_w then navigation_w.entry() end
  if perception_w then perception_w.entry()end
  if rosio_w then rosio_w.entry()end
end


--local depth_array = carray.float(depth.data, n_pixels)
local depth_fl = ffi.new('float[?]', 1)
local n_depth_fl = ffi.sizeof(depth_fl)
local fl_sz = ffi.sizeof('float')



local plancount=0

function WebotsBody.update(Body)
		local get_time = webots.wb_robot_get_time
    local t = get_time()
		local cmds = Body.get_command_position() -- Set actuator commands from shared memory
		local poss = Body.get_position()
    local cmdt = Body.get_command_torque()
		local cmdv = Body.get_command_velocity()
		for idx, jtag in ipairs(tags.joints) do
			local cmd, pos,vel = cmds[idx], poss[idx], cmdv[idx]
			local en = Body.get_torque_enable()[idx]
			if en>0 and jtag>0 then -- Only update the joint if the motor is torqued on
        -- Update the PID
			--[[
          local new_P, old_P = Body.get_position_p()[idx], PID_P[idx]
          if new_P ~= old_P then
            PID_P[idx] = new_P
            webots.wb_motor_set_control_pid(jtag, new_P, 0, 0)
          end
			--]]
        local rad = servo.direction[idx] * (cmd + servo.rad_offset[idx])
        if en==1 then --position control
					set_pos(jtag, rad)
        elseif en==2 then --torque control
					--for whatever reason, torque directions are inverted
					--webots.wb_motor_set_torque(jtag,servo.direction[idx]*cmdt[idx])
					webots.wb_motor_set_velocity(jtag,math.abs(vel))
          webots.wb_motor_set_torque(jtag,-servo.direction[idx]*cmdt[idx])
				elseif en==3 then --velocity control
					if vel>0 then	set_pos(jtag,1E6)
					else set_pos(jtag,-1E6) end
					webots.wb_motor_set_velocity(jtag,math.abs(vel))
        end
      end
		end --for

		-- Step the simulation, and shutdown if the update fails
		if webots.wb_robot_step(Body.timeStep) < 0 then os.exit() end
		t = get_time()

    if ENABLE_IMU then
      -- Accelerometer data (verified)
      local accel = webots.wb_accelerometer_get_values(tags.accelerometer)
      dcm.sensorPtr.accelerometer[0] = (accel[1]-512)/128 * 9.801
      dcm.sensorPtr.accelerometer[1] = (accel[3]-512)/128 * 9.801
      dcm.sensorPtr.accelerometer[2] = (accel[2]-512)/128 * 9.801 --positive z axis
      -- Gyro data (verified)
      local gyro = webots.wb_gyro_get_values(tags.gyro)
      dcm.sensorPtr.gyro[0] = (gyro[3]-512)/512*39.24
      dcm.sensorPtr.gyro[1] = (gyro[2]-512)/512*39.24
      dcm.sensorPtr.gyro[2] = -(gyro[1]-512)/512*39.24
    end
    -- GPS and compass data
    -- Webots x is our y, Webots y is our z, Webots z is our x,
    -- Our x is Webots z, Our y is Webots x, Our z is Webots y
    if ENABLE_POSE then
      local gps     = webots.wb_gps_get_values(tags.gps)
      local compass = webots.wb_compass_get_values(tags.compass)
      local angle   = math.atan2( compass[2], -compass[1] )+math.pi --corrected
      local pose    = vector.pose{gps[1], -gps[3], angle} --corrected

      gps_pose0=Config.map_zero_pose
      wcm.set_robot_pose0(Config.map_zero_pose)
			-- if gps_pose0 then --relative pose from the starting position

			local rel_pose = util.pose_relative(pose,gps_pose0)
			wcm.set_robot_pose_gps( rel_pose )
      wcm.set_robot_pose( rel_pose ) --use ground truth for now
      if rosio_w then rosio_w.webots_tf() end
			-- else
			-- 	gps_pose0 = pose
			-- 	wcm.set_robot_pose_gps({0,0,0})
			-- end
    end

		-- Update the sensor readings of the joint positions
		local rad, val
		local positions = dcm.get_sensor_position()
    local read_devices = tags.jointsensors  --Webots 8 handling (separate joint encoder)
    for idx, jtag in ipairs(read_devices) do
    if jtag>0 then
--        print("position reading, tag:",jtag)
			val = get_pos(jtag)
      rad = servo.direction[idx] * val - servo.rad_offset[idx]
			rad = rad==rad and rad or 0
			positions[idx] = rad
--HACK
--[[
      print("torque reading at joint",jtag)
			local tq = webots.wb_motor_get_force_feedback(jtag)
			dcm.sensorPtr.current[idx-1] = tq==tq and tq*servo.direction[idx] or 0
--]]

    end
    end

		dcm.set_sensor_position(positions)




    -- Grab a camera frame
    if ENABLE_CAMERA and t >= NEXT_CAMERA then
      local w = webots.wb_camera_get_width(tags.front_camera)
      local h = webots.wb_camera_get_height(tags.front_camera)
      -- local img = ImageProc.rgb_to_yuyv(webots.to_rgb(tags.front_camera), w, h)
			-- local w = webots.wb_camera_get_width(tags.back_camera)
			-- local h = webots.wb_camera_get_height(tags.back_camera)
			-- local img = ImageProc.rgb_to_yuyv(webots.to_rgb(tags.back_camera), w, h)
--[[
--quick test... and it works fine
local rgb=webots.to_rgb(tags.head_camera)
local buf=ffi.new("uint8_t[?]",w*h*3)
ffi.copy(buf,rgb,w*h*3)
print(buf[0], buf[1], buf[2])
--]]
--			WebotsBody.update_head_camera(img, 2*w*h, 0, t)
			NEXT_CAMERA = t + camera_timeStep / 1000
    end




    -- Grab a kinect frame
    if ENABLE_KINECT and t >= NEXT_KINECT then
    -- if false then
      local fl_sz = ffi.sizeof('float')
      local d_w = webots.wb_range_finder_get_width(tags.kinectD)
      local d_h = webots.wb_range_finder_get_height(tags.kinectD)
      local depth = {data = webots.get_rangefinder_ranges(tags.kinectD),width = d_w,height = d_h,	t = t,}

      local w = webots.wb_camera_get_width(tags.kinectRGB)
      local h = webots.wb_camera_get_height(tags.kinectRGB)
      local fov = webots.wb_camera_get_fov(tags.kinectRGB)

      -- print("WH:",d_w,d_h,w,h, fov)
			local rgb = {data = webots.to_rgb(tags.kinectRGB),width = w,height = h,t = t,}
			local buf_rgb=ffi.new("uint8_t[?]",w*h*3)
      ffi.copy(buf_rgb,rgb.data,w*h*3)

      local depth = {data = webots.get_rangefinder_ranges(tags.kinectD),width=d_w, height=d_h,t=t,}
      local buf_depth=ffi.new("float[?]",d_w*d_h)
      ffi.copy(buf_depth,depth.data,d_w*d_h*fl_sz)

      if rosio_w then rosio_w.webots_rgbd(count,
        w,h, ffi.string(buf_rgb,w*h*3),
        d_w,d_h, ffi.string(buf_depth,d_w*d_h*fl_sz), fov) end
      NEXT_KINECT = t + kinect_timeStep / 1000
    end

    --Grab a lidar scan
    if ENABLE_HEAD_LIDAR and t >= NEXT_HEAD_LIDAR then
      local n = webots.wb_lidar_get_horizontal_resolution(tags.head_lidar)
      local fov = webots.wb_lidar_get_fov(tags.head_lidar)
      local ranges = webots.get_lidar_ranges(tags.head_lidar)
      local fl_sz = ffi.sizeof('float')
      if rosio_w then rosio_w.webots_lidar(count,fov/2, -fov/2, n, ffi.string(ranges,n*fl_sz),"base_range_sensor_link" ) end


			local xbuf,ybuf,zbuf,count, xbuf2, ybuf2, count2 = Velodyne.process_webots_lidar(
				ffi.string(ranges,n*fl_sz),n,	Config.lidar.xyz,Config.lidar.rpy,
				-1, Config.lidar.maxrange, fov/2, -fov/2)

			local count_ffi = ffi.new("int[1]");count_ffi[0]=count
			local count_buf = ffi.string(count_ffi,ffi.sizeof("int"))
			-- if s3w then s3w.update(xbuf,ybuf,zbuf,pose_ffi,count_buf) end


			local pose=wcm.get_robot_pose_gps()
      -- print("GPS pose:",unpack(pose))
			-- if Config.slam and (Config.slam.enable or Config.slam.encoderonly) then
			-- 	pose=wcm.get_robot_pose() end

			local pose_ffi = ffi.new("float[3]")
			pose_ffi[0],pose_ffi[1],pose_ffi[2]=pose[1],pose[2],pose[3]
			local pose_buf = ffi.string(pose_ffi,3*ffi.sizeof("float"))

			local rgb_ffi,index_ffi,time_ffi=ffi.new("uint8_t[?]",3*count),ffi.new("uint8_t[1]"),ffi.new("float[1]")
			index_ffi[0],time_ffi[0]=1,Body.get_time()
			local rgb_buf = ffi.string(rgb_ffi,3*count)
			local index_buf = ffi.string(index_ffi,1)
			local time_buf = ffi.string(time_ffi,ffi.sizeof("float"))
      msg_lidar = {count_buf..xbuf..ybuf..zbuf}



			-- if obsw then obsw.update(xbuf2,ybuf2,nil,pose_ffi,count2,do_broadcast) end

      NEXT_HEAD_LIDAR = t + lidar_timeStep / 1000
    end

    -- if unix.time()>= NEXT_VELODYNE_BROADCAST then
    --   do_broadcast = true
    --   NEXT_VELODYNE_BROADCAST=unix.time()+Config.velodyne_monitor_interval/1000
    --   if msg_lidar then velodyne0_raw_ch:send(msg_lidar) end
    --   if msg_kinect then velodyne1_raw_ch:send(msg_kinect) end
    -- end

    if t >= NEXT_NAVIGATION then
      if navigation_w then navigation_w.update() end
      NEXT_NAVIGATION=t+navigation_timeStep/1000
    end

--


--     end

    -- Grab a velodyne lidar scan
--     if ENABLE_VELODYNE1 and t >= NEXT_VELODYNE1 then
--       local n = webots.wb_lidar_get_horizontal_resolution(tags.velodyne1)
--       local m = webots.wb_lidar_get_number_of_layers(tags.velodyne1)
--       local fov = webots.wb_lidar_get_fov(tags.velodyne1)
--       local v_fov = webots.wb_lidar_get_vertical_fov(tags.velodyne1)
--       local buf = webots.get_lidar_ranges(tags.velodyne1)
--       local fl_sz = ffi.sizeof('float')
--       local ranges=ffi.new("float[?]",m*n)
--       ffi.copy(ranges,buf,m*n*fl_sz)
--
--       local res,v_res,rpy = fov / n, v_fov/m,Body.get_rpy()
-- 			local headPitch = Body.get_head_position()[1]
--
-- 			local xbuf,ybuf,zbuf,count
-- 			if Config.velodyne[1].neckxyz then
-- 				xbuf,ybuf,zbuf,count = Velodyne.process_webots_velodyne(
-- 					ffi.string(ranges,m*n*fl_sz),n,m,
-- 					Config.velodyne[1].xyz, Config.velodyne[1].neckxyz,
-- 					-headPitch, 0,  -1)
-- 			else
-- 				xbuf,ybuf,zbuf,count = Velodyne.process_webots_velodyne(
-- 					ffi.string(ranges,m*n*fl_sz),n,m,
-- 					Config.velodyne[1].xyz,
-- 					{0,0,0},0,0,  -1)
-- 			end
--
--
-- --THIS CRASHES!!!!!!
-- 			process_pfield(xbuf, ybuf, zbuf, count)
    --
    --
		-- 	local count_ffi = ffi.new("int[1]");count_ffi[0]=count
		-- 	local count_buf = ffi.string(count_ffi,ffi.sizeof("int"))
    --
		-- 	--RUN SLAM HERE TO UPDATE THE POSE
		--   local odom = mcm.get_status_odometry()
		-- 	if s3w then s3w.update(xbuf,ybuf,zbuf,pose_ffi,count_buf, odom) end
    --
		-- 	local pose=wcm.get_robot_pose_gps()
		-- 	if Config.slam and Config.slam.enable  then pose=wcm.get_robot_pose() end
		-- 	local pose_ffi = ffi.new("float[3]")
		-- 	pose_ffi[0],pose_ffi[1],pose_ffi[2]=pose[1],pose[2],pose[3]
    --
		-- 	local pose_buf = ffi.string(pose_ffi,3*ffi.sizeof("float"))
		-- 	local index_ffi,time_ffi=ffi.new("uint8_t[1]"),ffi.new("float[1]")
    --
		-- 	index_ffi[0],time_ffi[0]=1,Body.get_time()
		-- 	local index_buf = ffi.string(index_ffi,1)
		-- 	local time_buf = ffi.string(time_ffi,ffi.sizeof("float"))
    --
    --
		-- 	local do_broadcast = false
		-- 	if unix.time()>= NEXT_VELODYNE_BROADCAST then
		-- 		do_broadcast = true
		-- 		NEXT_VELODYNE_BROADCAST=unix.time()+Config.velodyne_monitor_interval/1000
		-- 	end
		-- 	if do_broadcast then
		-- 		velodyne0_raw_ch:send({count_buf..xbuf..ybuf..zbuf..index_buf..pose_buf..time_buf})
		-- 	end
    --
		-- 	if obsw then obsw.update(xbuf,ybuf,zbuf,pose_buf,count_buf,do_broadcast) end
    --
    --
    --
    --   NEXT_VELODYNE1 = t + velodyne_timeStep / 1000 + hcm.get_teleop_velodyne_offset()/1000
		-- 	hcm.set_teleop_velodyne_offset(0)
    -- end

		-- Receive webot messaging
		while webots.wb_receiver_get_queue_length(tags.receiver) > 0 do

----UPDATE CUBE POSE INFO (FROM GROUND TRUTH DATA)
	    ndata = webots.wb_receiver_get_data_size(tags.receiver)
	    msg = webots.wb_receiver_get_data(tags.receiver)
      -- print(msg)
      -- if msg then
      --   loadstring("msg_v="..msg)()
      --   -- print("MSG: "..msg)
      --
      --   local pose=wcm.get_robot_pose()
      --   local pose0=wcm.get_robot_pose0()
      --   local inv_pose0=util.pose_relative({0,0,0},pose0)
      --
      --   -- print("pose0",unpack(pose0))
      --   -- print("pose",unpack(pose))
      --   -- print("xyz:",unpack(msg_v.xyz))
      --   -- print("new xyz:",new_xya[1],new_xya[2],msg_v.xyz[3])
      --
      --   local new_xya=util.pose_global({msg_v.xyz[1],msg_v.xyz[2],0},inv_pose0)
      --   local new_rpy=util.shallow_copy(msg_v.rpy)
      --   new_rpy[3]=new_rpy[3]-pose0[3]
      --
      --   if rosio_w and Config.skip_perception then
      --     rosio_w.webots_item(
      --     msg_v.name,msg_v.id,
      --     {new_xya[1],new_xya[2],msg_v.xyz[3]},
      --     msg_v.rpy)
      --    end
      --  end
	    webots.wb_receiver_next_packet(tags.receiver)
	  end

    --local key_code = webots.wb_robot_keyboard_get_key() 		-- Grab keyboard input, for modifying items
		--8.5.0 API change... now accept multiple keypresses
		local key_code = webots.wb_keyboard_get_key() 		-- Grab keyboard input, for modifying items
		local key_count=0
		while (key_code>0 and key_count<5) do
			-- print("KEY #:",key_count,key_code)

			--for now lets just use a single key
			if WebotsBody.USING_KB and key_code>0 and key_count==0 then
				kb.update(key_code)
			end

			key_count=key_count+1
			key_code = webots.wb_keyboard_get_key() 		-- Grab keyboard input, for modifying items
		end

		if ww then ww.update() end
  	if fw then fw.update() end
  	if rw then rw.update() end
    if rosio_w then rosio_w.update() end
    if perception_w then perception_w.update() end

end

function WebotsBody.exit() if ww then ww.exit() end end

return WebotsBody
