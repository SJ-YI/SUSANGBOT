local state = {}
state._NAME = ...
require'mcm'
require'wcm'
-- IDLE IS A RELAX STATE
-- NOTHING SHOULD BE TORQUED ON HERE!
local Body = require'Body'
local t_entry, t_update
local rossub = require'rossub'
local rospub = require 'tb3_rospub'
local sub_idx_mapcmd
local is_mapping=false

function state.entry()
  print(state._NAME..' Entry' )
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  wcm.set_task_index(1)
  hcm.set_path_execute(0)

  rossub.init('bodycontrol_sub')
  rospub.init('bodycontrol_pub')
  sub_idx_mapcmd=rossub.subscribeInt32('/mapcmd')
end


local function check_map_command()
	local mapcmd=rossub.checkInt32(sub_idx_mapcmd)
	if mapcmd and unix.time()>t_entry+1.0 then
		print("MAP COMMAND:",mapcmd)
		if mapcmd==1 then
			if not is_mapping then
				print"STARTING SLAM!!!!!"
				os.execute('rosnode kill /amcl')
				os.execute('rosnode kill /map_server')
				unix.usleep(1E6*1)
				os.execute('roslaunch pnu_tb3_launch ssb_slam.launch &')
				is_mapping=true
			else
				local robot_pose=rossub.checkTF("map","base_footprint")
				if not robot_pose then print("No tf, returning")
        else
  				local pose={robot_pose[1],robot_pose[2],robot_pose[6]}
  				print"ENDING SLAM!!!"
  				os.execute('rosrun map_server map_saver -f ~/Desktop/SUSANGBOT/Data/map')
  				unix.usleep(1E6*1)
  				print"RESTARTING AMCL!!!"
  				os.execute('rosnode kill /hector_mapping')
  				os.execute('roslaunch pnu_tb3_launch ssb_amcl.launch &')
  				unix.usleep(1E6*3)
  				rospub.posereset(pose)
  				print("POSE RESETTED TO",unpack(pose))
  				is_mapping=false
        end
			end
    elseif mapcmd==99 then
      print("POSE RESET!!!")
      rospub.posereset({0,0,0})
		elseif mapcmd==3 then
      print("MARKER ADDED")
			local robot_pose=rossub.checkTF("map","base_footprint")
			local pose={robot_pose[1],robot_pose[2],robot_pose[6]}
--			marker_pose_list[#marker_pose_list+1]=pose
--			save_markers()
		elseif mapcmd==4 then
      print("MARKER REMOVE")
--			if #marker_pose_list>1 then
  --		  marker_pose_list[#marker_pose_list]=nil
--			  save_markers()
		--	end
		elseif mapcmd==9 then
      print("MOVE START")
		--	rospub.posereset({0,0,0})
		--	unix.usleep(1E6*1)
		--	body_ch:send'start'
		end
	end
end



function state.update()
  local t = Body.get_time()
  local t_diff = t - t_update
  t_update = t
  check_map_command()
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
