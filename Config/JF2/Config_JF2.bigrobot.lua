------------------------------
--ROBOT NAMES
------------------------------
--JongHo Friend 2, 3 Omni wheel + 2 4DOF arms

if not IS_WEBOTS and (HOSTNAME=="sj_9900K" or HOSTNAME=="sj") then end
IS_LOCALHOST = false

-- Global Config
Config = {PLATFORM_NAME = 'JF2',demo = false,}
local exo = {'FSM','Robot'}

Config.robot_startup={
  start_wizards={
    -- {'check', },
    -- {'dynamixel_pro', },
    {'xb360', },
    {'state', },


    {'arm_send', 1},
    {'arm_recv', 1},
    {'arm_send', 2},
    {'arm_recv', 2},

    {'wheel_send', },
    {'ros_displayupdate', },

  },
  start_processes={
    "roscore",
    "python displayupdate_wizard.py"
  }
}

Config.webots_startup={
	world_name="JF2_test",--webots world name
  test_file="test_jf2",--keyboard I/O file for webots control
	start_wizards={{'xb360', },},
  start_processes={}
}

Config.sensors={}
Config.debug={}
-----------------------------------
-- Load Paths and Configurations --
-----------------------------------
-- Custom Config files
local pname = {HOME, '/Config/',Config.PLATFORM_NAME, '/?.lua;', package.path}
package.path = table.concat(pname)
for _,v in ipairs(exo) do
	--print('Loading', v)
	local fname = {'Config_', Config.PLATFORM_NAME, '_', v}
	local filename = table.concat(fname)
  assert(pcall(require, filename))
end

-- Custom motion libraries
if Config.fsm.libraries then
	for i,sm in pairs(Config.fsm.libraries) do
		local pname = {HOME, '/Player/', i,'/' ,sm, '/?.lua;', package.path}
		package.path = table.concat(pname)
	end
end

-- Finite state machine paths
if Config.fsm.enabled then
	for sm, en in pairs(Config.fsm.enabled) do
		if en then
			local selected = Config.fsm.select and Config.fsm.select[sm]
			if selected then
				local pname = {HOME, '/Player/', sm, 'FSM/', selected, '/?.lua;', package.path}
				package.path = table.concat(pname)
			else --default fsm
				local pname = {HOME, '/Player/', sm, 'FSM/', '?.lua;', package.path}
				package.path = table.concat(pname)
			end
		end
	end
end


return Config
