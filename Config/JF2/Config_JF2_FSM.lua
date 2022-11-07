assert(Config, 'Need a pre-existing Config table!')
local vector = require'vector'

local fsm = {}
-- Update rate in Hz
fsm.update_rate = 200

-- Which FSMs should be enabled?
fsm.libraries = {}
-- fsm.enabled = {Motion = true,Arm=true, Body=true}

fsm.enabled = {Motion = true, Body=true}
fsm.select = {Motion = 'Omni',Body = 'SSB',Arm='JF'}
fsm.Motion = {{'motionIdle', 'init', 'motionOmni'},}
fsm.Arm = {
  {'armIdle', 'init', 'armInit'},
  {'armInit', 'done', 'armTeleop'},
}
fsm.Body = {
  {'bodyIdle', 'done', 'bodyIdle'},
  -- {'bodyIdle', 'start', 'bodyNavigate'},
  -- {'bodyNavigate', 'done', 'bodyIdle'},
}

Config.fsm = fsm


Config.pathplan={
  max_vel_webots=0.4,
  max_vel=0.2,
  max_avel=0.8,
  max_acceleration=0.8,

  waypoint_th=0.10,
  targetpose_th={0.06,2*DEG_TO_RAD}, --larger

  navigate_minvf1=0.5, --velocity factor for start
  navigate_minvf2=0.5,--velocity factor for end
  navigate_minvffin=0.1,

  navigate_r1 = 0.1,
--  navigate_r2 = 0.1,
  navigate_r2 = 0.05,
  navigate_rfin = 0.15,

  rotate_th1=0.3,
  rotate_th2=0.5,--face the target at the final waypoint
--  direct_angle_th=1 --roatate to target angle if closer than this
  direct_angle_th=0.6 --roatate to target angle if closer than this
}

-- Add all FSM directories that are in Player
for _,sm in ipairs(fsm.enabled) do
  local pname = {HOME, '/Player/', sm, 'FSM', '/?.lua;', package.path}
  package.path = table.concat(pname)
end

return Config
