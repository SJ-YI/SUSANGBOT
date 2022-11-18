assert(Config, 'Need a pre-existing Config table!')
local vector = require'vector'

local fsm = {}
-- Update rate in Hz
fsm.update_rate = 200

-- Which FSMs should be enabled?
fsm.libraries = {}
-- fsm.enabled = {Motion = true,Arm=true, Body=true}

fsm.enabled = {Motion = true, Body=true, Arm=true}
fsm.select = {Motion = 'Omni',Body = 'SSB',Arm='JF'}
fsm.Motion = {{'motionIdle', 'init', 'motionOmni'},}
fsm.Arm = {
  {'armIdle', 'init', 'armInit'},
  {'armInit', 'done', 'armTeleop'},

  {'armTeleop', 'greet', 'armGreet'},
  {'armTeleop', 'left', 'armGreetLeft'},
  {'armTeleop', 'right', 'armGreetRight'},

  {'armGreet', 'stop', 'armIdle'},
  {'armGreet', 'left', 'armIdle'},
  {'armGreet', 'right', 'armIdle'},

  {'armGreetLeft', 'stop', 'armIdle'},
  {'armGreetLeft', 'greet', 'armIdle'},
  {'armGreetLeft', 'right', 'armIdle'},

  {'armGreetRight', 'stop', 'armIdle'},
  {'armGreetRight', 'left', 'armIdle'},
  {'armGreetRight', 'greet', 'armIdle'},

  {'armTeleop', 'dance1', 'armDance1'},
  {'armTeleop', 'dance2', 'armDance2'},
  {'armTeleop', 'dance3', 'armDance3'},
  {'armTeleop', 'dance4', 'armDance4'},
  {'armDance1', 'done', 'armIdle'},
  {'armDance2', 'done', 'armIdle'},
  {'armDance3', 'done', 'armIdle'},
  {'armDance4', 'done', 'armIdle'},

  {'armDance1', 'stop', 'armIdle'},
  {'armDance2', 'stop', 'armIdle'},
  {'armDance3', 'stop', 'armIdle'},
  {'armDance4', 'stop', 'armIdle'},

  {'armDance1', 'stop', 'armIdle'},
  {'armDance2', 'stop', 'armIdle'},
  {'armDance3', 'stop', 'armIdle'},
  {'armDance4', 'stop', 'armIdle'},
}
fsm.Body = {
  {'bodyIdle', 'done', 'bodyIdle'},
  {'bodyIdle', 'move', 'bodyNavigate'},

  {'bodyNavigate', 'done', 'bodyIdle'},
  {'bodyNavigate', 'arrived', 'bodyNavigateWait'},
  {'bodyNavigateWait', 'done', 'bodyNavigate'},

  {'bodyNavigateWait', 'wait', 'bodyWait'},
  {'bodyNavigate', 'wait', 'bodyWait'},

  {'bodyWait','move','bodyNavigate'},
}

Config.pose_wait=3


Config.dancemotion={
  36, -- spider
  26, -- little rabbit
  45, -- pig market
  32, -- indian
  3, --greet
  9, --right
  10 --left
}

Config.startframe={
  75,
  15,  --60 and 98fps good for dance 2
  35,  --60 and 98fps good for dance 3
  15,   --40 and 98fps good for dance 4
  1,
  1,
  1
}

--delay the motion (video starting lag)
Config.startframe={
  75-50,
  15-50,  --60 and 98fps good for dance 2
  35-50,  --60 and 98fps good for dance 3
  15-50,   --40 and 98fps good for dance 4
  1,
  1,
  1
}





Config.dance_fps=98

Config.fsm = fsm

Config.pathplan={
  max_vel_webots=0.3,
  max_vel=0.3,
  max_avel=0.8,
  max_acceleration=0.8,

  waypoint_th=0.10,
  targetpose_th={0.06,2*DEG_TO_RAD}, --larger

  navigate_minvf1=0.5, --velocity factor for start
  navigate_minvf2=0.5,--velocity factor for end
  navigate_minvffin=0.1,

  navigate_r1 = 0.1,
  navigate_r2 = 0.1,
--  navigate_r2 = 0.05,
  navigate_rfin = 0.15,

  --face the target at the final waypoint
  rotate_th1=0.2,
  rotate_th2=0.2,  --start at 0.2+0.2, and end at 0.2

  direct_angle_th=0.6, --roatate to target angle if closer than this

  -- pfield_soft_th = 0.4,
  -- pfield_hard_th = 0.2,
  -- pfield_soft_th = 0.45,
  -- pfield_hard_th = 0.25,
  -- pfield_soft_th2 = 0.3, --for side obstacles
  pfield_soft_th = 0.55,
  pfield_hard_th = 0.35,

  pfield_soft_th2 = 0.40, --for side obstacles
}

-- Add all FSM directories that are in Player
for _,sm in ipairs(fsm.enabled) do
  local pname = {HOME, '/Player/', sm, 'FSM', '/?.lua;', package.path}
  package.path = table.concat(pname)
end

return Config
