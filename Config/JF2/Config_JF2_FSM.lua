assert(Config, 'Need a pre-existing Config table!')
local vector = require'vector'

local fsm = {}
-- Update rate in Hz
fsm.update_rate = 200
if IS_WEBOTS then
  fsm.update_rate = 250
end

-- Which FSMs should be enabled?
fsm.enabled = {
  -- Head = true,
  -- Motion = true,
  -- Body = true,
  Arm=true,
}

fsm.select = {
  --Body = 'M2',
  -- Motion = 'M2',
  Arm='JF'
}

fsm.libraries = {
  -- MotionLib = 'M1',
}
fsm.Arm = {
  {'armIdle', 'init', 'armInit'},
  {'armInit', 'done', 'armTeleop'},
  {'armTeleop', 'pickup', 'armPickup'},
  {'armPickup', 'done', 'armTeleop'},
  {'armTeleop', 'release', 'armRelease'},
  {'armRelease', 'done', 'armTeleop'},
}

--[[
fsm.Body = {
  {'bodyIdle', 'init', 'bodyInit'},
  {'bodyInit', 'done', 'bodyTeleop'},
}


fsm.Motion = {
  {'motionIdle', 'idle', 'motionIdle'},
  {'motionIdle', 'init', 'motionBias'},

  {'motionBias', 'done', 'motionInit'},

  {'motionInit', 'done', 'motionStand'},
  -- {'motionStand', 'walk', 'motionTrot'},
  {'motionStand', 'walk', 'motionTrot2'},
--  {'motionStand', 'bound', 'motionBound'},
  -- {'motionCrawl', 'stop', 'motionStand'},
  {'motionTrot', 'stop', 'motionStand'},
  {'motionTrot2', 'stop', 'motionStand'},
  {'motionBound', 'stop', 'motionStand'},

--  {'motionStand', 'pronk', 'motionPronk'},
  {'motionPronk', 'stop', 'motionStand'},
}
--]]

Config.fsm = fsm

-- Add all FSM directories that are in Player
for _,sm in ipairs(fsm.enabled) do
  local pname = {HOME, '/Player/', sm, 'FSM', '/?.lua;', package.path}
  package.path = table.concat(pname)
end

return Config
