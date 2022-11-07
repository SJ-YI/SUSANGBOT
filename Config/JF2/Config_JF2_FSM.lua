assert(Config, 'Need a pre-existing Config table!')
local vector = require'vector'

local fsm = {}
-- Update rate in Hz
fsm.update_rate = 200

-- Which FSMs should be enabled?
fsm.libraries = {}
-- fsm.enabled = {Motion = true,Arm=true, Body=true}

fsm.enabled = {Motion = true}
fsm.select = {Motion = 'Omni',Body = 'SSB',Arm='JF'}
fsm.Motion = {{'motionIdle', 'init', 'motionOmni'},}
fsm.Arm = {
  {'armIdle', 'init', 'armInit'},
  {'armInit', 'done', 'armTeleop'},
}
fsm.Body = {
  {'bodyIdle', 'start', 'bodyNavigate'},
  {'bodyNavigate', 'done', 'bodyIdle'},
}

Config.fsm = fsm

-- Add all FSM directories that are in Player
for _,sm in ipairs(fsm.enabled) do
  local pname = {HOME, '/Player/', sm, 'FSM', '/?.lua;', package.path}
  package.path = table.concat(pname)
end

return Config
