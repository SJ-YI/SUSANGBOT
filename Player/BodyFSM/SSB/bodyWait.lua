local state = {}
state._NAME = ...
require'mcm'
require'wcm'
-- IDLE IS A RELAX STATE
-- NOTHING SHOULD BE TORQUED ON HERE!
local Body = require'Body'
local t_entry, t_update

function state.entry()
  print(state._NAME..' Entry' )
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
end


function state.update()
  local t = Body.get_time()
  local t_diff = t - t_update
  t_update = t
  if t-t_entry>5.0 then
    return "done"
  end
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
