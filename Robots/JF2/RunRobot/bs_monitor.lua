#!/usr/bin/env luajit
local pwd = os.getenv'PWD'
local ok = pcall(dofile,'../fiddle.lua')
if not ok then ok=pcall(dofile,'./fiddle.lua') end
if not ok then ok=dofile'../../fiddle.lua' end

local unix=require'unix'
local util=require'util'
require'dcm'
local vector = require'vector'

local t0=unix.time()
local t_debug=t0

local leg_names={"FL","FR","RL","RR"}
local rxcount_old={0,0,0,0}
local t_last=t0



local function getcolorcount(num)
  str=string.format("[  %4d    ]",num)
  if num<900 then str=util.color(str,'red')
  else str=util.color(str,'green')
  end
  return str
end
local function getcolortemp(num)
  str=string.format("%2d ",num)
  if num>60 then str=util.color(str,'red')
  else str=util.color(str,'green')
  end
  return str
end


while true do
  local t= unix.time()
  if t-t_debug>1.0 then
    local cpos_raw=dcm.get_mitmotor_current_position_raw()
    local cpos=dcm.get_mitmotor_current_position()
    local cvel=dcm.get_mitmotor_current_velocity()
    local ctemp=dcm.get_mitmotor_temp()
    local cenc=dcm.get_mitmotor_encoder_position()
    local cvoltage=dcm.get_mitmotor_voltage()


    local rxcount=dcm.get_mitmotor_rxcount()

    local hiproll,e_hiproll={0,0,0,0},{0,0,0,0}
    local hippitch,e_hippitch={0,0,0,0},{0,0,0,0}
    local kneepitch,e_kneepitch={0,0,0,0},{0,0,0,0}
    local t_hiproll, t_hippitch, t_kneepitch={0,0,0,0},{0,0,0,0},{0,0,0,0}



    for i=1,2 do
      local dcmmap=Config.mit_legs[i].dcmmap
      hiproll[i],hippitch[i],kneepitch[i]=cpos[dcmmap[1]],cpos[dcmmap[2]],cpos[dcmmap[3]]
      e_hiproll[i],e_hippitch[i],e_kneepitch[i]=cpos_raw[dcmmap[1]],cpos_raw[dcmmap[2]],cpos_raw[dcmmap[3]]
      t_hiproll[i],t_hippitch[i],t_kneepitch[i]=ctemp[dcmmap[1]],ctemp[dcmmap[2]],ctemp[dcmmap[3]]
    end



    print("--------------------------------------------------------------------")
    str=string.format("% 5ds",t-t0)
    print(str.."         FL          FR          RL         RR             TEMP")


    str=string.format(
           "FPS:      %s%s%s%s",

      getcolorcount(   (rxcount[1]-rxcount_old[1])/(t-t_last)/3 ),
      getcolorcount(   (rxcount[2]-rxcount_old[2])/(t-t_last)/3 ),
      getcolorcount(   (rxcount[3]-rxcount_old[3])/(t-t_last)/3 ),
      getcolorcount(  (rxcount[4]-rxcount_old[4])/(t-t_last)/3  )
    )

    t_last=t
    rxcount_old=rxcount
    print(str)



    str="HipRoll  :"
    for i=1,2 do str=str..string.format("[P% 4d E% 3d]",hiproll[i],e_hiproll[i],t_hiproll[i]) end
    str=str.."  ( "
    for i=1,2 do str=str..getcolortemp( t_hiproll[i]) end
    print(str..")")

    str="HipPitch :"
    for i=1,2 do str=str..string.format("[P% 4d E% 3d]",hippitch[i],e_hippitch[i]) end
    str=str.."  ( "
    for i=1,2 do str=str..getcolortemp(t_hippitch[i]) end
    print(str..")")


    str="KneePitch:"
    for i=1,2 do str=str..string.format("[P% 4d E% 3d]",kneepitch[i],e_kneepitch[i]) end
    str=str.."  ( "
    for i=1,2 do str=str..getcolortemp(t_kneepitch[i]) end
    print(str..")")


    print(unpack(cvoltage))

--    print("--------------------------------------------------------------------")

    t_debug=t
    -- t_delay,t_send_count=0,0
  end
  unix.usleep(1E6*0.1);
end
