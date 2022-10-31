
assert(Config, 'Need a pre-existing Config table!')

local vector = require'vector'
-- print("HERE")
Config.nJoint = 3+8
local jointNames={
  "OmniFL","OmniFR","OmniR",
  "LShoulderPitch","LShoulderRoll","LShoulderYaw","LElbowPitch",
  "RShoulderPitch","RShoulderRoll","RShoulderYaw","RElbowPitch",
}
Config.jointNames = jointNames

local indexWheel=1
local nJointWheel=3
local indexArm=4
local nJointArm=8

Config.parts = {
  Wheel = vector.count(indexWheel,nJointWheel),
  Arm = vector.count(indexArm,nJointArm),
}
local servo={}

servo.min_rad = vector.ones(Config.nJoint)*-180*DEG_TO_RAD
servo.max_rad = vector.ones(Config.nJoint)*180*DEG_TO_RAD
servo.direction=vector.new({-1,1,-1, 1,1,1,1,  1,1,1,1})
servo.rad_offset=vector.new({0,0,0,  0,0,0,0,   0,0,0,0})*DEG_TO_RAD
servo.torque_enable={3,3,3, 1,1,1,1, 1,1,1,1} --for webots

if IS_WEBOTS then
  servo.direction=vector.new({-1,1,-1, 1,1,1,1,  1,1,1,1})
  servo.rad_offset=vector.new({0,0,0,  0,0,0,90,   0,0,0,90})*DEG_TO_RAD
end

Config.nMITjoint=11

--Config.mit_wheels={dev="can5",direction={-1,-1,-1},dcmmap={1,2,3},reduction_ratio=6} --Wheel info
--Config.mit_wheels={dev="can5",direction={-1,-1,-1},dcmmap={1,2,3},reduction_ratio=6} --Wheel info
--Config.mit_wheels={dev="can5",direction={1,1,1},dcmmap={3,1,2},reduction_ratio=6} --Wheel info
Config.mit_wheels={dev="can5",direction={1,1,1},dcmmap={1,3,2},reduction_ratio=6} --Wheel info



Config.mit_arms={
  {dev="can3",direction={1,1,-1,-1},dcmmap={4,5,6,7},}, --Left arm
  {dev="can8",direction={-1,1,-1,1},dcmmap={8,9,10,11},}, --Right arm
}

Config.mit_arms.angle_min=vector.new({-90,-15,-90,-110,  -90,-85,-30,-110})*DEG_TO_RAD
Config.mit_arms.angle_max=vector.new({0, 85,30,0,        0,15,90,0})*DEG_TO_RAD
Config.mit_arms.reduction_ratio={9,9,8,8} --9 for X8v2, 8 for X6


Config.mitmotors={
  encoder_offset={
       0,0,0, --wheels
       0,0,0,0, --arms
       0,0,0,0  --arms
  }
}


-- Config.mit_arms.max_torque=vector.ones(Config.nJoint)*200
Config.mit_arms.max_torque=vector.ones(Config.nJoint)*50
Config.mit_arms.pgain=vector.ones(Config.nJoint)*10
Config.mit_arms.dgain=vector.ones(Config.nJoint)*(-0.5)

Config.servo=servo


Config.max_acc={1.0, 1.0, 4.0}
Config.max_vel={0.5, 0.5, 2.0}


return Config
