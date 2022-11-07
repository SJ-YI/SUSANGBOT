
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
-- servo.direction=vector.new({-1,-1,-1, 1,1,1,1,  1,1,1,1}) --INDEXED BY DCM
servo.max_vel = vector.new({180,180,180,    180,180,180,180,  180,180,180,180})*DEG_TO_RAD--INDEXED BY DCM
servo.torque_enable={3,3,3, 1,1,1,1, 1,1,1,1} --for webots

--DYNAMIXEL info
servo.direction=vector.new({1,1,1,   1,-1,-1,-1,     -1,-1,-1,1}) --INDEXED BY DCM
servo.rad_offset=vector.new({ 0,0,0,     0,0,0,0,      0,0,0,0 })*DEG_TO_RAD --INDEXED BY DCM
servo.dynamixel_ids={1,2,3,   11,12,13,14,  15,16,17,18}
servo.dynamixel_mode={1,1,1,   3,3,3,3,  3,3,3,3} --0 for current, 1 for velocity, 3 for pos, 4 for multi-turn pos, 5 for curret-limited pos
servo.dynamixel_map={1,2,3,    4,5,6,7,  8,9,10,11} --dcm mapping
servo.dynamixel_type={1,1,1,  1,1,1,1,  1,1,1,1} --1 for XM, 2 for pro 20W, 3 for pro 100/200W

if IS_WEBOTS then
  servo.direction=vector.new({-1,1,-1, 1,1,1,1,  1,1,1,1})
  servo.rad_offset=vector.new({0,0,0,  0,0,0,90,   0,0,0,90})*DEG_TO_RAD
end


Config.mit_arms={
  angle_min=vector.new({-90,-15,-90,-110,  -90,-85,-30,-110})*DEG_TO_RAD,
  angle_max=vector.new({0, 85,30,0,        0,15,90,0})*DEG_TO_RAD
}


Config.servo=servo
Config.max_acc={1.0, 1.0, 1.0}
Config.max_vel={0.5, 0.5, 0.5}

local wheel_r, body_r=0.05, 0.14
Config.wheels={
  wheel_r=wheel_r,
  body_r=body_r,

  --2.5d vel to wheel velocity
  xcomp=vector.new( {-0.86602,0, 0.86602} )/wheel_r,
  ycomp=vector.new( {-0.5, 1, -0.5 } )/wheel_r,
  acomp=vector.new( {-body_r, -body_r, -body_r} )/wheel_r,

  --wheel velocity to 2.5d vel
  -- velxcomp={-0.57735,0,0.57735},
  -- velycomp={-1/3, 2/3, -1/3},
  -- velacomp={-1/3/body_r,-1/3/body_r,-1/3/body_r}

  velxcomp=vector.new({-0.57735,-1/3, -1/3/body_r}),
  velycomp=vector.new({0, 2/3, -1/3/body_r}),
  velacomp=vector.new({0.57735,-1/3,-1/3/body_r})
}



return Config
