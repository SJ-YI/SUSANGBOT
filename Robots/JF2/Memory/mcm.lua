--------------------------------
-- Motion Communication Module
-- (c) 2013 Stephen McGill, Seung-Joon Yi
--------------------------------
local memory = require'memory'
local vector = require'vector'

-- shared properties
local shared = {}
local shsize = {}




--Now we use shm variable here to toggle dcm leg reading (which is needed only for init)
shared.servo={}
shared.servo.offline=vector.zeros(1) --to indicate that the servo is offline (should restart the initialization motion)
shared.servo.read=vector.zeros(1)

--Now we put debug messages to DCM first
--So that we can show them in synchronous way
shared.servo.larm = vector.zeros(3)
shared.servo.rarm = vector.zeros(3)
shared.servo.lleg = vector.zeros(3)
shared.servo.rleg = vector.zeros(3)
shared.servo.imu = vector.zeros(17)


--Leg bias and walk params info
shared.leg={}
shared.leg.bias=vector.zeros(12)



-- Storing current stance info
shared.stance = {}
shared.stance.bodyTilt   = vector.zeros(1)
shared.stance.bodyHeight = vector.zeros(1)
shared.stance.bodyHeightTarget = vector.zeros(1)
shared.stance.uTorsoComp = vector.zeros(2) --For quasi-static balancing
shared.stance.waistPitchBias = vector.zeros(1) --To cancel out body sag
shared.stance.waistPitchBiasTarget = vector.zeros(1) --To cancel out body sag
shared.stance.singlesupport = vector.zeros(1) --we are doing quasi-static motion, so need more roll compensation
shared.stance.last_support = vector.zeros(3) --We keep last support point here
shared.stance.COMoffset = vector.zeros(3)--relative COM position from waist joint
shared.stance.COMoffsetPose1 = vector.zeros(3) --com offset for 'default' arm pose


shared.stance.COMoffsetBias = vector.zeros(1)







--Arm info

shared.arm = {}

--7 larm biases AND 7 rarm biases, 1-7 for larm, 8-14 for rarm
shared.arm.bias = vector.zeros(14)

--Target arm position (w/o compensation)
shared.arm.qlarm = vector.zeros(7)
shared.arm.qrarm = vector.zeros(7)

--Target transform (with fixed center frame)
shared.arm.trlarm = vector.zeros(7)
shared.arm.trrarm = vector.zeros(7)

--Torso-Compensated arm position (from moving torso frame)
shared.arm.qlarmcomp = vector.zeros(7)
shared.arm.qrarmcomp = vector.zeros(7)

--left and right hand offsets
shared.arm.lhandoffset = vector.zeros(3)
shared.arm.rhandoffset = vector.zeros(3)
shared.arm.handmass = vector.zeros(2) --Hand weight
shared.arm.holdmass = vector.zeros(2) --Additional holding mass weight
shared.arm.endpoint_compensation = vector.zeros(2) --should we apply ik-based endpoint compensation?

-- Walk Parameters (for tuning on the fly)
shared.walk = {}
shared.walk.tStep      = vector.zeros(1)
shared.walk.tZmp 	   = vector.zeros(1)
shared.walk.bodyHeight = vector.zeros(1)
shared.walk.stepHeight = vector.zeros(1)
shared.walk.torsoX     = vector.zeros(1)
shared.walk.footY      = vector.zeros(1)
shared.walk.supportX   = vector.zeros(1)
shared.walk.supportY   = vector.zeros(1)
shared.walk.hipRollCompensation = vector.zeros(2)

--Walk state variables
shared.walk.bodyOffset = vector.zeros(3)
shared.walk.targetvel        = vector.zeros(3)
shared.walk.vel        = vector.zeros(3)
shared.walk.command_t        = vector.zeros(1)
shared.walk.bipedal    = vector.zeros(1)
shared.walk.stoprequest= vector.zeros(1)
shared.walk.ismoving= vector.zeros(1)

-- Walk-step transition
shared.walk.steprequest= vector.zeros(1)
shared.walk.step_supportleg= vector.zeros(1)

-- kick type
shared.walk.kickphase= vector.zeros(1)
shared.walk.kicktype = vector.zeros(1)
shared.walk.kickfoot = vector.zeros(1)



--feet shift/tilt info


shared.walk.zShift = vector.zeros(2)
shared.walk.zvShift = vector.zeros(2)

shared.walk.aShiftX = vector.zeros(2)
shared.walk.avShiftX = vector.zeros(2)

shared.walk.aShiftY = vector.zeros(2)
shared.walk.avShiftY = vector.zeros(2)

shared.walk.t_last = vector.zeros(1)

shared.walk.footlift = vector.zeros(2)
shared.walk.heeltoewalk = vector.zeros(1)




--During SS, swinging foot sags below
--So they should be lifted up more
shared.walk.zSag = vector.zeros(2)


--direct leg compensation info

shared.walk.angleShift=vector.zeros(4) --ankleX ankleY kneeX hipY
shared.walk.delta_legs = vector.zeros(12)

--torso x-y shift (to keep zmp in the middle)
shared.walk.torsoShift = vector.zeros(2)


shared.walk.global_angle = vector.zeros(2) --surface gradient




-------------------------------------------




-- Motion Status
shared.status = {}

shared.status.battery=vector.zeros(1)
shared.status.velocity   = vector.zeros(3)
shared.status.odometry   = vector.zeros(3)
shared.status.bodyOffset = vector.zeros(3)
shared.status.falling    = vector.zeros(1)

--Foot xyz poses for quadruped robot
shared.status.pFLLeg = vector.zeros(3)
shared.status.pFRLeg = vector.zeros(3)
shared.status.pRLLeg = vector.zeros(3)
shared.status.pRRLeg = vector.zeros(3)



--Current Foot and Torso Poses
shared.status.uLeft = vector.zeros(3)
shared.status.uRight = vector.zeros(3)

shared.status.zLeg0 = vector.zeros(2) -- left, right height at the beginning of the step
shared.status.zLeg = vector.zeros(2) -- left, right height of the last frame

shared.status.zLegComp = vector.zeros(2) --used to compensate for leg lift for swinging leg due to flex

shared.status.aLeg = vector.zeros(2) --foot pitch angles of the last frame
shared.status.zGround = vector.zeros(0) -- if feet on a higher ground


shared.status.uTorso = vector.zeros(3)
shared.status.uTorsoZMPComp = vector.zeros(3) --zmp-based reactive torso compensation
shared.status.uTorsoNeutral = vector.zeros(3) --center torso position between two legs


shared.status.uSupport = vector.zeros(3)
shared.status.supportLeg = vector.zeros(1)
shared.status.ph = vector.zeros(1)

shared.status.velActual = vector.zeros(3)


--We store the torso velocity (to handle stopping)
shared.status.uTorsoVel = vector.zeros(3)

--ZMP is stored here for external monitoring
shared.status.uZMP = vector.zeros(3)
shared.status.uZMPMeasured = vector.zeros(3)

--We monitor whole mass points here
shared.status.com_pos = vector.zeros(66)

--We keep 2 previous frames of joint angles
--to calculate accelleration
shared.status.com_pos0 = vector.zeros(66)
shared.status.com_pos1 = vector.zeros(66)
shared.status.t0 = vector.zeros(1)
shared.status.t1 = vector.zeros(1)


shared.status.LFT = vector.zeros(3) --fZ tx ty
shared.status.RFT = vector.zeros(3) --fz tx ty
shared.status.LZMP = vector.zeros(3)
shared.status.RZMP = vector.zeros(3)
shared.status.IMU = vector.zeros(6) --rpy vr vp vy



--are arms and bodies intialized?
shared.status.motion_init = vector.zeros(0)
shared.status.body_init = vector.zeros(0)
shared.status.arm_init = vector.zeros(0)

--Mecanum wheel positions are stored here!
shared.status.mecanum_wheels = vector.zeros(4)







--how much of the upper body weight is held by left leg?
-- 0.5 for double support
-- 1 for full left single support

--Torso mass: 14.832lg, Arm mass: 5.714/6.74kg
--Total upper body mass: 27.286kg
--Leg mass: 11.86kg for each leg
--so if ratio is less than 11.86 / (11.86*2+27.286), the left leg is in mid-air
--and we should compensate the COM position so that we can hold the left leg mass
shared.status.leftSupportRatio = vector.ones(1)*0.5


shared.status.lleg_torque = vector.zeros(6)
shared.status.rleg_torque = vector.zeros(6)

--stabilization mode: SS (0) / DS (1) / Load (2)
shared.status.stabilization_mode = vector.zeros(1)
shared.status.temp_tZMP = vector.zeros(1)

--Stores current toe/heel lift information
shared.status.footTilt=vector.zeros(2)

--Current time
shared.status.t = vector.zeros(1)

-- Foot support
--SJ: they are bit misleading as they are different from 'support' positions
shared.support = {}
shared.support.uLeft_now   = vector.zeros(3)
shared.support.uRight_now  = vector.zeros(3)
shared.support.uTorso_now  = vector.zeros(3)
shared.support.uLeft_next  = vector.zeros(3)
shared.support.uRight_next = vector.zeros(3)
shared.support.uTorso_next = vector.zeros(3)


--Motion states
shared.motion={}
shared.motion.state = vector.zeros(1)
--Idle 0
--Init 1
--Stance 2
--HybridWalkInit 3
--HybridWalk 4
--HybridWalkEnd 5
--HybridWalkKick 6

shared.teach = {}
shared.teach.sway = 'none'


local maxSteps = 8
shared.step = {}

--Footsteps queue
--SJ: now 15 entries per item (we have the zmp mod for the final zmp of the trapzoid)
--{[rx ry ra supportLeg t0 t1 t2 zmpmodx zmpmody zmpmoda stepparam1 stepparam2 stepparam3      zmpxmod2 zmpymod2 ]}
shared.step.footholds  = vector.zeros(15*maxSteps)
shared.step.nfootholds = vector.zeros(1)



memory.init_shm_segment(..., shared, shsize)
