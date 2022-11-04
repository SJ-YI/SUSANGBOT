local memory = require'memory'
local vector = require'vector'


local MAX_TRAJ= 10
local MAX_J_TRAJ = 100
--Human input memory

-- shared properties
local shared = {};
local shsize = {};

shared.xb360 = {};
shared.xb360.vel = vector.zeros(3)
shared.xb360.trigger = vector.zeros(2)
shared.xb360.lstick = vector.zeros(2)
shared.xb360.rstick = vector.zeros(2)
shared.xb360.dpad = vector.zeros(2)
shared.xb360.buttons = vector.zeros(9)
shared.xb360.seq = vector.zeros(1)
shared.xb360.t = vector.zeros(1)

shared.spacemouse={}
shared.spacemouse.button=vector.zeros(1)


















--They are for controlling HSR arm
shared.hsrarm={}
shared.hsrarm.qTargets=vector.zeros(5*MAX_J_TRAJ)
shared.hsrarm.qTargetNum=vector.zeros(1)
shared.hsrarm.qTargetCurrent=vector.zeros(1)
shared.hsrarm.qTargetT=vector.zeros(MAX_J_TRAJ)
shared.hsrarm.dPoseX=vector.zeros(MAX_J_TRAJ)
shared.hsrarm.dPoseY=vector.zeros(MAX_J_TRAJ)
shared.hsrarm.dPoseA=vector.zeros(MAX_J_TRAJ)
shared.hsrarm.maxreachx=vector.zeros(1)

shared.hsrarm.pTargetNum=vector.zeros(1)
shared.hsrarm.pTargetCurrent=vector.zeros(1)
shared.hsrarm.pTargets=vector.zeros(5*10)
shared.hsrarm.execute=vector.zeros(1)
shared.hsrarm.motionstate=vector.zeros(1)

shared.hsrarm.gripperTarget=vector.zeros(1)
shared.hsrarm.gripperExecute=vector.zeros(1)

--They are for controlling the base movement

--for impedance control
shared.hsrarm.controlmode=vector.zeros(1)
shared.hsrarm.controlmode_status=vector.zeros(1)
shared.hsrarm.controlforce=vector.zeros(3)
shared.hsrarm.controlexecute=vector.zeros(1)




shared.voice={}
shared.voice.execute=vector.zeros(1)
shared.voice.str="defaultstrxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"

shared.voice.replyenable=vector.zeros(1)
shared.voice.replyenable_t=vector.zeros(1)
shared.voice.reply=vector.zeros(0)
shared.voice.replystr="defaultstrxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"

shared.voice.answer=vector.zeros(0)
shared.voice.answerstr="defaultstrxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"





local MAX_PATH_NO=255

shared.path={}
shared.path.targetpose=vector.zeros(3)
shared.path.execute=vector.zeros(1)
shared.path.num=vector.zeros(1)
shared.path.index=vector.zeros(1)
shared.path.x=vector.zeros(MAX_PATH_NO)
shared.path.y=vector.zeros(MAX_PATH_NO)
shared.path.maxvel=vector.zeros(1)
shared.path.maxavel=vector.zeros(1)
shared.path.approachtype=vector.zeros(1) --0 for fine mode, 1 for quick mode








shared.base={}
shared.base.teleop_t=vector.zeros(1)
shared.base.teleop_velocity=vector.zeros(3)
shared.base.command_velocity=vector.zeros(3)
shared.base.velocity=vector.zeros(3)
shared.base.target=vector.zeros(3)



shared.head={}
shared.head.target=vector.zeros(3)
shared.head.execute=vector.zeros(1)



shared.arm={} --They are for UR5 arm
shared.arm.qTarget = vector.zeros(6)
shared.arm.pTarget = vector.zeros(6)
shared.arm.pRelTarget = vector.zeros(6)
shared.arm.rpyTarget = vector.zeros(3)
shared.arm.gripperTarget = vector.zeros(2)
shared.arm.execute = vector.zeros(1)


shared.arm.state=vector.zeros(1) --low level
shared.arm.motionstate=vector.zeros(1) --high level

shared.arm.grabxyz=vector.zeros(3)
shared.arm.grabrpy=vector.zeros(3)
shared.arm.releasexyz=vector.zeros(3)
shared.arm.releaserpy=vector.zeros(3)
shared.arm.grabwidth=vector.zeros(1)

shared.arm.velTarget=vector.zeros(6)
shared.arm.vel6Target=vector.zeros(6)
shared.arm.pHaptic = vector.zeros(6)

-- Call the initializer
memory.init_shm_segment(..., shared, shsize)
