--------------------------------
-- Joint Communication Module
-- (c) 2013 Stephen McGill
--------------------------------
local Config = Config or require'Config'
assert(Config, 'DCM requires a config, since it defines joints!')
local nJoint = Config.nJoint
local memory = require'memory'
local vector = require'vector'


local shared_data = {}
local shared_data_sz = {}

-- Sensors from the robot
shared_data.sensor = {}

--setup sensor variables
shared_data.sensor.position = vector.zeros(nJoint)

-- These should not be tied in with the motor readings,
-- so they come after the read/tread setup
-- Raw inertial readings
shared_data.sensor.accelerometer = vector.zeros(3)
shared_data.sensor.gyro          = vector.zeros(3)
shared_data.sensor.magnetometer  = vector.zeros(3)
shared_data.sensor.imu_t  = vector.zeros(1) --we timestamp IMU so that we can run IMU later than state wizard
shared_data.sensor.imu_t0  = vector.zeros(1) --run_imu start time

shared_data.sensor.rpy           = vector.zeros(3) -- Filtered Roll/Pitch/Yaw
shared_data.sensor.battery       = vector.zeros(1) -- Battery level (in volts)
shared_data.sensor.compass       = vector.zeros(3)

-- Sensors from the robot
shared_data.tsensor = {}

--  Write to the motors
shared_data.actuator = {}

-- Additional memory variables (to make old dcm code work)
local nJoint=20
shared_data.actuator.gainLevel = vector.zeros(1)
shared_data.actuator.torqueEnable = vector.zeros(1)
shared_data.actuator.torqueEnableChanged = vector.zeros(1)
shared_data.actuator.hardnessChanged = vector.zeros(1)
shared_data.actuator.gainChanged = vector.zeros(1)
shared_data.actuator.bias = vector.zeros(nJoint)
shared_data.actuator.hardness = vector.ones(nJoint)
shared_data.sensor.updatedCount  = vector.zeros(1)


--Actuator variables
shared_data.actuator.command_position = vector.zeros(nJoint) --target position
shared_data.actuator.command_position2 = vector.zeros(nJoint) --current target position (with vel limit)
shared_data.actuator.torque_enable = vector.zeros(nJoint) --joint specific torque enable values
shared_data.actuator.torque_enable_changed = vector.zeros(1) --joint specific torque enable values
shared_data.actuator.command_current = vector.zeros(nJoint)
shared_data.actuator.command_velocity = vector.zeros(nJoint)
shared_data.actuator.command_torque = vector.zeros(nJoint)

shared_data.actuator.offline=vector.zeros(1)

--ADDITIONAL variables for MIT motor access


local nMITjoint=12
shared_data.mitmotor = {}
shared_data.mitmotor.read_mode=vector.zeros(1) --0 for velocity, 1 for position
shared_data.mitmotor.reset_zero=vector.zeros(1) --0 for velocity, 1 for position

shared_data.mitmotor.operation_mode=vector.zeros(nJoint) --0 for velocity, 1 for position
shared_data.mitmotor.temp=vector.zeros(nJoint) --0 for velocity, 1 for position
shared_data.mitmotor.command_position=vector.zeros(nJoint) --0 for velocity, 1 for position
shared_data.mitmotor.command_velocity=vector.zeros(nJoint) --0 for velocity, 1 for position
shared_data.mitmotor.current_position=vector.zeros(nJoint) --multi-turn position with offset applied
shared_data.mitmotor.zero_position=vector.zeros(nJoint) --zero position for multi-turn position
shared_data.mitmotor.current_position_raw=vector.zeros(nJoint) --single-turn position
shared_data.mitmotor.encoder_position=vector.zeros(nJoint) --single-turn position
shared_data.mitmotor.encoder_offset=vector.zeros(nJoint) --single-turn position
shared_data.mitmotor.current_velocity=vector.zeros(nJoint)
shared_data.mitmotor.command_torque=vector.zeros(nJoint)
shared_data.mitmotor.max_torque=vector.zeros(nJoint)
shared_data.mitmotor.pgain=vector.zeros(nJoint)
shared_data.mitmotor.igain=vector.zeros(nJoint)
shared_data.mitmotor.dgain=vector.zeros(nJoint)


shared_data.mitmotor.txcount=vector.zeros(4)
shared_data.mitmotor.rxcount=vector.zeros(4)
shared_data.mitmotor.voltage=vector.zeros(nJoint)

------------------------
-- Call the initializer
memory.init_shm_segment(..., shared_data, shared_data_sz)
