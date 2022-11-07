--------------------------------
-- World Communication Module
-- (c) 2013 Stephen McGill
--------------------------------
local vector = require'vector'
local memory = require'memory'

-- shared properties
local shared = {}
local shsize = {}
local MAX_OBJECTS=100
local MAX_TASKS=255

shared.robot = {}
shared.robot.pose = vector.zeros(3)
shared.robot.pose0 = vector.zeros(3)
shared.robot.pose_odom = vector.zeros(3)
shared.robot.pose_gps = vector.zeros(3)
shared.robot.rostime = vector.zeros(1)
shared.robot.webots = vector.zeros(1)

shared.robot.location = vector.zeros(1) --robot's location, kitchen/living room/ et
shared.robot.front_clearance = vector.zeros(1)
shared.robot.wristforce=vector.zeros(3)
shared.robot.joints=vector.zeros(5)
shared.robot.pfield=vector.zeros(12)
shared.robot.posereset=vector.zeros(1)
shared.robot.proceed=vector.zeros(1)

shared.robot.enable_stop=vector.zeros(1)
shared.robot.stopped=vector.zeros(1)
shared.robot.resume_t=vector.zeros(1)



shared.robot.lastqarm=vector.zeros(5)


shared.task={}
shared.task.num=vector.zeros(1)
shared.task.index=vector.zeros(1)
shared.task.action=vector.zeros(MAX_TASKS)
shared.task.target=vector.zeros(MAX_TASKS)
shared.task.isrunning=vector.zeros(1)



--for general purpose object manipulation
shared.objects = {}
shared.objects.num=vector.zeros(1)
shared.objects.ids=vector.zeros(MAX_OBJECTS)
shared.objects.types=vector.zeros(MAX_OBJECTS)
shared.objects.locations=vector.zeros(MAX_OBJECTS)
shared.objects.xpos=vector.zeros(MAX_OBJECTS)
shared.objects.ypos=vector.zeros(MAX_OBJECTS)
shared.objects.zpos=vector.zeros(MAX_OBJECTS)
shared.objects.depths=vector.zeros(MAX_OBJECTS)

shared.objects.selected=vector.zeros(1)
shared.objects.releasepos=vector.zeros(1)
shared.objects.holdingid=vector.zeros(1)
shared.objects.grabheight=vector.zeros(1)
shared.objects.lookingfor=vector.zeros(1)

shared.objects.test=vector.zeros(1)
shared.objects.picktype=vector.zeros(1)
shared.objects.targetxyz=vector.zeros(3)

shared.objects.clear=vector.zeros(1)
shared.objects.enable=vector.zeros(1)
shared.objects.enable_t=vector.zeros(1)
shared.objects.detection_count=vector.zeros(1)
shared.objects.lid_xyz=vector.zeros(3)

shared.objects.tableX=vector.zeros(1)
shared.objects.tableA=vector.zeros(1)
shared.objects.scanpose=vector.zeros(3)


shared.objects.enable_vertical_pickup=vector.zeros(1) --automaticall pickup vertically if item is far away



--Currently visible objects
shared.cobjects = {}
shared.cobjects.num=vector.zeros(1)
shared.cobjects.ids=vector.zeros(MAX_OBJECTS)
shared.cobjects.xpos=vector.zeros(MAX_OBJECTS)
shared.cobjects.ypos=vector.zeros(MAX_OBJECTS)
shared.cobjects.zpos=vector.zeros(MAX_OBJECTS)


shared.grippos={}
shared.grippos.num=vector.zeros(1)
shared.grippos.posx=vector.zeros(255)
shared.grippos.posy=vector.zeros(255)
shared.grippos.posz=vector.zeros(255)
shared.grippos.yaw=vector.zeros(255)
shared.grippos.isvertical=vector.zeros(255)
shared.grippos.selected_no=vector.zeros(1) --which # is selected for grab
shared.grippos.len=vector.zeros(255)

shared.grippos.execute=vector.zeros(1) --execute detection once
shared.grippos.execute_t=vector.zeros(1) --execute detection time
shared.grippos.detection_count=vector.zeros(1) --execute detection time

shared.bookshelf={}
shared.bookshelf.enabledetect=vector.zeros(1)
shared.bookshelf.detected=vector.zeros(1)
shared.bookshelf.type=vector.zeros(1)
shared.bookshelf.pose=vector.zeros(3)


shared.garbagecan={}
shared.garbagecan.enabledetect=vector.zeros(1)
shared.garbagecan.detected=vector.zeros(1)
shared.garbagecan.posxyz=vector.zeros(3)
shared.garbagecan.yaw=vector.zeros(1)
shared.garbagecan.len=vector.zeros(1)
shared.garbagecan.lidheight=vector.zeros(1)


shared.mixer={}
shared.mixer.enabledetect=vector.zeros(1)
shared.mixer.detected=vector.zeros(1)
shared.mixer.posxyz=vector.zeros(3)
shared.mixer.width=vector.zeros(1)
shared.mixer.fruits=vector.zeros(3)


--Ground objects array
shared.gobject={}
shared.gobject.num=vector.zeros(1) --number of ground objects
shared.gobject.posex=vector.zeros(255)
shared.gobject.posey=vector.zeros(255)
shared.gobject.posez=vector.zeros(255)
shared.gobject.yaw=vector.zeros(255) --Global yaw angle of major axis of the ground object
shared.gobject.std=vector.zeros(255) --standard deviation
shared.gobject.selected_no=vector.zeros(1) -- which # is selected for pickup


shared.heightmap={}
shared.heightmap.enabledetect=vector.zeros(1)
shared.heightmap.detected=vector.zeros(1)
shared.heightmap.posxyz=vector.zeros(3)
shared.heightmap.basez=vector.zeros(1)
shared.heightmap.yaw=vector.zeros(1)
shared.heightmap.minheight=vector.zeros(1)






--for cube pickup task
shared.cubes={}
shared.cubes.num=vector.zeros(1)
shared.cubes.xpos=vector.zeros(7)
shared.cubes.ypos=vector.zeros(7)
shared.cubes.zpos=vector.zeros(7)
shared.cubes.r=vector.zeros(7)
shared.cubes.p=vector.zeros(7)
shared.cubes.y=vector.zeros(7)
shared.cubes.xoffsets=vector.zeros(7)
shared.cubes.yoffsets=vector.zeros(7)
shared.cubes.pickupseq=vector.zeros(98)


shared.slam={}
shared.slam.t=vector.zeros(1)
shared.lobstacle={}
shared.lobstacle.t=vector.zeros(1)
-- Call the initializer
memory.init_shm_segment(..., shared, shsize)
