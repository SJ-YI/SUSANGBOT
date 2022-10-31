#!/usr/bin/env luajit
assert(arg, "Run from the shell")
dofile'include.lua'

unix.chdir(HOME)
fd = io.open("start_robot.sh", "w")
local str=""
str=str.."tmux kill-session\nsleep 1\n"
str=str.."tmux new-session -d\n"
str=str.."tmux set -g mouse on\n"
local num_windows=
	#Config.robot_startup.start_wizards+#Config.robot_startup.start_processes
print("num_windows for start_robot:",num_windows)

local wsiz=math.floor((num_windows+1)/2)
-- print(wsiz)
-- for i=1,num_windows-1 do
-- 	str=str.."tmux split-window -h\ntmux select-pane -R\n"
-- end

for i=1,wsiz-1 do
	local div = math.floor(100-100/(wsiz-i+1))
	-- print(div)
	-- str=str.."tmux split-window -h\ntmux select-pane -R\n"
	-- str=str.."tmux split-window -h -p "..div.."\n"
	str=str.."tmux split-window -h -p "..div.."\n"
	str=str.."tmux select-pane -L\n"
	str=str.."tmux split-window -v\n"
	str=str.."tmux select-pane -R\n"
end
str=str.."tmux split-window -v\n"


local pre_str="tmux send \"source ~/.bashrc\" C-m\ntmux send \"cd RunRobot\" C-m\n"
if Config.robot_startup.ros_dep then
	for j=1,#Config.robot_startup.ros_dep do
		pre_str=pre_str.."tmux send \""..Config.robot_startup.ros_dep[j].."\" C-m\n"
	end
end


local window_count=0
local pre_cmd=Config.robot_startup.pre_cmd
for i, process in ipairs(Config.robot_startup.start_processes) do
	str=str.."tmux select-pane -t "..window_count.."\n"
	str=str..pre_str
	-- str=str.."tmux send \"cd RunRobot;"..process.."\" C-m\n"
	if pre_cmd then str=str.."tmux send \""..pre_cmd.."\" C-m\n" end
	str=str.."tmux send \""..process.."\" C-m\n"
	window_count=window_count+1
end
for i, wizard in ipairs(Config.robot_startup.start_wizards) do
	local wizard_name=wizard[1]..'_wizard.lua'
	if #wizard==2 then wizard_name=wizard_name..' '..wizard[2] end
	str=str.."tmux select-pane -t "..window_count.."\n"
	str=str..pre_str
	--	str=str.."tmux send \"cd RunRobot;./"..wizard_name.."\" C-m\n"
	if pre_cmd then str=str.."tmux send \""..pre_cmd.."\" C-m\n" end
	str=str.."tmux send \"luajit "..wizard_name.."\" C-m\n"
	window_count=window_count+1
end

-- str=str.."tmux select-layout even-horizontal\n"
str=str.."tmux attach-session -d\n"
fd:write(str)
fd:close()
os.execute("chmod +x start_robot.sh")


fd = io.open("webots_start.sh", "w")
local str=""
str=str.."tmux kill-session\nsleep 1\n"
str=str.."tmux new-session -d\n"
str=str.."tmux set -g mouse on\n"
local num_windows=
	#Config.webots_startup.start_wizards+#Config.webots_startup.start_processes+1
-- print(num_windows)

local wsiz=math.floor((num_windows+1)/2)
for i=1,wsiz-1 do
	local div = math.floor(100-100/(wsiz-i+1))
	-- print(div)
	str=str.."tmux split-window -h -p "..div.."\n"
	str=str.."tmux select-pane -L\n"
	str=str.."tmux split-window -v\n"
	str=str.."tmux select-pane -R\n"
end
str=str.."tmux split-window -v\n"

-- for i=1,num_windows-1 do
-- 	str=str.."tmux split-window -h\ntmux select-pane -R\n"
-- end
str=str.."tmux select-pane -t 0\n"
str=str.."tmux send \"source ~/.bashrc\" C-m\n"
str=str.."tmux send \"webots Webots/worlds/"..Config.webots_startup.world_name..".wbt\" C-m\n"

local window_count=1
for i, wizard in ipairs(Config.webots_startup.start_wizards) do
	local wizard_name=wizard[1]..'_wizard.lua'
	if #wizard==2 then wizard_name=wizard_name..' '..wizard[2] end
	str=str.."tmux select-pane -t "..window_count.."\n"
	str=str.."tmux send \"source ~/.bashrc\" C-m\n"
	str=str.."tmux send \"cd RunRobot\" C-m\n"
	str=str.."tmux send \"luajit "..wizard_name.."\" C-m\n"
	window_count=window_count+1
end
for i, process in ipairs(Config.webots_startup.start_processes) do
	str=str.."tmux select-pane -t "..window_count.."\n"
	str=str.."tmux send \"source ~/.bashrc\" C-m\n"
	str=str.."tmux send \"cd RunRobot;"..process.."\" C-m\n"
	window_count=window_count+1
end
-- str=str.."tmux select-layout even-horizontal\n"
str=str.."tmux attach-session -d\n"
fd:write(str)
fd:close()
os.execute("chmod +x webots_start.sh")
