#!/usr/bin/env luajit
dofile('../../include.lua')

local ffi=require'ffi'
local tcp = require 'tcp'
local ip,port="169.254.186.72",7
local tcp_sock=tcp.connect(ip, port)
local vector = require'vector'

local d_open0=0.5

local d_fold=-0.8
local d_release=1


local d_fold=-0.8
local d_open0=0
local d_release=0

local finger_sequence={}
for i=1,16 do
	-- if i==1 or i==5 or i==9 or i==13 then
	if i==1 or i==5 or  i==13 then
	else
		finger_sequence[#finger_sequence+1]=vector.ones(20) * d_open0 --release
		finger_sequence[#finger_sequence][i]=d_fold --fold
		finger_sequence[#finger_sequence+1]=vector.ones(20) * d_open0 --release
		finger_sequence[#finger_sequence][i]=d_release --release
	end
end

local fo1, fo2, fo3, fo4


--INIT zero position

f1,f2,f3,f4=tcp.hyuhand_send(
	tcp_sock,
	{0,d_release,d_release,d_release},
	{0,d_release,d_release,d_release},
	{0,0,d_release,d_release},
	{0,d_release,d_release,d_release}
)
unix.usleep(1E6*1) --500hz
fo1,fo2,fo3,fo4=tcp.hyuhand_send(
	tcp_sock,
	{0,d_release,d_release,d_release},
	{0,d_release,d_release,d_release},
	{0,0,d_release,d_release},
	{0,d_release,d_release,d_release}
)




local function print_fingers(f1,f2,f3,f4,t)
	print( "\027[H\027[2J")
	-- print(string.format("Thumb : %5.1f %5.1f %5.1f %5.1f",f1[1],f1[2],f1[3],f1[4]))
	-- print(string.format("Index : %5.1f %5.1f %5.1f %5.1f",f2[1],f2[2],f2[3],f2[4]))
	-- print(string.format("Middle: %5.1f %5.1f %5.1f %5.1f",f3[1],f3[2],f3[3],f3[4]))
	-- print(string.format("Ring  : %5.1f %5.1f %5.1f %5.1f",f4[1],f4[2],f4[3],f4[4]))


	print(string.format("Thumb : %5.1f %5.1f %5.1f %5.1f",f1[1]-fo1[1],f1[2]-fo1[2],f1[3]-fo1[3],f1[4]-fo1[4]))
	print(string.format("Index : %5.1f %5.1f %5.1f %5.1f",f2[1]-fo2[1],f2[2]-fo2[2],f2[3]-fo2[3],f2[4]-fo2[4]))
	print(string.format("Middle: %5.1f %5.1f %5.1f %5.1f",f3[1]-fo3[1],f3[2]-fo3[2],f3[3]-fo3[3],f3[4]-fo3[4]))
	print(string.format("Ring  : %5.1f %5.1f %5.1f %5.1f",f4[1]-fo4[1],f4[2]-fo4[2],f4[3]-fo4[3],f4[4]-fo4[4]))


	if t then
		print(string.format("Time elapsed: %.2f ms (%.1f hz)",t*1000,1/t))
	end
end

local count=0
local t_acc=0
local t_old=unix.time()
local fc=1
local t_finger=unix.time()
local lc=0


local finger_interval=1.5

while true do
	local f1,f2,f3,f4

	if fc>#finger_sequence then
		fc=1
		lc=lc+1
	end
	if lc>3 then
		f1,f2,f3,f4=tcp.hyuhand_send(
			tcp_sock,
			{0,0,0,0},--thumb
			{0,0,0,0},--index
			{0,0,0,0},--middle
			{0,0,0,0}
		)
	else
		fs=finger_sequence[fc]
		f1,f2,f3,f4=tcp.hyuhand_send(
				tcp_sock,
				{0,fs[2],fs[3],fs[4]}, --thumb
				{0,fs[6],fs[7],fs[8]}, --index
				{0,fs[10],fs[11],fs[12]},--middle
				{0,fs[14],fs[15],fs[16]}
			)
	end

	local t=unix.time()
	if t>t_finger+finger_interval then
		fc=fc+1
		t_finger=t
	end

	local t_elapsed=unix.time()-t_old
	if t_elapsed>0.1 then
		print_fingers(f1,f2,f3,f4, t_elapsed/count)
		t_old=unix.time()
		count=0
	end
	count=count+1
	-- unix.usleep(1E6*0.001) --500hz
end
