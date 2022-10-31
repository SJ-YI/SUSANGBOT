module(..., package.seeall);

local unix = require('unix');
local Config = require('Config')

local volume = 55;
local lang = 'en-us';
local gender = Config.dev.gender or 1;
if gender == 1 then
  girl = '';
else
  girl = '+f1';
end
enable = Config.speakenable or 1

if enable == 1 then
  -- define speak queue file
  fifo = '/tmp/speakFIFO'..(os.getenv('USER') or '');
  
  -- clean up old fifo if it exists
  unix.system('rm -f '..fifo);
  
  -- create directory if needed
  unix.system('mkdir -p /tmp/');
  
  -- create the queue file (438 = 0666 permissions)
  assert(unix.mkfifo(fifo, 438) == 0, 'Could not create FIFO: '..fifo);
  
  -- open the fifo
  fid = io.open(fifo, 'a+')
	assert(fid,string.format('could not open fifo (%s) ',fifo))
  
  -- start espeak background process
  if (unix.system('(/usr/bin/env espeak --stdout -v '..lang..girl..' -s 130 -a '..volume..' < '..fifo..' | aplay) > /dev/null 2>&1 &') ~= 0) then
    error('Could not run speak process');
  end
end

function talk(text)
  if enable==1 then
    print('Speak: '..text);
    fid:write(text..'\n');
    fid:flush()
  end
end

function play(filename)
  if enable == 1 then
    unix.system('aplay '..filename..' &');
  end
end

