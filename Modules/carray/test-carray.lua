dofile('../../include.lua')

local carray = require 'carray'
local vector = require 'vector'

dd = carray.double(5)
dd[2] = 4
dd[5] = 342
for i = 1, 5 do
  print(dd[i])
end
carray.cast()


print("===============================")
print(type(dd))

local ffi = require 'ffi'

--[[
cdata = ffi.cast('double*', dd:pointer())
print(type(cdata))
print(cdata)
print(cdata[1])
print(cdata[2])
--]]

d=ffi.new("double[4]")
d[0]=1
d[1]=2
d[2]=4
d[3]=3
for i=0,3 do
	print("d: ",i,d[i])
end

print("===============================")

for i=1,5 do
	print("dd: ",i,dd[i])
end


ffi.copy(dd:pointer(),d,4*ffi.sizeof('double'))

print("===============================")

for i=1,5 do
	print("dd: ",i,dd[i])
end




print("===============================")

--FFI cdata to luastring
local tmp=ffi.new("float[?]",3)
tmp[0]=1.3
tmp[1]=53.5
tmp[2]=2.1
b=ffi.string(tmp,3*ffi.sizeof("float"))

print(#b)
print(string.byte(b,1,#b))

--lua string to FFI cdata
c=ffi.new("float[?]",3)
ffi.copy(c,b,ffi.sizeof(c))
print (c[0],c[1],c[2])




print("===============================")
print("CARRAY_TORCH")
--carray to torch
local torch = require'torch'
local tmp   = torch.DoubleTensor(dd:table())
print(tmp)

--torch to carray
local tmp2   = torch.FloatTensor( {{1,2,3},{4,5,6}} )
print(tmp2)

local tmp2_ptr=torch.data(tmp2:contiguous())
d=ffi.new("float[?]",6)
ffi.copy(d,tmp2_ptr,6*ffi.sizeof("float"))
for i=0,5 do print(d[i]) end



print("===============================")



local tmp2   = torch.DoubleTensor( {{1,2,3},{4,5,6}} )
print(tmp2)
local row1=tmp2:select(2,3)
row1:copy(torch.DoubleTensor({10,9}))

print(tmp2)


print("===============================")
fd=io.open('temp.dat','w')
local float_ffi=ffi.new("float[?]",3)
float_ffi[0]=1.2
float_ffi[1]=2.5
float_ffi[2]=7.8
fd:write(ffi.string(float_ffi,3*ffi.sizeof("float")))
fd:close()
fd=io.open('temp.dat','r')
local buf_ffi=ffi.new("float[?]",3)
local buf_str=fd:read(3*ffi.sizeof("float"))
ffi.copy(buf_ffi, buf_str, ffi.sizeof(buf_ffi))
for i=0,2 do print(buf_ffi[i]) end







--temp: 3-point cubic spline

--http://letslearncomputing.blogspot.kr/2013/04/c-program-for-cubic-spline-interpolation.html


local function print_matr(m,siz)
  for i=1,siz do
    local buf=""
    for j=1, siz do
      buf=buf..string.format("   %.1f",m[i][j])
    end
    print(buf)
  end
end







local function get_spline_coeffs(t,x)
  local n = #t
  local s = vector.zeros(n)
  local dt = vector.zeros(n)
  local slope=vector.zeros(n)
  local m = {}
  local a,b,c,d = vector.zeros(n),vector.zeros(n),vector.zeros(n),vector.zeros(n)
  for i=1,n do m[i]=vector.zeros(n) end

  for i=2,n do
    dt[i-1]=t[i]-t[i-1]
    slope[i]=(x[i]-x[i-1])/dt[i-1]
  end

  for i=2,n-1 do
    m[i][i] = 2*(dt[i-1]+dt[i])
    if i>2 then  m[i][i-1],m[i-1][i] = dt[i-1], dt[i-1]  end
    m[i][n] = 6*(slope[i+1]-slope[i])
  end

  --forward elimination
  for i=2,n-1 do
    temp=m[i+1][i]/m[i][i]
    for j=2,n do
      m[i+1][j] = m[i+1][j]-temp*m[i][j]
    end
  end

  --backward substitution
  for i=n-1, 2, -1 do
    local sum=0
    for j=i,n-1 do sum=sum+m[i][j]*s[j] end
    s[i]=(m[i][n]-sum)/m[i][i]
  end

  for i=1,n-1 do
    a[i]=(s[i+1]-s[i])/(6*dt[i])
    b[i]=s[i]/2
    c[i]=(x[i+1]-x[i])/dt[i] - (2*dt[i]*s[i]+s[i+1]*dt[i])/6
    d[i]=x[i]
  end

  local ret={}
  ret.n=n
  ret.a, ret.b, ret.c, ret.d = a,b,c,d
  ret.t = t
  return ret
end

local function spline_eval(coeff, val)
  local t = coeff.t
  for i=1,coeff.n-1 do
    if t[i]<=val and val<t[i+1] then
      sum=coeff.a[i]*math.pow(val-t[i], 3)+
          coeff.b[i]*math.pow(val-t[i],2)+
          coeff.c[i]*(val-t[i]) +
          coeff.d[i]
      print(string.format("Eval at %.1f: %.1f",val, sum))
      return sum
    end
  end
  return coeff.d[0]
end


t={1,2,3,4,5}
x={1,3,5,6,9}
local coeff = get_spline_coeffs(t,x)
spline_eval(coeff,1)
spline_eval(coeff,1.5)

--TODO: carray to tensor wrapper
--[[
local torch = require'torch'
local tmp   = torch.FloatTensor(5):zero()
local tt    = carray.float(5)
tt[1] = 5
tt[2] = 64
tt[3] = 3.14
local tbl = tt:table()
print(tbl[1])
tt:tensor( tmp, 3, 2 )
print('Tensor[1:5]:',tmp[1],tmp[2],tmp[3],tmp[4],tmp[5])
--]]
