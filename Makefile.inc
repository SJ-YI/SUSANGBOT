# Makefile Includes
OSTYPE=$(shell uname -s|awk '{print tolower($$0)}')

# Compiler Definitions




ifdef NOTUNE
CXX=g++
CC=gcc
  ifdef WARNINGS
    CXXFLAGS=-fpic -pedantic -Wall
    CFLAGS=-fpic -pedantic -Wall
    else
    CXXFLAGS=-fpic
    CFLAGS=-fpic
  endif
else
CXX=c++
CC=cc
  ifdef WARNINGS
    CXXFLAGS=-O3 -fpic -pedantic -Wall
    CFLAGS=-O3 -fpic -pedantic -Wall
    else
    CXXFLAGS=-O3 -fpic
    CFLAGS=-O3 -fpic
    endif
endif

# Linker Definitions
LD=g++ -shared
#LD=ld -shared -lstdc++
LDFLAGS=-L/usr/local/lib
LIBRT=-lrt
SHLIBEXT=so
STRIP=strip
USE_TORCH=1
INCLUDES=-I/usr/local/include

# Lua 5.1
#INCLUDES+=-I/usr/include/lua5.1 -I/usr/local/include/lua5.1
#LUA_INC=$(pkg-config --cflags-only-I lua5.1)
# LuaJIT 2.1
#LUA_INC=$(pkg-config --cflags-only-I luajit)
LUA_INC=-I/usr/local/include/luajit-2.1
INCLUDES+=$(LUA_INC)

# OS Specific overrides
ifeq ($(OSTYPE),darwin)
  LD=g++
  #LD=ld -macosx_version_min 10.8
	#LDFLAGS=-lc++ -bundle -undefined dynamic_lookup
	LDFLAGS=-lstdc++ -bundle -undefined dynamic_lookup -L/usr/local/lib
  LIBRT=
	#	CFLAGS+=$(pkg-config lua5.1 --cflags)
	# CXXFLAGS+=$(pkg-config lua5.1 --cflags)
else # Linux
#  CFLAGS+= -mtune=native -march=native `pkg-config lua5.1 --cflags`
#  CXXFLAGS+= -mtune=native -march=native `pkg-config lua5.1 --cflags`
ifndef NOTUNE
  CFLAGS+= -mtune=native -march=native
  CXXFLAGS+= -mtune=native -march=native
endif
endif


# Verbose mode check
V=@
ifdef VERBOSE
V=
endif

# From Julia Make.inc
ENDCOLOR="\033[0m"
SRCCOLOR="\033[33m"
CCCOLOR="\033[34m"
CXXCOLOR="\033[32;1m"
LINKCOLOR="\033[34;1m"
BINCOLOR="\033[37;1m"
INFOCOLOR="\033[32m"
CLEANCOLOR="\033[35m"

# Default compiling for modules
all none: $(LIBNAME)
%.o: %.cpp
	@printf '\t%b %b\n' $(CXXCOLOR)CXX$(ENDCOLOR) $(SRCCOLOR)$@$(ENDCOLOR);
	$(V)$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cc
	@printf '\t%b %b\n' $(CXXCOLOR)CXX$(ENDCOLOR) $(SRCCOLOR)$@$(ENDCOLOR);
	$(V)$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	@printf '\t%b %b\n' $(CCCOLOR)CC$(ENDCOLOR) $(SRCCOLOR)$@$(ENDCOLOR);
	$(V)$(CC) $(CFLAGS) $(INCLUDES) -o $@ -c $<
$(LIBNAME): lua_$(LIBNAME).o $(EXTRA_OBJ)
	@printf '\t%b %b\n' $(LINKCOLOR)LINK$(ENDCOLOR) $(BINCOLOR)$@$(ENDCOLOR);
	$(V)$(LD) -o $@.$(SHLIBEXT) $^ $(EXTRA_LINK) $(LDFLAGS)
clean:
	@printf '\t%b %b\n' $(CLEANCOLOR)Cleaning$(ENDCOLOR) $(shell pwd);
	$(V)rm -f *.so *.o $(EXTRA_CLEAN)
