# Master Makefile to compile all Lua/C++ libraries
CWD= $(shell pwd)
PWD= $(subst /,\/,$(CWD)/Player/Lib)
# .PHONY: all none clean transform modules modules2 modules3 robots
.PHONY: all none clean transform modules modules2 robots

# all none: modules modules2 modules3 robots
all none: modules modules2 robots


%:
	@cd Robots/Transform && make && cd $(CWD)
	@printf "  %s %s\n" Making $@;
	@cd Robots/$@ && make && cd $(CWD)
	@printf "  %s %s\n" Configuring $@;
	@rm -f $(CWD)/Config/Config.lua
	@ln -s $(CWD)/Config/$@/Config_$@.lua $(CWD)/Config/Config.lua
#else
#	@echo "WE HAVE ADDITIONAL VARIABLE "$(VAR)
#	@ln -s $(CWD)/Config/$@/$(VAR)/Config_$@_$(VAR).lua $(CWD)/Config/Config.lua
#endif

	@rm -f $(CWD)/Webots/worlds
	@ln -s $(CWD)/Robots/$@/Webots/worlds $(CWD)/Webots/worlds
# @rm -f $(CWD)/RunRobot/*.lua
# @rm -f $(CWD)/RunRobot/*.sh
# @rm -f $(CWD)/RunRobot/*.py
	@mkdir -p $(CWD)/RunRobot/
	@rm -f $(CWD)/RunRobot/*
	@ls $(CWD)/RunRobot/
# @ln -s $(CWD)/Robots/$@/RunRobot/*.* $(CWD)/RunRobot/
	@ln -s $(CWD)/Robots/$@/RunRobot/* $(CWD)/RunRobot/
	@ln -s $(CWD)/Run/*.lua $(CWD)/RunRobot/
	@rm -f $(CWD)/Memory/*.lua
	@ln -s $(CWD)/Robots/$@/Memory/*.lua $(CWD)/Memory/
	@cd $(CWD) && luajit gen_start_script.lua

modules:
	@for dir in `ls Modules`; do \
	printf "  %b \n" $$dir ; \
	$(MAKE) -C Modules/$$dir clean; \
	$(MAKE) -C Modules/$$dir; \
	done

modules2:
	@for dir in `ls Modules2`; do \
	printf "  %b \n" $$dir ; \
	$(MAKE) -C Modules2/$$dir clean; \
	$(MAKE) -C Modules2/$$dir; \
	done

# Must make transform first
robots:
	@cd Robots/Transform && make && cd $(CWD)
	@for dir in `ls Robots`; do \
	printf "  %b \n" $$dir ; \
	$(MAKE) -C Robots/$$dir clean; \
	$(MAKE) -C Robots/$$dir; \
	done

clean:
	@for dir in `ls Modules`; do \
	printf "  %b \n" $$dir ; \
	$(MAKE) -C Modules/$$dir clean; \
	done
	@for dir in `ls Modules2`; do \
	printf "  %b \n" $$dir ; \
	$(MAKE) -C Modules2/$$dir clean; \
	done
	# @for dir in `ls Modules3`; do \
	# printf "  %b \n" $$dir ; \
	# $(MAKE) -C Modules3/$$dir clean; \
	# done
	@for dir in `ls Robots`; do \
	printf "  %b \n" $$dir ; \
	$(MAKE) -C Robots/$$dir clean; \
	done
	@rm -f $(CWD)/Config/Config.lua
	@rm -f $(CWD)/RunRobot/*.lua
	@rm -f $(CWD)/RunRobot/*.sh
	@rm -f $(CWD)/RunRobot/*.py
	@rm -f $(CWD)/Memory/*.lua
	@rm -f $(CWD)/start_robot.sh
	@rm -f $(CWD)/webots_start.sh
