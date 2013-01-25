# Makefile able to recursively execute the Makefiles inside the subdirectories

#List of subdirectories
	SUBDIRS =  ausiliarie netus2pic  sensor  core kalman  xbee_lib joystick  hokuyo_laser probstavoid  wifi main

#Declaration of Phony targets  (cfr. http://www.gnu.org/software/automake/manual/make/Phony-Targets.html)
.PHONY: all Makefile.pre $(SUBDIRS) 

#Default all target
all: Makefile.pre $(SUBDIRS)

Makefile.pre:
	$(SHELL) $@

#Subdir target (make and install)
$(SUBDIRS):
	$(MAKE) -C $@ && $(MAKE) -C $@ install

#Clean target
clean:
	for dir in $(SUBDIRS); do $(MAKE) -C $$dir clean; done
	$(RM) -rf bin/
	
