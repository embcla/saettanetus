#---------------------------------------#
#Executable program file
PROGS     = main
#Object file
OBJS      = main.o
#Installation Directory
INSTDIR = $(prefix)/usr/bin
#Compiler flags
CFLAGS += `c_urg-config --cflags`
#Linker flags
LDFLAGS+= -lm -lpthread -lprobstavoid -D_REENTRANT -ljoystick -lxbee -lrobotcore  -lkalman  -lrobot_comm -lausiliarie -lrobot_sensors -lserial_comm -lpic2netus -lhokuyomiddle  `c_urg-config --libs`
#---------------------------------------#

#main rule
all: $(PROGS)
#--------#

#replace rule for .c source files
.c.o:	
	$(CC) -fpic -shared -c $(CFLAGS) -o $@ $< -I../bin/include -L../bin/librerie
#-------------------------------#

#linking
$(PROGS): $(OBJS) 
	$(CC) $(OBJS)  -O2 -I../bin/include -L../bin/librerie $(LDFLAGS) -o $(PROGS)
#-------------------------------#

#Installation rule
install: $(PROGS)
	mv $(PROGS) ../bin/exec/
	cp exec.sh ../bin/exec/
	cp *.txt ../bin/exec/
#-------------------------------#

#clean rule
clean:
	rm -f $(PROGS) *.o core
#---------#
