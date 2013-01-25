#---------------------------------------#
#Executable program file (library)
PROGS     = libhokuyomiddle.so
#Object file
OBJS      = hokuyomiddle.o
#Installation Directory
INSTDIR = $(prefix)/usr/bin
#FLAGS for the compiling step
CFLAGS += -O2 `c_urg-config --cflags --libs` 
#FLAGS for the linking step
LDFLAGS += `c_urg-config --libs` -lm
#---------------------------------------#

#main rule
all: $(PROGS)
#--------#

#replace rule for .c source files
.c.o:	
	$(CC) $(CFLAGS) -fpic -shared -c -o $@ $< -I../bin/include 
#-------------------------------#

#linking library
$(PROGS): $(OBJS) 
	$(CC) $(OBJS) $(LDFLAGS) -shared -o $(PROGS) -I../bin/include -L../bin/librerie -lpthread
#-------------------------------#

#Installation rule
install: $(PROGS)
	mv *.so ../bin/librerie
	cp *.h ../bin/include
#-------------------------------#

#clean rule
clean:
	rm -f $(PROGS) *.o core
#---------#
