#---------------------------------------#
#Executable program file (library)
PROGS     = libviapointservoing.so
#Object file
OBJS      = viapoint_servoing.o
#Installation Directory
INSTDIR = $(prefix)/usr/bin
#---------------------------------------#

#main rule
all: $(PROGS)
#--------#

#replace rule for .c source files
.c.o:	
	$(CC) -fpic -shared -c -o $@ $< -I../bin/include -I../core -I.
#-------------------------------#

#linking library
$(PROGS): $(OBJS) 
	$(CC) $(OBJS) -O2 -shared -o $(PROGS) -I../bin/include -L../bin/librerie
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
