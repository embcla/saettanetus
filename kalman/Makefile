#---------------------------------------#
#Executable program file (library)
PROGS     = libkalman.so
#Object file
OBJS      = ekf.o matrix.o
#Installation Directory
INSTDIR = $(prefix)/usr/bin
#---------------------------------------#

#main rule
all: $(PROGS)
#--------#

#replace rule for .c source files
.c.o:	
	$(CC) -fpic -shared -c -o $@ $< -I../bin/include
#-------------------------------#

#linking library
$(PROGS): $(OBJS) 
	$(CC) $(OBJS) -O2 -shared -o $(PROGS) -I../bin/include -L../bin/librerie
#-------------------------------#

#Installation rule
install: $(PROGS)
	mv *.so ../bin/librerie
	cp *.h ../bin/include
	cp ekf_map.txt ../bin/exec
#-------------------------------#

#clean rule
clean:
	rm -f $(PROGS) *.o core
#---------#
