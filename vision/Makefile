#---------------------------------------#
#Executable program file (library)
PROGS     = libvision.so
#Object file
OBJS      = coordinates.o frame.o labelling.o lists.o log.o v4l2_capture.o simple_image.o
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
	$(CC) $(OBJS) -shared -o $(PROGS) -I../bin/include -lm
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
