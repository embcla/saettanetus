#---------------------------------------#
#Executable program file (library)
PROGS     = libiphone.so
#Object file
OBJS      = iphone.o
#Installation Directory
INSTDIR = $(prefix)/usr/bin
#LINKER FLAGS
LDFLAGS+= -lm
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
	$(CC) $(OBJS) $(LDFLAGS) -O2 -shared -o $(PROGS) -I../bin/include
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
