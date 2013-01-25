#---------------------------------------#
#Executable program file (library)
PROGS     = libsincos.so
#Object file
OBJS      = lookupsincos.o
#Installation Directory
INSTDIR = $(prefix)/usr/bin
#Compiling flags
CFLAGS +=  
#Linking flags
LDFLAGS += 
#---------------------------------------#

#main rule
all: $(PROGS)
#--------#

#replace rule for .c source files
.c.o:	
	$(CC) -fpic -shared -c $(CFLAGS) -o $@ $< -I../bin/include 
#-------------------------------#

#linking library
$(PROGS): $(OBJS) 
	$(CC) $(OBJS) -O2 -shared $(LDFLAGS) -o $(PROGS) -I../bin/include -L../bin/librerie -lpthread
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
