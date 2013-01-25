#include "XbeePro_API.h"
#include <stdio.h>

#define DEVICE "/dev/ttyS2"

/* 
 * Param1: number of package to send
 * Param2: Hz
 */
int main(int argc, char *argv[]) {

	int fd, size_load, n, i, dest;
	struct payload_pkg_rob_command *pkg;	// Struct to send
	float freq;
	
	/* Number of packets to send */
	if (argc<3)
		printf("Missing parameters\n");
	else {
		n = atoi(argv[1]);
		freq = 1000000/atoi(argv[2]);
	}
	
	/* Opend the serial port */
	fd = open_port (DEVICE, 1, 1, 0);
	
	/* Load size */
	size_load = sizeof(struct payload_pkg_rob_command);
	
	printf("Load size: %d\n",size_load);
	
	/* Memory allocation */
	pkg = (struct payload_pkg_rob_command *)malloc(size_load);
	
	pkg->command = 'A'; 
	pkg->value =    4;
	
	/* Destination */
	dest = 0x09;
	
	/* Send n packets */
	for (i=0; i<n; i++) {
		
#ifdef VERBOSE
		printf("Sending packet: %d\n",i+1);
#endif	
			
		/* Send the packet with Destination 'dest', packet type 'pkg_time' */
		Send_Data_16(fd, dest, pkg_rob_command, (void *)pkg, size_load, 1);
		
		usleep(freq);
	}
	
	printf("\nFinish.\n");
	
	/* Free memory */
	free(pkg);
	
	/* Close serial port */
	close_port (fd);
	
	return 0;
}
