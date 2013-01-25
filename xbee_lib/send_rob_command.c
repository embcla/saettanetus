#include "xbee.h"
#include <stdio.h>

#define DEVICE "/dev/ttyS1"

/* 
 * Param1: number of package to send
 * Param2: Hz
 */
int main(int argc, char *argv[]) {

	unsigned int packets=0;
	int fd, size_load, n, i, dest;
	struct payload_pkg_trajectory *pkg;	// Struct to send
	float freq;
	//char *comando='A';

	/* Number of packets to send */
	if (argc<3)
		printf("Missing parameters\n");
	else {
		n = atoi(argv[2]);
		freq = 1000000/atoi(argv[3]);
	}
	
	/* Opend the serial port */
	fd = open_port(argv[1], 0);
	
	/* Load size */
	size_load = sizeof(struct payload_pkg_trajectory);
	
	//printf("Load size: %d\n",size_load);
	
	/* Memory allocation */
	pkg = (struct payload_pkg_trajectory *)malloc(size_load);
	
	pkg->v = 0; 
	pkg->w = 0;
	
	/* Destination */
	if ( argc == 5 )
		dest=atoi(argv[4]);	
	else
		dest = 0xFF;
	
	/* Send n packets */
	for (i=0; i<n; i++) {
		
//#ifdef VERBOSE
		printf("Sending packet: %d\n",i+1);
//#endif	
			
		/* Send the packet with Destination 'dest', packet type 'pkg_time' */
		Send_Data_16(fd, dest, pkg_trajectory, (void *)pkg, size_load, 0);
		packets++;
		usleep(freq);
	}
	
	printf("\nFinish.\n");
	printf("Sent packets %d\n",packets);
	/* Free memory */
	free(pkg);
	
	/* Close serial port */
	close_port (fd);
	
	return 0;
}
