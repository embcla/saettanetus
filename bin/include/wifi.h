#ifndef _WIFI_H_

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <pthread.h>

#include <fcntl.h>		/* File control definitions */

#include <termios.h>	/* POSIX terminal control definitions */
#include <math.h>		/* Mathematical function definitions */

#include <sys/uio.h>
#include <unistd.h>
#include "console_commands.h"
#include "serial_comm.h"
//#include "xbee.h"

#define WIFI_PORT 5069

#define	MSG_STOP	1

int wifi_fd;
struct sockaddr_in server_addr , client_addr;

struct payload_pkg_trajectory1 {
	float  v;		//!\brief vel for the left motor (ticks)
	float  w;	//!\brief vel for the right motor (ticks)
};

typedef enum{
        SPEED,
        MSG
}pkg_t;

typedef struct {
        float v;
        float w;
} wifi_vel;

typedef struct {
        int msg;        
} wifi_msg;


typedef unsigned char uint8; 

pthread_t thread_wifi;

void init_wifi();

void close_wifi();


#endif
