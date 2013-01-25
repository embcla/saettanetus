#ifndef _JOYSTICK_H_

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
#include "xbee.h"

#define JOY_PORT 5069

int joy_fd;
struct sockaddr_in server_addr , client_addr;


pthread_t thread_joy;

void init_joystick();

void close_joystick();


#endif
