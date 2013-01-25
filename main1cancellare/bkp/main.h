#ifndef _LANDER_H_
#define _LANDER_H_

//Libraries inclusion
//===================
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <pthread.h>
#include <errno.h>
#include <sys/select.h>


#include "robot_core.h"
#include "console.h"
#include "xbee.h"
#include "consensus.h"
#include "globals.h"
#include "kalman.h"
#include "particle.h"
#include "iphone.h"
#include "pic2netus.h"
#include "pic_rel.h"
#include "robot_sensors.h"
#include "viapoint_servoing.h"
#include "v4l2_capture.h"
#include "orizzontiFuzzy.h"
#include "hokuyomiddle.h"
//===================


//Constant values define
//===================

#define 			XBEE
//#define				O_AVOIDANCE_FUZZY
#define				PRINT_LOOP_TIME
//#define 			WEBCAM
//#define			IPHONE

//#define 			CONSOLE
//#define 			RFID
//#define 			XBEE_SEND
//#define			CONSENSUS_PROTOCOL_ACTIVE
//#define				VIAPOINTS_SERVOING
//#define			MAIN_DEBUG

//#define			USE_KALMAN
//#define			MAGNETO_CALIBRATION
//#define			GET_MAGNETO_BEARING
//#define			USE_PARTICLE_FILTER

//#define 			MAIN_PRINT	//abilito la stampa a video

//===================

//Variables needed for a normal execution of the main thread
//===================

int	xbee_fd_r;
int 	xbee_fd_w;

unsigned char 	c;

char 		file_log[20]="acc.txt";
float		*robot_state=NULL;
struct timeval  *tempi=NULL;
//===================
float w_xbee=0,v_xbee=0;


//===================
//Variables needed in order to instanciate a new thread
pthread_t 	thread_xbee;
pthread_t	thread_main;
pthread_t	thread_laser;
pthread_mutex_t xbee_serial_mutex;
//===================

/**Termination handler is the function able to catch CTRL_C SIGNAL SIGINT (SIGTERM)
*@param[in] signum Integer representing signal
*/
void 		termination_handler(int signum);
#endif
