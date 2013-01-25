#ifndef _LANDER_H_
#define _LANDER_H_

//Constant values define
//===================

//#define 			XBEE
//#define				O_AVOIDANCE_FUZZY
//#define				PRINT_LOOP_TIME
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


//#define		JOYSTICK
//#define 	EKF_LOC
//#define		HOKUYO

//#define CARTESIAN_REGULATOR
//#define OB_AV

#define WIFI

//===================

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
#include <sys/time.h>		// for the gettimeofday
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>


#include "robot_core.h"
// #include "globals.h"
#ifdef EKF_LOC
	#include "ekf.h"
#endif
#include "pic2netus.h"
#include "pic_rel.h"
#include "robot_sensors.h"
#include "xbee.h"
#ifdef HOKUYO
	#include "hokuyomiddle.h"
#endif

#ifdef JOYSTICK
	#include "joystick.h"
#endif

#ifdef WIFI
	#include "wifi.h"
#endif

#ifdef OB_AV
#include "probstavoid.h"
#endif

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

#ifdef DEBUG_TIMING
	int debug_fd;
#endif

//===================
//Variables needed in order to instanciate a new thread

#define LOG_FROM_MAIN_THREAD	// To log cycle timing
#define LOG_FROM_PIC_THREAD	// To log cycle BEFORE signals

pthread_t	thread_main;
pthread_t	thread_pic;
pthread_cond_t  cond;
pthread_mutex_t  mutex;
int log_fd;
int pic_log_fd;


#ifdef XBEE
	pthread_t 	thread_xbee;
	pthread_mutex_t xbee_serial_mutex;
#endif

#ifdef HOKUYO
	pthread_t	thread_laser;
	urg_t *urg;
#endif

#ifdef JOYSTICK
	pthread_t	thread_joy;
	struct payload_pkg_trajectory pay;
#endif

#ifdef WIFI
	pthread_t	thread_wifi;
	struct payload_pkg_trajectory pay;
#endif

#ifdef EKF_LOC
	ekf_data xpred;
	ekf_data xpost;
	float u[2];
	float obsv[NUM_SENS];
#endif


#ifdef CARTESIAN_REGULATOR
#define N_VIA 	5
	float *goal_state;
	float *curr_state;
	int curr_via;
	float viapoints[N_VIA][3];
	int via_done;	
#endif

//===================

/**Termination handler is the function able to catch CTRL_C SIGNAL SIGINT (SIGTERM)
*@param[in] signum Integer representing signal
*/
void 		termination_handler(int signum);
#endif
