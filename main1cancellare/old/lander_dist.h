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


#include "robot.h"
#include "console.h"
#include "xbee.h"
#include "consensus.h"
#include "globals.h"
#include "kalman.h"
#include "particle.h"
//===================


//Constant values define
//===================

//#define CONSOLE
//#define CONSOLE_XBEE
//#define XBEE
//#define RFID
//#define XBEE_SEND
//#define	WEBCAM
//#define	CONSENSUS_PROTOCOL_ACTIVE
//#define	CONSOLE_ETHERNET
//#define	VAFFA_CONTROLLER
//#define	MAIN_DEBUG

//#define		USE_KALMAN
//#define		MAGNETO_CALIBRATION
//#define		GET_MAGNETO_BEARING
//#define		USE_PARTICLE_FILTER




#define	MAX_KILLER_COUNTER	400
//===================

//Variables needed for a normal execution of the main thread
//===================

unsigned char c;
unsigned int num_passi;

char carattere[4];
char porta[20]="/dev/ttyS1";
char file_log[20]="acc.txt";
FILE *foutput;
int micro_secondi=0;    
int killer_counter;
int i;
int v_sx;
int v_dx;
int   angle_degrees;
int v1,v2;
int cv=0;
int v_croc;
float distanza, velocita;
float angle_rad;
unsigned int contatore_pacchetti=0;
int	flag_obstacle=FALSE;
int flag_imposed=0;
//===================

//===================
//Variables needed in order to instanciate a new thread
pthread_t thread;
pthread_t thread_w;
pthread_t thread_rfid;
//===================

FILE* statusFile;

/**Termination handler is the function able to catch CTRL_C SIGNAL SIGINT (SIGTERM)
*@param[in] signum Integer representing signal
*/
void termination_handler(int signum);
/**
*Initial state setup done by parsine pose.txt file
*/
void setup_state();
#endif
