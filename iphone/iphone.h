#ifndef IPHONE_H
#define IPHONE_H

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

/**Server port for iphone data receiving*/
#define PORTIPHONE 10552
/**Buffer length for iphone data receiving*/
#define BUFLEN 280// 164
/**Define the minimum accelerometer value to activate the robot motion*/
#define SENSIBILITY_THRESHOLD 0.2
/**This is the maximum value reacheable by the x-y-z axis projections of iphone gravity vector*/
#define MAXIMUM_SLOPE 0.7
/**This normalizaction factor is able to normalize the maximum robot linear speed*/
#define IPHONE_LINEAR_NORMALIZATION_FACTOR 30

/**\brief Data structure representing iphone accelerometer data
*\struct dataIphone
*/
struct dataIphone {
	double ts;
	double xacc;
	double yacc;
	double zacc;
};

/**Iphone log file*/
FILE*           iphoneLogging;

/**Socket descriptor*/
int sd=-1;

/* To handle the UDP socket server */

/**\brief Init Iphone-related data structures
*\param[out] Socket descriptor 
*/
int  init_iphone();
/**\brief Close Iphone Server socket
*
*/
void close_iphone();
/**\brief Get the data packet coming from Iphone
*\param[in] buf Buffer to write on
*\param[out] number of read byte
*/
int  getIPhonePacket(char *buf);
/**\brief Function to parse Iphone data into a structure
*\param[in] buf received buffer
*\param[in] data parsing data result
*/
void parseIPhoneData(char *buf, struct dataIphone *data);
/**\brief Main function in iphone library. this function provides an abstraction layer for remaining parsing functions.
*\param[in] data Data structure to write on
*\param[out] number of read bytes
*/
int getIPhoneAcc(struct dataIphone *data);
/**\brief Function to extract the desidered speed.
*This function is used to extract the linear and angular reference speed from iphone accelerometers data.
*\param[out] lin_ref Linear speed
*\param[out] ang_ref Angular speed
*/
void getIPhoneRefVel(float *lin_ref,float *ang_ref);
#endif
