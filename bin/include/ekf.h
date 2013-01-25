#ifndef EKF_H
#define EKF_H

#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>		// open
#include <sys/stat.h>	// stat
#include <ctype.h>		// isdigit
#include <math.h>		// fabs

#include "matrix.h"		// Matrix floating operations

#define IR_SENSOR
//#define HOKUYO_SENSOR

// #define LOG_EKF_DATA		// To log data


#define RAD2DEG(x) ((float) x*180/M_PI )

#ifdef IR_SENSOR
	#define MAX_DIFF_EKF	10	// cm
	#define MAP_SCALE	1	// cm
	#define NUM_TOL		1E-10	// 10^(-10)
	#define MAX_DIST	80	//[cm]
#endif

#ifdef HOKUYO_SENSOR
// 	  mm
//        #define MAX_DIFF_EKF    100	 // mm
//        #define MAP_SCALE       10       // mm (assuming the ekf_map.txt is written in cm!!!!)
//        #define NUM_TOL         1E-10   // 10^(-10)
//        #define MAX_DIST        4000    //[mm]

//	 cm
        #define MAX_DIFF_EKF    10	// cm
        #define MAP_SCALE       1       // cm (assuming the ekf_map.txt is written in cm!!!!)
        #define NUM_TOL         1E-10   // 10^(-10)
        #define MAX_DIST        400     //[cm]

#endif

#define MAP_COL		7		// x1 y1 x2 y2 a b c
#define NUM_COORD	4		// x1 y1 x2 y2

#ifdef IR_SENSOR
	#define NUM_SENS	5		// Define the number of sensors the robot is equipped with
#endif

#ifdef HOKUYO_SENSOR
	#define NUM_SENS	19		// Number of laser beams		
	extern const float ANGLE_H[NUM_SENS]; // Defined in ekf.c
	int ANGLE_IDX[NUM_SENS];
	#define RADIUS_H	1		// mm radious of the laser
#endif

#define X1	0
#define Y1	1
#define X2	2
#define Y2	3
#define A	4
#define B	5
#define C	6

// Indexes for the sensor array
#define SX	0		
#define SY	1
#define STH	2

// State
#define X_SIZE	3
#define X
#define Y
#define TH

// Input
#define U_SIZE	2
#define U_V	0
#define U_W	1

#define sign(x) (x > 0) - (x < 0)

// Ekf Types


typedef struct odo_ odo;
struct odo_ {
	float x;
	float y;
	float th;	
};


typedef struct ekf_data_ ekf_data;
struct ekf_data_ {
	float x;
	float y;
	float th;
};


// Map Variables
float **map;				// Map Structure
float Tc;					// Sampling time
int	n_line;					// Number of segments describing the environment
float sbo [NUM_SENS][3];	// Sensor displacement: (ray, psi, th_sb)
float sa [NUM_SENS][3];		// Sensor base odo, i.e., with respect to the robot odo (0,0,0)
//ekf_data xpred;				// x prediction
//ekf_data xcorr;				// x correction

float **mPpred; // [X_SIZE][X_SIZE];	// Prediction Covariance Matrix
float **mPcorr;	// [X_SIZE][X_SIZE];	// Correction Covariance Matrix
float **mQ;		// [U_SIZE][U_SIZE];	// Input Noise Covariance Matrix
float **mV;		// [X_SIZE][X_SIZE];	// System Noise Covariance Matrix
float *vectR;		// [NUM_SENS];


#ifdef LOG_EKF_DATA
	#include <string.h> 	// required for bzero
	#define LOG_EKF_DATA_FILENAME "log_ekf_data.txt"
	int ekf_fd;
	#define LOG_EKF_BUF_LEN	256
	char log_ekf_buf[LOG_EKF_BUF_LEN];
#endif

// Ekf function
void init_ekf();
void close_ekf();

// Xprev, Xpred, u
void EKF_Prediction(ekf_data, ekf_data *, float *);
// Xpred, Xcorr, obs
void EKF_Correction(ekf_data, ekf_data *, float *);

// Map Function
int load_map(char *filename);
void free_map();

// Expected Sensor Reading
int ExpSensReading(odo , float *, int *);

int checkEq(float , float );
float compDist(odo , int );

//#define ANGLE_H { -1.5708, -1.3963,  -1.2217, -1.0472, -0.8727, -0.6981, -0.5236, -0.3491, -0.1745, 0, 0.1745, 0.3491, 0.5236, 0.6981, 0.8727, 1.0472, 1.2217, 1.3963, 1.5708 };

#endif
