/*****************************************************************************
*  This program is free software: you can redistribute it and/or modify      *
*  it under the terms of the GNU General Public License as published by      *
*   the Free Software Foundation, either version 3 of the License, or        *
*  (at your option) any later version.                                       *
*                                                                            *
*  This program is distributed in the hope that it will be useful,           *
*  but WITHOUT ANY WARRANTY; without even the implied warranty of            *
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
*  GNU General Public License for more details.                              *
*                                                                            *
*  You should have received a copy of the GNU General Public License         *
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.     *
*****************************************************************************/

/**This library implements a simple version of the fuzzy horizon obstacle avoidance.
 * Actually, in this version only a reactive behaviour is implemented. In future
 * versions I hope to include some improvements based upon the temporary situation 
 * update of the surrounding environment.
 *  @author Attilio Priolo
 *  @file probstavoid.h
 * */
 
/*cfr. http://panzieri.dia.uniroma3.it/FuzzyAppicacations/IA-FuzzyApplications2006.pdf (Italian)*/


#ifndef _PROBSTAVOID_
#define _PROBSTAVOID_



#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "funzioni_ausiliarie.h"
#include "robot_hardware.h"
#include "robot_sensors.h"
#include "hokuyomiddle.h"
#include "xbee.h"

/*
*@define SIDELOBE_THRESHOLD This define is used to slighty modify the attractive horizon
*/
#define SIDELOBE_THRESHOLD 50
/*
*If defined, all horizons are stored in files
*/
//#define LOG_TO_FILE
/**
*@define MAX_INT_FUZZY_MEMBERSHIP This is the maximum membership function with integer programming 
*/
#define MAX_INT_FUZZY_MEMBERSHIP 255
/**
*These defines state which sensor has to be used.
*/
//#define USE_URG_LASER
#define USE_IR
/**
 * @define HORIZON_ELEMENTS This is the number of the horizon elements.
 * */
#define HORIZON_ELEMENTS 48
/**
 * @define SENSOR_MIN_THRESHOLD Minimum value for sensor readings. Under this threshold, the value of "danger" is maximum.
 * 
 * */
#define SENSOR_MIN_THRESHOLD 10.0
/**
 * @define SENSOR_MAX_THRESHOLD Maximum value for sensor readings. Over this threshold, the value of "danger" is zero.
 * 
 * */
#define SENSOR_MAX_THRESHOLD 70.0
/**
 * @define ROBOT_ENCUMBRANCE_RADIANS This is the robot encumbrance in radians. It is necessary to compute the robot inclusion.
 * */
#define ROBOT_ENCUMBRANCE_RADIANS M_PI/6
/**
 * @define DIAGONAL_SENSOR_SHIFT_RADIANS This is the gap (in degrees) between the central and diagonal IR.
 * */
#define DIAGONAL_SENSOR_SHIFT_RADIANS M_PI/6
/**
 * @define SENSOR_TRIANGLE_BASE_RADIANS This is the wideness of the triangle base (in degrees).
 * */
#define SENSOR_TRIANGLE_BASE_RADIANS M_PI/2 
/**
 *@define DESTINATION_TRIANGLE_BASE This const represents the amplitude of the destination triangle base in degrees.
 * */
#define DESTINATION_TRIANGLE_BASE_RADIANS M_PI/2
/**
 *@define KINEMATIC_TRIANGLE_BASE This const represents the amplitude of the kinematic triangle base in degrees.
 * */
#define KINEMATIC_TRIANGLE_BASE_RADIANS M_PI
/**
 *@const IGNORED_LASER_SAMPLES This is the number of the ignored samples of the laser range finder. In this version of
 * the source code, only 180° are taken into account, but the laser range finder can sense 240°. This const is used to 
 * fix this discrepancy.
 * ( 30° / 0.3515625º )
 * */
#define IGNORED_LASER_SAMPLES 86
/**
 *@define DANGER_THRESHOLD This define represents the danger threshold (cm). 
 * */
#define DANGER_THRESHOLD 40
/**
*This variable represents the shifting index for the diagonal IR sensors
*/
int diagonal_ir_shift;
/**
*This is the wideness of the triangle base (in array elements).
*/
int triangle_base_degrees;
/**
\struct Resulting values of the algorithm
*
*/
typedef struct horizon_result
{
	/**Linear speed*/
	float v_ref_hor_fuz;
	/**Angular speed*/
	float w_ref_hor_fuz;
	/**Boolean if robot is blocked or not*/
	int stuck;
}fuzzy_horizon_result;

/**
*@typedef uint8_ Unsigned 8bit integer (0-255)
*/
//typedef unsigned char   uint8_;

/**This integer represents the step of the laser range finder according to HORIZON_ELEMENTS*/
int laser_step;
/**
 * Attractive horizon. 
 * */
uint8_ attractive_horizon[HORIZON_ELEMENTS];
/**
 * Attractive horizon file. 
 * */

FILE *attractive_horizon_file;
/**
 * Not(Attractive horizon). 
 * */
uint8_ not_attractive_horizon[HORIZON_ELEMENTS];
/**
 * Not(Attractive horizon) file. 
 * */
FILE *not_attractive_horizon_file;
/**
 * Repulsive horizon. 
 * */
uint8_ repulsive_horizon[HORIZON_ELEMENTS];
/**
 * Repulsive horizon file. 
 * */
FILE *repulsive_horizon_file;
/**
 * Kinematic horizon. 
 * */
uint8_ kinematic_horizon[HORIZON_ELEMENTS];
/**
 * Kinematic horizon file. 
 * */
FILE *kinematic_horizon_file;
/**
 * Not(Kinematic horizon). 
 * */
uint8_ not_kinematic_horizon[HORIZON_ELEMENTS];
/**
 * Not(Kinematic horizon) file. 
 * */
FILE *not_kinematic_horizon_file;
/**
 * Not robot inclusion horizon (based on its encumbrance).
 * */
uint8_ inclusion_horizon[HORIZON_ELEMENTS];
/**
 * Not robot inclusion horizon (based on its encumbrance) file.
 * */
FILE *inclusion_horizon_file;
/**
 * Resulting array.
 * */
uint8_ union_horizon[HORIZON_ELEMENTS];
/**
 * Resulting array file.
 * */
FILE *union_horizon_file;
/**
 * Function to create the repulsive horizon basing upon ir readings. 
 * */
void create_repulsive_horizon_ir();

/**
 * Function to create the repulsive horizon basing upon laser readings. 
 * */
void create_repulsive_horizon_laser();

/**
 * Function to create the attractive horizon. It takes the destination angle as input. 
 *@param[in] destination_angle Destination relative angle (according to current robot bearing). It has to be contained into the interval [-PI,PI]
 * */
void create_attractive_horizon(float destination_angle);

/**
 * This function must not be used outside this library. It is intended only for debugging purposes.
 * */
void create_triangle(int base,int index,uint8_ max_triangle_value,uint8_ *horizon);

/**
 * Function to create the kinematic horizon.
 * */	
void create_kinematic_horizon();

/**
 * This function constructs negative attractive and kinematic horizons 
 */
void construct_negative_horizons(); 
/**
 * Module Init
 */
void init_probstavoid_module();
/**
 * Function to compute the union horizon
 */
void compute_resulting_horizon();
/**
 * Function to compute the inclusion horizon
 */
void compute_inclusion();
/**
 * Module Close
 */
void close_probstavoid_module();
/**
*Main algorithm function
*@param v_ref Linear speed (by other modules).
*@param w_ref Angular speed (by other modules).
*@param result Result (filled by this function).
*/
void apply_fuzzy_horizon_algorithm(float *v_ref,float *w_ref,fuzzy_horizon_result *result);
#endif
