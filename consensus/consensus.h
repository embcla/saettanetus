#ifndef _CONSENSUS_H_
#define _CONSENSUS_H_

#include <pthread.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "robot_core.h"
#include "funzioni_ausiliarie.h"

///\brief       number of agents involved in consensus
#define         CONSENSUS_AGENTS 15
///\brief	norm error into which next via point is considered
#define		CONSENSUS_ERROR_NORM		30.0
///\brief	minimum desidere velocity able to activate the robot motion
#define         DESIDERED_VELOCITY_THRESHOLD 3.0
///\brief	Buffer able to contain other robots state (required for consensus)
float** consensus_state_others;
///\brief	Buffer that contains 1 when a robot id is valid. A robot id is valid if robot state is arrived.
int* consensus_valid;
///\brief	mutex for locking consensus buffer
pthread_mutex_t mutex_consensus;

/**
Function able to initializa consensus related structures
\brief	Consensus computation

*/
void init_consensus();

/**
Function able to compute velocities according to consensus protocol
\brief	Consensus computation
@param[out] v_x X desidered velocity
@param[out] v_y Y desidered velocity
@param[in] state_x X coordinate of robot
@param[in] state_y Y coordinate of robot
*/
void calculate_consensus_velocities(float* v_x, float* v_y, float state_x, float state_y);

/**
Function able to destroy consensus related structures
\brief	Consensus computation
*/
void close_consensus();

/**
Function able to convert from X and Y component of velocity to linear and angular speed
\brief	V_X V_Y conversion
@param[in] V_X X component of velocity
@param[in] V_Y Y component of velocity
@param[in] k_v linear speed proportional constant
@param[in] k_w angular speed proportional constant
@param[out] V_D linear speed
@param[out] W_D angular speed
*/
void get_vel_desiderate_consensus(float V_X, float V_Y, float* V_D, float* W_D, float k_v, float k_w);
#endif
