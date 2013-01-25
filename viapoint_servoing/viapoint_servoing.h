#ifndef VIA_POINT_SERVOING_H
#define VIA_POINT_SERVOING_H

#include <stdio.h>
#include <stdlib.h>
#include "robot_core.h"
///\brief	norm error into which next via point is considered
#define		VIA_POINT_ERROR_NORM		4.0
///\brief	goal del robot
float  		goal[3]={-50.0, 30.0,0.0};
//  ____________________________________ PSEUDO TRAIETTORIA ____________________________
///\brief	a set of via points
float  		*goal_trajectory=0;
///\brief	number of via points
unsigned int 	traj_num_viapoints;
///\brief	number of via points served
unsigned int 	traj_num_viapoints_served;
///\brief	cartesian error norm from the current via point
float 		cartesian_error_norm=0;
///\brief	error state respect to the current via point
float 		error_state[3]={0,0,0};

/*Constant value for scaling linear speed*/
const float k_v=0.2;
/*Constant value for scaling angular speed*/
const float k_w=-1.0;
/**
*\brief Via points initialization
*Function to init the via points servoing module
*\return returns 0 if ok and -1 if not
*/
int init_via_points_servoing();

/**
*\brief 
*
*\param[in,out] 
*/
void run_via_points_servoing(float *lin_ref,float *ang_ref);

void close_via_points_servoing();
#endif
