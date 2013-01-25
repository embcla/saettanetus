#include "viapoint_servoing.h"
//------------------------------------------------------------------------------

//This array is hidden from the other modules
float *robot_state=0;

int init_via_points_servoing() {
    //Viapoints
    traj_num_viapoints = 4;
    //Initializing the served viapoint counter
    traj_num_viapoints_served = 0;
    
    //This is the array containing the viapoints
    if (!goal_trajectory) {

	//Allocation
	goal_trajectory = malloc(sizeof (float) *3 * traj_num_viapoints);

	if (!goal_trajectory) {
	    fprintf(stderr, "Error on goal trajectory memory allocation\n");
	    return -1;
	}
	
	/*First Viapoint*/	
	*goal_trajectory = 90.0;
	*(goal_trajectory + 1) = 0.0;
	*(goal_trajectory + 2) = 0.0;
	/****************/

	/*Second Viapoint*/
	*(goal_trajectory + 3) = 90.0;
	*(goal_trajectory + 4) = 90.0;
	*(goal_trajectory + 5) = 0.0;
	/*****************/

	/*Third Viapoint */
	*(goal_trajectory + 6) = 0.0;
	*(goal_trajectory + 7) = 90.0;
	*(goal_trajectory + 8) = 0.0;
	/*****************/

	/*Fourth Viapoint*/
	*(goal_trajectory + 9) = 0.0;
	*(goal_trajectory + 10) = 0.0;
	*(goal_trajectory + 11) = 0.0;
	/*****************/
	return 0;
    } 
    else
	return -1;
}

void close_via_points_servoing()
{
	free(goal_trajectory);
}

void run_via_points_servoing(float *lin_ref, float *ang_ref){
    
   
    
    if (!robot_state) {
	    robot_state=malloc(sizeof(float)*3);
	    //fprintf(stderr, "Error on robot state memory allocation\n");
	    //return -1;
    }

    //Acquiring the robot state
    get_robot_state(&robot_state);
 
    //Computing the error with the desidered position   
    error_state[STATE_X] = *(goal_trajectory + 3 * traj_num_viapoints_served) - robot_state[STATE_X];
    error_state[STATE_Y] = *(goal_trajectory + 3 * traj_num_viapoints_served + STATE_Y) - robot_state[STATE_Y];
    error_state[STATE_THETA] = atan2(error_state[STATE_Y], error_state[STATE_X]) - robot_state[STATE_THETA];
    
   
    //Computing the distance to the goal
    cartesian_error_norm = sqrt(error_state[STATE_X] * error_state[STATE_X] + error_state[STATE_Y] * error_state[STATE_Y]);

    //Go to the next viapoint if the robot is close to the current one
    if (cartesian_error_norm < VIA_POINT_ERROR_NORM && traj_num_viapoints_served < traj_num_viapoints) {
	traj_num_viapoints_served++;
	//traj_num_viapoints_served %= traj_num_viapoints;

	printf("-----------------------------------------\n");
	printf("SERVED VIAPOINT %d\n", traj_num_viapoints_served);
	printf("-----------------------------------------\n");

	*lin_ref=0.0;
	*ang_ref=0.0;
    }

    //It is arrived to the last waypoint
    if (traj_num_viapoints_served == traj_num_viapoints) {
	printf("ARRIVED\n");
    }

    //Use the pose controller
    if (traj_num_viapoints_served < traj_num_viapoints) {

	pose_controller(error_state, k_v, k_w, lin_ref, ang_ref);

    } 

    else {
	*lin_ref = 0.0;
	*ang_ref = 0.0;
    }
}
//------------------------------------------------------------------------------
