#include "consensus.h"

void init_consensus()
{
//Temporary variable
    int i;
//Consensus Mutex
    pthread_mutex_init(&mutex_consensus, NULL);
//=====================================
    
//Consensus structures
    consensus_state_others=(float**)malloc(sizeof(float*)*(CONSENSUS_AGENTS));
    for (i=0;i<CONSENSUS_AGENTS;i++)
      /*3 is the dimension of the state space*/
	consensus_state_others[i]=(float*)malloc(sizeof(float)*3);
    consensus_valid=(int*)malloc(sizeof(int)*CONSENSUS_AGENTS);
    for (i=0;i<CONSENSUS_AGENTS;i++)
	consensus_valid[i]=0;
    
//=====================================
}

void calculate_consensus_velocities(float* v_x, float* v_y, float state_x, float state_y)
{
    int i;
    *v_x=0;
    *v_y=0;
    pthread_mutex_lock(&mutex_consensus);
    for (i=0;i<CONSENSUS_AGENTS;i++) {
	if (consensus_valid[i]==1) {
	    consensus_valid[i]=0;
	    *v_x=*v_x+consensus_state_others[i][STATE_X]-state_x;
	    *v_y=*v_y+consensus_state_others[i][STATE_Y]-state_y;
	}
	
    }
    //*v_x=*v_x/
    pthread_mutex_unlock(&mutex_consensus);
    
}

void get_vel_desiderate_consensus(float V_X, float V_Y, float* V_D, float* W_D, float k_v, float k_w)
{
    float ratio;
    *V_D=sqrt(V_X*V_X+V_Y*V_Y);
    if(*V_D<=DESIDERED_VELOCITY_THRESHOLD) {
	*V_D=0;
	*W_D=0;
    }
    else{
	if (V_Y==0 && V_X==0) {
	    *W_D=0;
	    *V_D=0;	
	}
	else {
	    *W_D=atan2(V_Y, V_X)-state[STATE_THETA];
	    if (*W_D > M_PI) {
		*W_D -= (2 * M_PI);
	    }
	    if (*W_D<-M_PI) {
		*W_D += (2 * M_PI);
	    }
	  *V_D=k_v*(*V_D);
	  *W_D=k_w*(*W_D);
	  ratio = min(MAX_LIN_VEL / (modulo(*V_D)), MAX_TURN_RATE / modulo(*W_D));
    
    //printf("INGRESSI: %f %f RATIO %f\n", *v_return, *w_return, ratio);
	    if (ratio < 1.0) {
		*V_D *= ratio;
		*W_D *= ratio;
	    }

	}   
  }
}

void close_consensus()
{
    //Temporary variable
    int i;
//Destroyin' consensus Mutex
    pthread_mutex_destroy(&mutex_consensus);
//=====================================
    
//Destroying consensus structures
    for (i=0;i<CONSENSUS_AGENTS;i++)
	free(consensus_state_others[i]);
    free(consensus_valid);
//=====================================

}
