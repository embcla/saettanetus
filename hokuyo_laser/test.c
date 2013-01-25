#include <stdlib.h>
#include <stdio.h>
#include "urg_ctrl.h"
#include "math.h"
#include "ekf.h"

int main(int argc, char * argv[])  {
	
	int i;
	int ret;
	int idx[NUM_SENS];
	urg_t urg;
	long *data;
	int data_max=get_data_max();
	ret = urg_connect(&urg, "/dev/ttyACM0", 115200);


	for (i=0; i< NUM_SENS; i++)
		printf("angle: %f   \tidx: %d\n",ANGLE_H[i],urg_rad2index(&urg,ANGLE_H[i]));


	read_laser_data(urg,&data);



	for (i=0; i<data_max ;i++){
		printf("%ld\n",data);

	}

	printf("\n");

	close_urg_laser(urg);

}
