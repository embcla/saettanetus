#include "ekf.h"

int main(int argc, char *argv[]) {

	int i;

	for (i=0; i < NUM_SENS_H; i++)
		printf("%f\n",ANGLE_H[i]);

}
