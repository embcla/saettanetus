#include "ekf.h"

int main(int argc, char *argv[])
{
	int i,j;
	odo test;
	int idseg;
	float dist;
	ekf_data xpre_;	
	ekf_data xpost_;
	float obs[NUM_SENS];
	float u[2];
	
	init_ekf();
	
	load_map(argv[1]);
	
	printf("n_line: %d\n",n_line);

	
//	for (i=0; i<n_line; i++) {
//		for (j=0; j<MAP_COL; j++) {
//			printf("%2.2f\t",map[i][j]);
//		}
//		printf("\n");
//	}

//	test.x=atof(argv[2]);
//	test.y=atof(argv[3]);
//	test.th=atof(argv[4]);	
	
//	ExpSensReading(test,&dist,&idseg);
//	printf("\nWall: %d, Dist: %f\n",idseg+1, dist);
//	printf("x: %f\ty: %f\t th: %f\n",test.x,test.y,test.th);
	

	xpre_.x=0;
	xpre_.y=0;
	xpre_.th=0;	
	
	xpost_.x=100;
	xpost_.y=0;
	xpost_.th=0;
	
	obs[0]=100;
	obs[1]=565.6854;
	obs[2]=400;
	obs[3]=565.6854;
	obs[4]=800;
	
	
	u[0]=0;
	u[1]=0;
	
	EKF_Prediction(xpost_,&xpre_,u);
	printf("x: %f\ty: %f\t th: %f\n",xpre_.x,xpre_.y,xpre_.th);
	
//	xpre_.x=90;//
//	xpre_.y=10;//	
//	xpre_.th=M_PI/40;

//	xpre_.y=480;
//	xpre_.x=650;
	xpre_.x=498;
	xpre_.y=629;
	xpre_.th=0;	
	EKF_Correction(xpre_, &xpost_, obs);
	printf("x: %f\ty: %f\t th: %f\n",xpre_.x,xpre_.y,xpre_.th);	
	printf("x: %f\ty: %f\t th: %f\n",xpost_.x,xpost_.y,xpost_.th);	
	
	
	
	close_ekf();
	return 0;


}
