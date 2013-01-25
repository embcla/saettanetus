#include "ekf.h"

// The size must be NUM_SENS_H
const float ANGLE_H[] = {
   -1.5708,
   -1.3963,
   -1.2217,
   -1.0472,
   -0.8727,
   -0.6981,
   -0.5236,
   -0.3491,
   -0.1745,
         0,
    0.1745,
    0.3491,
    0.5236,
    0.6981,
    0.8727,
    1.0472,
    1.2217,
    1.3963,
    1.5708
};

float tanf0, tanfM_PI;


void init_ekf(){
	int i;
	float qN[] = {1, 0.1};		// Input NoiSe Covariance
	float vN[] = {1, 1, 0.01};		// System Noise Covariance
	float pN[] = {10, 10, 10};			

	// Sensor Noise Covariance
	vectR = (float *) malloc(sizeof(float)*NUM_SENS);
	for (i=0;i<NUM_SENS;i++)
#ifdef HOKUYO_SENSOR
		vectR[i]=1000000;
#endif
#ifdef IR_SENSOR
		vectR[i]=10000000;			// [1cm] (good for a laser)
#endif	
	
	tanf0 = tanf(0);
	tanfM_PI = tanf(M_PI);
	
	// Build the Sensor array
	// Note: this should be changed if we use an array of laser beams 
	// deployed over a ring of ray R. Indeed, in this case the equation 
	// to compute the base odometry would be:
	// sa[i][0]=ray*cos(psi)
	// sa[i][1]=ray*sin(psi)
	// sa[i][2]=th_sb
	//	  0		1
	//	-------------
	//	|			|
	//	|	 0--->x	| 2
	//	|			|
	//	-------------
	//	  4		3	
	//	
	// This should be automated
//	for (i=0; i<NUM_SENS; i++) {
//		sd[i][0]=0;
//		sd[i][1]=M_PI/2 -(M_PI/(NUM_SENS-1))*i;
//		sd[i][2]=sd[i][1];
//		printf("%f[%f]\t",sd[i][1],sd[i][1]*180/M_PI);
//	}
//	printf("\n");
//

#ifdef HOKUYO_SENSOR
	// Sensor odo base
	for (i=0; i<NUM_SENS; i++) {
		sbo[i][SX]=RADIUS_H*cos(ANGLE_H[i]);
		sbo[i][SY]=RADIUS_H*sin(ANGLE_H[i]);
		sbo[i][STH]=ANGLE_H[i];		
	}	
#endif

	// SAETTA CONFIGURATION IR
#ifdef IR_SENSOR
	sbo[0][SX]=-7.5;
	sbo[0][SY]=9.0;
	sbo[0][STH]=M_PI/2;		

	sbo[1][SX]=-7.5;
	sbo[1][SY]=-9.0;
	sbo[1][STH]=-M_PI/2;		

	sbo[2][SX]=11;
	sbo[2][SY]=9.5;
	sbo[2][STH]=M_PI/18;		

	sbo[3][SX]=11;
	sbo[3][SY]=-9.5;
	sbo[3][STH]=-M_PI/18;		

	sbo[4][SX]=11;
	sbo[4][SY]=0;
	sbo[4][STH]=0;		
#endif 	
	
	
	// Sampling Time
	Tc=0.25;	// 5 Hz
	
	// Input Noise Covariance Matrix
	// Note: this should be made parametric! :-)
	mDiag(&mQ,U_SIZE,qN);
#ifdef EKF_DEBUG	
	mPrint(mQ,U_SIZE,U_SIZE);
#endif	
	
	// System Noise Covariance Matrix
	mDiag(&mV,X_SIZE,vN);
#ifdef EKF_DEBUG		
	mPrint(mV,X_SIZE,X_SIZE);	
#endif

	mDiag(&mPpred,X_SIZE,pN);	
	mDiag(&mPcorr,X_SIZE,pN);
#ifdef EKF_DEBUG		
	mPrint(mPpred,X_SIZE,X_SIZE);	
	mPrint(mPcorr,X_SIZE,X_SIZE);		
#endif	
	

#ifdef  LOG_EKF_DATA

	ekf_fd = open(LOG_EKF_DATA_FILENAME, O_CREAT | O_WRONLY | O_TRUNC , S_IWUSR | S_IRUSR | S_IRGRP | S_IROTH);
	printf("Ekf FD: %d\n",ekf_fd);
#endif	
	
	
}

void close_ekf() {
	free(vectR);
	free_map();
	mFree(mQ,U_SIZE);
	mFree(mV,X_SIZE);	
	mFree(mPpred,X_SIZE);			
	mFree(mPcorr,X_SIZE);		

#ifdef LOG_EKF_DATA
	close(ekf_fd);
#endif
	
}

int isNumber(char c) {
	// A number is composed of
	// 1) all digits
	// 2) '+' '-' '.'
	return (isdigit(c) || (c=='.') || (c=='-') || (c=='+'))?1:0;
}

// To load the map from a file
int load_map(char *filename) {
	int fd;

	fd = open(filename, O_RDONLY );
#ifdef EKF_DEBUG					
	printf("File Descriptor: %d\n",fd);	
#endif	
	// If the file exists	
	if (fd>0) {
		struct stat fs;
		char *buf, *sbuf;
		int n;
		int i;
		int j;
		
		stat(filename,&fs);
		n=(int)fs.st_size;
#ifdef EKF_DEBUG						
		printf("Size: %d\n",n);
#endif		
		// Allocate buffer to access the file. 
		// Note: This is not the most efficient
		// approach but it is very convenient! :-)
		buf = (char*) malloc(sizeof(char)*n);
		sbuf=buf;	// To access the buffer

		i=read(fd,buf,n);
		
#ifdef EKF_DEBUG						
		printf("#Read: %d\n",i);		
		printf("\n%s\n",buf);
#endif		

		// Parse the number of line

		
		n_line=atoi(sbuf);
		for (; isNumber(sbuf[0]); sbuf++);
		for (; isspace(sbuf[0]); sbuf++);

		// Allocate map structure
		map=(float **) malloc(sizeof(float *)*n_line);			
		for (i=0; i<n_line; i++) {
			*(map+i)=(float *) malloc(sizeof(float)*MAP_COL);
			// Parse line by line			
			for (j=0; j<NUM_COORD; j++){
//				printf("i%d\tj%d\t%f\n",i,j,atof(sbuf));
				*(*(map+i)+j)=atof(sbuf)*MAP_SCALE;
				// You must parse both numbers and point, e.g. 12.10
				for (; isNumber(sbuf[0]) ; sbuf++);
				for (; isspace(sbuf[0]); sbuf++);												
			}
			// To compute the cosine directors let us consider a rect
			// passing for two points P1=(x1,y1) and P2=(x2,y2)
			// a = (y1-y2)
			// b = (x2-x1)
			// c = (x1-x2)*y1 + (y2-y1)*x1
//			*(*(map+i)+A)=*(*(map+i)+Y1)-*(*(map+i)+Y2);
//			*(*(map+i)+B)=*(*(map+i)+X2)-*(*(map+i)+X1);
//			*(*(map+i)+C)=-*(*(map+i)+B) * *(*(map+i)+Y1) -*(*(map+i)+A) * *(*(map+i)+X1);
			// It is the same but way simpler to read :-)
			*(*(map+i)+A)=map[i][Y1]-map[i][Y2];
			*(*(map+i)+B)=map[i][X2]-map[i][X1];;
			*(*(map+i)+C)=-map[i][B]*map[i][Y1]-map[i][A]*map[i][X1];
			printf("%2.4f\t%2.4f\t%2.4f\t%2.4f\n",map[i][0],map[i][1],map[i][2],map[i][3]);
#ifdef EKF_DEBUG			
			printf("segment: %d\ta: %f\tb: %f\tc: %f\n",i+1,map[i][A],map[i][B],map[i][C]);
#endif
		}
		
		// Note: If we just used buf to access the buffer (buf++) 
		// instead of using sbuf then we would not have been able
		// to free the memory!
		free(buf);	
	}
	printf("\n");
	
	int ii;
	int jj;
	for (ii=0; ii<n_line; ii++) {
		for (jj=0; jj<4; jj++){
				printf("%f\t",map[ii][jj]);	
		}
		printf("\n");
	}
	
	close(fd);

}

// To free the map pointer
void free_map() {
	int i;
	
	for (i=0; i<n_line; i++) {
		free(*(map+i));
	}
	free(map);
}

// To check the difference among two numbers
// Note: the 
int checkEq(float m1, float m2) {
//	printf("%f\t%f\t-->%d \n",m1,m2,( fabsf(m1-m2) < NUM_TOL)?1:0);
	return ( fabsf(m1-m2) < NUM_TOL)?1:0;
}


// The formulae to compute the distance between
// a line and a point along a line is used.
// Let us describe the line with {a,b,c}
// and let us describe the point along a line
// by mean of the position and orientation of the
// sensor (lx, ly, th) 
// The distance can be then computed as
// d = | a*lx+b*ly+c| / |a*cos(th)+b*sin(th)|

float compDist(odo o, int idseg) {
	float num;
	float den;
	// 1) Let us check if the sensor is oriented parallel to 
	// the y axis.  check=1 -> sensor is NOT // to axis y
#ifdef EKF_VERBOSE	
	printf("\nCheck 0: sensor y-axis parallel\n");
#endif	
	// The sensor is NOT parallel to the y axis
	// Note: this check should be done with the cos but it
	// is computationally more expensive. By using the angle
	// only pi/2, 3/2*pi, and -pi/2 have been considered.
	// This might not be an exhaustive check if you use
	// any multiple of these numbers x+2*k*pi, k in N.
	if ( !checkEq(o.th,M_PI/2) && !checkEq(o.th,3*M_PI/2) && !checkEq(o.th,-M_PI/2)) {
		// parameter of the line defined by the sensor		
		float as, bs, cs;
#ifdef EKF_VERBOSE		
		printf("Sensor is NOT parallel to the y-axis!\n");		
#endif		
		// Note: if sens_y=0 then tanf=inf.
		// Note: this could passed as a parameter to
		// reduce the computational complexity
		as=-tanf(o.th);		// m= -tan(th)
		bs=1;				// b =1
		cs=-o.y-as*o.x;		// c = -y -m*x
	
		// 1a) Let us check if the sensor intersect the wall
		//     Note: This check fails only if the sensor and the
		//	   wall lie on the same line parallel to the x axis
#ifdef EKF_VERBOSE		
		printf("\nCheck 1: sensor-wall intersection\n");
#endif		
#ifdef EKF_DEBUG						
		printf("Wall: %d\n",idseg+1);
		printf("%f\t%f\t%f\n",o.x,o.y,o.th);		
		printf("[as] %f\n[bs] %f\n[cs] %f\n",as,bs,cs);
		printf("%f\n",(as*map[idseg][X1]+bs*map[idseg][Y1]+cs));
		printf("%f\n",(as*map[idseg][X2]+bs*map[idseg][Y2]+cs));
#endif		
		if ((as*map[idseg][X1]+bs*map[idseg][Y1]+cs) * (as*map[idseg][X2]+bs*map[idseg][Y2]+cs) <= 0) {
#ifdef EKF_VERBOSE			
			printf("Intersect!\n");
#endif			
			
			// 1b) Let us check if the sensor and the wall lie on the same line parallel to 
			// the x-axis. In fact, in this case the previus check would have failed since
			// the parameter as=0, and both the extrems were such that y=sensor.y. Therefore
			// the product would always have been 0 even without an intersaction.
#ifdef EKF_VERBOSE							
			printf("\nCheck 2: sensor and wall are the same line // to x-axis\n");
#endif
#ifdef EKF_DEBUG							
			printf("idseg: %d\n",idseg);
			printf("%f[as]\t%f[-tanf(o)]\t%d[as==-tanf(0)]\n",as, -tanf0, as==-tanf0);
			printf("%f[as]\t%f[-tanf(M_PI)]\t%d[as==-tanf(M_PI)]\n",as, -tanfM_PI, as==-tanfM_PI);
			printf("%f[map[idseg][A]]\t%d[map[idseg][A]==0]\n", map[idseg][A], map[idseg][A]==0);
			printf("!((as==-tanf(0) || as==-tanf(M_PI) ) &&  (map[idseg][A] == 0) ) %d\n",
				   !((as==-tanf0 || as==-tanfM_PI ) &&  (map[idseg][A] == 0) ));
#endif			
			// If both the sensor and the wall are not parallel to the x axis
			if (!((as==-tanf0 || as==-tanfM_PI ) &&  (map[idseg][A] == 0) )) {
#ifdef EKF_VERBOSE				
				printf("They are not the same line parallel to x-axis!\n");
#endif				
				float xp, yp, p_den;
				float cth;
				cth=cos(o.th);
				// 1c) Let us check if the wall is visible from the sensor				
#ifdef EKF_VERBOSE				
				printf("\nCheck 3: wall visibility\n");
#endif				
				// To check the visibility we must compute the intersecting point of
				// the rect passing for the sensor and the wall. 
				// p_den=as*bw-aw*bs
				p_den=as*map[idseg][B] - map[idseg][A]*bs;
				// xp = (bs*cw-bw*cs)/p_den
				xp= (bs*map[idseg][C]-map[idseg][B]*cs)/p_den;
				// yp = (aw*cs-as*cw)/p_den
				yp= (map[idseg][A]*cs-as* map[idseg][C])/p_den;
#ifdef EKF_DEBUG			
				printf("Intersecting Point:\n");
				printf("xl: %f\txp: %f\n",o.x,xp);
				printf("yl: %f\typ: %f\n",o.y,yp);				
				printf("cth: %f\n",cth);
				printf("((cos(th)>0) && (xl<xp)): %d\n",((cth>0) && (o.x<xp)));
				printf("((cos(th)<=0) && (xl >= xp)): %d\n",((cth<=0) && (o.x >= xp)));
#endif			
				// 1) Let us check if the wall is visible from the sensor
				if ( ((cth>0) && (o.x<xp)) || ((cth<=0) && (o.x >= xp)) ) {
#ifdef EKF_VERBOSE					
					printf("Visible!\n");
#endif					
					num = fabs(map[idseg][A]*o.x + map[idseg][B]*o.y + map[idseg][C]);
					den = fabs(map[idseg][A]*cth+map[idseg][B]*sin(o.th));	
					return (float) num/den;					
				}
#ifdef EKF_VERBOSE				
				else {
					printf("Not visibile!\n");
				}
#endif				
			}
#ifdef EKF_VERBOSE			
			else { 
				printf("They are same line parallel to x-axis!\n");
			}
#endif			
		}
#ifdef EKF_VERBOSE		
		else {
			printf("Not Intersect\n");
		}
#endif		
		return MAX_DIST;
	}
	// The sensor is parallel to the y axis
	else {	
		int cond;
#ifdef EKF_VERBOSE		
		printf("Sensor is parallel to the y-axis!\n");
#endif	
		// 2a) Let us check if the sensor intersect the wall
#ifdef EKF_VERBOSE		
		printf("\nCheck 1: sensor-wall intersection\n");
#endif		
		if ( ( (map[idseg][X1] <= o.x) && (map[idseg][X2] >=o.x) ) ||  
			( (map[idseg][X2] <= o.x) && (map[idseg][X1] >=o.x) ) ) {
			float xp, yp;
#ifdef EKF_VERBOSE			
			printf("Intersect!\n");
#endif			
			// Note: If the sensor is parallel to the y axis, then the 
			// x coordinate of the intersection with the wall must be 
			// the same as the x coordinate of the sensor line
			xp = o.x;
			
			// 2b) Let us check if the wall is parallele to the y axis
			//     Note: This check includes also the case the sensor and
			//	   the wall are the same line
#ifdef EKF_VERBOSE			
			printf("\nCheck 2: wall // to y axis\n");
#endif			
			if (map[idseg][B]!=0) {
				float num; den;
#ifdef EKF_VERBOSE				
				printf("Not parallel!\n");				
#endif				
				yp = -(map[idseg][A]*xp+map[idseg][C])/map[idseg][B];
				
				// 2c) Let us check if the wall is visible from the sensor
#ifdef EKF_VERBOSE				
				printf("\nCheck 3: wall visibility\n");
#endif				
				float sth;
				sth=sin(o.th);
				if ( ((sth>0) && (o.y<yp)) || ((sth<=0) && (o.y >= yp)) ) {
#ifdef EKF_VERBOSE									
					printf("Visible!\n");
#endif					
					return fabs(o.y-yp);
				}
#ifdef EKF_VERBOSE				
				else {
					printf("Not visible!\n");
				}
#endif				
			}
#ifdef EKF_VERBOSE			
			else {
				printf("Parallel!\n");
			}
#endif			
		}		
		return MAX_DIST;				
	}
}

// ExpSensReading
int ExpSensReading(odo o, float *dist, int *idseg) {

	int i;
	float d;
	
	d=MAX_DIST;
	*dist=MAX_DIST;
	*idseg=-1;
	
	for (i=0; i<n_line; i++) {
		// Compute distance
		d=compDist(o,i);
#ifdef EKF_VERBOSE						
		printf("Segment: %d\t Distance: %f\n",i+1,d);
#endif		
		if (*dist>d) {
			*dist=d;
			*idseg=i;
		}
	}
	return 0;
}



//void EKF_Prediction_static(ekf_data xprev, ekf_data *xpred, float *u){
//	
//	float ctha,stha;
//	float mA[X_SIZE][X_SIZE];
//	float mB[X_SIZE][U_SIZE];
//	float mP1[X_SIZE][X_SIZE];
//	float mP2[X_SIZE][X_SIZE];
//	
//	int i,j;
//	
//	ctha= cos(xprev.th+ u[W]*Tc/2);
//	stha= sin(xprev.th+ u[W]*Tc/2);
//	
//	// State update
//	xpred->x = xprev.x + u[V]*Tc*ctha;
//	xpred->y = xprev.y + u[V]*Tc*stha;
//	xpred->th = xprev.th + u[W]*Tc;
//	
//	// Populate Matrices
//	// A = [	1	0	-v*tc*sin(th + w*tc/2)
//	//			0	1	v*tc*cos(th + w*tc/2)
//	//			0	0			1				]
//	// row 1
//	mA[0][0]=1;
//	mA[0][1]=0;
//	mA[0][2]=-u[V]*Tc*stha;
//	// row 2
//	mA[1][0]=0;
//	mA[1][1]=1;
//	mA[1][2]=u[V]*Tc*ctha;
//	// row 3
//	mA[2][0]=0;
//	mA[2][1]=0;
//	mA[2][2]=1;
//	
//	// B = [	tc*cos(th + w*tc/2)		-v*tc*sin(th + w*tc/2)*tc/2
//	//			tc*sin(th + w*tc/2)		v*tc*cos(th + w*tc/2)*tc/2
//	//					0						tc					]	
//	// row 1
//	mB[0][0]=Tc*ctha;
//	mB[0][1]=mA[0][2]*Tc/2;
//	// row 2
//	mB[1][0]=Tc*stha;
//	mB[1][1]=mA[1][2]*Tc/2;
//	// row 3
//	mB[2][0]=0;
//	mB[2][1]=Tc;
//	
//	
//	// Covariance Update
//	// Pprev= A*P*A' + B*Q*B'
//	
//	//	A*P*A' closed form
//	//	+-                                                                                                                               -+
//	//	|  a00 (a00 p00 + a02 p20) + a02 (a00 p02 + a02 p22), a11 (a00 p01 + a02 p21) + a12 (a00 p02 + a02 p22), a22 (a00 p02 + a02 p22)  |
//	//	|                                                                                                                                 |
//	//	|  a00 (a11 p10 + a12 p20) + a02 (a11 p12 + a12 p22), a11 (a11 p11 + a12 p21) + a12 (a11 p12 + a12 p22), a22 (a11 p12 + a12 p22)  |
//	//	|                                                                                                                                 |
//	//	|                                                                                                                   2             |
//	//	|              a00 a22 p20 + a02 a22 p22,                         a11 a22 p21 + a12 a22 p22,                     a22  p22         |
//	//	+-                                                                                                                               -+
//	
//	// a00 (a00 p00 + a02 p20) + a02 (a00 p02 + a02 p22)
//	mP1[0][0]=	mA[0][0]*(mA[0][0]*xprev.P[0][0]+mA[0][2]*xprev.P[2][0])+
//				mA[0][2]*(mA[0][0]*xprev.P[0][2]+mA[0][2]*xprev.P[2][2]);
//	// a11 (a00 p01 + a02 p21) + a12 (a00 p02 + a02 p22)
//	mP1[0][1]=	mA[1][1]*(mA[0][0]*xprev.P[0][1]+mA[0][2]*xprev.P[2][1])+
//				mA[1][2]*(mA[0][0]*xprev.P[0][2]+mA[0][2]*xprev.P[2][2]);
//	// a22 (a00 p02 + a02 p22) 	
//	mP1[0][2]=	mA[2][2]*(mA[0][0]*xprev.P[0][2]+mA[0][2]*xprev.P[2][2]);				
//
//
//	// a00 (a11 p10 + a12 p20) + a02 (a11 p12 + a12 p22)
//	mP1[1][0]=	mA[0][0]*(mA[1][1]*xprev.P[1][0]+mA[1][2]*xprev.P[2][0])+
//				mA[0][2]*(mA[1][1]*xprev.P[1][2]+mA[1][2]*xprev.P[2][2]);
//	// a11 (a11 p11 + a12 p21) + a12 (a11 p12 + a12 p22)
//	mP1[1][1]=	mA[1][1]*(mA[1][1]*xprev.P[1][1]+mA[1][2]*xprev.P[2][1])+
//				mA[0][2]*(mA[1][1]*xprev.P[1][2]+mA[1][2]*xprev.P[2][2]);
//	// a22 (a11 p12 + a12 p22)
//	mP1[1][2]=	mA[2][2]*(mA[1][1]*xprev.P[1][2]+mA[1][2]*xprev.P[2][2]);				
//	
//
//	// a00 a22 p20 + a02 a22 p22
//	mP1[2][0]=	mA[0][0]*mA[2][2]*xprev.P[2][0] + mA[0][2]*mA[2][2]*xprev.P[2][2];
//	// a11 a22 p21 + a12 a22 p22
//	mP1[2][1]=	mA[1][1]*mA[2][2]*xprev.P[2][1] + mA[1][2]*mA[2][2]*xprev.P[2][2];
//	// a22^2  p22
//	mP1[2][2]=	mA[2][2]*mA[2][2]*xprev.P[2][2];
//
//
//	
//	//	B*Q*B' closed form
//	//	+-                                                                                 -+
//	//	|            2          2                                                           |
//	//	|     q00 b00  + q11 b10 ,    b00 b01 q00 + b10 b11 q11, b00 b02 q00 + b10 b12 q11  |
//	//	|                                                                                   |	
//	//	|                                       2          2                                |
//	//	|  b00 b01 q00 + b10 b11 q11,    q00 b01  + q11 b11 ,    b01 b02 q00 + b11 b12 q11  |
//	//	|                                                                                   |
//	//	|                                                                  2          2     |
//	//	|  b00 b02 q00 + b10 b12 q11, b01 b02 q00 + b11 b12 q11,    q00 b02  + q11 b12      |
//	//	+-                                                                                 -+	
//	
//	// q00 b00^2  + q11 b10^2
//	mP2[0][0]= mQ[0][0]*mB[0][0]*mB[0][0]+mQ[1][1]*mB[1][0]*mB[1][0];
//	// b00 b01 q00 + b10 b11 q11
//	mP2[0][1]= mB[0][0]*mB[0][1]*mQ[0][0]+mB[1][0]*mB[1][1]*mQ[1][1];
//	// b00 b02 q00 + b10 b12 q11
//	mP2[0][2]= mB[0][0]*mB[0][2]*mQ[0][0]+mB[1][0]*mB[1][2]*mQ[1][1];	
//	
//	// b00 b01 q00 + b10 11 q11
//	mP2[1][0]= mP2[0][1];
//	// q00 b01^2  + q11 b11^2
//	mP2[1][1]= mQ[0][0]*mB[0][1]*mB[0][1]+mQ[1][1]*mB[1][1]*mB[1][1];	
//	// b01 b02 q00 + b11 b12 q11
//	mP2[1][2]= mB[0][1]*mB[0][2]*mQ[0][0]+mB[1][1]*mB[1][2]*mQ[1][1];	
//
//	// b00 b02 q00 + b10 b12 q11
//	mP2[2][0]= mP1[0][2];
//	// b01 b02 q00 + b11 b12 q11
//	mP2[2][1]= mP1[1][2];
//	// q00 b02^2  + q11 b12^2 
//	mP2[2][2]= mQ[0][0]*mB[0][2]*mB[0][2]+mQ[1][1]*mB[1][2]*mB[1][2];		
//	
//	
//	// Pprev= A*P*A' + B*Q*B'
//	for (i=0; i<X_SIZE; i++) {
//		for (j=0; j<X_SIZE; j++) {
//			xpred->P[i][j] = mP1[i][j]+mP2[i][j];
//		}
//	}
//	
//}

void EKF_Prediction(ekf_data xprev, ekf_data *xpred, float *u){
	
	float ctha,stha;
	float **mA;		//[X_SIZE][X_SIZE];
	float **mB;		//[X_SIZE][U_SIZE];
	float **mPprev;	//[X_SIZE][X_SIZE];
	float **mP1;	//[X_SIZE][X_SIZE];
	float **mP2;	//[X_SIZE][X_SIZE];



	printf("V: %f \t W: %f\n",u[0],u[1]);		
	int i,j;
	
	ctha= cos(xprev.th+ u[U_W]*Tc/2);
	stha= sin(xprev.th+ u[U_W]*Tc/2);
	
	// State update
	xpred->x = xprev.x + u[U_V]*Tc*ctha;
	xpred->y = xprev.y + u[U_V]*Tc*stha;
	xpred->th = xprev.th + u[U_W]*Tc;
	if ( xpred->th > 2*M_PI)
		xpred->th -= 2*M_PI;
	
	if ( xpred->th < -2*M_PI)
		xpred->th += 2*M_PI;
	
	// Populate Matrices
	// A = [	1	0	-v*tc*sin(th + w*tc/2)
	//			0	1	v*tc*cos(th + w*tc/2)
	//			0	0			1				]
	mBuild(&mA,X_SIZE,X_SIZE);
	// row 1
	mA[0][0]=1;
	mA[0][1]=0;
	mA[0][2]=-u[U_V]*Tc*stha;
	// row 2
	mA[1][0]=0;
	mA[1][1]=1;
	mA[1][2]=u[U_V]*Tc*ctha;
	// row 3
	mA[2][0]=0;
	mA[2][1]=0;
	mA[2][2]=1;
	
	// B = [	tc*cos(th + w*tc/2)		-v*tc*sin(th + w*tc/2)*tc/2
	//			tc*sin(th + w*tc/2)		v*tc*cos(th + w*tc/2)*tc/2
	//					0						tc					]	
	mBuild(&mB,X_SIZE,U_SIZE);
	// row 1
	mB[0][0]=Tc*ctha;
	mB[0][1]=mA[0][2]*Tc/2;
	// row 2
	mB[1][0]=Tc*stha;
	mB[1][1]=mA[1][2]*Tc/2;
	// row 3
	mB[2][0]=0;
	mB[2][1]=Tc;
	
	
	// Covariance Update (The second term is neglected)
	// Pprev= A*P*A' + B*Q*B' + V
	
	// Allocate matrix
	mBuild(&mPprev,X_SIZE,X_SIZE);
	mBuild(&mP1,X_SIZE,X_SIZE);
	mBuild(&mP2,X_SIZE,X_SIZE);	
	
	// mPprex = xprev.P
	// The pPrev at time k is the mPcorr at time k-1
//	printf("\nmPcorr k-1\n");
	mCopy(mPprev,X_SIZE,X_SIZE,mPcorr,X_SIZE,X_SIZE);	
//	mPrint(mPprev,X_SIZE,X_SIZE);
	
	// mP1 = P*A';
	mMultTr2(mP1,X_SIZE,X_SIZE,mPprev,X_SIZE,X_SIZE,mA,X_SIZE,X_SIZE);
	// mP2 = A*P*A'
	mMultTr2(mP2,X_SIZE,X_SIZE,mA,X_SIZE,X_SIZE,mP1,X_SIZE,X_SIZE);
	// mPpred = A*P*A' + V
	mAdd2(mPpred,X_SIZE,X_SIZE,mV,X_SIZE,X_SIZE);
//	printf("\nmPpred k\n");
//	mPrint(mPpred,X_SIZE,X_SIZE);
	
	// Free memory
	mFree(mA,X_SIZE);
	mFree(mB,X_SIZE);
	mFree(mPprev,X_SIZE);		
	mFree(mP1,X_SIZE);	
	mFree(mP2,X_SIZE);	
	
}


// Data
// 

void EKF_Correction(ekf_data xpred, ekf_data *xcorr, float *obs ){
	
	float expMeas[NUM_SENS];	// Expected measurements
	int idseg[NUM_SENS];		// Segment id for expected measurements
	int idJac[NUM_SENS];		// To store the indexes required to build the Jacobian
	odo s_odo[NUM_SENS];
	
	float cth, sth;
	int i;
	int h_line;				// Number of lines for the Jacobian
	float **mJ;
	float **dMeas;			// z-h

	cth=cos(xpred.th);
	sth=sin(xpred.th);
	

	// Compute the expected measurements
	for (i=0; i<NUM_SENS; i++) {

		// pxs= px + cos(th)*px_sb -sin(th)*py_sb
		s_odo[i].x= xpred.x+cth*sbo[i][SX]-sth*sbo[i][SY];
		// pys= py + sin(th)*px_sb +cos(th)*py_sb
		s_odo[i].y= xpred.y+sth*sbo[i][SX]+cth*sbo[i][SY];
		// pth_s= th + th_sb
		s_odo[i].th= xpred.th+sbo[i][STH];		
		ExpSensReading(s_odo[i], expMeas+i, idseg+i);
#ifdef EKF_DEBUG		
		printf("expM: %f\tidseg: %d\n",expMeas[i],idseg[i]);			
		printf("s_b -- x: %f\ty: %f\t th: %f\n",sbo[i][SX],sbo[i][SY],sbo[i][STH]);		
		printf("s_odo -- x: %f\ty: %f\t th: %f\n",s_odo[i].x,s_odo[i].y,s_odo[i].th);		
#endif
	}
	
	// Let us find out the good observations along with the good expected measurements
	h_line=0;
	for (i=0; i<NUM_SENS; i++) {
		idJac[i]=-1;
		if ( obs[i] < MAX_DIST && expMeas[i]<MAX_DIST && fabs(expMeas[i]-obs[i])<MAX_DIFF_EKF) {
			idJac[h_line]=i;	// Id of the "used" sensors
			h_line++;
		}
		
	}


/*
#ifdef EKF_DEBUG
	printf("\nh_line: %d\n", h_line);
	for (i=0; i<NUM_SENS; i++) {
		printf("%d [%d]\n",idJac[i],i);
	}
#endif	
*/
	
	// Main data Structure for the Jacobian
//	mJ = (float **) malloc(sizeof(float *)* h_line);
	mBuild(&mJ, h_line, X_SIZE);

	
	// Compute the Jacobian
	for (i=0; i<h_line; i++) {
		float num, den, d_num, d_den, s_num, s_den;
		float cths, sths;
		
//		*(mJ+i)=(float *) malloc(sizeof(float)*X_SIZE);
		
		// Compute the Jacobian
		// Let us consider the prediction robot odo x=(px, py, th)
		// Let us consider the sensor base odometry (px_sb, py_sb, th_sb)
		// Let us consider the sensor odometry (px_s, py_s, th_s)
		//			 | a*px_s + b*py_s +c |		  |num|
		// h(x,M) = ------------------------	= ----
		//			| a*cos(th_s) + b*sin(th_s)	  |den|
		// with:
		// pxs= px + cos(th)*px_sb -sin(th)*py_sb
		// pys= py + sin(th)*px_sb +cos(th)*py_sb	
		// pth_s= th + th_sb
		//
		cths=cos(s_odo[idJac[i]].th);
		sths=sin(s_odo[idJac[i]].th);
		
		num = map[idseg[idJac[i]]][A]*s_odo[idJac[i]].x +
			  map[idseg[idJac[i]]][B]*s_odo[idJac[i]].y +
			  map[idseg[idJac[i]]][C];
		s_num = sign(num);
		den = map[idseg[idJac[i]]][A]*cths+ map[idseg[idJac[i]]][B]*sths;
		s_den = sign(den);
		
		// 
		//  d num			d |num|				   d num
		//  ------ = a	--> ------- = sign(num)*  ------ = sign(num) * a
		//	d px			d px				   d px
		// 					
		//  d num			d |num|				   d num
		//  ------ = b	--> ------- = sign(num)*  ------ = sign(num) * b
		//	d py			d py				   d py
		//
		//			d num
		// d_num = ------ = a*(-px_sb*sin(th)-py_sb*cos(th))+b*(px_sb*cos(th)-py_sb*sin(th))
		//			d th
		//			d num
		// d_den = ------ = -a*sin(th_s)+b*cos(th_s)
		//			d th
		// 
		//		 
		d_num = map[idseg[idJac[i]]][A]*(-sbo[idJac[i]][SX]*sth-sbo[idJac[i]][SY]*cth)+
				map[idseg[idJac[i]]][B]* (sbo[idJac[i]][SX]*cth -sbo[idJac[i]][SY]*sth );
		d_den = -map[idseg[idJac[i]]][A]*sths + map[idseg[idJac[i]]][B]*cths;
		
		//	Let us build the Jacobian
		//
		//	d h		  1		d |num|		1			
		//	---- =  ----- * ------- =  ----- * sign(num) * a
		//	d px	|den|	d px	   |den|
		//
	
		mJ[i][0]=s_num*map[idseg[idJac[i]]][A]/fabs(den);
	
		//	d h		  1		d |num|		1			
		//	---- =  ----- * ------- =  ----- * sign(num) * b
		//	d py	|den|	d py	   |den|
		//
	
		mJ[i][1]=s_num*map[idseg[idJac[i]]][B]/fabs(den);;
		
		//	d h		sign(num)*d_num*den - sign(den)*d_den*num
		//	---- =  ------------------------------------------
		//	d tx					den^2
		//
		
		mJ[i][2]=(s_num*d_num*den - s_den*d_den*num)/(den*den);
	}
	
	
//#ifdef EKF_DEBUG
	printf("\nReal\n");
	for (i=0; i<NUM_SENS; i++) {
//		printf("Sensor: %d\tReading: %f\tWall: %d\n",i+1,expMeas[i],(idseg[i]>-1)?idseg[i]+1:idseg[i]);
		printf("%f\n",obs[i]);
	}
	printf("\n");
	
	printf("\nExpected\n");
	for (i=0; i<NUM_SENS; i++) {
//		printf("Sensor: %d\tReading: %f\tWall: %d\n",i+1,expMeas[i],(idseg[i]>-1)?idseg[i]+1:idseg[i]);
		printf("%f\n",expMeas[i]);
	}
	printf("\n");
	printf("Xpre --- x: %f\ty: %f\t th: %f\n",xpred.x,xpred.y,xpred.th);	
//#endif	
	
#ifdef EKF_DEBUG	
	for (i=0; i<h_line; i++){
		printf("Jx: %f\tJy: %f\tJth: %f\n",mJ[i][0],mJ[i][1],mJ[i][2]);
		printf("Sensor odo --- x: %f\ty: %f\t th: %f\n",s_odo[i].x,s_odo[i].y,s_odo[i].th);
		printf("A: %f\tB: %f\tC: %f\n",map[idseg[idJac[i]]][A],map[idseg[idJac[i]]][B],
map[idseg[idJac[i]]][C]);
	}
#endif
	
	float **mP1;
	float **mP2;
	float **mP3;	
	float **mK;
	
	// Let us build the matrix 
	// P (n x n)		J (q x n)
	// P1=mPpred*J^T	(n x n) x (n x q) = (n x q)
	mBuild(&mP1,X_SIZE,h_line);
	mMultTr2(mP1, X_SIZE, h_line, mPpred, X_SIZE, X_SIZE, mJ, h_line, X_SIZE);
//	printf("\nP1=Ppred*J^T\n");	
//	mPrint(mP1,X_SIZE,h_line);	
	
	// P2= J*P1			(q x n) x (n x q) = (q x q)
	mBuild(&mP2,h_line,h_line);
	mMult(mP2,h_line, h_line, mJ, h_line, X_SIZE, mP1, X_SIZE, h_line);
//	printf("\nP2=J*P1\n");
//	mPrint(mP2,h_line,h_line);	
	
	// P2= P2+R			(q x q) + (q x q) = (q x q)
	for (i=0; i<h_line; i++)
		mP2[i][i]+=vectR[idJac[i]];
//	printf("\nP2=P2+R\n");	
//	mPrint(mP2,h_line,h_line);		
	
	
	// P3= inv(P2)		(q x q)
	mBuild(&mP3,h_line,h_line);
	mInv(mP3,h_line,h_line,mP2,h_line,h_line);
//	printf("\nP3=inv(P2)\n");
//	mPrint(mP3,h_line,h_line);		
	
	
	// K= P1*P3			(n x q) x (q x q) =	(n x q)
	mBuild(&mK,X_SIZE,h_line);
	mMult(mK,X_SIZE,h_line,mP1, X_SIZE, h_line, mP3, h_line, h_line);	
//	printf("\nK=P1*P3\n");
//	mPrint(mK,X_SIZE,h_line);
	

	// To build the innovation vector
	mBuild(&dMeas, h_line,1);
	for (i=0; i<h_line; i++) {
		dMeas[i][0]=obs[idJac[i]]-expMeas[idJac[i]];	
	}
//	mPrint(dMeas,h_line,1);	
	
	float **mP4;
	float **mP5;
	float **mInn;
	// Pk = (I-KJ)P


	// K*J
	// (n x q) x (q x n) = (n x n)
	mBuild(&mP4,X_SIZE,X_SIZE);	
	mMult(mP4,X_SIZE,X_SIZE,mK,X_SIZE,h_line,mJ,h_line,X_SIZE);
//	printf("\nmJ\n");
//	mPrint(mJ,h_line,X_SIZE);
	
//	printf("\nmP4\n");
//	mPrint(mP4,X_SIZE,X_SIZE);
	
	
	// (I-K*J)
	mEye(&mP5,X_SIZE);	
	mSub2(mP5,X_SIZE,X_SIZE,mP4,X_SIZE,X_SIZE);
	
	// (I-K*J)*P
	mMult(mPcorr, X_SIZE, X_SIZE, mP5, X_SIZE, X_SIZE, mPpred, X_SIZE, X_SIZE);

//	printf("\nmPcorr\n");
//	mPrint(mPcorr,X_SIZE,X_SIZE);

	// K*vec
	// (n x h_line) x (h_line  x 1)
	mBuild(&mInn, X_SIZE, 1);
	mMult(mInn, X_SIZE, 1, mK, X_SIZE, h_line, dMeas, h_line, 1);

	printf("\ndMeas\n");	
	mPrint(dMeas,h_line,1);
//	int kkk;
//	for (kkk=0; kkk<h_line; kkk++)
//		printf("[%d]\t",idJac[kkk]);
//	printf("\n");
	printf("\nmInn\n");	
	mPrint(mInn,X_SIZE,1);
	

	// Xpost
	xcorr->x = xpred.x + mInn[0][0];
	xcorr->y = xpred.y + mInn[1][0];
	xcorr->th = xpred.th + mInn[2][0];

	if ( xcorr->th > 2*M_PI)
		xcorr->th -= 2*M_PI;
	
	if ( xcorr->th < -2*M_PI)
		xcorr->th += 2*M_PI;
	
	
	printf("\n");
	printf("Xcorr --- x: %f\ty: %f\t th: %f\n",xcorr->x,xcorr->y,xcorr->th);	

	// Free Memory
	mFree(mJ,h_line);
	mFree(mP1,X_SIZE);
	mFree(mP2,h_line);
	mFree(mP3,h_line);
	mFree(mP4,X_SIZE);
	mFree(mP5,X_SIZE);
	mFree(mK,X_SIZE);
	mFree(mInn,X_SIZE);
	mFree(dMeas,h_line);

#ifdef LOG_EKF_DATA
	bzero(log_ekf_buf,LOG_EKF_BUF_LEN);
	sprintf(log_ekf_buf,"%f\t%f\t%f\n",xcorr->x,xcorr->y,xcorr->th);
	printf("[%d] %s\n",strlen(log_ekf_buf),log_ekf_buf);
	printf("Written: %d\n",write(ekf_fd, log_ekf_buf, strlen(log_ekf_buf)));
#endif

}


