#include "iphone.h"

	// Socket File Descriptor


/* Initialize the Server */
int init_iphone() {
	sd=-1;	
	/* Data Structure */
	struct sockaddr_in addr;
	int bufsize;

	iphoneLogging = fopen("logIphone.txt", "w");

	/* create socket */ 
	if ( (sd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) { 
		printf("Socket creation error\n"); 
		return -1; 
	}	 

	// int n = 1024 * 1024;
	bufsize=BUFLEN;
	if (setsockopt(sd, SOL_SOCKET, SO_RCVBUF, (void *) &bufsize, sizeof(bufsize)) == -1) {
	  // deal with failure, or ignore if you can live with the default size
	  printf("Socket buf reduced\n");  
	}

	/* initialize address */ 
	//memset((void *)&addr, 0, sizeof(addr));     	/* clear server address */ 
	addr.sin_family = AF_INET;                  	/* address type is INET */ 
	addr.sin_port = htons(PORTIPHONE);                  	/* daytime port is 13 */ 
	addr.sin_addr.s_addr =  htonl(INADDR_ANY);   	/* connect from anywhere */ 
	bzero(&(addr.sin_zero),8);
	
	/* bind socket */ 
	if (bind(sd, (struct sockaddr *)&addr, sizeof(struct sockaddr)) < 0) { 
		printf("bind error\n"); 
		return -1; 
	}

	return 0;

}

/* Close the Server */
void close_iphone() {
	close(sd);
	fclose(iphoneLogging);
}

/*  getData */
int getIPhonePacket(char *buf) {
	int n, slen;
	struct sockaddr_in si_other;

//	buf = (char *) malloc(sizeof(char)*BUFLEN);
	memset(buf,BUFLEN,'\0');	
#ifndef IPHONEBLOCK
	n = recvfrom(sd, buf, BUFLEN, MSG_DONTWAIT, (struct sockaddr *) &si_other, &slen);
#else
	n = recvfrom(sd, buf, BUFLEN, 0, (struct sockaddr *) &si_other, &slen);
#endif
//	printf("Read: %d\n",n);
	// printf("Received packet from %s:%d\nData: %s\n\n", 
//			inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port), buf);
	
	return n;
}

void parseIPhoneData(char *buf, struct dataIphone *data) {
	char *idx;

	/* Timestamp */
	idx=strstr(buf,",");
	idx++;
	data->ts=atof(idx);
	/* X Acc*/
	idx=strstr(idx,",");
	idx++;
	data->xacc=atof(idx);
	/* y Acc*/
	idx=strstr(idx,",");
	idx++;
	data->yacc=atof(idx);
	/* z Acc*/
	idx=strstr(idx,",");
	idx++;
	data->zacc=atof(idx);


}


int getIPhoneAcc(struct dataIphone *data) {

	int n;
	char buf[BUFLEN];

	n=getIPhonePacket(buf);
	if (n>0)
		parseIPhoneData(buf,data);
	else
		data=NULL;

	return n;
}

void getIPhoneRefVel(float *lin_ref,float *ang_ref)
{
  struct dataIphone dataI;
  float qx, qy, qz;
  
  if (getIPhoneAcc(&dataI) > 0) {
		
		printf("[%3.3f] Xacc: %1.3f\tYacc: %1.3f\tZacc: %1.3f\n", dataI.ts, dataI.xacc, dataI.yacc, dataI.zacc);
		fprintf(iphoneLogging, "[%3.3f] Xacc: %1.3f\tYacc: %1.3f\tZacc: %1.3f\n", dataI.ts, dataI.xacc, dataI.yacc, dataI.zacc);

		/*Thresholding data*/
		if (fabs(dataI.yacc) > SENSIBILITY_THRESHOLD)
		    qy = dataI.yacc - (dataI.yacc / fabs(dataI.yacc)) * SENSIBILITY_THRESHOLD;
		else
		    qy = 0;

		if (fabs(dataI.zacc) > SENSIBILITY_THRESHOLD) {
		    qz = dataI.zacc - (dataI.zacc / fabs(dataI.zacc)) * SENSIBILITY_THRESHOLD;
		    if (fabs(qz) > MAXIMUM_SLOPE)
			qz = qz / fabs(qz) * MAXIMUM_SLOPE;
		}
		else
		    qz = 0;

		if (fabs(dataI.xacc) > SENSIBILITY_THRESHOLD)
		    qx = dataI.xacc - (dataI.xacc / fabs(dataI.xacc)) * SENSIBILITY_THRESHOLD;
		else
		    qx = 0;
		/*-----------------*/

	      

		*lin_ref = -(qz) * IPHONE_LINEAR_NORMALIZATION_FACTOR;
		*ang_ref = -qy;

  }
  else {
		printf("No IPhone data\n");
		*lin_ref = 0.0;
		*ang_ref = 0.0;
  }
}
