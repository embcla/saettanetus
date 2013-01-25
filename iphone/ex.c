#include "iphone.h"


void parseDataIphone(char *buf, struct dataIphone *data) {
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

int main() {

	int i,n;
	char buf[BUFLEN];
	struct dataIphone data;

	/* Initialize Server */
	if (serverIPhoneInit(1)<0)
		return -1;

	i=1;
	/* Get DAta*/
	while (1) {
		n=getIPhonePacket(buf);
		if (n>0) {
			printf("Read: %d\n",n);
			printf("Data: %s\n",buf);
			parseDataIphone(buf,&data);
			printf("%d -- [%3.3f] Xacc: %1.3f\tYacc: %1.3f\tZacc: %1.3f\n",i,data.ts,data.xacc,data.yacc,data.zacc);
		}
//		else
//			printf("Data not available\n");		
		
		usleep(1000000);
	//	sleep(1);
		i++;
	}

	/* Close Server */
	closeIPhoneServer();

	if (buf!=NULL)
		free(buf);
	
	return 0;

}

