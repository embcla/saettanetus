#include "../iphone.h"
#include <fcntl.h>
#include <sys/stat.h>

#define IP "127.0.0.1"
//#define IP "193.204.161.207"
#define DATA "0558119a6989173ae99871ca96a841474605ba98,313.693,0.036,-0.580,-0.761"
#define FILENAME "log.txt"

int main(int argc, char *argv[]) {

	struct sockaddr_in si_other;
	int s, i, slen=sizeof(si_other);
	int n;
	int fd;
	int len;

	char buf[BUFLEN];
    	char *filebuf;
	struct stat fInfo;
	char *fIdx1,*fIdx2;

        if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1) {
		printf("socket\n");
		return -1;
	}


	memset((char *) &si_other, 0, sizeof(si_other));
	si_other.sin_family = AF_INET;
        si_other.sin_port = htons(PORTIPHONE);
        
	if (inet_aton(IP, &si_other.sin_addr)==0) {
		printf("inet_aton() failed\n");
          	return -1;
        }


#ifdef FROM_FILE
	/* Open File */
	fd = open(FILENAME,O_RDONLY);
	fstat(fd,&fInfo);
	printf("Size: %d\n",(int) fInfo.st_size);
	
	filebuf = (char *) malloc(sizeof(char)*fInfo.st_size);
	read(fd,filebuf,fInfo.st_size);

//	printf("%s\n",filebuf);
/*
	printf("Create packet...\n");
	memset(buf,BUFLEN,'\0');
*/

	fIdx1=NULL;
	fIdx2=NULL;
	fIdx1=strstr(filebuf,"ACC");
//	printf("-------------\n%s\n",fIdx1);
	fIdx2=strstr(fIdx1+1,"ACC");
//	printf("-------------\n%s\n",fIdx2);

	len = fIdx2 -  fIdx1;
	printf("Len: %d\n",len);
	
	i=1;
	while (fIdx2!=NULL & fIdx1!=NULL) { 
		len =  fIdx2 -  fIdx1;
		printf("Len: %d\n",len);
		memset(buf,BUFLEN,'\0');
		strncpy(buf,fIdx1,len);
		n =sendto(s, buf, len, 0, (struct sockaddr *) &si_other, slen);
		printf("%s\n",buf);
		fIdx1=strstr(fIdx2+1,"ACC");
		fIdx2=strstr(fIdx1+1,"ACC");
		usleep(100000);
		//sleep(1);
		printf("%d\n",i);
		i++;
	};
	close(fd);       
	
#else
	n =sendto(s, DATA, strlen(DATA), 0, (struct sockaddr *) &si_other, slen);
	
	printf("Data %s\n",DATA);
	if ( n < 0)
            printf("sendto()\n");
  	else
	      printf("Sent %d bytes\n",n);	

#endif	
	
	close(s);
	return 0;
}
	

	/*
	printf("argc: %d\n",argc);	
	if (argc>1)
        	sprintf(buf, "This is packet %s\n", argv[1]);
	else
		sprintf(buf,"Simple void packet\n");
 
	n =sendto(s, buf, strlen(buf), 0, (struct sockaddr *) &si_other, slen);
	*/


