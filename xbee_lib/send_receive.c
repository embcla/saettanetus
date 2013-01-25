#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "xbee.h"


#define device "/dev/ttyS1"

pthread_t       thread_xbee_r;
pthread_t       thread_xbee_w;

int fd_r;
int fd_w;

int sent;
int received;

void* tf_write(void *args) {
	struct payload_pkg_state state;

	printf("Open Port to Write\n");
//	fd_w = open(device, O_WRONLY | O_NOCTTY | O_NDELAY   );
	fd_w = open_port_write(device,1);	
	printf("Write fd: %d\n",fd_w);


	state.x=0;
	state.y=0;
	state.th=0;

	sleep(5);
	sent=0;
	while (1) {
		printf("Sending...\n");
		Send_Data_16(fd_w, 0xff, pkg_state, (void *) &state, sizeof(struct payload_pkg_state), NO_ACK);
		sent++;
		sleep(1);
	}	

}


void* tf_read(void *args) {
	void *load;
	int id;
	int iid;
	int sender;
	int flags;

	printf("Open Port to Read\n");
	//fd_r = open(device, O_RDONLY | O_NOCTTY | O_NDELAY   );
	fd_r = open_port_read(device,1);
	printf("Read fd: %d\n",fd_r);

         flags=fcntl(fd_r, F_GETFL);
         fcntl(fd_r,F_SETFL,flags & ~O_NONBLOCK);

	sleep(1);

	received=0;
	while(1) {

             id=pkg_parser(fd_r,&iid,&load,&sender);

             printf("ID %02x\t",id);
             printf("IID %02x\n",iid);
             if (id>-1){
             	printf("%d\n",id);
                        switch(iid){
                                case pkg_trajectory:
                                        printf("v: %f\n",( (struct payload_pkg_trajectory* )load)->v);
                                        printf("w: %f\n",( (struct payload_pkg_trajectory* )load)->w);
                                        free(load);
                                        break;
                                case pkg_state:
                                        printf("x: %f\n",((struct payload_pkg_state *) load)->x);
                                        printf("y: %f\n",((struct payload_pkg_state *) load)->y);
                                        printf("th: %f\n",((struct payload_pkg_state *) load)->th);
                                        free(load);
                                        break;

                              default:
	                              break;
              
                        }

		}
		received++;
	}
}




void termination_handler(int signum) {
	printf("Closing ports\n");
	printf("Sent: %d\n",sent);
	printf("Received: %d\n", received);
	close_port(fd_w);
	close_port(fd_r);
}


int main(){

    pthread_attr_t attr_xbee_r;
    pthread_attr_init(&attr_xbee_r);
    pthread_attr_setdetachstate(&attr_xbee_r, PTHREAD_CREATE_DETACHED);
    pthread_create(&thread_xbee_r, &attr_xbee_r, &tf_read, NULL);

    pthread_attr_t attr_xbee_w;
    pthread_attr_init(&attr_xbee_w);
    pthread_attr_setdetachstate(&attr_xbee_w, PTHREAD_CREATE_DETACHED);
    pthread_create(&thread_xbee_w, &attr_xbee_w, &tf_write, NULL);


    if (signal(SIGINT, termination_handler) == SIG_IGN) signal(SIGINT, SIG_IGN);
    if (signal(SIGHUP, termination_handler) == SIG_IGN) signal(SIGHUP, SIG_IGN);
    if (signal(SIGTERM, termination_handler) == SIG_IGN) signal(SIGTERM, SIG_IGN);

    pause();

}
