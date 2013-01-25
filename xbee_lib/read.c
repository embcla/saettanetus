#include "xbee.h"


#define DEVICE "/dev/ttyS1"

int main(int argc, char *argv[]){
	
	int fd;
	int flags;
	int id;
	int iid;
    int sender;
	void *load;
	struct payload_pkg_trajectory *lload;	
int np=0;

		
	if (argc>1)
		fd=open_port(argv[1],0);
	else
		fd=open_port(DEVICE,0);

	if (fd<0)
		return -1;

	while(1) {
		id=pkg_parser(fd,&iid,&load,&sender);
        
 		printf("ID %02x\t",id);
 		printf("IID %02x\n",iid);
 		if (id>-1){
            printf("%d\n",id);
// 			switch(iid){
// 				case pkg_state:
					lload = (struct payload_pkg_trajectory *) load;
					printf("v: %f\n",lload->v);
					printf("w: %f\n",lload->w);
                    free(load);
// 					break;
// 				default:
// 				break;
// 		
// 			}
		np++;
		printf("np: %d\n",np);
		}
		sleep(0.1);
	}
//	process_pkg(fd);

	close_port(fd);
	

	
}

