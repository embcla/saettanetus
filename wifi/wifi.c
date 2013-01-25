#include "wifi.h"

void init_wifi()
{
        int addr_len, bytes_read;
        struct sockaddr_in server_addr , client_addr;
	//struct payload_pkg_trajectory1 pay;
	int yes;

        if ((wifi_fd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
            perror("Socket");
            //exit(1);
        }

        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(WIFI_PORT);
        server_addr.sin_addr.s_addr = INADDR_ANY;
        bzero(&(server_addr.sin_zero),8);

	yes=0;	
		if (setsockopt(wifi_fd,SOL_SOCKET,SO_REUSEADDR,&yes,sizeof(int)) == -1) { 
			perror("setsockopt"); 
			//exit(1); 
		}  

        if (bind(wifi_fd,(struct sockaddr *)&server_addr,
            sizeof(struct sockaddr)) == -1)
        {
            perror("Bind");
            //exit(1);
        }

	if (listen(wifi_fd, 5) == -1) {
            perror("Listen");
            exit(1);
        }


        addr_len = sizeof(struct sockaddr);
		
	printf("\nUDPServer Waiting for client on port 5000\n");
        fflush(stdout);
}

void close_wifi()
{
	close(wifi_fd);
}

