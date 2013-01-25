#include "main.h"

void main_init() {

    int i;
    float starting_speed = 0.0;
    init_robot();
    flag = 0;
    robot_state = malloc(sizeof (float) *3);

    for (i = 0; i < LEN_PIC_BUFFER; i++) {
        set_vel_2_array(pic_buffer[i], starting_speed, starting_speed);
    }

    write(pic_fd, pic_message_reset_steps_acc, PACKET_TIMING_LENGTH + 1);
    tcflush(pic_fd, TCOFLUSH);
    sync();
    steps_anomaly = 0;


#ifdef OB_AV
	init_probstavoid_module();
#endif

#ifdef CARTESIAN_REGULATOR
	viapoints[0][0]=38;
	viapoints[0][1]=119;
	viapoints[0][2]=0;

	viapoints[1][0]=98;
	viapoints[1][1]=161;
	viapoints[1][2]=0;


	viapoints[2][0]=187;
	viapoints[2][1]=179;
	viapoints[2][2]=0;

	viapoints[3][0]=158;
	viapoints[3][1]=238;
	viapoints[3][2]=0;

	viapoints[4][0]=187;
	viapoints[4][1]=268;
	viapoints[4][2]=0;
	curr_via=0;
	via_done=0;

#endif

#ifdef HOKUYO
	init_urg_laser(&urg,ON_DEMAND);
#endif

#ifdef JOYSTICK
	init_joystick();
#endif

#ifdef WIFI
	init_wifi();
#endif
	
#ifdef EKF_LOC	
	load_map("ekf_map.txt");
        init_ekf();

#ifdef HOKUYO_SENSOR
    for (i=0; i< NUM_SENS; i++){
        ANGLE_IDX[i]=urg_rad2index(urg,ANGLE_H[i]);
        //printf("angle: %f   \tidx: %d\n",ANGLE_H[i],urg_rad2index(urg,ANGLE_H[i]));
    }
#endif
	
        xpost.x=50;
	xpost.y=45;
	xpost.th=M_PI/2;	
	
	xpred.x=0;
	xpred.y=0;
	xpred.th=0;
	
#endif	
	
}

//=========================================================================================================

//=========================================================================================================

void termination_handler(int signum) {

    int i;


    pthread_mutex_destroy(&mutex);  
    pthread_cancel(thread_pic);

    // Clean the bufffer	
    tcflush(pic_fd, TCIFLUSH);
    tcflush(pic_fd, TCOFLUSH);	
 

   pthread_cancel(thread_main);


#ifdef LOG_FROM_PIC_THREAD
	close(pic_log_fd);
#endif

#ifdef LOG_FROM_MAIN_THREAD
    close(log_fd);
#endif

#ifdef OB_AV
    close_probstavoid_module();
#endif

 #ifdef JOYSTICK
	pthread_cancel(thread_joy);
	close_joystick();
#endif

#ifdef WIFI
	pthread_cancel(thread_wifi);
	close_wifi();
#endif

#ifdef EKF_LOC
	close_ekf();
#endif


#ifdef HOKUYO
	close_urg_laser(urg);
#endif


#ifdef CARTESIAN_REGULATOR
	free(goal_state);
	free(curr_state);
#endif

   fprintf(fp_log, "\nNum packet data: correct  %d wrong  %d sent_wrong: %d \n", num_packet_data_ok, num_packet_data_wrong, num_packet_sent_wrong);
    fflush(fp_log);
    fclose(fp_log);
    printf("scritto\n");
    set_vel_2_array(pic_buffer[pic_last_vel_2_send], 0, 0);
    write(pic_fd, pic_buffer[pic_last_vel_2_write], PACKET_SPEED_LENGTH + 1);
    sync();
 
    exit(0);
}
//=========================================================================================================

//=========================================================================================================

void setup_termination() {
    if (signal(SIGINT, termination_handler) == SIG_IGN) signal(SIGINT, SIG_IGN);
    if (signal(SIGHUP, termination_handler) == SIG_IGN) signal(SIGHUP, SIG_IGN);
    if (signal(SIGTERM, termination_handler) == SIG_IGN) signal(SIGTERM, SIG_IGN);
}
//=========================================================================================================



//=========================================================================================================
#ifdef HOKUYO
void* tf_laser_read(void* thread_arg)
{
  while(1)
  {
  read_laser_data(urg);
  usleep(100000);
  }
}
#endif
//=========================================================================================================

#ifdef JOYSTICK
void* tf_joy_read(void* thread_arg)
{
	int bytes_read, addr_len;
	int sin_size, connected;
	
	pay.v=0;
	pay.w=0; 
	addr_len = sizeof(struct sockaddr);
	
	sin_size = sizeof(struct sockaddr_in);
	
	
	while(1) {
        connected = accept(joy_fd, (struct sockaddr *)&client_addr,&sin_size);
		
        printf("\n I got a connection from (%s , %d)",
			   inet_ntoa(client_addr.sin_addr),ntohs(client_addr.sin_port));
		
		while (1)
		{
						
			bytes_read=recv(connected,&pay,sizeof(struct payload_pkg_trajectory),0);
			
			if (bytes_read==0) {
				pay.v=0;
				pay.w=0;				
				break;
			}

#ifdef JOYSTICK
			// Set Velocity	for the PIC			
			set_robot_speed(&pay.v, &pay.w);
#endif
	
						
		}
	}


}

#endif

#ifdef WIFI
void* tf_wifi_read(void* thread_arg){
setup_termination();
	int bytes_read, addr_len;
	int sin_size, connected;
	uint8 *sizes= ( uint8* ) malloc ( sizeof ( uint8 ) *2 );
	uint8 *buffer;
    	/* Receive message from client */
	unsigned int size;
	size = sizeof ( struct sockaddr_in );

	
	pay.v=0;
	pay.w=0; 
	addr_len = sizeof(struct sockaddr);
	
	sin_size = sizeof(struct sockaddr_in);
	
	
	while(1) {
	

		connected = accept(wifi_fd, (struct sockaddr *)&client_addr,&sin_size);
	        printf("\n I got a connection from (%s , %d)\n", inet_ntoa(client_addr.sin_addr),ntohs(client_addr.sin_port));
		bytes_read=1;
		while (bytes_read>0){
			bytes_read = recv ( connected, ( void* ) sizes, sizeof ( uint8 ) *2, 0 );

		        if ( bytes_read <= 0 ){
		            printf ( "error: %s\n",strerror ( errno ) );
			    //exit(1); 
			}       
            		buffer=malloc ( sizes[1] );
            		bytes_read = recv ( connected, buffer, sizes[1], 0 );

            		if ( connected < 0 ){
		                printf ( "error: %s\n",strerror ( errno ) );
 				exit(1);               
		        }

                        switch ( sizes[0] ){
                		case SPEED:
				if (bytes_read==0) {
					((wifi_vel*)buffer)->v=0;
					((wifi_vel*)buffer)->w=0;																
					}
					else
{
						// Set Velocity	for the PIC			
						set_robot_speed(&(((wifi_vel*)buffer)->v),&(((wifi_vel*)buffer)->w));
printf("RECEIVED DATA: linear %9.7f   angular %9.7f\n",((wifi_vel*)buffer)->v,((wifi_vel*)buffer)->w);
}
					break;
		                case MSG:
					if(((wifi_msg*)buffer)->msg==MSG_STOP){
						//termination_handler(0);
						//raise(SIGINT);
pid_t process_id=getpid();
kill(process_id,SIGINT);	
					}
                    			break;
                	}
           	}
        }
        close ( connected );
	free ( sizes );
}
#endif

//=========================================================================================================
void* tf_main(void* thread_arg) {

    float angle;
    float v_ref, w_ref;
	int i;
    long int counter=0;

	

#ifdef EKF_LOC	
    int data[NUM_SENS];	// provided by ekf.h	
#endif
	
#ifdef HOKUYO
	int maximum_laser_data=get_data_max();	
#endif
	
    setup_termination();

    angle = 0.0;
    gyro_integral = 0.0;
    gyro_sup_integral = 0.0;
    float a,b;

#ifdef CARTESIAN_REGULATOR
	float v_return, w_return;
	int get_goal=0;
	fuzzy_horizon_result fh;
#endif

    a=4;
    b=0;
//    set_robot_speed(&a,&b);

 #define RAD2DEG(x) ((float) x*180/M_PI )

	struct timeval tvb, tva;


#ifdef LOG_FROM_MAIN_THREAD
	int cycle;
	char debug_buf[24];
	log_fd=open("timing_main.txt",O_CREAT | O_WRONLY | O_TRUNC , S_IWUSR | S_IRUSR | S_IRGRP | S_IROTH);
#endif

#ifdef HOKUYO_SENSOR

#endif


	// The timing of the first cycle is not allined due to the 
	// fact that the two threads are not setup exactly at the
	// same time
	pthread_cond_wait(&cond, &mutex);
	pthread_mutex_unlock(&mutex);		


#ifdef CARTESIAN_REGULATOR
	goal_state = (float *) malloc(sizeof(float)*3);
	curr_state = (float *) malloc(sizeof(float)*3);
	goal_state[0]=viapoints[curr_via][0];
	goal_state[1]=viapoints[curr_via][1];
	goal_state[2]=0;
#endif

	while(1){
		counter++;
		pthread_cond_wait(&cond, &mutex);
		printf("Alive! [%d]\n",counter);
		pthread_mutex_unlock(&mutex);
		gettimeofday(&tvb,NULL);
	
		//
		get_robot_state(&robot_state);

#ifdef CARTESIAN_REGULATOR
		// Cartesian Regulator
//		get_goal=cartesian_controller(robot_state, goal_state, .1, .1, &v_return, &w_return);		
		if (!via_done) {
			get_goal=cartesian_controller(curr_state, goal_state, 0.3, 0.25, &v_return, &w_return);		
			printf("v:%f  \tw:%f\n",v_return,w_return);		
			if (get_goal==1){
				curr_via++;
				goal_state[0]=viapoints[curr_via][0];
				goal_state[1]=viapoints[curr_via][1];
				get_goal=0;
			}
			if (curr_via==N_VIA) {
				via_done=1;
				v_return=0;
				w_return=0;	
			}	
		}
#ifdef OB_AV		
		apply_fuzzy_horizon_algorithm(&v_return,&w_return,&fh );
		set_robot_speed(&fh.v_ref_hor_fuz, &fh.w_ref_hor_fuz);
#else
		set_robot_speed(&v_return, &w_return);
#endif

#endif
	
#ifdef EKF_LOC
#ifdef HOKUYO_SENSOR		
		// To select the subset of laser beams of interests
		// These are defined in ekf.c by ANGLE_H		
		for (i=0; i<NUM_SENS; i++) {
			obsv[i]=data_laser[ANGLE_IDX[i]]/10;
		}
#endif
		
#ifdef IR_SENSOR		
		for (i=0; i<NUM_SENS; i++){
			obsv[i]= *(ir->range+i);
//			printf("%f\n ", obsv[i]);
		}		
		
#endif		
		
		// Get the latest control input
//		u[0]=last_v_ref;
//		u[1]=last_w_ref;
		// Prediction
		//EKF_Prediction(xpost, &xpred, u);
		// Ekf Correction
		//EKF_Correction(xpred, &xpost, obsv);		


		//printf("%f\t%f\t%f\n",xpred.x, xpred.y, RAD2DEG(xpred.th));
		//printf("%f\t%f\t%f\n",xpost.x, xpost.y, RAD2DEG(xpost.th));	
		
#ifdef CARTESIAN_REGULATOR
		curr_state[0]=xpost.x;
		curr_state[1]=xpost.y;
		curr_state[2]=xpost.th;	
#endif

#endif

		//printf("%f\t%f\t%f\n",state[0],state[1],RAD2DEG(state[2]));
		printf("lu: %9.7f   \tlw: %9.7f\n",last_v_ref,last_w_ref);


		gettimeofday(&tva,NULL);



                if (tva.tv_sec==tvb.tv_sec){
			//printf("%ld\n", tva.tv_usec-tvb.tv_usec);
#ifdef LOG_FROM_MAIN_THREAD 
	           	cycle=tva.tv_usec-tvb.tv_usec;
#endif
       		}
                else {
                        int delta;
                        delta = 1000000-tvb.tv_usec;
                        //printf("%ld\n",tva.tv_usec+delta);
#ifdef LOG_FROM_MAIN_THREAD
                	cycle=tva.tv_usec+delta;
#endif
		}
#ifdef LOG_FROM_MAIN_THREAD
                sprintf(debug_buf,"%ld\n",cycle);
                write(log_fd, debug_buf,strlen(debug_buf));
#endif	
	}



    //========================================================================
    while (0) {


	
//		packet_type = analizza_pacchetto();
                       
        if (packet_type == LOAD_PACKET_ANALYZED) {
            //============================================            
            pthread_mutex_lock(&mutex_fp);
            fflush(fp_log);
            pthread_mutex_unlock(&mutex_fp);
            flag = 0;
        }

		
#ifdef EKF_LOC		
#ifdef HOKUYO_SENSOR		
		// To select the subset of laser beams of interests
		// These are defined in ekf.c by ANGLE_H		
		for (i=0; i<NUM_SENS; i++) {
			obsv[i]=data_laser[ANGLE_IDX[i]];
		}
#endif
		
#ifdef IR_SENSOR		
		for (i=0; i<NUM_SENS; i++){
			obsv[i]= *(ir->range+i);
//			printf("%f\n ", obsv[i]);
		}		
		
#endif		
		
		// Get the latest control input
		u[0]=last_v_ref;
		u[1]=last_w_ref;
		
		// EKF Prediction
//		EKF_Prediction(xpost, &xpred, u);
		// Ekf Correction
//		EKF_Correction(xpred, &xpost, obsv);		


//	printf("%f\t%f\t%f\n",xpred.x, xpred.y, RAD2DEG(xpred.th));
//	printf("%f\t%f\t%f\n",xpost.x, xpost.y, RAD2DEG(xpost.th));	
//	printf("%f\t%f\t%f\n",state[0],state[1],RAD2DEG(state[2]));
//	printf("u: %f   \tw: %f\n",u[0],u[1]);
//	printf("lu: %f   \tlw: %f\n",last_v_ref,last_w_ref);


		
#endif
		
		
    }
    //========================================================================

}

void* tf_read(void *args) {

    int oldstate;
    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, &oldstate);

    
    setup_termination();
    void *load;
    int nRead;
    uint8_ buf[3];
    uint8_ *pkg;
    int len;
    int iid;
    int id;
    int sender;
    struct payload_pkg_rob_command *pkg_rob_comm;
    struct payload_pkg_trajectory  *pach;

    xbee_packet_received = 0;
    xbee_ack_received = 0;

    while (1) {

        printf("------------\n");
        
        
        iid = -1;

        /* Look for a command */
        /* Lock the mutex on the job queue. */
        nRead = readport(xbee_fd_r, buf, 3);

        if (nRead == 3) {
		if (buf[0] == 0x7e) {
                printf("%02x\t%02x\n", buf[1], buf[2]);
                len = MSB_LSB_2_INT(buf + 1, 2);
                if (len > 0) {
                    printf("Len pkg: %d\t", len);
                    pkg = (uint8_ *) malloc(sizeof (uint8_)*(len + 1));
                    nRead = readport(xbee_fd_r, pkg, len + 1);
                    sender = pkg[2];
                    printf("Sender: %2x\t%d\n", pkg[2], sender);
                    ++xbee_packet_neighborhood[sender];
                    printf("Read pkg: %d\n", nRead);
                    id = pkg_dispatcher(pkg, nRead, &iid, &load);
                    if (id == rx_pack_16 && iid == pkg_1) {
                        printf("---PARSER----\n");
                        printf("Long: %ld\n", ((struct payload_pkg1 *) load)->i);
                        printf("Float: %f\n", ((struct payload_pkg1 *) load)->j);
                    }
		xbee_packet_received++;
                    switch (iid) {

			   
                        case pkg_state:
                            //printf("pkg_state\n");
			    break;
			case pkg_trajectory:
			    pach=(struct payload_pkg_trajectory*)load; 
			    v_xbee=pach->v;
			    w_xbee=pach->w;
			    fprintf(stderr,"VELOCITA' RICEVUTE DA MATLAB: line=%f ang=%f\n",pach->v,pach->w);
			    break;
                        case pkg_webcam_data:
                            printf("pkg_webcam_data\n");
                            break;

                        case pkg_rob_command:
                            printf("pkg_rob_command\n");
                            pkg_rob_comm = (struct payload_pkg_rob_command *) load;
                            console_current_command = pkg_rob_comm->command[0];
                            console_current_value1 = pkg_rob_comm->value;

                            switch (console_current_command) {
                                case 'v':
                                    printf("Command v\tValue: %d\n", console_current_value1);
                                    set_vel_2_array(pic_buffer[pic_last_vel_2_send], console_current_value1, console_current_value1);
                                    pic_last_vel_2_send %= LEN_PIC_BUFFER;
                                    break;

                                case 'p':
                                    printf("Command p\tValue: %d\n", console_current_value1);
                                    break;

                                case 'd':
                                    printf("Command d\tValue: %d\n", console_current_value1);
                                    break;
                                case 't':
                                    printf("Command d\tValue: %d\n", console_current_value1);
                                    break;
                                case 's':
                                    printf("Command s\tValue: %d\n", console_current_value1);
                                    break;
                                default:
                                    break;
                            }

                            break;

                        default:
                            break;

                    }


                    free(pkg);
                }
                printf("Len: %d\n", len);

            }
#ifdef VERBOSE_XBEE
            printf("nRead(in): %d\t", nRead);
            print_pkg(buf, nRead);
#endif
        }
#ifdef VERBOSE_XBEE
        printf("nRead(out): %d\t", nRead);
        print_pkg(buf, nRead);
#endif
    }
    pthread_exit(NULL);

}



void* tf_pic2netus(void *args) {
	
	
	int i;
	unsigned char buf[256];
	struct timeval tvb, tva;	
	int counter;
	int byte_read;
	int tot_byte_read;
	int an_ret;

#ifdef LOG_FROM_PIC_THREAD
	int cycle;
	unsigned char debug_buf[128];
	pic_log_fd=open("timing_pic.txt",O_CREAT | O_WRONLY | O_TRUNC , S_IWUSR | S_IRUSR | S_IRGRP | S_IROTH);
#endif	


#ifdef LOG_FROM_PIC_THREAD	
	cycle=0;
#endif
	
	counter=0;

	tcflush(pic_fd, TCIFLUSH);
	tcflush(pic_fd, TCOFLUSH);

	// Hook cycle
	do {	
		read(pic_fd, buf, 1);
	}
	while(buf[0]!=0x0A);
	
	while(1) {
		memset(buf,'\0',128);
		counter++;
		byte_read=0;
		printf("Cycle: %d\n",counter);
             // Get the start
                do {
                        read(pic_fd, buf, 1);
                }
                while(buf[0]!='S');
                byte_read++;
 		gettimeofday(&tva,NULL);

 	        if (tva.tv_sec==tvb.tv_sec){
        	        printf("%ld  \t%d\n", tva.tv_usec-tvb.tv_usec, byte_read);
#ifdef LOG_FROM_PIC_THREAD
                	cycle=tva.tv_usec-tvb.tv_usec;
#endif
        	}
        	else {
                	int delta;
                	delta = 1000000-tvb.tv_usec;
                	printf("%ld  \t%d\n",tva.tv_usec+delta, byte_read);
#ifdef LOG_FROM_PIC_THREAD
                	cycle=tva.tv_usec+delta;
#endif
        	}

#ifdef LOG_FROM_PIC_THREAD
                sprintf(debug_buf,"%ld  \t%d\n",cycle,byte_read);
                write(pic_log_fd, debug_buf,strlen(debug_buf));
#endif


              
		// Get the whole pkg    
                do {
                        read(pic_fd,buf+byte_read,1);
                        byte_read++;
                }
                while(*(buf+byte_read-1)!='\n');
/*                for (i=0; i< byte_read; i++) {
                        printf("%02x\t",buf[i]);
                //      printf("%c\t",buf[i]);
                }       
                printf("\n");
*/                tot_byte_read=byte_read;
//		write(pic_fd, pic_message_timing, 4);
		analizza_pacchetto(buf,byte_read);
		byte_read=0;
                // Get the whole pkg    
                memset(buf,'\0',128);
                byte_read=0;
                do {
                        read(pic_fd,buf+byte_read,1);
                        byte_read++;
                }
                while(*(buf+byte_read-1)!='\n');
/*                for (i=0; i< byte_read; i++) {
                        printf("%02x\t",buf[i]);
                //      printf("%c\t",buf[i]);
                }
                printf("\n");
*/                tot_byte_read+=byte_read;
		an_ret=analizza_pacchetto(buf,byte_read);
		gettimeofday(&tvb,NULL);
	
		if (an_ret == LOAD_PACKET_ANALYZED )
			pthread_cond_signal(&cond);
		else
			printf("problem!\n");

	}
	
}





//=========================================================================================================

int main(int argc, char* argv[]) {

    printf("INIT\n");

    main_init();


    fp_log = fopen(file_log, "w");
    setup_termination();

	pthread_mutex_init(&mutex, NULL);


    pthread_attr_t attr_main;
    pthread_attr_init(&attr_main);
	// Main thread
	pthread_create(&thread_main, &attr_main, &tf_main, NULL);
	pthread_create(&thread_pic, &attr_main, &tf_pic2netus, NULL);	

#ifdef JOYSTICK
    pthread_create(&thread_joy, &attr_main, &tf_joy_read, NULL);	
#endif

#ifdef WIFI
    pthread_create(&thread_wifi, &attr_main, &tf_wifi_read, NULL);	
#endif

#ifdef HOKUYO
    pthread_create(&thread_laser, &attr_main, &tf_laser_read, NULL);	
#endif
	
    printf("FD Pic: %d\n", pic_fd);
    printf("FD Xbee Write: %d\n ", xbee_fd_w);
    printf("FD Xbee Read: %d\n ", xbee_fd_r);
    pthread_join(thread_main, NULL);
    return 0;
}
//=========================================================================================================



