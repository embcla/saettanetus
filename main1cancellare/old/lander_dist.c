// source code

#include "lander_dist.h"


float  camera_x;
float  camera_y;
pthread_mutex_t mutex_camera_kalman;


	
//=========================================================================================================
void mutex_init() {
    pthread_mutex_init(&mutex_ir, NULL);
    pthread_mutex_init(&mutex_gyro, NULL);
    pthread_mutex_init(&mutex_acc, NULL);
    pthread_mutex_init(&mutex_magneto, NULL);
    pthread_mutex_init(&mutex_state, NULL);
    pthread_mutex_init(&mutex_fp, NULL);
    pthread_mutex_init(&mutex_consensus, NULL);
    pthread_mutex_init(&m_analizza_pacchetto, NULL);
	pthread_mutex_init(&mutex_v_ref, NULL);
	pthread_mutex_init(&mutex_w_ref, NULL);
    pthread_cond_init(&cv_analizza_pacchetto, NULL);
    flag=0;
}
//=========================================================================================================

//=========================================================================================================
void termination_handler(int signum) {
    
	int i;
#ifdef MAIN_DEBUG
    printf("\nNum packet data: correct  %d wrong  %d sent_wrong: %d \n", num_packet_data_ok, num_packet_data_wrong, num_packet_sent_wrong);
    printf("Num xbee packet sent: %d, ack received: %d \n", xbee_packet_sent, xbee_ack_received);
    printf("Num xbee packet received: %d \n", xbee_packet_received);
	for(i=0;i<MAX_NEIGHBORS;i++){printf("%d: %d\n",i,xbee_packet_neighborhood[i]);}
#endif

  //  pthread_cancel(thread);
 //   pthread_cancel(thread_w);

//    pthread_cancel(thread_rfid);

    set_vel_2_array(pic_buffer[pic_last_vel_2_send], 0, 0);
    write(pic_fd, pic_buffer[pic_last_vel_2_write], PACKET_SPEED_LENGTH + 1);
    sync();
#ifdef CONSENSUS_PROTOCOL_ACTIVE
    destroy_consensus_structures(&consensus_state_others,&consensus_valid);
#endif
    tcflush(pic_fd, TCIOFLUSH);
    if (pic_fd > 0) {
        tcsetattr(pic_fd, TCSANOW, &oldtio);
    }
    close(pic_fd);
#ifdef	XBEE
    close_port(console_fd);
#endif

	fprintf(fp_log, "\nNum packet data: correct  %d wrong  %d sent_wrong: %d \n", num_packet_data_ok, num_packet_data_wrong, num_packet_sent_wrong);
    fprintf(fp_log, "Num xbee packet sent: %d, ack received: %d \n", xbee_packet_sent, xbee_ack_received);
    fprintf(fp_log, "Num xbee packet received: %d \n", xbee_packet_received);
	for(i=0;i<MAX_NEIGHBORS;i++){fprintf(fp_log, "%d: %d\n",i,xbee_packet_neighborhood[i]);}
	fflush(fp_log);
	fclose(fp_log);
	printf("scritto\n");
#ifdef MAIN_DEBUG
    printf("\nExit\n");
#endif
#ifdef USE_KALMAN
    distruggiTabellaMagneto(pMag);
    fclose(foutput);
    distruggiDatiKalman(dKalman);
#endif

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
void *tf_write(void* param_write) {
    struct sched_param p;
    int dest, size_load;
    float stato_s[3];
    float array[100];
    int n_pkg = 1000;
    float freq = 1000000/10;
    struct payload_pkg_webcam_data *webcam_data;
    float dati[6];
    int k;
    // Priority
    //p.sched_priority = sched_get_priority_max(SCHED_RR);
    //p.sched_priority = 20;
    //pthread_setschedparam(pthread_self(), SCHED_OTHER, &p);
    //p.sched_priority= sched_get_priority_max(SCHED_OTHER);
    //pthread_setschedparam(pthread_self(), SCHED_OTHER, &p);
    for(k=0; k<MAX_NEIGHBORS;k++){xbee_packet_neighborhood[k]=0;}
    xbee_packet_sent = 0;	
    size_load = sizeof (float)*3;
    dest = 0xff;
    usleep(3000000);
    while (1) {
        pthread_mutex_lock(&mutex_state);
        Send_Data_16(console_fd, dest, pkg_state, (void *)state, size_load, NO_ACK);
        pthread_mutex_unlock(&mutex_state);
        ++xbee_packet_sent;
        usleep(freq);
    }
    pthread_exit(NULL);
}

//=========================================================================================================

//=========================================================================================================
void* tf_rfid(void* thread_arg){
	
	unsigned char rfid_message[100];
	unsigned char command[]="\r001400\r";;
	unsigned char rfid_stop[]="\n";
	int n;
	while(1){
		printf("Writing on RFID ====================================\n");
		write(rfid_fd, command, 100);
		//stampa_pacchetto(command,8);
		n=read(rfid_fd, rfid_message,100);
		if(n>5){printf("%s \n", rfid_message );}
		//write(rfid_fd, rfid_stop, 1);
	    tcflush(rfid_fd, TCIFLUSH);
		sync();
		usleep(500000);
	}

}
//=========================================================================================================


//=========================================================================================================
void* tf_read(void* thread_arg){


	float *puntatore;
	int id, iid, sender;
	void *load;
    struct payload_pkg_rob_command *pkg;
    struct payload_pkg_trajectory *pkg_traj;
	struct sched_param p;
    struct payload_pkg_webcam_data *webcam;
    int j;
	uint8 *buffer;
	int N = 1000;
	int n_read, len, byte_parsati;
	uint8 *pkg_parsato;
    // Priority
    //p. sched_priority = sched_get_priority_max(SCHED_RR);
    //p. sched_priority = 20;
    //pthread_setschedparam(pthread_self(), SCHED_OTHER, &p);
	xbee_packet_received=0;
	xbee_ack_received = 0;
	buffer = (uint8 *) malloc(sizeof(uint8)*N);
   	while (1) {

		//printf("tf_r\n");
		usleep(100000);
		//continue;

		byte_parsati = 0;
		id = 0xff;
		iid = 0xff;
		pthread_mutex_lock (&m_io);
//		n_read = read (console_fd,buffer,N); 
		pthread_mutex_unlock (&m_io);
		while (byte_parsati<n_read-1) {
			if (buffer[byte_parsati] == 0x7e) {
				byte_parsati++;
				len = MSB_LSB_2_INT (buffer+byte_parsati,2);
				if ( (len>0) & (len<=MAX_LOAD_SIZE) & (byte_parsati+2+len+1<=n_read) ) {
					byte_parsati+=2;
					pkg_parsato = (uint8 *) malloc(sizeof(uint8)*(len+1)); // there is also 
					memcpy (pkg_parsato, buffer+byte_parsati, len+1);
					id = pkg_dispatcher (pkg_parsato,len+1,&iid,&load);
					sender = pkg_parsato[2];			
					++xbee_packet_neighborhood[sender];
        			if (id==rx_pack_16) {
            			switch (iid) {
							//==========================================================================
			                case pkg_state:
			                    puntatore= (float *) load;
								#ifdef CONSENSUS_PROTOCOL_ACTIVE
                    			pthread_mutex_lock(&mutex_consensus);
			                    consensus_state_others[sender][STATE_X]=*(puntatore+STATE_X);
			                    consensus_state_others[sender][STATE_Y]=*(puntatore+STATE_Y);
			                    consensus_state_others[sender][STATE_THETA]=*(puntatore+STATE_THETA);
			                    consensus_valid[sender]=1;
			                    pthread_mutex_unlock(&mutex_consensus);
								#endif
			                    break;
								//=======================================================================
							//===========================================================================
			                case pkg_webcam_data:
			                    webcam = (struct payload_pkg_webcam_data *)load;
								//printf("ricevuto\n");
								pthread_mutex_lock(&mutex_camera_kalman);
								if(	webcam->n_obj==1 && webcam->data[1]>-30 && webcam->data[1]<200 && webcam->data[0]>-30 && webcam->data[0]<200){
			                    	for (j=0; j<webcam->n_obj; j++){
										//printf("NUM: %d\n", webcam->n_obj);
										camera_x=webcam->data[j*2+1];
										camera_y=webcam->data[j*2];
										//printf("valori: %f %f \n", camera_x, camera_y);
										#ifdef MAIN_DEBUG
			                        	printf("Obj: %d \t x: %f y: %f\n", j, webcam->data[j*2], webcam->data[j*2+1]);
										#endif
									}
								}
								//else{printf("-- NV --\n");}
								pthread_mutex_unlock(&mutex_camera_kalman);
			                    break;
								//=======================================================================
							//==========================================================================
							case pkg_trajectory:
			                    pkg_traj = (struct payload_pkg_trajectory *) load;
								get_vel_motori(pkg_traj->v, pkg_traj->w, &v_m1, &v_m2);
						        calcola_velocita(v_m1, TRISTEPPING, &pulse_m1);
			        			calcola_velocita(v_m2, TRISTEPPING, &pulse_m2);
								pulse_m1=min_int(max_int(-1000, pulse_m1), 1000);
						        pulse_m2=min_int(max_int(-1000, pulse_m2), 1000);
						        pulse_m1=pulse_m1/modulo(pulse_m1)*(1024-modulo(pulse_m1));
						        pulse_m2=pulse_m2/modulo(pulse_m2)*(1024-modulo(pulse_m2));
								set_vel_2_array(pic_buffer[pic_last_vel_2_send],pulse_m1,pulse_m2);
								break;
								//======================================================================
							//==========================================================================
			                case pkg_rob_command:
			                    pkg = (struct payload_pkg_rob_command *) load;
			                    console_current_command =pkg->command[0];
			                    console_current_value1 =pkg->value;
			                    switch (console_current_command) {
									//=================
            			            case 'v':
										#ifdef MAIN_DEBUG
            			                printf("-------------------------------------------- COMANDO ----------------\n");
										#endif
            			                set_vel_2_array(pic_buffer[pic_last_vel_2_send], console_current_value1, console_current_value1);
            			                pic_last_vel_2_send%=LEN_PIC_BUFFER;
            			                break;
										//============
									//=================
            			            case 'p':
            			                set_pos_2_array(pic_buffer[pic_last_vel_2_send], v_sx, v_dx, console_current_value1, console_current_value1);
            			                pic_last_vel_2_send%=LEN_PIC_BUFFER;
            			                break;
										//============
									//=================
            			            case 'l':
            			                v_sx=console_current_value1;
            			                break;
										//============
									//=================
            			            case 'r':
            			                v_dx=console_current_value1;
            			                break;
										//============
									//=================
            			            case 'o':
            			                set_vel_2_array(pic_buffer[pic_last_vel_2_send], 0, 0);
            			                pic_last_vel_2_send%=LEN_PIC_BUFFER;
            			                write(pic_fd, pic_buffer[pic_last_vel_2_send], PACKET_SPEED_LENGTH + 1);
            			                sync();
            			                break;
										//============
            		                //=================
            			            case 'd':
            			                distanza=console_current_value1;
            			                break;
										//============
            		                //=================
            			            case 't':
            			                velocita=console_current_value1;
            			                break;
										//============
		                            //=================
        			                case 's':
        			                    genera_rettilineo(distanza, velocita , TRISTEPPING, &num_passi, &v_croc);
        			                    set_pos_2_array(pic_buffer[pic_last_vel_2_send], 1024-v_croc, 1024-v_croc, num_passi, num_passi);
        			                    pic_last_vel_2_send%=LEN_PIC_BUFFER;
        			                    break;
										//============
									//=================
        			                case CONSOLE_COMMAND_SET_ANGLE:
        			                    angle_degrees=console_current_value1;
        			                    break;
										//============
									//=================
        			                case CONSOLE_COMMAND_SET_CIRC:
        			                    calcola_circonferenza(velocita/100.0, distanza/100.0,  &v1, &v2);
        			                    set_vel_2_array(pic_buffer[pic_last_vel_2_send], 1024-v1, 1024-v2);
        			                    break;
										//============
									//=================		
        			                case CONSOLE_COMMAND_SET_SELF_ROTATION:
        			                    angle_rad=M_PI*angle_degrees/180.0;
        			                    calcola_angolo(angle_rad, velocita,  &v1 , &v2, &num_passi);
        			                    v1=v1/modulo(v1)*(1024-modulo(v1));
        			                    v2=v2/modulo(v2)*(1024-modulo(v2));
        			                    set_pos_2_array(pic_buffer[pic_last_vel_2_send], v1, v2, num_passi, num_passi);
										gyro_sup_integral=0;
        			                    break;
										//============
									//=================
        			                default:
        			                    break;
										//============
        		                    id=-1;
        			            }
								break;
								//==================================================================
							//======================================================================
                			default:
                    			break;
								//==================================================================
            			}
		            	tcflush(console_fd, TCIFLUSH);
						#ifdef MAIN_DEBUG
        		    	printf("-----\n");
        		    	printf("Xbee packet received: %d \t sender: %02x\n", ++xbee_packet_received, sender);
        		    	printf("-----\n");
						#endif            
        		    	if(iid!=-1){
							#ifdef MAIN_DEBUG
							printf("ALLOCAZIONE\n");
							#endif
							free(load);}
							#ifdef MAIN_DEBUG
							printf("vaffa\n");
							#endif
		        		}
		        		else if (id==tx_status) {
							#ifdef MAIN_DEBUG
			            	printf("-----2\n");
			            	printf("Xbee ack packet received: %d\n", ++xbee_ack_received);
			            	printf("-----\n");
							#endif
		        		}
		
						byte_parsati += (int)len+1;
						#ifdef MAIN_DEBUG
						printf ("\nbyte_parsati = %d\n",byte_parsati);
						#endif
						free (pkg_parsato);
				}  
				else {
					do {byte_parsati++;}
					while (buffer[byte_parsati]==0x7e);
				}
			} 
			else {
				do {byte_parsati++;} 
				while (buffer[byte_parsati]==0x7e);
			}
		}
   		usleep(100000);
    }
    pthread_exit(NULL);
}
//=========================================================================================================



//=========================================================================================================
void setup_state() {
    size_t len = 50;
    char* line=NULL;
    ssize_t read;
    statusFile=fopen("pose.txt", "r");
    if(statusFile==NULL) {
		#ifdef MAIN_DEBUG
        printf("Error in pose.txt opening: %s\n", strerror(errno));
		#endif
        exit(EXIT_FAILURE);
    }
    else{
        if(read=getline(&line, &len, statusFile)!=-1){
            state[STATE_X]=atof(strtok(line, " "));
            state[STATE_Y]=atof(strtok(NULL, " "));
            state[STATE_THETA]=atof(strtok(NULL, " "));
        }
    }
    
}
//=========================================================================================================


//=========================================================================================================
inline void	algorithm_via_points_servoing(){


            pthread_mutex_lock(&mutex_state);
            error_state[STATE_X] = *(goal_trajectory+3*traj_num_viapoints_served)-state[STATE_X];
            error_state[STATE_Y] = *(goal_trajectory+3*traj_num_viapoints_served+STATE_Y)-state[STATE_Y];
            error_state[STATE_THETA] = atan2(error_state[STATE_Y], error_state[STATE_X])-state[STATE_THETA];
            pthread_mutex_unlock(&mutex_state);
            cartesian_error_norm=sqrt(error_state[STATE_X]*error_state[STATE_X]+error_state[STATE_Y]*error_state[STATE_Y]);
            if(cartesian_error_norm<VIA_POINT_ERROR_NORM && traj_num_viapoints_served<traj_num_viapoints){
				traj_num_viapoints_served++ ;
				//traj_num_viapoints_served%=traj_num_viapoints;
				#ifdef MAIN_DEBUG
                printf("-----------------------------------------\n");
                printf("SERVED VIAPOINT %d\n", traj_num_viapoints_served );
                printf("-----------------------------------------\n");
				#endif
                set_vel_2_array(pic_buffer[pic_last_vel_2_send], 0, 0);
            }
			#ifdef MAIN_DEBUG
            if(traj_num_viapoints_served==traj_num_viapoints){printf("ARRIVED\n");v_ref=0.0;w_ref=0.0;}
			#endif
            
            if(traj_num_viapoints_served<traj_num_viapoints){
                vaffa_controller(error_state, 1.2, -20.0, &v_ref, &w_ref);
				//vaffa_controller(error_state, 0.1, -1.0, &v_ref, &w_ref);

				//printf("INGRESSI: %f %f\n", v_ref, w_ref);
				//printf("ERRORE STATO: %f %f %f\n", *error_state,*(error_state+1),*(error_state+2));								
                get_vel_motori_evolution(&v_ref, &w_ref, &v_m1, &v_m2);
				//printf("INGRESSI FILTRATI: %f %f\n", v_ref, w_ref);
                calcola_velocita_evolution(v_m1, TRISTEPPING, &pulse_m1);
                calcola_velocita_evolution(v_m2, TRISTEPPING, &pulse_m2);
				
                //printf("motori: %d %d\n", pulse_m1, pulse_m2);
                //=======================================
                //		      Pulses Saturation
                pulse_m1=min_int(max_int(-900, pulse_m1), 900);
                pulse_m2=min_int(max_int(-900, pulse_m2), 900);
                pulse_m1=pulse_m1/modulo(pulse_m1)*(1024-modulo(pulse_m1));
                pulse_m2=pulse_m2/modulo(pulse_m2)*(1024-modulo(pulse_m2));
                //=======================================
                set_vel_2_array(pic_buffer[pic_last_vel_2_send],pulse_m1,pulse_m2);
            }
            else{set_vel_2_array(pic_buffer[pic_last_vel_2_send], 0, 0);}
}
//=========================================================================================================


//=========================================================================================================
inline	void	consensus_randezvous(){

		float v_x, v_y;
		pthread_mutex_lock(&mutex_state);
        calculate_consensus_velocities(&v_x, &v_y, state[STATE_X], state[STATE_Y]);
        get_vel_desiderate_consensus(v_x, v_y, &v_ref, &w_ref,0.5,10.0);
		*(error_state+STATE_X)=v_x;
		*(error_state+STATE_Y)=v_y;
		*(error_state+STATE_THETA)=w_ref;	
		cartesian_error_norm=sqrt(error_state[STATE_X]*error_state[STATE_X]+error_state[STATE_Y]*error_state[STATE_Y]);

		//printf("===================\n");
		//printf("ERROR NORM: %f\n", cartesian_error_norm);
		//printf("===================\n");


		if(cartesian_error_norm > CONSENSUS_ERROR_NORM){
			//printf("oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo\n");
			
					vaffa_controller(error_state, 1.2, -16.0, &v_ref, &w_ref);
                	pthread_mutex_unlock(&mutex_state);
                	get_vel_motori(v_ref, w_ref, &v_m1, &v_m2);
                
                
                	calcola_velocita(v_m1, TRISTEPPING, &pulse_m1);
                	calcola_velocita(v_m2, TRISTEPPING, &pulse_m2);
                
                
                	//=======================================
                	//		      Pulses Saturation
                	pulse_m1=min_int(max_int(-900, pulse_m1), 900);
                	pulse_m2=min_int(max_int(-900, pulse_m2), 900);
                	pulse_m1=pulse_m1/modulo(pulse_m1)*(1024-modulo(pulse_m1));
                	pulse_m2=pulse_m2/modulo(pulse_m2)*(1024-modulo(pulse_m2));
                	//=======================================
                
                	set_vel_2_array(pic_buffer[pic_last_vel_2_send],pulse_m1,pulse_m2);
			killer_counter=0;
			//printf("end_cons\n");
		}
		else{
			//printf("arrvied\n");
			pthread_mutex_unlock(&mutex_state);
			set_vel_2_array(pic_buffer[pic_last_vel_2_send],0,0);
			if(cartesian_error_norm !=0){
				termination_handler(0);
				exit(0);
			}
			else if((++killer_counter)==MAX_KILLER_COUNTER){
				termination_handler(0);
				exit(0);
			}
		}


}
//=========================================================================================================

//=========================================================================================================
inline	void	global_initialization(){


#ifdef WEBCAM
   InitCamera(); 
#endif
#ifdef CONSENSUS_PROTOCOL_ACTIVE
    init_consensus_structures();
    setup_state();
#endif
    robot_init();
    mutex_init();
	pthread_mutex_init(&mutex_camera_kalman, NULL);
    write(pic_fd, pic_message_reset_steps_acc, PACKET_TIMING_LENGTH+1);
    tcflush(pic_fd, TCOFLUSH);
    sync();
    steps_anomaly=0;
#ifdef	XBEE
    console_fd=open_port ((char *)porta, NO_BLOCKING, V_MIN, NO_ACK);
    if(console_fd<0) {
        printf("Error to open port \"%s\"\n", porta);
        exit(0);
    }
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    pthread_create(&thread, 		&attr, &tf_read, NULL);
#ifdef RFID
	tty_open_rfid();
	pthread_create(&thread_rfid, 	&attr, &tf_rfid, NULL);
#endif
#ifdef XBEE_SEND
    pthread_create(&thread_w, &attr, &tf_write, NULL);
#endif
	//printf("thread_creato\n"); while(getchar()!='\n');
    pthread_attr_destroy(&attr);
#endif
    
    v_sx=0;v_dx=0;
    v_croc=0;
    for(i=0;i<LEN_PIC_BUFFER;i++){set_vel_2_array(pic_buffer[i], v_croc, v_croc);}


	killer_counter=0;
	fp_log=fopen(file_log, "w");
    setup_termination();

}
//=========================================================================================================

//=========================================================================================================
inline	void 	gyro_zero_setting(){

	int 	local_counter=0;
	int 	local_value=0;
    while (local_counter<3000) {
        pthread_mutex_lock(&m_analizza_pacchetto);
        while(!flag){pthread_cond_wait(&cv_analizza_pacchetto, &m_analizza_pacchetto);}
            pthread_mutex_unlock(&m_analizza_pacchetto);
            packet_type=analizza_pacchetto();
			if(gyro->is_valid==TRUE){
				if(local_value==*(gyro->range+1)){local_counter++;printf("lv: %d\n", local_value);}
				else{local_counter=0;local_value=*(gyro->range+1);}
				if(local_counter>10){gyro_table_effective_zero=local_value;local_counter=3001;}
		}
	}
	printf("zero_trovato= %d su %d iterazioni\n", gyro_table_effective_zero, local_counter);
}
//=========================================================================================================

//=========================================================================================================
inline	void	camera_processing(){

	        // 			      Vision step
	        //Frame Grabbing
	        GetFrame();  //blocking
	        frame_counter++;
	        //Processing Frame
	        //ProcessFrame();
	        //gettimeofday(&vision_start_time,NULL);
	        //EvaluateCoordinates(); // commentata la parte che accede alla mappa
	        //gettimeofday(&vision_end_time,NULL);
	        //LogTimeData();
}
//=========================================================================================================

//=========================================================================================================
inline	void	calibrazione_magneto(){

	int	magneto_flag=0;
	velocita = 2.0;
	angle_degrees = 360.0;
	angle_rad=M_PI*angle_degrees/180.0;
	fpm_log=fopen(magneto_log, "w");
	calcola_angolo(angle_rad, velocita,  &v1 , &v2, &num_passi);
	v1=v1/modulo(v1)*(1024-modulo(v1));
	v2=v2/modulo(v2)*(1024-modulo(v2));
	set_pos_2_array(pic_buffer[pic_last_vel_2_send], v1, v2, num_passi, num_passi);	

	while (flag_servoing_completed!=1) {
        pthread_mutex_lock(&m_analizza_pacchetto);
        while(!flag){pthread_cond_wait(&cv_analizza_pacchetto, &m_analizza_pacchetto);}
        pthread_mutex_unlock(&m_analizza_pacchetto);
        packet_type=analizza_pacchetto();
		if(magneto->is_valid==TRUE){print_sensore_on_file(magneto, fpm_log);fprintf(fpm_log,"\n");}
	}
	fclose(fpm_log);
}
//=========================================================================================================


//=========================================================================================================
inline	void 	magneto_zero_setting(){

	int 	local_counter=0;
	int 	local_value=0;
	int		x=0;
	int 	y=0;
    while (local_counter<16) {
        pthread_mutex_lock(&m_analizza_pacchetto);
        while(!flag){pthread_cond_wait(&cv_analizza_pacchetto, &m_analizza_pacchetto);}
        pthread_mutex_unlock(&m_analizza_pacchetto);
        packet_type=analizza_pacchetto();
		if(magneto->is_valid==TRUE){
			x+=*(magneto->range);
			y+=*(magneto->range+1);
			++local_counter;
		}
	}
	x=x>>4;
	y=y>>4;

	pMag->nordRad = pMag->tabMag[(x-pMag->tozerox)*(pMag->colonneTabMag)+(y-pMag->tozeroy)];   
	printf("Gradi inziali:%d %d %f\n",x,y,pMag->nordRad);
}
//=========================================================================================================



	    /*//==========================================
		gettimeofday(&vision_start_time,NULL);   
	    gettimeofday(&vision_end_time,NULL);
	    if(vision_end_time.tv_sec==vision_start_time.tv_sec){
			micro_secondi=vision_end_time.tv_usec-vision_start_time.tv_usec;
		}
	    else{
			micro_secondi=1000000-vision_start_time.tv_usec+vision_end_time.tv_usec;
	    }
		printf("TEMPO: %f\n", micro_secondi/1000.0);
		//==========================================*/

//=========================================================================================================
int	obstacle_avoidance(){

	velocita=1.0;
	if(ir->is_valid==TRUE){

		if(ir_distance[1023-*(ir->range+2)] <400 || ir_distance[1023-*(ir->range+3)] <400 || ir_distance[1023-*(ir->range+4)] <400){

				flag_obstacle=TRUE;
				if(flag_imposed==0){
				w_ref=0.0;3.0*(((float)rand()/RAND_MAX))*(MAX_TURN_RATE_EVO);;
				calcola_angolo(w_ref, velocita,  &v1 , &v2, &num_passi);
				v1=v1/modulo(v1)*(1024-modulo(v1));
        		v2=v2/modulo(v2)*(1024-modulo(v2));
        		set_pos_2_array(pic_buffer[pic_last_vel_2_send], v1, v2, num_passi, num_passi);
				flag_servoing_completed = 0;
				flag_imposed=1;
				}
				flag_imposed=flag_servoing_completed;
				/*v_ref=0.0;
				w_ref=(((float)rand()/RAND_MAX))*(MAX_TURN_RATE_EVO);
				printf("INGRESSI:%f %f %f\n", v_ref, w_ref, ((float)rand()/RAND_MAX));
				printf("STO DENTRO\n");
				get_vel_motori_evolution(&v_ref, &w_ref, &v_m1, &v_m2);
				//printf("INGRESSI FILTRATI: %f %f\n", v_ref, w_ref);
		        calcola_velocita_evolution(v_m1, TRISTEPPING, &pulse_m1);
		        calcola_velocita_evolution(v_m2, TRISTEPPING, &pulse_m2);
				
		        //printf("motori: %d %d\n", pulse_m1, pulse_m2);
				//=======================================
				//		      Pulses Saturation
        		pulse_m1=min_int(max_int(-900, pulse_m1), 900);
        		pulse_m2=min_int(max_int(-900, pulse_m2), 900);
        		pulse_m1=pulse_m1/modulo(pulse_m1)*(1024-modulo(pulse_m1));
        		pulse_m2=pulse_m2/modulo(pulse_m2)*(1024-modulo(pulse_m2));
				//=======================================
        		set_vel_2_array(pic_buffer[pic_last_vel_2_send],pulse_m1,pulse_m2);*/

		}
		else{flag_obstacle=FALSE;}


 		
	}




}
//=========================================================================================================




//=========================================================================================================
int main(int argc, char* argv[]){

	float angle;
	int i,j,k,z;
	#ifdef	USE_PARTICLE_FILTER
	int	n_part;
	int particle_iterations;
	n_part=20;
	particle_iterations=1000;
	long tempo;
	int livello_tabella;
	#endif








	printf("INIT\n");
	global_initialization();
	//printf("GYRO\n");
	//gyro_zero_setting();
	printf("inizio calibrazione\n");
	////---------------------------------
	#ifdef	MAGNETO_CALIBRATION	
	calibrazione_magneto();
	printf("fine calibrazione\n");
	//create_magneto_table();
	#endif




	printf("Fine creazione tabella\n");
	//magneto_table_2_struct(&pMag);
	printf("caricamento completato\n");
	//pMag->nordRad=0;
	#ifdef	GET_MAGNETO_BEARING
	magneto_zero_setting();
	#endif
	printf("qua\n");
	//---------------------------------
	camera_x=0;
	camera_y=0;


	#ifdef USE_KALMAN
	//================================================
		inizializzaDatiKalman(&dKalman);
		printf("inizializzato kalman\n");
        //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
        //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
        foutput=fopen("kalman_output/outputKSaetta.txt","w");
        //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
	#endif

	//================================================


	#ifdef	USE_PARTICLE_FILTER
	angle=M_PI/4;
	livello_tabella=PARTICLE_ROBOT_BEARING_TO_TABLE(angle);

	printf("init particle\n");
	gettimeofday(&step_start_time,NULL);
	particle_init(NUMERO_PARTICLES);
	create_random_particles(particles, 0, n_part/2, ambience_A_borders);
	create_random_particles(particles, (int)n_part/2, n_part-n_part/2, ambience_B_borders);
	gettimeofday(&step_end_time,NULL);
	GetElapsedTime("PRED", &step_start_time, &step_end_time, &tempo );
	printf("TEMPO: %d\n", tempo);
	//while(getchar()!='\n');

	#endif

	//usleep(7000000);
	angle=0.0;gyro_integral=0.0;gyro_sup_integral=0.0;
	//========================================================================
    while (1) {

        //pthread_mutex_lock(&m_analizza_pacchetto);
        //while(!flag){pthread_cond_wait(&cv_analizza_pacchetto, &m_analizza_pacchetto);}
        //pthread_mutex_unlock(&m_analizza_pacchetto);
		packet_type=analizza_pacchetto();
	if(packet_type==LOAD_PACKET_ANALYZED ){

			#ifdef WEBCAM
			printf("camera\n");
			camera_processing();
			#endif
			//if(flag_obstacle==FALSE){
			#ifdef VAFFA_CONTROLLER 
			algorithm_via_points_servoing();
			#endif
			//}
			#ifdef CONSENSUS_PROTOCOL_ACTIVE
			consensus_randezvous();
			#endif
			#ifdef MAIN_DEBUG
			printf("==================================================================\n");
			
			//printf("STATE: %f %f %f |	%f	|	%f	\n", state[STATE_X], state[STATE_Y], state[STATE_THETA], angle, gyro_integral);
			//printf("gyro pic: %f - gyro fox: %f\n", gyro_sup_integral, gyro_integral);
			printf("==================================================================\n");
			#endif

			//if(ir->is_valid==TRUE){printf("IR: %d %d %d\n",ir_distance[1023-*(ir->range+2)],ir_distance[1023-*(ir->range+3)],ir_distance[1023-*(ir->range+4)] );}
			//bstacle_avoidance();

			//============================================
			angle=0.0;

			#ifdef	GET_MAGNETO_BEARING
			if(magneto->is_valid){radMagneto(pMag, *(magneto->range), *(magneto->range+1), &angle);}
			#endif


			
			#ifdef	USE_KALMAN
			pthread_mutex_lock(&mutex_camera_kalman);
			kalman_input_m[0]	=	camera_x+ 2.0*((float)(rand()/RAND_MAX)-0.5);
			kalman_input_m[1]	=	camera_y+ 2.0*((float)(rand()/RAND_MAX)-0.5);;
			pthread_mutex_unlock(&mutex_camera_kalman);
			kalman_input_m[2]	=	angle;
			//kalman_input_m[2] =	0.0;
			kalman_input_c[0]	=	v_ref;
			// è giusto il "meno"
			kalman_input_c[1]	=	-w_ref;
			//printf("(%f) (%f)\n",state[STATE_X],state[STATE_Y]);
			passoAlgoritmoKalman(&dKalman, kalman_input_c, kalman_input_m);					
			//fprintf(foutput,"%f %f %f %f %f %f | %f %f %f | %f\n", dKalman->xkp[0], dKalman->xkp[1], dKalman->xkp[2], kalman_input_m[2], v_ref, w_ref,state[STATE_X],state[STATE_Y],state[STATE_THETA], angle);			
			pthread_mutex_lock(&mutex_camera_kalman);		
			fprintf(foutput,"%f %f %f %f %f %f  %f %f %f  %f %f %f\n", 
			dKalman->xkp[0], dKalman->xkp[1], dKalman->xkp[2], 
			kalman_input_m[2], 
			v_ref, w_ref,
			state[STATE_X],state[STATE_Y],state[STATE_THETA], angle, 
			camera_x, camera_y);					
			pthread_mutex_unlock(&mutex_camera_kalman);
			#endif



			#ifdef	USE_PARTICLE_FILTER
			printf("init\n");
			particle_accumulator=0;
			pthread_mutex_lock(&mutex_ir);
			for(i=0;i<PARTICLE_NUM_SENSORS;i++){particle_int_current_distances[i]=*(ir->range+i);}
			pthread_mutex_unlock(&mutex_ir);

			pthread_mutex_lock(&mutex_v_ref);
			pthread_mutex_lock(&mutex_w_ref);

			/*gettimeofday(&step_start_time,NULL);

			particles_prediction(particles, NUMERO_PARTICLES, v_ref, w_ref,0.25);
			gettimeofday(&step_end_time,NULL);
			GetElapsedTime("PRED", &step_start_time, &step_end_time, &tempo );
			printf("TEMPO EVO: %d\n", tempo);
			gettimeofday(&step_start_time,NULL);

			//------------------------------------
			particle_int_accumulator=0;
			int_point_to_line_dist(livello_tabella);
			gettimeofday(&step_end_time,NULL);
			GetElapsedTime("PRED", &step_start_time, &step_end_time, &tempo );
			printf("ITER---------------------: %d\n", tempo);

			particle_int_get_iteration_weights(particle_int_current_distances);
			particle_int_update_weights(&particle_int_accumulator);


			//particle_int_evolution( livello_tabella, &particle_int_accumulator, particle_int_current_distances);
			//int_point_to_line_dist(livello_tabella);
			//particle_int_get_iteration_weights(particle_int_current_distances);
			//particle_int_update_weights(&particle_int_accumulator);
			particle_int_renormalization(particle_int_accumulator, &particle_faileds_number);
			particles_resampling(particles, particle_faileds_number);
			//------------------------------------*/
			particle_accumulator=0;
			particles_evolution( v_ref, w_ref, 0.25, &particle_accumulator, particle_int_current_distances);
			/*particles_renormalization(particles_weights, particle_accumulator, &particle_faileds_number);
			particles_resampling(particles, particle_faileds_number);*/

			gettimeofday(&step_end_time,NULL);
			GetElapsedTime("PRED", &step_start_time, &step_end_time, &tempo );
			printf("ITER---------------------: %d\n", tempo);



			pthread_mutex_unlock(&mutex_v_ref);
			pthread_mutex_unlock(&mutex_w_ref);
			
			

			printf("end	\n");			
			#endif










			//printf("stima: %f %f %f\n", dKalman->xkp[0], dKalman->xkp[1], dKalman->xkp[2]);					

			//iControllo++;
			//============================================

			

            pthread_mutex_lock(&mutex_fp);
            fflush(fp_log);
            pthread_mutex_unlock(&mutex_fp);
            flag=0;
        }
    }
	//========================================================================
    return 0;
}
//=========================================================================================================


