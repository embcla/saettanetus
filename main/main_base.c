#include "main.h"

void main_init() {

    int i;
    float starting_speed = 0.0;
    init_robot();
    flag = 0;
    robot_state = malloc(sizeof (float) *3);

#ifdef VIAPOINTS_SERVOING
    init_via_points_servoing();
#endif

    for (i = 0; i < LEN_PIC_BUFFER; i++) {
        set_vel_2_array(pic_buffer[i], starting_speed, starting_speed);
    }

    write(pic_fd, pic_message_reset_steps_acc, PACKET_TIMING_LENGTH + 1);
    tcflush(pic_fd, TCOFLUSH);
    sync();
    steps_anomaly = 0;

#ifdef XBEE
    xbee_fd = open_port(XBEE_DEVICE, 0);

    pthread_mutex_init(&xbee_serial_mutex, NULL);
#endif

#ifdef WEBCAM
    GetLogFileName(log_fname);
    InitCamera();
    InitPixelMap();
    //GetLogFileName(log_fname);
    InitLogs();
    CreateColorStruct(&colorStruct); //struttura contenente le informazione dei colori
#endif

#ifdef O_AVOIDANCE_FUZZY
    init_obstacle_avoidance();
#endif

#ifdef IPHONE
    init_iphone();
#endif
}

//=========================================================================================================

//=========================================================================================================

void termination_handler(int signum) {

    int i;

#ifdef MAIN_DEBUG
    printf("\nNum packet data: correct  %d wrong  %d sent_wrong: %d \n", num_packet_data_ok, num_packet_data_wrong, num_packet_sent_wrong);
    printf("Num xbee packet sent: %d, ack received: %d \n", xbee_packet_sent, xbee_ack_received);
    printf("Num xbee packet received: %d \n", xbee_packet_received);
    for (i = 0; i < MAX_NEIGHBORS; i++) {
        printf("%d: %d\n", i, xbee_packet_neighborhood[i]);
    }
#endif

#ifdef	XBEE
    close_port(xbee_fd);
    fprintf(fp_log, "Num xbee packet sent: %d, ack received: %d \n", xbee_packet_sent, xbee_ack_received);
    fprintf(fp_log, "Num xbee packet received: %d \n", xbee_packet_received);
    for (i = 0; i < MAX_NEIGHBORS; i++) {
        fprintf(fp_log, "%d: %d\n", i, xbee_packet_neighborhood[i]);
    }
#endif

#ifdef WEBCAM
    CloseCamera();
    //DestroyColorStruct(colorStruct);
    CloseLogs();
#endif

#ifdef IPHONE
    close_iphone();

#endif

#ifdef VIAPOINTS_SERVOING
    close_via_points_servoing();
#endif

#ifdef PRINT_LOOP_TIME
    if (tempi)
        free(tempi);
#endif    
    fprintf(fp_log, "\nNum packet data: correct  %d wrong  %d sent_wrong: %d \n", num_packet_data_ok, num_packet_data_wrong, num_packet_sent_wrong);
    fflush(fp_log);
    fclose(fp_log);
    printf("scritto\n");
    set_vel_2_array(pic_buffer[pic_last_vel_2_send], 0, 0);
    write(pic_fd, pic_buffer[pic_last_vel_2_write], PACKET_SPEED_LENGTH + 1);
    sync();
    close_serial_comm();
    close_robot_comm();
    close_robot();
    close(pic_fd);

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
void* tf_laser_read(void* thread_arg)
{
  urg_t *urg;
  
  init_urg_laser(&urg,ON_DEMAND);
  
	
  
  read_laser_data(urg);
  
  
  //get_laser_data(pluto);
  
  //printf("%ld\n",pluto[0]);
}
//=========================================================================================================




//=========================================================================================================

void* tf_main(void* thread_arg) {

    float angle;
    float v_ref, w_ref;
    struct payload_pkg_state tosend;
    struct payload_pkg_trajectory *lala;

    lala=malloc(sizeof(struct payload_pkg_trajectory));	
   

    setup_termination();

#ifdef PRINT_LOOP_TIME
    tempi = malloc(sizeof (struct timeval) *2);
#endif

    angle = 0.0;
    gyro_integral = 0.0;
    gyro_sup_integral = 0.0;
    //========================================================================
    while (1) {

        /*Condition variable implementation*/
        //pthread_mutex_lock(&m_analizza_pacchetto);
        //while(!flag){pthread_cond_wait(&cv_analizza_pacchetto, &m_analizza_pacchetto);}
        //pthread_mutex_unlock(&m_analizza_pacchetto);
        /***********************************/

	tosend.x=state[0];
	tosend.y=state[1];
	tosend.th=state[2];

	lala->v=1;
	lala->w=2;
	pthread_mutex_lock (&xbee_serial_mutex);
//            Send_Data_16(xbee_fd, 0xff, pkg_trajectory, (void *)lala, sizeof(struct payload_pkg_trajectory), NO_ACK);
       Send_Data_16(xbee_fd, 0xff, pkg_state, (void *) &tosend, sizeof(struct payload_pkg_state), NO_ACK);
	
	    pthread_mutex_unlock (&xbee_serial_mutex);
       	xbee_packet_sent++;
	 packet_type = analizza_pacchetto();

            
           
        if (packet_type == LOAD_PACKET_ANALYZED) {
#ifdef PRINT_LOOP_TIME
            gettimeofday(&(tempi[0]), NULL);
#endif

            /*
                          printf("Sensore IR 1: %d  ",*(ir->range));
                          printf("Sensore IR 2: %d  ",*(ir->range+1));
                          printf("Sensore IR 3: %d  ",*(ir->range+2));
                          printf("Sensore IR 4: %d  ",*(ir->range+3));
                          printf("Sensore IR 5: %d  \n",*(ir->range+4));
             */
#ifdef WEBCAM
#ifdef MAIN_PRINT
            printf("camera\n");
#endif
            camera_processing();
#endif


#ifdef VIAPOINTS_SERVOING

            run_via_points_servoing(&v_ref, &w_ref);
            fprintf(stderr, "INGRESSI: %f %f\n", v_ref, w_ref);
            printf("ERRORE STATO: %f %f %f\n", *error_state, *(error_state + 1), *(error_state + 2));
            set_robot_speed(&v_ref, &w_ref);
            get_robot_state(&robot_state);
            fprintf(stderr, "STATE: %f %f %f\n", robot_state[STATE_X], robot_state[STATE_Y], robot_state[STATE_THETA]);
#endif

#ifdef O_AVOIDANCE_FUZZY

            int o_av_state;
            fuzzy_horizon_result result_fuzzy;

            fuzzyHorizon(&result_fuzzy);

            get_o_avoidance_state(&o_av_state);

            v_ref = result_fuzzy.v_ref_hor_fuz;
            w_ref = result_fuzzy.w_ref_hor_fuz;


            set_robot_speed(&v_ref, &w_ref);
            printf("v_ref %f\n",v_ref);
	    printf("w_ref %f\n",w_ref);

            get_robot_state(&robot_state);
#endif

#ifdef IPHONE
#ifdef MAIN_PRINT
            printf("Query iphone...\n");
#endif

            getIPhoneRefVel(&v_ref, &w_ref);
            printf("v_ref: %f\tw_ref: %f\n", v_ref, w_ref);

            set_robot_speed(&v_ref, &w_ref);

            fprintf(iphoneLogging, "m1: %f\tm2: %f\n", v_m1, v_m2);
            printf("m1: %f\tm2: %f\n", v_m1, v_m2);



#endif


#ifdef MAIN_DEBUG
            printf("==================================================================\n");

            //printf("STATE: %f %f %f |	%f	|	%f	\n", state[STATE_X], state[STATE_Y], state[STATE_THETA], angle, gyro_integral);
            //printf("gyro pic: %f - gyro fox: %f\n", gyro_sup_integral, gyro_integral);
            printf("==================================================================\n");
#endif

            //============================================
            

#ifdef	GET_MAGNETO_BEARING
            angle = 0.0;
            if (magneto->is_valid) {
                radMagneto(pMag, *(magneto->range), *(magneto->range + 1), &angle);
            }
#endif


            pthread_mutex_lock(&mutex_fp);
            fflush(fp_log);
            pthread_mutex_unlock(&mutex_fp);
            flag = 0;

#ifdef PRINT_LOOP_TIME
           /* pthread_mutex_lock (&xbee_serial_mutex);
            Send_Data_16(xbee_fd, 0xff, pkg_state, (void *)state, sizeof(float)*3, NO_ACK);
            pthread_mutex_unlock (&xbee_serial_mutex);
            */
	    set_robot_speed(&v_xbee,&w_xbee);
            gettimeofday(&(tempi[1]), NULL);
            printf("Tempo di ciclo in millisecondi: %ld\n", ((tempi[1].tv_usec + 1000000 * tempi[1].tv_sec)-(tempi[0].tv_usec + 1000000 * tempi[0].tv_sec)) / 1000);
#endif
        }
    }
    //========================================================================

}

void* tf_read(void *args) {

    int oldstate;
    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, &oldstate);

    
    fd_set read_fd;
    FD_ZERO(&read_fd);
    FD_SET(xbee_fd,&read_fd);

    
    setup_termination();
    void *load;
    int nRead;
    uint8 buf[3];
    uint8 *pkg;
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

	select(xbee_fd+1,&read_fd,NULL,NULL,NULL);
        /* Look for a command */
        /* Lock the mutex on the job queue. */
        pthread_mutex_lock(&xbee_serial_mutex);
        nRead = readport(xbee_fd, buf, 3);
	pthread_mutex_unlock(&xbee_serial_mutex);

        if (nRead == 3) {
		if (buf[0] == 0x7e) {
                printf("%02x\t%02x\n", buf[1], buf[2]);
                len = MSB_LSB_2_INT(buf + 1, 2);
                if (len > 0) {
                    printf("Len pkg: %d\t", len);
                    pkg = (uint8 *) malloc(sizeof (uint8)*(len + 1));
                    pthread_mutex_lock(&xbee_serial_mutex);
                    nRead = readport(xbee_fd, pkg, len + 1);
                    pthread_mutex_unlock(&xbee_serial_mutex);
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

//=========================================================================================================

int main(int argc, char* argv[]) {




    printf("INIT\n");

    main_init();

    fp_log = fopen(file_log, "w");
    setup_termination();


    pthread_attr_t attr_main;
    pthread_attr_init(&attr_main);
    
   pthread_create(&thread_main, &attr_main, &tf_main, NULL);

#ifdef XBEE
    pthread_attr_t attr_xbee;
    pthread_attr_init(&attr_xbee);
    pthread_attr_setdetachstate(&attr_xbee, PTHREAD_CREATE_DETACHED);

    pthread_create(&thread_xbee, &attr_xbee, &tf_read, NULL);
#endif

   // pthread_create(&thread_laser,&attr_main,&tf_read_laser,NULL);
    printf("FD Pic: %d\n", pic_fd);
    printf("FD Xbee: %d\n ", xbee_fd);
    pthread_join(thread_main, NULL);
    return 0;
}
//=========================================================================================================



