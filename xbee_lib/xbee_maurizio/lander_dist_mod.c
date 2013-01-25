// source code

#include "lander_dist.h"
#include "XbeePro_API.h"

//----------------------







    unsigned char c;
	int i;
	char *pv;		
	int v_sx;
	int v_dx;
	//--------------------------
	unsigned int num_passi;
	float distanza, velocita;
	float angle_rad;
	int   angle_degrees;
	int v1,v2;
	//--------------------------
	char carattere[4];
	int cv=0;
	int v_croc;












#ifdef	CONSOLE
void* thread_function (void* thread_arg)
{
	int id, iid;
	void *load;
	struct payload_pkg_rob_command pkg;
	
    while(1) {

		//console_parsing(console_fd);
		id=pkg_parser(console_fd,&iid,load);
		
		if(console_flag_cmd_rcvd==TRUE){



		/*********			

		LA PARTE XBEE VA QUI: LA STRUTTURA PACCHETTO è pkg_command:
			
			è costituita da un char "comando" e da un int "valore"
			lo switch qui sotto deve avere come argomento "comando"

			il valore "console_current_value1" deve essere rimpiazzato da "valore"
		*********/
			pkg = (struct payload_pkg_rob_command *) load;
			
			console_current_command = pkg->command;
			console_current_current_value1 = pkg->value;
			
			
			// qui ci andrà pacchetto->comando
			switch(console_current_command){
			
				case 'v':
						set_vel_2_array(pic_buffer[pic_last_vel_2_send], console_current_value1, console_current_value1);
						//++pic_last_vel_2_send;
						pic_last_vel_2_send%=LEN_PIC_BUFFER;
						//write(pic_fd, pic_buffer[pic_last_vel_2_write], PACKET_SPEED_LENGTH + 1);
						break;
				
				case 'p':

					
						set_pos_2_array(pic_buffer[pic_last_vel_2_send], v_sx, v_dx, console_current_value1, console_current_value1);
						//++pic_last_vel_2_send;
						pic_last_vel_2_send%=LEN_PIC_BUFFER;
						//write(pic_fd, pic_buffer[pic_last_vel_2_write], PACKET_SERVOING_LENGTH + 1);
						break;
				case 'l':
						v_sx=console_current_value1;
						break;
				case 'r':
						v_dx=console_current_value1;
						break;
				case 'o':

						set_vel_2_array(pic_buffer[pic_last_vel_2_send], 0,0);
						//++pic_last_vel_2_send;
						pic_last_vel_2_send%=LEN_PIC_BUFFER;
						write(pic_fd, pic_buffer[pic_last_vel_2_send], PACKET_SPEED_LENGTH + 1);
						break;
				
				// 	SETTAGGIO DISTANZA
				case 'd':
						distanza=console_current_value1;
//						printf("DIST: %f  cm\n", distanza);
						break;
				//	SETTAGGIO VELOCITA'
				case 't':
						velocita=console_current_value1;
//						printf("VEL: %f cm/s\n", velocita);
						break;
				//	SETTAGGIO SERVOING:
				case 's':
						printf("S: %d\n", num_passi);
						genera_rettilineo(distanza, velocita , TRISTEPPING, &num_passi, &v_croc);
//						printf("INPUT: vel %d dist %d\n", v_croc, num_passi);
						set_pos_2_array(pic_buffer[pic_last_vel_2_send], 1024-v_croc, 1024-v_croc, num_passi, num_passi);
						//++pic_last_vel_2_send;
						pic_last_vel_2_send%=LEN_PIC_BUFFER;
						//set_pos_2_array(pic_buffer[pic_last_vel_2_write], 200,200,200,200);
						break;
				
				case CONSOLE_COMMAND_SET_ANGLE:
						angle_degrees=console_current_value1;
						break;
				
				case CONSOLE_COMMAND_SET_CIRC:
					
						//printf("RAGGIO: %f  VEL_LINEARE: %f\n", distanza, velocita);
						calcola_circonferenza(velocita/100.0, distanza/100.0,  &v1, &v2);
//						printf("OUTPUT: %d %d\n", 1024-v1, 1024-v2);
						set_vel_2_array(pic_buffer[pic_last_vel_2_send], 1024-v1,1024-v2);
						break;
				
				case CONSOLE_COMMAND_SET_SELF_ROTATION:
						//calcola_circonferenza(velocita, distanza,  &v1, &v2);	
						
						angle_rad=M_PI*angle_degrees/180.0;
				
						//printf("ANGOLO %f RADIANTI\n", angle_rad);
				
						//printf("VEL LINEARE: %f\n", velocita);
						calcola_angolo(angle_rad, velocita,  &v1 , &v2, &num_passi);
						printf("S: %d\n", num_passi);
						//printf("OUTPUT1: v1 %d  v2 %d  num_passi %d\n", v1, v2, num_passi);
				
						
						v1=v1/modulo(v1)*(1024-modulo(v1));
						v2=v2/modulo(v2)*(1024-modulo(v2));
						//printf("OUTPUT2: v1 %d  v2 %d  num_passi %d\n", v1, v2, num_passi);
						set_pos_2_array(pic_buffer[pic_last_vel_2_send], v1, v2, num_passi, num_passi);
						break;
				
				
				default:
						break;
			
			}
			console_flag_cmd_rcvd=FALSE;
			//calcola_velocita(20.724, TRISTEPPING, &v_croc);
			//calcola_passi(20.724, TRISTEPPING, &v_croc);
			genera_rettilineo(20.724, 20.724, TRISTEPPING, &num_passi, &v_croc);
//			printf("VEL: %d  PASSI: %d\n", v_croc, num_passi);
		}



		
    }
}

#endif

//---------------------------------------------------------------------------------------------

const char file_log[]="acc.txt";
unsigned	int contatore_pacchetti=0;
        
int main(int argc, char* argv[]){


	carattere[cv]=getchar();
	//printf("dsadas: %c\n", carattere[cv]);
	while(carattere[cv]!='\n'){
		cv++;
		carattere[cv]=getchar();
	}
	v_croc=atoi(carattere);
	fp_log=fopen(file_log, "w");
	
	//delay_ms(999);
	
	
	robot_init();
	
#ifdef CONSOLE
	pv=malloc(sizeof(char)*30);	
	//console_init();
#endif
#ifdef	CONSOLE_XBEE
		console_fd=xbee_fd;
		pthread_attr_t attr;
		pthread_t thread;
		pthread_attr_init (&attr);
		pthread_attr_setdetachstate (&attr, PTHREAD_CREATE_DETACHED);
		pthread_create (&thread, &attr, &thread_function, NULL);
		pthread_attr_destroy (&attr);
	


#endif
	
	wait_flag=TRUE;
	v_sx=0;v_dx=0;
	for(i=0;i<LEN_PIC_BUFFER;i++){set_vel_2_array(pic_buffer[i],v_croc,v_croc);}
	
	
    while (1) {
		

		if(wait_flag==FALSE){
			analizza_pacchetto();
			//cartesian_controller(state, goal, LINEAR_GAIN,ANGULAR_GAIN, v_d, w_d);
			//decoupled_controller(state, goal, &p_norm_d, &p_norm_d);
			
			
			fflush(fp_log);
			wait_flag=TRUE;
		}
    }

	

	return 0;
}
