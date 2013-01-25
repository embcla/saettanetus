#include "console.h"

int init_console(void) {
struct termios tattr;


		//console_fd=STDIN_FILENO;
	
  	// Make sure stdin is a terminal
	if (!isatty (STDIN_FILENO)) {
		fprintf (stderr,"stdin is not a terminal\n");
	  return -1;
	}
	
	// Save the terminal attributes so we can restore them later.
	tcgetattr (STDIN_FILENO, &stdin_saved_attributes);

  // Set the funny terminal modes. 
  tcgetattr (STDIN_FILENO, &tattr);
  tattr.c_lflag &= ~(ICANON | ECHO); /* Clear ICANON and ECHO. */
  tattr.c_lflag|=O_NONBLOCK;
  tattr.c_cc[VMIN] = 0;
  tattr.c_cc[VTIME] = 0;
  tcsetattr (STDIN_FILENO, TCSAFLUSH, &tattr);

  return 0;
}


void	console_parsing(int fd){
//void	console_parsing(){

		int n;
		int i,j;
		//n=read (STDIN_FILENO, console_buffer, CONSOLE_LEN_BUFFER);
		n=read (fd, console_buffer, CONSOLE_LEN_BUFFER);

		if (n>0) {
			console_current_command=console_buffer[0];
			//i=3;
			//while(console_buffer[i]!=0x20){i++;}
			//memcpy(&(console_first_value[0]), &(console_buffer[3]), 2);
			//printf("%s\n", console_first_value);
			console_current_value1=atoi(&(console_buffer[1]));
			//console_current_value1=atoi(console_first_value);
			
			
			//printf("PARSATO: %c %d \n",console_buffer[0], console_current_value1);
			switch(console_buffer[0]){

				case CONSOLE_COMMAND_VEL:
					console_flag_cmd_rcvd=TRUE;
					//printf("%c %d %d\n", console_current_command, console_current_value1, console_flag_cmd_rcvd);
					break;
				case CONSOLE_COMMAND_SERVOING:
					console_flag_cmd_rcvd=TRUE;					
					break;
				case CONSOLE_COMMAND_V_SX:
					console_flag_cmd_rcvd=TRUE;					
					break;
				case CONSOLE_COMMAND_V_DX:
					console_flag_cmd_rcvd=TRUE;					
					break;
				case CONSOLE_COMMAND_MOTOR_OFF:
					console_flag_cmd_rcvd=TRUE;					
					break;
				case CONSOLE_COMMAND_SET_DIST:
					console_flag_cmd_rcvd=TRUE;					
					break;
				case CONSOLE_COMMAND_SET_VEL:
					console_flag_cmd_rcvd=TRUE;					
					break;
				case CONSOLE_COMMAND_SET_RECT:
					console_flag_cmd_rcvd=TRUE;					
					break;
				
				case CONSOLE_COMMAND_SET_CIRC:
					console_flag_cmd_rcvd=TRUE;					
					break;

				case CONSOLE_COMMAND_SET_SELF_ROTATION:
					console_flag_cmd_rcvd=TRUE;					
					break;
				
				case CONSOLE_COMMAND_SET_ANGLE:
					console_flag_cmd_rcvd=TRUE;					
					break;

				default:
					console_flag_cmd_rcvd=FALSE;
					//printf("NR\n");
					break;
			}
	  		
		}		
}




