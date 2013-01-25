/*
		serial_test.c
		Very simple serial tester
		For more info see: http://www.acmesystems.it/?id=50

		Copyright (C) 2005 Acme Systems srl (http://www.acmesystems.it)
		
		This is free software; you can redistribute it and/or modify
		it under the terms of the GNU General Public License as published by
		the Free Software Foundation; either version 2 of the License, or
		(at your option) any later version.
		
		This example is distributed in the hope that it will be useful,
		but WITHOUT ANY WARRANTY; without even the implied warranty of
		MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
		GNU General Public License for more details.
		
		To have a copy of the GNU General Public License write to the Free Software
		Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <stdio.h>     
#include <string.h>    
#include <unistd.h>    
#include <fcntl.h>     
#include <errno.h>     
#include <termios.h>   
#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <stdarg.h>
#include <signal.h>

struct termios stdin_saved_attributes;
struct termios tty_saved_attributes;
int tty_fd;

int tty_open(char* tty_dev) {
	struct termios new_attributes;

	tty_fd = open(tty_dev,O_RDWR| O_NOCTTY | O_NONBLOCK);
	
  if (tty_fd<0) {
  	return -1;
  } else {
		tcgetattr(tty_fd,&tty_saved_attributes);
		tcgetattr(tty_fd,&new_attributes);
		
		// Set the new attributes for the serial port
		// http://linux.about.com/library/cmd/blcmdl3_termios.htm
		// http://www.gnu.org/software/libc/manual/html_node/Low_002dLevel-I_002fO.html#Low_002dLevel-I_002fO
		
		// c_cflag
		new_attributes.c_cflag |= CREAD;		 	// Enable receiver
  	new_attributes.c_cflag |= CS8;			 	// 8 data bit
  	
		// c_iflag
  	new_attributes.c_iflag |= IGNPAR;		 	// Ignore framing errors and parity errors. 
  	
		// c_lflag
  	new_attributes.c_lflag &= ~(ICANON); 	// DISABLE canonical mode. 
  																				// Disables the special characters EOF, EOL, EOL2, 
  																				// ERASE, KILL, LNEXT, REPRINT, STATUS, and WERASE, and buffers by lines.
  	new_attributes.c_lflag &= ~(ECHO);		// DISABLE this: Echo input characters.
  	new_attributes.c_lflag &= ~(ECHOE);		// DISABLE this: If ICANON is also set, the ERASE character erases the preceding input 
  																				// character, and WERASE erases the preceding word.
  	new_attributes.c_lflag &= ~(ISIG);		// DISABLE this: When any of the characters INTR, QUIT, SUSP, 
  																				// or DSUSP are received, generate the corresponding signal.
 	  
 	  new_attributes.c_cc[VMIN]=1;					// Minimum number of characters for non-canonical read.
	  new_attributes.c_cc[VTIME]=0;					// Timeout in deciseconds for non-canonical read.

		cfsetospeed(&new_attributes,B9600);		// Set the baud rate
		cfsetispeed(&new_attributes,B9600);


    tcsetattr(tty_fd, TCSANOW, &new_attributes);
	}
  return tty_fd;
}

// Serial version of printf

void tty_printf(char *format, ...) {
  va_list argptr;
  char buffer[200];
  
  va_start(argptr,format);
  vsprintf(buffer,format,argptr);
  va_end(argptr);
  
  write(tty_fd,buffer,strlen(buffer));
}

void termination_handler (int signum) {
	tcsetattr(STDIN_FILENO,TCSANOW,&stdin_saved_attributes);
	if (tty_fd>0) tcsetattr (tty_fd,TCSANOW,&tty_saved_attributes);
	close(tty_fd);
	printf("Exit\n");
	exit(0);
}

int stdin_init(void) {
  struct termios tattr;

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
  tattr.c_cc[VMIN] = 0;
  tattr.c_cc[VTIME] = 0;
  tcsetattr (STDIN_FILENO, TCSAFLUSH, &tattr);
  return 0;
}

int main(int argc, char *argv[]) {
  char rxChar;
  char txChar;

	printf("Acme Serial Test (press ctrl-c to exit)\n");

  if (tty_open(argv[1])<0) {
  	fprintf (stderr,"tty open error %s\n", strerror(errno));
	  exit(EXIT_FAILURE);
  } 
	
  if (stdin_init()<0) {
  	printf("stdin init error %s\n", strerror(errno));
	  exit(EXIT_FAILURE);
  } 

  if (signal (SIGINT, termination_handler) == SIG_IGN) signal (SIGINT, SIG_IGN);
  if (signal (SIGHUP, termination_handler) == SIG_IGN) signal (SIGHUP, SIG_IGN);
  if (signal (SIGTERM, termination_handler) == SIG_IGN) signal (SIGTERM, SIG_IGN);

  while (1) {
		if (read (STDIN_FILENO, &txChar, 1)>0) {
  		tty_printf("TX: 0x%02X",txChar);
			if (txChar>=32 && txChar<=126) tty_printf(" [%c]",txChar);
			tty_printf("\n");
	  }		
	  
		if (read(tty_fd,&rxChar,1)>0) {
			printf("RX = 0x%02X",rxChar);
			if (rxChar>=32 && rxChar<=126) printf(" [%c]",rxChar);
			printf("\n");
		}	
  }
	return EXIT_SUCCESS;
}

     
