/*
 *  XbeePro_API.c
 *  XbeePro_API
 *
 *  Created by Andrea on 2/5/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#include "xbee.h"
#include <sys/ioctl.h>


/**************** FILE ****************/ 

/*
 * 'open_port()' - Open serial port 1.
 *
 * Returns the file descriptor on success or -1 on error.
 */

int
open_port(char *device, int blocking ) {
	
int fd; /* File descriptor for the port */
	int flags;
	struct termios options;
	
	/* open the port */
 	fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY   );
	// 	fd = open(device, O_RDWR | O_NOCTTY);
	
	
	if (fd == -1) {
		perror("open_port: Unable to open /dev/ttyf1 - ");
		return (fd);
	}
	
	/* Set as blocking */
	fcntl(fd, F_SETFL, 0);
	
/*	 flags=fcntl(fd, F_GETFL);	
	 if(!blocking)
	    fcntl(fd,F_SETFL,flags |O_NONBLOCK);
	 else
	 fcntl(fd,F_SETFL,flags & (~O_NONBLOCK) ); 
*/

	/* get the current options */
	tcgetattr(fd, &options);
	
	/* Control flags */
	options.c_cflag |= (CLOCAL | CREAD);
	options.c_cflag |= CRTSCTS;  /* Enable hw flow controlo */ 
	options.c_lflag &= ~FLUSHO;
	
	/* Input */
	options.c_iflag &= ~(IXON | IXOFF | IXANY); /* Enable sw flow control ... 
												(disabilitato per XON,XOFF)  */
	options.c_iflag &= ~ICRNL;
	options.c_iflag &= ~(INPCK | ISTRIP );
	
	//options.c_iflag &= ~(INLCR | ICRNL);   // prova
	//options.c_iflag &= ~INLCR;  			 // prova 2
	
	//options.c_iflag &= ~IGNCR;   // tolto anche questo, va disabilitando ...
								   // ... il sw flow control
	
	/* set raw input, 1 second timeout */
	options.c_lflag     &= ~(ICANON | ECHO | ECHOE | ISIG);
	options.c_cc[VMIN]  = 1; //vmin;   // Minimum number of characters to read ...
							      // ... set 0 to wait VTIME ds
	options.c_cc[VTIME] = 0; //vtime;  // Time to wait for data (tenths of seconds) ...	
							      // ... if 0 set VMIN>0
	
	/* Raw Output */
	options.c_oflag     &= ~OPOST;
	
	/* 8N1 */
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	
	/*  Set the baud ratrees to B115200 */
	cfsetispeed(&options, B115200);
	cfsetospeed(&options, B115200);

	// cfsetispeed(&options, B38400);
	// cfsetospeed(&options, B38400);

	
	/* set the options */
	tcsetattr(fd, TCSANOW, &options);
	
	/* Flush IO */	
	tcflush(fd,TCIOFLUSH);	
	
	return (fd);
	
}		

int
open_port_read(char *device, int blocking ) {
	
int fd; /* File descriptor for the port */
	int flags;
	struct termios options;
	
	/* open the port */
 	fd = open(device, O_RDONLY | O_NOCTTY | O_NDELAY   );
	// 	fd = open(device, O_RDWR | O_NOCTTY);
	
	
	if (fd == -1) {
		perror("open_port: Unable to open /dev/ttyf1 - ");
		return (fd);
	}
	
	/* Set as blocking */
	fcntl(fd, F_SETFL, 0);
	
/*	 flags=fcntl(fd, F_GETFL);	
	 if(!blocking)
	    fcntl(fd,F_SETFL,flags |O_NONBLOCK);
	 else
	 fcntl(fd,F_SETFL,flags & (~O_NONBLOCK) ); 
*/

	/* get the current options */
	tcgetattr(fd, &options);
	
	/* Control flags */
	options.c_cflag |= (CLOCAL | CREAD);
	options.c_cflag |= CRTSCTS;  /* Enable hw flow controlo */ 
	options.c_lflag &= ~FLUSHO;
	
	/* Input */
	options.c_iflag &= ~(IXON | IXOFF | IXANY); /* Enable sw flow control ... 
												(disabilitato per XON,XOFF)  */
	options.c_iflag &= ~ICRNL;
	options.c_iflag &= ~(INPCK | ISTRIP );
	
	//options.c_iflag &= ~(INLCR | ICRNL);   // prova
	//options.c_iflag &= ~INLCR;  			 // prova 2
	
	//options.c_iflag &= ~IGNCR;   // tolto anche questo, va disabilitando ...
								   // ... il sw flow control
	
	/* set raw input, 1 second timeout */
	options.c_lflag     &= ~(ICANON | ECHO | ECHOE | ISIG);
	options.c_cc[VMIN]  = 1; //vmin;   // Minimum number of characters to read ...
							      // ... set 0 to wait VTIME ds
	options.c_cc[VTIME] = 0; //vtime;  // Time to wait for data (tenths of seconds) ...	
							      // ... if 0 set VMIN>0
	
	/* Raw Output */
	options.c_oflag     &= ~OPOST;
	
	/* 8N1 */
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	
	/*  Set the baud ratrees to B115200 */
	cfsetispeed(&options, B115200);
	cfsetospeed(&options, B115200);

	// cfsetispeed(&options, B38400);
	// cfsetospeed(&options, B38400);

	
	/* set the options */
	tcsetattr(fd, TCSANOW, &options);
	
	/* Flush IO */	
	tcflush(fd,TCIOFLUSH);	
	
	return (fd);
	
}		


int
open_port_write(char *device, int blocking ) {
	
int fd; /* File descriptor for the port */
	int flags;
	struct termios options;
	
	/* open the port */
 	fd = open(device, O_WRONLY | O_NOCTTY | O_NDELAY   );
	// 	fd = open(device, O_RDWR | O_NOCTTY);
	
	
	if (fd == -1) {
		perror("open_port: Unable to open /dev/ttyf1 - ");
		return (fd);
	}
	
	/* Set as blocking */
	fcntl(fd, F_SETFL, 0);
	
/*	 flags=fcntl(fd, F_GETFL);	
	 if(!blocking)
	    fcntl(fd,F_SETFL,flags |O_NONBLOCK);
	 else
	 fcntl(fd,F_SETFL,flags & (~O_NONBLOCK) ); 
*/

	/* get the current options */
	tcgetattr(fd, &options);
	
	/* Control flags */
	options.c_cflag |= (CLOCAL | CREAD);
	options.c_cflag |= CRTSCTS;  /* Enable hw flow controlo */ 
	options.c_lflag &= ~FLUSHO;
	
	/* Input */
	options.c_iflag &= ~(IXON | IXOFF | IXANY); /* Enable sw flow control ... 
												(disabilitato per XON,XOFF)  */
	options.c_iflag &= ~ICRNL;
	options.c_iflag &= ~(INPCK | ISTRIP );
	
	//options.c_iflag &= ~(INLCR | ICRNL);   // prova
	//options.c_iflag &= ~INLCR;  			 // prova 2
	
	//options.c_iflag &= ~IGNCR;   // tolto anche questo, va disabilitando ...
								   // ... il sw flow control
	
	/* set raw input, 1 second timeout */
	options.c_lflag     &= ~(ICANON | ECHO | ECHOE | ISIG);
	options.c_cc[VMIN]  = 1; //vmin;   // Minimum number of characters to read ...
							      // ... set 0 to wait VTIME ds
	options.c_cc[VTIME] = 0; //vtime;  // Time to wait for data (tenths of seconds) ...	
							      // ... if 0 set VMIN>0
	
	/* Raw Output */
	options.c_oflag     &= ~OPOST;
	
	/* 8N1 */
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	
	/*  Set the baud ratrees to B115200 */
	cfsetispeed(&options, B115200);
	cfsetospeed(&options, B115200);

	// cfsetispeed(&options, B38400);
	// cfsetospeed(&options, B38400);

	
	/* set the options */
	tcsetattr(fd, TCSANOW, &options);
	
	/* Flush IO */	
	tcflush(fd,TCIOFLUSH);	
	
	return (fd);
	
}		




void close_port(int fd) {
	/* Flush IO  before to close the device */	
	tcflush(fd,TCIOFLUSH);	
	
	close(fd);
}



/*
 *
 *
 */

int readport(int fd, uint8_ *buffer, int len) {
	
	uint8_ *buf;
	int n,p;
	
	buf = (uint8_ *) malloc(sizeof(uint8_)*len);
	memset(buffer,'\0',len);
	p=0;
	
	do {
		memset(buf,'\0',len);
//		pthread_mutex_lock (&m_io);
		n=read(fd,buf,len-p);
//		pthread_mutex_unlock (&m_io);	
		
		if (n>0) {
			memcpy(buffer+p,buf,n);
			p+=n;
		}
#ifdef VERBOSE_XBEE
			//printf("Read %d\n",n);
#endif
//		usleep(10000);  // aggiunto per porta bloccante con VMIN = 0
	} while (n>0 && p < len);
	

	free(buf);
	
	return p;
	
}


/*
 *
 *
 */

int writeport(int fd, uint8_ *buffer, int len) {
	
	int nWrote;
	int l;
	
	nWrote=0;

	do {
//		pthread_mutex_lock (&m_io);
		l=write(fd,buffer+nWrote,len-nWrote);
//		pthread_mutex_unlock (&m_io);
		nWrote+=l;
	} while (nWrote < len & l > 0);
	
	return nWrote;
	
}



/**************** FILE ****************/ 


/**************** DEBUG ****************/ 

/* 
 * To convert hexadecimal to decimal from characters
 */
long hex2dec_char(uint8_ *buf, int len) {
	long dec;
	long value;
	uint8_ *ptr;
	int i;
	int power;
	
	dec=0;
	value=-1;
	ptr=buf;
	power=len-1;
	
#ifdef VERBOSE_XBEE		
	printf("%s\n",buf);
#endif	
		
		for (i=0;i<len;i++) {
			
			
			switch(*ptr) {
				
				case '0': value = 0; break;				
				case '1': value = 1; break;				
				case '2': value = 2; break;				
				case '3': value = 3; break;				
				case '4': value = 4; break;				
				case '5': value = 5; break;				
				case '6': value = 6; break;				
				case '7': value = 7; break;				
				case '8': value = 8; break;				
				case '9': value = 9; break;				
				case 'a': value = 10; break;				
				case 'b': value = 11; break;				
				case 'c': value = 12; break;				
				case 'd': value = 13; break;				
				case 'e': value = 14; break;				
				case 'f': value = 15; break;			
				case 'A': value = 10; break;				
				case 'B': value = 11; break;				
				case 'C': value = 12; break;				
				case 'D': value = 13; break;				
				case 'E': value = 14; break;				
				case 'F': value = 15; break;
				
				default: value=0; break;
			}	
			
			/* Add this contribution */
			dec +=pow(16,power)*value;
			power--;
			ptr++;		
		}
	return dec;
}


/*  Checksums:
 To calculate:	Not including frame delimiters and length, 
 add all bytes keeping only the lowest 8 
 bits of the result and subtract from 0xFF. 
 */
int checksum_char(uint8_ *pkg) {
	void *ptr;
	int i,len;
	long sum;
	
	sum=0;
	
	ptr=pkg+6;
	
	len=hex2dec_char(pkg+2,4);
	
#ifdef VERBOSE_XBEE	
	printf("Len: %d [%s]\n",len, pkg+2);
#endif	
		
		/* 
		 * Len gives the number of bytes. As 1 byte is 
		 * made of 2 hex characters. Hereby, the len
		 * has to be doubled.
		 */
	for (i=0;i<len*2;i++,i++)
		sum+=hex2dec_char(ptr+i,2);	
	
	
	/* In order to have the right checksum we need to
	 * take the lowest 8 bits and subtract to 0xff
	 */
#ifdef VERBOSE_XBEE		 
	printf("Sum: %ld[%2x]\n",sum,sum);
#endif
		
	sum&=0xff;
	
#ifdef VERBOSE_XBEE		 
	printf("Sum: %ld[%2x]\n",sum,sum);
#endif
		
	return ( 0xff - sum);
	
}


/**************** DEBUG ****************/ 




/**************** API ****************/ 

/* 
 * To convert hexadecimal to decimal
 */

#define BASE16 16
long hex2dec(uint8_ *buf, int len) {
	
	long val;
	int i;
	
	val=0;
	for (i=0;i<len;i++) {
#ifdef VERBOSE_XBEE
		printf("%d\t%ld\t%ld\n",buf[i], (long) pow(BASE16,(len-1-i)*2), (long) buf[i]*(long)pow(BASE16,(len-1-i)*2));
#endif
		val+=  buf[i]*pow(BASE16,(len-1-i)*2);
	}
	
	return val;
	
}

/* 
 * To convert hexadecimal to decimal
 */

#define BASE10 10
long dec2hex(uint8_ *buf, int len) {
	
	/* TODO */ 
}


void INT_2_MSB_LSB(uint8_ *buf, int val) {
	
	buf[0]=0xff & (val>>8);
	buf[1]=0xff & val;
	
}

/* the parameter len is not used, it has been declared only for
 back compability with the old code
 */
int MSB_LSB_2_INT(uint8_ *buf, int len) {
	
	int val=0;
	
	switch (len) {
		case 1:
			val = buf[0];
			break;
		case 2:
			val = buf[0];
			val =  (val << 8) | buf[1];
			break;
		default:
			val=-1;
			break;
	}
	//	printf("%d\n",val);
	return val;
}


/*	To compute checksum 
 
 To calculate:	
 Not including frame delimiters and length, add all bytes 
 keeping only the lowest 8 bits of the result and subtract from 0xFF. 
 */
int checksum(uint8_ *pkg) {
	
	int i,len;
	long sum;
	
	sum=0;
	
	len=MSB_LSB_2_INT(pkg+1,2);
	
#ifdef VERBOSE_XBEE	
	printf("Len: %d\n",len);
#endif	
		
	/* 
	 * Compute sum
	 */
	for(i=0; i<len; i++) {
/* 			if (pkg[3+i]==0x7D) {
 * 				sum += (pkg[3+i+1]^0x20);
 * 				printf("\nXor: %02x\n",pkg[3+i+1]^0x20);
 * 				i++;
 * 			} else {
 */
		sum += MSB_LSB_2_INT(pkg+3+i,1);	
				//printf("\nMSB_LSB... %02x",MSB_LSB_2_INT(pkg+3+i,1));
	//		}
	}
	
#ifdef VERBOSE_XBEE		 
	printf("Sum: %ld[%2x]\n",sum,sum);
#endif
		
	/* 
	 * In order to have the right checksum we need to
	 * take the lowest 8 bits and subtract to 0xff
	 */	 
		
	return ( 0xff - (sum & 0xff));
}


/*
 * Add all bytes (include checksum, but not the delimiter and length).
 * If the checksum is correct, the sum will equal 0xFF
 */
int verify_checksum(uint8_ *pkg) {
	
	int i,len;
	long sum;
	
	sum=0;
	
	/* I must consider also the checksum */
	len=MSB_LSB_2_INT(pkg+1,2)+1;	
	
	/* 
	 * Compute sum
	 */
	for(i=0; i<len; i++)
		sum+=MSB_LSB_2_INT(pkg+3+i,1);	
	
#ifdef VERBOSE_XBEE		 
	printf("Sum: %ld[%2x]\n",sum,sum);
#endif
		
	/* 
	 *	If the lowest byte of the result is 0xff 
	 *	the checksum is correct
	 */	 
	return ( (sum & 0xFF) == 0xFF);
	
}



void print_dbg (uint8_ *buf, int len) {
	
	int i;
	
	for (i=0; i<len; i++)
		printf("%02x ",buf[i]);	
	
	printf("\t");	
}


void print_pkg (uint8_ *buf, int len) {
	
	int i;
	
	for (i=0; i<len; i++)
		printf("%02x ",buf[i]);	
	
	printf("\n");
}


print_pkg_char(uint8_ *buf, int len){
	int i;
	for (i=0;i<len;i++)
		printf("%c",buf[i]);	
	printf("\n");
	
}


/*
int process_pkg(int fd) {
	uint8_ buf[3];
	uint8_ *pkg;
	int len;
	int nRead;
	
	// Look for a command
	nRead=readport(fd,buf,3);
	
	
	if ( nRead== 3) {
		if (buf[0] == 0x7e) {
			len=MSB_LSB_2_INT(buf+1,2);
			if (len>0) {
				printf("Len pkg: %d\t",len);
				pkg = (uint8_ *) malloc(sizeof(uint8_)*(len+1));
				nRead=readport(fd,pkg,len+1);
				print_dbg(buf,3);
				print_pkg(pkg,nRead);
				print_pkg(pkg,nRead);
				print_pkg_char(pkg,nRead);
				free(pkg);
				return 0;
			}
			return -1;
		}
		printf("nRead(in): %d\t",nRead);
		print_pkg(buf,nRead);
		return -1;
	}
	printf("nRead(out): %d\t",nRead);
	print_pkg(buf,nRead);
	return -1;
	
}
*/



/* To parse AT command response pkg */
void at_cmd_response (uint8_* pkg, int len){
	
	printf("API ID: %02x\t",pkg[0]);
	printf("Frame ID: %02x\t",pkg[1]);
	printf("AT Cmd: %c%c\t",pkg[2],pkg[3]);
	printf("Status: %02x\t",pkg[4]);
	
	printf("Value\t");
	print_pkg(pkg+5,len-5);		// We don't wont to plot the checksum :-)
}


/* To parse the TX status pkg */
int ati_tx_status (uint8_* pkg) {
	
#ifdef VERBOSE_XBEE
	printf("API ID: %02x\t",pkg[0]);
	printf("Frame ID: %02x\t",pkg[1]);
	printf("Status:\t");
#endif
	
#ifdef VERBOSE_XBEE
	switch(pkg[2]){
		case 0x00:
			printf("Success [%02x]\n",pkg[2]);
			break;
		case 0x01:
			printf("No ACK Received [%02x]\n",pkg[2]);
			break;
		case 0x02:
			printf("CCA Failure [%02x]\n",pkg[2]);
			break;
		case 0x03:
			printf("Purged [%02x]\n",pkg[2]);
			break;
		default: 
			break;		
	}
#endif
	return pkg[2];
}


/* To parse the TX transmit request pkg */
void at_tx_req_64(uint8_* pkg, int len){
	/* TODO */
	return;
}

/* To parse the TX transmit request pkg */
void at_tx_req_16(uint8_* pkg, int len) {
	
#ifdef VERBOSE_XBEE	
	printf("Frame ID: %02x\t",pkg[0]);
	printf("Dest MSB: %02x\t",pkg[1]);
	printf("Dest LSB: %02x\t",pkg[2]);
#endif
	switch (pkg[3]) {
		case 0x01:
			printf("Disable ACK [%02x]\t",pkg[3]);
			break;
		case 0x04:
			printf("Broadcast PAN ID [%02x]\t",pkg[3]);
			break;
		default: 
			break;		
	}
#ifdef VERBOSE_XBEE
	print_pkg(pkg+4,len-4);
#endif
}


/* To parse the RX package pkg */
int  at_rx_pack_16(uint8_* pkg, int len, void **load){
	
	int iid;
	int size_load;
	
#ifdef VERBOSE_XBEE	
	print_pkg(pkg,len);
	printf("API ID: %02x\t",pkg[0]);
	printf("Source MSB: %02x\t",pkg[1]);
	printf("Source LSB: %02x\t",pkg[2]);
	printf("RSSI: %02x\t",pkg[3]);
	printf("Options: %02x\t",pkg[4]);
	printf("Internal ID: %02x\t",pkg[5]);
	
	switch(pkg[4]){
		case 0x01:
			printf("Address Broadcast [%02x]\t",pkg[4]);
			break;
		case 0x02:
			printf("PAN Broadcast [%02x]\t",pkg[4]);
			break;
		default: 
			break;		
	}
#endif
	
	/* Dispatcher of the internal protocol */
	switch (pkg[5]) {
#ifdef VERBOSE_XBEE
		printf("\nIC: %02x\n",pkg[5]);
#endif
		case pkg_1:
			size_load = sizeof(struct payload_pkg1);
			iid=pkg_1;
#ifdef VERBOSE_XBEE			
			printf("---AT_RX----\n");
			printf("Long: %ld\n",((struct payload_pkg1 *) *load)->i);
			printf("Float: %f\n",((struct payload_pkg1 *) *load)->j);
#endif
			read_pkg(pkg+6,len-6,load,size_load);
				break;
		case pkg_2:
			size_load = sizeof(struct payload_pkg2);
			iid=pkg_2;
#ifdef VERBOSE_XBEE
			printf("---AT_RX----\n");
			printf("Double: %f\n",((struct payload_pkg2 *) *load)->i);
			printf("String: %s\n",((struct payload_pkg2 *) *load)->buf);
#endif
			read_pkg(pkg+6,len-6,load,size_load);
				break;	
		case pkg_time:
			size_load = sizeof(struct payload_pkg_time);
			iid = pkg_time;
			read_pkg(pkg+6,len-6,load,size_load);
				break;
		case pkg_rob_command:
			size_load = sizeof(struct payload_pkg_rob_command);
			iid = pkg_rob_command;
			read_pkg(pkg+6,len-6,load,size_load);
				break;
		case pkg_ir:
			size_load = sizeof(struct payload_pkg_ir);
			iid = pkg_ir;
			read_pkg(pkg+6,len-6,load,size_load);
				break;
		case pkg_acc:
			size_load = sizeof(struct payload_pkg_acc);
			iid = pkg_acc;
			read_pkg(pkg+6,len-6,load,size_load);
				break;
		case pkg_gyro:
			size_load = sizeof(struct payload_pkg_gyro);
			iid = pkg_gyro;
			read_pkg(pkg+6,len-6,load,size_load);
				break;
		case pkg_magneto:
			size_load = sizeof(struct payload_pkg_magneto);
			iid = pkg_magneto;
			read_pkg(pkg+6,len-6,load,size_load);
				break;
		case pkg_state:
			size_load = sizeof(struct payload_pkg_state);
			iid = pkg_state;
			read_pkg(pkg+6,len-6,load,size_load);
				break;
		case pkg_trajectory:
			size_load = sizeof(struct payload_pkg_trajectory);
			iid = pkg_trajectory;
			read_pkg(pkg+6,len-6,load,size_load);
				break;
		case pkg_multi:
			// Struct payload_pkg_multi allocation
			*load = (void *) malloc(sizeof(struct payload_pkg_multi));
			// Copy the four int field
			memcpy(*load, pkg+6, sizeof(struct payload_pkg_multi)-sizeof(void *));
			
			// Pointer inside payload_pkg_multi allocation
			((struct payload_pkg_multi *)*load)->data = (void *) malloc ( ((struct payload_pkg_multi *)*load)->frame_size );
			// Copy the data
			memcpy(((struct payload_pkg_multi *)*load)->data, pkg+6+sizeof(struct payload_pkg_multi), 
				   ((struct payload_pkg_multi *)*load)->frame_size+sizeof(void *));
			
			iid = pkg_multi;
				break;
		case pkg_webcam_data:
			// Struct payload_pkg_webcam_data allocation
			*load = (void *) malloc(sizeof(struct payload_pkg_webcam_data));
			// Copy the two field
			memcpy(*load, pkg+6, sizeof(struct payload_pkg_webcam_data)-sizeof(float *));
			
			// Pointer inside payload_pkg_webcam_data allocation
			((struct payload_pkg_webcam_data *)*load)->data = (void *) malloc ( (((struct payload_pkg_webcam_data *)*load)->n_obj) * sizeof(float)*2);
			// Copy the data
			memcpy(((struct payload_pkg_webcam_data *)*load)->data, pkg+6+sizeof(struct payload_pkg_webcam_data), 
				  ((((struct payload_pkg_webcam_data *)*load)->n_obj) * sizeof(float)*2)+sizeof(float *));
			
			iid = pkg_webcam_data;
				break; 
		case pkg_sensor:
			// Struct payload_pkg_webcam_data allocation
			*load = (void *) malloc(sizeof(struct payload_pkg_sensor));
			
			// Copy the three field
			memcpy(*load, pkg+6, sizeof(struct payload_pkg_sensor)-sizeof(unsigned int *)*2-sizeof(float *)); // 
			
			// Pointer inside payload_pkg_webcam_data allocation
			((struct payload_pkg_sensor *)*load)->range = 
				(void *) malloc ( (((struct payload_pkg_sensor *)*load)->num_canali)*sizeof(unsigned int) );
			((struct payload_pkg_sensor *)*load)->converted = 
				(void *) malloc ( (((struct payload_pkg_sensor *)*load)->num_canali)*sizeof(float) );
			((struct payload_pkg_sensor *)*load)->bias = 
				(void *) malloc ( (((struct payload_pkg_sensor *)*load)->num_canali)*sizeof(unsigned int) );
			
			// Copy the data			
			memcpy(((struct payload_pkg_sensor *)*load)->range, 
				   pkg+6+sizeof(struct payload_pkg_sensor), 
				  (((struct payload_pkg_sensor *)*load)->num_canali)*sizeof(unsigned int));
			memcpy(((struct payload_pkg_sensor *)*load)->converted, 
				   pkg+6+sizeof(struct payload_pkg_sensor)+(((struct payload_pkg_sensor *)*load)->num_canali)*sizeof(unsigned int), 
				  (((struct payload_pkg_sensor *)*load)->num_canali)*sizeof(float));
			memcpy(((struct payload_pkg_sensor *)*load)->converted, 
				   pkg+6+sizeof(struct payload_pkg_sensor)+(((struct payload_pkg_sensor *)*load)->num_canali)*sizeof(unsigned int)+(((struct payload_pkg_sensor *)*load)->num_canali)*sizeof(float), 
				  (((struct payload_pkg_sensor *)*load)->num_canali)*sizeof(unsigned int));
			
			iid = pkg_sensor;
				break;
		default:
#ifdef VERBOSE_XBEE
			print_pkg(pkg+6,len-6);
			printf("No internal command\n");
#endif
			iid=-1;
	}

	/*if (iid!=pkg_multi & iid!=pkg_webcam_data)
		read_pkg(pkg+6,len-6,load,size_load);
	 */
	
	return iid;
}

/*  Dispatcher */
int pkg_dispatcher(uint8_ *pkg, int len, int *iid, void **load) {
	
	int id, i;	
	*iid=-1;
	
	/* API types */
	switch(pkg[0]){
		case (modem_status):	
#ifdef VERBOSE_XBEE
			printf("modem_status:\t");
			print_pkg(pkg,len);
#endif
			id = modem_status;			
			break;
		case (at_cmd):	
#ifdef VERBOSE_XBEE			
			printf("at_cmd:\t");
			print_pkg(pkg,len);
#endif			
			id=at_cmd;			
			break;
		case (at_cmd_queue):	
#ifdef VERBOSE_XBEE			
			printf("at_cmd queue:\t");
			print_pkg(pkg,len);
#endif			
			id=at_cmd_queue;			
			break;
		case (at_cmd_resp):	
#ifdef VERBOSE_XBEE			
			printf("at_cmd_response:\t");
			at_cmd_response(pkg,len-1);		
#endif			
			id=at_cmd_resp;
			break;			
		case (tx_req_64):	
#ifdef VERBOSE_XBEE			
			printf("tx_req_64:\t");
			print_pkg(pkg,len);
#endif			
			id=tx_req_64;			
			break;
		case (tx_req_16):	
#ifdef VERBOSE_XBEE			
			printf("tx_req_16:\t");
#endif
			//			print_dbg(pkg,len);	
			at_tx_req_16(pkg,len);
			id=tx_req_16;
			break;
		case (tx_status):	
#ifdef VERBOSE_XBEE			
			printf("tx_status:\t");
#endif			
			//			print_dbg(pkg,len);
			ati_tx_status(pkg);	
			id=tx_status;			
			break;
		case (rx_pack_64):	
#ifdef VERBOSE_XBEE			
			printf("rx_pack_64:\t");
			print_pkg(pkg+1,len-1);
#endif			
			id=rx_pack_64;			
			break;
		case (rx_pack_16):	
#ifdef VERBOSE_XBEE			
			printf("rx_pack_16:\t");
#endif			
			id=rx_pack_16;
			*iid = at_rx_pack_16 (pkg,len,load);
			
#ifdef VERBOSE_DISPATCHER
			printf("\n----------- DISPATCHER -----------\n");
			if (*iid==pkg_1) {	
				printf("Long: %ld\n",((struct payload_pkg1 *) *load)->i);
				printf("Float: %f\n",((struct payload_pkg1 *) *load)->j);
			} else if (*iid==pkg_2) {
				printf("Double: %lf\n",((struct payload_pkg2 *) *load)->i);
				printf("String: %s\n",((struct payload_pkg2 *) *load)->buf);
			} else if (*iid==pkg_time) {
				printf("Long: %ld\n",((struct payload_pkg_time *) *load)->i);
				printf("Seconds: %ld\n",((struct payload_pkg_time *) *load)->time.tv_sec);
				printf("Microseconds: %ld\n",((struct payload_pkg_time *) *load)->time.tv_usec);
			} else if (*iid==pkg_rob_command) {
				printf("Command: %c\n",((struct payload_pkg_rob_command *) *load)->command[0]);
				printf("Value: %d\n",((struct payload_pkg_rob_command *) *load)->value);
			} else if (*iid==pkg_ir) {
				printf("Canali: %d \t d2c: %f \t is_valid: %d\n",((struct payload_pkg_ir * )*load)->num_canali,
																 ((struct payload_pkg_ir * )*load)->d2c,
																 ((struct payload_pkg_ir * )*load)->is_valid);
				for (i=0; i<((struct payload_pkg_ir * )load)->num_canali; i++) {
					printf("range: %d \t converted: %f \t bias: %d\n",((struct payload_pkg_ir * )*load)->range[i],
						   ((struct payload_pkg_ir * )*load)->converted[i],((struct payload_pkg_ir * )*load)->bias[i]);
				}
			}
			else if (*iid==pkg_acc) {
				printf("Canali: %d \t d2c: %f \t is_valid: %d\n",((struct payload_pkg_acc * )*load)->num_canali,
																 ((struct payload_pkg_acc * )*load)->d2c,
																 ((struct payload_pkg_acc * )*load)->is_valid);
				for (i=0; i<((struct payload_pkg_acc * )*load)->num_canali; i++) {
					printf("range: %d \t converted: %f \t bias: %d\n",((struct payload_pkg_acc * )*load)->range[i],
						   ((struct payload_pkg_acc * )*load)->converted[i],((struct payload_pkg_acc * )*load)->bias[i]);
				}
			}
			else if (*iid==pkg_gyro) {
				printf("Canali: %d \t d2c: %f \t is_valid: %d\n",((struct payload_pkg_gyro * )*load)->num_canali,
																 ((struct payload_pkg_gyro * )*load)->d2c,
																 ((struct payload_pkg_gyro * )*load)->is_valid);
				for (i=0; i<((struct payload_pkg_gyro * )*load)->num_canali; i++) {
					printf("range: %d \t converted: %f \t bias: %d\n",((struct payload_pkg_gyro * )*load)->range[i],
						   ((struct payload_pkg_gyro * )*load)->converted[i],((struct payload_pkg_gyro * )*load)->bias[i]);
				}
			}
			else if (*iid==pkg_magneto) {
				printf("Canali: %d \t d2c: %f \t is_valid: %d\n",((struct payload_pkg_magneto * )*load)->num_canali,
																 ((struct payload_pkg_magneto * )*load)->d2c,
																 ((struct payload_pkg_magneto * )*load)->is_valid);
				for (i=0; i<((struct payload_pkg_magneto * )*load)->num_canali; i++) {
					printf("range: %d \t converted: %f \t bias: %d\n",((struct payload_pkg_magneto * )*load)->range[i],
						   ((struct payload_pkg_magneto * )*load)->converted[i],((struct payload_pkg_magneto * )*load)->bias[i]);
				}
			}
			else if (*iid==pkg_state) {
				printf("x: %f \t y: %f \t th: %f\n",
					((struct payload_pkg_state *)*load)->x,
					((struct payload_pkg_state *)*load)->y,
					((struct payload_pkg_state *)*load)->th);
			}
			else if (*iid==pkg_multi) {
				printf ("Multi_pkg_ID: %d \t total_size: %d \t frame: %d \t frame_size: %d\n",((struct payload_pkg_multi *)*load)->id,
						((struct payload_pkg_multi *)*load)->total_size, ((struct payload_pkg_multi *)*load)->frame,
						((struct payload_pkg_multi *)*load)->frame_size);
			}
			else if (*iid==pkg_webcam_data) {
				printf("N_obj: %d \t Time: %ld\n",((struct payload_pkg_webcam_data *)*load)->n_obj,((struct payload_pkg_webcam_data *)*load)->time);
				for (i=0; i<((struct payload_pkg_webcam_data * )*load)->n_obj; i++)
					printf("Object %d \t x: %f y: %f\n",i,((struct payload_pkg_webcam_data * )*load)->data[i*2],
														  ((struct payload_pkg_webcam_data * )*load)->data[i*2+1]);
			}
			else if (*iid==pkg_sensor) {
				printf("Numero canali: %u\n",((struct payload_pkg_sensor *)*load)->num_canali);
				printf("d2c: %f\n",((struct payload_pkg_sensor *)*load)->d2c);
				printf("is_valid: %u\n",((struct payload_pkg_sensor *)*load)->is_valid);
				for (i=0; i<((struct payload_pkg_sensor *)*load)->num_canali; i++) {
					printf("-------------------------- Channel %d ----------------------------------\n",i);
					printf("Range: %u\t",((struct payload_pkg_sensor * )*load)->range[i]);
					printf("Converted: %f\t",((struct payload_pkg_sensor * )*load)->converted[i]);
					printf("Bias: %u\n",((struct payload_pkg_sensor * )*load)->bias[i]);
				}
			}
#endif 
		default:
#ifdef VERBOSE_XBEE		
			printf("%2x\n",pkg[0]);
#endif			
			break;	
	}
	return id;
}



/*  */
int pkg_parser(int fd, int *iid, void **load, int *sender) {
	
	uint8_ buf[100];
	uint8_ *pkg;
	int len;
	int nRead;
	int id, ver_checksum;
	uint8_ *total_pkg;
	
	*iid = -1; 
		pthread_mutex_lock (&m_io);
	nRead=read(fd,buf,3);
		pthread_mutex_unlock (&m_io);	
	

	if ( nRead== 3) {
		if (buf[0] == 0x7e) {
			len=MSB_LSB_2_INT(buf+1,2);
			if (len>0) {
				pkg = (uint8_ *) malloc(sizeof(uint8_)*(len+1)); // there is also 
				nRead=readport(fd,pkg,len+1);
				if (nRead<len+1) {
                    *sender = pkg[2];
					return READ_ERROR;
				}
				id=pkg_dispatcher(pkg,nRead,iid,load);
				*sender = pkg[2];				     
				free(pkg);			
				return id;
			}
		}
		return -1;		
	}

	/*	pthread_mutex_lock (&m_io);
	nRead=read(fd,mau_buf,100);

		pthread_mutex_unlock (&m_io);
	
	if ( nRead>= 3) {
		printf("LETTO: %d******************\n", nRead);

		if (mau_buf[0] == 0x7e) {
			len=MSB_LSB_2_INT(mau_buf+1,2);
			printf("xxLEN: %d xx\n", len);
			if (len==nRead-4) {
				//pkg = (uint8_ *) malloc(sizeof(uint8_)*(len+1)); // there is also
				//pkg = (uint8_ *) malloc(sizeof(uint8_)*100); // there is also  
				//nRead=readport(fd,pkg,len+1);
				//printf("step1\n");
				//memcpy(pkg, &(buf[3]), len+1);
				//printf("step2\n");
				//if (nRead<len+1) {
                //    *sender = pkg[2];
				//	return READ_ERROR;
				//}
				//id=pkg_dispatcher(pkg,nRead,iid,load);
				id=pkg_dispatcher(&(mau_buf[3]),len,iid,load);
				printf("ID: %d\n", id);
				printf("step3\n");				
				//h    *sender = pkg[2];
				*sender = mau_buf[5];				     				     
				printf("step4\n");
				//free(pkg);	
				printf("step5\n");		
				return id;
			}


		}
		return -1;		
	}*/

	return -1;
}


/*	Packet Details...
 0x7e							1
 MSB Len						1
 LSB Len						1
 FRAME DATA	---------------------
 	API ID						1
 	FRAME ID					1
 	MSB	Dest					1
 	LSB Dest					1
 	OPT							1		
 	DATA	---------------------
 		INTERNAL PROTOCOL ID	1
 		REAL DATA			sizeof(struct payload_pkg1)		
 	END DATA --------------------
END FRAME DATA ------------------
CRC								1
	 
*/

int Send_Data_16(int fd, int dest, int iid, void *load, int size_load, int ack)
{
	uint8_* pkg;
	int lenFrame;
	int lenPCMD;
	int wrote, ver_chks;

	
	/* Length of the data frame  */ 
	lenFrame = 6 + size_load;  // size_load + preamble + checksum
	
	/* Length of the whole packet */
	lenPCMD = SIZE_FRAME + lenFrame ;
	
#ifdef VERBOSE_XBEE
	printf("\n\nNew packet to write\n");
	printf("Packet Len: %d\n",lenPCMD);
#endif
	
	/* Build Packet */
	pkg=(uint8_ *)malloc(sizeof(uint8_)*lenPCMD);
	
	memset(pkg,'0',lenPCMD);	
	pkg[0]= 0x7e;
	
	INT_2_MSB_LSB(pkg+1,lenFrame);	// MSB + LSB Packet Length (without crc)
#ifdef VERBOSE_XBEE
	printf("(Length) MSB: %02x\tLSB: %02x\n",pkg[1],pkg[2]);
#endif
	
	pkg[3]= 0x01;					// API ID
	
	
	/* FRAME ID */
	if (ack) {
		/* Randomly generate the frame id */
		//srand(getpid());				// FRAME ID
		//pkg[4]=  (uint8_) 30*(rand()/(double) RAND_MAX);	// Random frame ID

//		srand ( (unsigned)time ( NULL ) );
//		pkg[4] = (uint8_) ((rand() % 255) + 1);   // between 1 and 255
		
		if (++Frame_id == 0)
			Frame_id++;
		pkg[4] = (uint8_)Frame_id;
	}
	else
		pkg[4]= 0x00;  // No tx_status ack
	

	
#ifdef VERBOSE_XBEE	
	printf("\nFrame id: %d\n",Frame_id);
	printf("FRAME ID: %02x\n",pkg[4]);
#endif
	
	// Send broadcast
	if (dest==0xff) {
		pkg[5] = 0xff;
		pkg[6] = 0xff;
	}
	else
		INT_2_MSB_LSB(pkg+5,dest);		// MSB + LSB Destination Address

#ifdef VERBOSE_XBEE	
	printf("(Destination) MSB: %02x\tLSB: %02x\n",pkg[5],pkg[6]);
#endif
	pkg[7]= 0x00;					// Options: 0x01 to disable ack
									//          0x04 to send packet with broadcast PAN ID
	
	pkg[8]= iid;					// Internal Protocol ID
	
	// Copy the load into the pkg to send
	if (iid!=pkg_multi & iid!=pkg_webcam_data & iid!=pkg_sensor)
		memcpy(pkg+9,load,size_load);	// Payload 
	else if (iid==pkg_webcam_data) {
		memcpy(pkg+9,load,sizeof(struct payload_pkg_webcam_data));	// Payload 
		memcpy(pkg+9+sizeof(struct payload_pkg_webcam_data),((struct payload_pkg_webcam_data *)load)->data,
			               ((struct payload_pkg_webcam_data *)load)->n_obj*sizeof(float)*2);	// Payload 
	}
	else if (iid==pkg_multi) {
		memcpy(pkg+9,load,sizeof(struct payload_pkg_multi));	// Payload 
		memcpy(pkg+9+sizeof(struct payload_pkg_multi),((struct payload_pkg_multi *)load)->data,
			               ((struct payload_pkg_multi *)load)->frame_size);	// Payload 
	} else {
		
		int size_range = sizeof(unsigned int)*((struct payload_pkg_sensor *)load)->num_canali;
						 
		

		// Copy the three variables
		memcpy (pkg+9,load,sizeof(unsigned int)*2+sizeof(float));
		
		// Copy the 'range' pointer
		memcpy (pkg+9+sizeof(unsigned int)*2+sizeof(float),((struct payload_pkg_sensor *)load)->range,
				sizeof(unsigned int)*((struct payload_pkg_sensor *)load)->num_canali);
		
		// Copy the 'converted' pointer
		memcpy (pkg+9+sizeof(unsigned int)*2+sizeof(float)+sizeof(unsigned int)*((struct payload_pkg_sensor *)load)->num_canali,
				((struct payload_pkg_sensor *)load)->converted,
				sizeof(float)*((struct payload_pkg_sensor *)load)->num_canali);
		
		// Copy the 'bias' pointer
		memcpy (pkg+9+sizeof(unsigned int)*2+sizeof(float)+sizeof(unsigned int)*((struct payload_pkg_sensor *)load)->num_canali+sizeof(float)*((struct payload_pkg_sensor *)load)->num_canali,
				((struct payload_pkg_sensor *)load)->bias,
				sizeof(unsigned int)*((struct payload_pkg_sensor *)load)->num_canali);
		
		/*
		memcpy (pkg+9,load,sizeof(struct payload_pkg_sensor));
		
		memcpy (pkg+9+sizeof(struct payload_pkg_sensor), ((struct payload_pkg_sensor *)load)->range, 
				sizeof(unsigned int)*((struct payload_pkg_sensor *)load)->num_canali);
		
		memcpy (pkg+9+sizeof(struct payload_pkg_sensor)+sizeof(unsigned int)*((struct payload_pkg_sensor *)load)->num_canali,
				((struct payload_pkg_sensor *)load)->converted,
				sizeof(float)*((struct payload_pkg_sensor *)load)->num_canali);
				
		memcpy (pkg+9+sizeof(struct payload_pkg_sensor)+sizeof(unsigned int)*((struct payload_pkg_sensor *)load)->num_canali+sizeof(float)*((struct payload_pkg_sensor *)load)->num_canali,
				((struct payload_pkg_sensor *)load)->bias,
				sizeof(unsigned int)*((struct payload_pkg_sensor *)load)->num_canali);		
		 */
	}
	
	pkg[lenPCMD-1]=checksum(pkg);   // Checksum
#ifdef VERBOSE_XBEE	
	printf("Checksum: %02x",pkg[lenPCMD-1]);
#endif
	
	/* Send package */	
#ifdef VERBOSE_XBEE
	printf("\nTo Write: ");
	print_dbg(pkg,lenPCMD);
#endif
	
	wrote = writeport(fd,pkg,lenPCMD);
	
	if (wrote<0)
		printf("\nError to write");
	
	ver_chks = verify_checksum(pkg);
#ifdef VERBOSE_XBEE		
	printf("Wrote: %d\n",wrote);
	printf("Verify pkg: %d\n",ver_chks);
#endif
	
	free(pkg);
	
	return 1;
}


int Send_Data_16_multi(int fd, int dest, void *load, int size_load, int ack) {
	
	int n_frame,     // Number of frames to send
	size_last_frame, // Size of last frame
	i, j,
	max_data_size = MAX_LOAD_SIZE - sizeof(struct payload_pkg_multi);   // Max partial load size 
	int size_frame_prec = 0;                                            // Sum of partial load just wrote
	struct payload_pkg_multi *pkg_i; 		// Struct to send pkg with payload > 100 Bytes
	
	// Struct memory allocation
	pkg_i = (struct payload_pkg_multi *) malloc(sizeof (struct payload_pkg_multi));
	
	// Number of frames
	n_frame = size_load/max_data_size;
	
	// Size of last frame
	size_last_frame = size_load % max_data_size;
	
	if ( size_last_frame > 0)
		n_frame++;
	else if (size_last_frame == 0)
		size_last_frame = max_data_size;
	
	// Random ID
	srand(getpid());
	pkg_i->id = 30*(rand()/(double) RAND_MAX);	
	
	// Total size
	pkg_i->total_size = size_load;
	
	// Wrote necessary pkg
	for (i=0; i<n_frame; i++) {
		
		// Frame number
		pkg_i->frame = i;
		
		if (i==n_frame-1) {
#ifdef VERBOSE_XBEE
			printf ("\n-------------------- Ultimo pacchetto: %d ---------------------------------\n", i);
#endif
			// Size of last frame
			pkg_i->frame_size = size_last_frame;
		}
		else {
#ifdef VERBOSE_XBEE
			printf ("\n-------------------- Pacchetto numero: %d ---------------------------------\n", i);
#endif
			// Size of frame
			pkg_i->frame_size = max_data_size;
		}
		
		// Pointer memory allocation
		pkg_i->data = (void *) malloc (pkg_i->frame_size);
		
		// Copy partial data 
		memcpy (pkg_i->data, load + size_frame_prec, pkg_i->frame_size);
		
		// Update the number of wrote bytes
		size_frame_prec += pkg_i->frame_size;

		if (i==n_frame-1)
			// Send last frame
			Send_Data_16(fd, dest, pkg_multi, (void *)pkg_i, size_last_frame + sizeof(struct payload_pkg_multi), ack);
		else
			// Send frame
			Send_Data_16(fd, dest, pkg_multi, (void *)pkg_i, MAX_LOAD_SIZE, ack);
		
		// Free memory
		free (pkg_i->data);
		
		usleep (80000);
	}
	
	// Free memory
	free (pkg_i);
	
	return 1;
}

/* Package Size */

/*	Packet Details...
 0x7e									1
 MSB Len								1
 LSB Len				*load = (void *) malloc(size_load);
	memcpy(*load, buf, size_load);				1
 FRAME DATA	----------------------
 	API ID								1
 	MSB Source							1
 	LSB Source							1
 	RSSI								1
 	OPT									1		
 	DATA	----------------------
 		INTERNAL PROTOCOL ID			1
 		REAL DATA				sizeof(struct payload_pkgX)		
 	END DATA ---------------------
 END FRAME DATA ------------------
 CRC										1
 
 */


// READ GENERICA
void read_pkg(uint8_ *buf, int len, void **load, int size_load) {
	*load = (void *) malloc(size_load);
	memcpy(*load, buf, size_load);
}




#ifdef CONSOLE
//****************************************************************************************************************
//********************************* C O N S O L E ****************************************************************
//****************************************************************************************************************
//void 	console_exec_vel_cmd(int v);
//void 	console_exec_servoing_cmd(int steps);

int console_init(void) {
	struct termios tattr;
	
	//console_fd=STDIN_FILENO;
	
	// Make sure stdin is a terminal
	if (!isatty (STDIN_FILENO)) {
            #ifdef VERBOSE_XBEE
		fprintf (stderr,"stdin is not a terminal\n");
            #endif
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


void console_parsing(int fd) {
	
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
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
#endif
