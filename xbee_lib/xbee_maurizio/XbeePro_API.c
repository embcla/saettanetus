/*
 *  XbeePro_API.c
 *  XbeePro_API
 *
 *  Created by Andrea on 2/5/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#include "XbeePro_API.h"

/**************** FILE ****************/ 

/*
 * 'open_port()' - Open serial port 1.
 *
 * Returns the file descriptor on success or -1 on error.
 */

int
open_port(char *device, int blocking, int vmin, int vtime) {
	
	int fd; /* File descriptor for the port */
	int flags;
	struct termios options;
	
	/* Open the port.
	 * 
	 */
	fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);   
	//fd = open(device, O_RDWR | O_NOCTTY);   // prova
	
	 // Set the lines specified by the bit mask "iomask" to 1
	 //int iomask;  
	 //iomask = 1<<5;
	 //ioctl(fd,_IO(ETRAXGPIO_IOCTYPE,IO_SETBITS),iomask);

	
	if (fd == -1) {
		perror("open_port: Unable to open device - ");
		return (fd);
	}
	
	
	if (blocking)
		/* Set as blocking */
		fcntl(fd, F_SETFL, 0);
	else
		/* Set as non blocking */
		fcntl(fd, F_SETFL, FNDELAY);
	
	/*	 flags=fcntl(fd, F_GETFL);	
	fcntl(fd,F_SETFL,flags |O_NONBLOCK);
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
	options.c_cc[VMIN]  = vmin;   // Minimum number of characters to read ...
							      // ... set 0 to wait VTIME ds
	options.c_cc[VTIME] = vtime;  // Time to wait for data (tenths of seconds) ...	
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
	
	/* set the options */
	tcsetattr(fd, TCSANOW, &options);
	
	/* Flush IO  before to close the device */	
	tcflush(fd,TCIOFLUSH);	
	
	return (fd);
	
}


void
close_port(int fd) 
{
	/* Flush IO  before to close the device */	
	tcflush(fd,TCIOFLUSH);	
	
	close(fd);
}

void flushport(int fd) {
	tcflush(fd,TCIOFLUSH);	
}


/*
 *
 *
 */

int readport(int fd, uint8 *buffer, int len) {
	
	uint8 *buf;
	int n,p;
	
	buf = (uint8 *) malloc(sizeof(uint8)*len);
	memset(buffer,'\0',len);
	p=0;
	
	
	do{
		memset(buf,'\0',len);
		n=read(fd,buf,len-p);
		
		if (n>0) {
			memcpy(buffer+p,buf,n);
			p+=n;
		}
		//else	// Da commentare per porta non bloccante
		//	perror("Error");
#ifdef VERBOSE
		//printf("Read %d\n",n);
#endif
	usleep(10000);  // aggiunto per porta bloccante con VMIN = 0
	} while (n>0 && p < len);
	//} while ( p < len);   // diventa bloccante!!!

	free(buf);
	
	return p;
	
}


/*
 *
 *
 */

int writeport(int fd, uint8 *buffer, int len) {
	
	int nWrote;
	int l;
	
	nWrote=0;
	
	do {
		l=write(fd,buffer+nWrote,len-nWrote);
		nWrote+=l;
	} while (nWrote < len & l > 0);
	
	return nWrote;
	
}



/**************** FILE ****************/ 


/**************** DEBUG ****************/ 

/* 
 * To convert hexadecimal to decimal from characters
 */
long hex2dec_char(uint8 *buf, int len) {
	long dec;
	long value;
	uint8 *ptr;
	int i;
	int power;
	
	dec=0;
	value=-1;
	ptr=buf;
	power=len-1;
	
#ifdef VERBOSE		
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
int checksum_char(uint8 *pkg) {
	void *ptr;
	int i,len;
	long sum;
	
	sum=0;
	
	ptr=pkg+6;
	
	len=hex2dec_char(pkg+2,4);
	
#ifdef VERBOSE	
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
#ifdef VERBOSE		 
	printf("Sum: %ld[%2x]\n",sum,sum);
#endif
		
		sum&=0xff;
	
#ifdef VERBOSE		 
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
long hex2dec(uint8 *buf, int len) {
	
	long val;
	int i;
	
	val=0;
	for (i=0;i<len;i++) {
#ifdef VERBOSE
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
long dec2hex(uint8 *buf, int len) {
	
	/* TODO */ 
}


void INT_2_MSB_LSB(uint8 *buf, int val) {
	
	buf[0]=0xff & (val>>8);
	buf[1]=0xff & val;
	
}

/* the parameter len is not used, it has been declared only for
 back compability with the old code
 */
int MSB_LSB_2_INT(uint8 *buf, int len) {
	
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
int checksum(uint8 *pkg) {
	
	int i,len;
	long sum;
	
	sum=0;
	
	len=MSB_LSB_2_INT(pkg+1,2);
	
#ifdef VERBOSE	
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
	
#ifdef VERBOSE		 
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
int verify_checksum(uint8 *pkg) {
	
	int i,len;
	long sum;
	
	sum=0;
	
	/* I must consider also the checksum */
	len=MSB_LSB_2_INT(pkg+1,2)+1;	
	
	/* 
	 * Compute sum
	 */
	for(i=0; i<len; i++)
/* 		if (pkg[3+i]==0x7D) {
 * 			sum += (pkg[3+i+1]^0x20);
 * 			printf("\nXor: %02x\n",pkg[3+i+1]^0x20);
 * 			i++;
 * 		} else {
 */
			sum+=MSB_LSB_2_INT(pkg+3+i,1);	
			//printf("\nMSB_LSB... %02x",MSB_LSB_2_INT(pkg+3+i,1));
		//}
	
#ifdef VERBOSE		 
	printf("Sum: %ld[%2x]\n",sum,sum);
#endif
		
		/* 
		 *	If the lowest byte of the result is 0xff 
		 *	the checksum is correct
		 */	 
		return ( (sum & 0xFF) == 0xFF);
	
}



void print_dbg(uint8 *buf, int len){
	
	int i;
	
	for (i=0;i<len;i++)
		printf("%02x ",buf[i]);	
	
	printf("\t");	
}


void print_pkg(uint8 *buf, int len){
	
	int i;
	
	for (i=0;i<len;i++)
		printf("%02x ",buf[i]);	
	
	printf("\n");
}


print_pkg_char(uint8 *buf, int len){
	int i;
	for (i=0;i<len;i++)
		printf("%c",buf[i]);	
	printf("\n");
	
}


int process_pkg(int fd) {
	uint8 buf[3];
	uint8 *pkg;
	int len;
	int nRead;
	
	/* Look for a command */
	nRead=readport(fd,buf,3);
	
	
	if ( nRead== 3) {
		if (buf[0] == 0x7e) {
			len=MSB_LSB_2_INT(buf+1,2);
			if (len>0) {
				printf("Len pkg: %d\t",len);
				pkg = (uint8 *) malloc(sizeof(uint8)*(len+1));
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



/* To parse AT command response pkg */
void at_cmd_response(uint8* pkg, int len){
	
	printf("API ID: %02x\t",pkg[0]);
	printf("Frame ID: %02x\t",pkg[1]);
	printf("AT Cmd: %c%c\t",pkg[2],pkg[3]);
	printf("Status: %02x\t",pkg[4]);
	
	printf("Value\t");
	print_pkg(pkg+5,len-5);		// We don't wont to plot the checksum :-)
}


/* To parse the TX status pkg */
void ati_tx_status(uint8* pkg){
	
#ifdef VERBOSE
	printf("API ID: %02x\t",pkg[0]);
	printf("Frame ID: %02x\t",pkg[1]);
	printf("Status:\t");
#endif
	
#ifdef VERBOSE
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
			default: break;		
			
	}
#endif
}


/* To parse the TX transmit request pkg */
void at_tx_req_64(uint8* pkg, int len){
	return;
}

/* To parse the TX transmit request pkg */
void at_tx_req_16(uint8* pkg, int len){
	
#ifdef VERBOSE	
	printf("Frame ID: %02x\t",pkg[0]);
	printf("Dest MSB: %02x\t",pkg[1]);
	printf("Dest LSB: %02x\t",pkg[2]);
#endif
	switch(pkg[3]){
		case 0x01:
			printf("Disable ACK [%02x]\t",pkg[3]);
		case 0x04:
			printf("Broadcast PAN ID [%02x]\t",pkg[3]);
			break;
		default: 
			break;		
	}
#ifdef VERBOSE
	print_pkg(pkg+4,len-4);
#endif
}


/* To parse the RX package pkg */
int  at_rx_pack_16(uint8* pkg, int len, void **load){
	
	int iid;
	
#ifdef VERBOSE	
	print_pkg(pkg,len);
	printf("API ID: %02x\t",pkg[0]);
	printf("Source MSB: %02x\t",pkg[1]);
	printf("Source LSB: %02x\t",pkg[2]);
	printf("RSSI: -%02x\t",pkg[3]);
#endif
	
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
	
	/* Dispatcher of the internal protocol */
	
	switch (pkg[5]) {
		
		case pkg_1:
			printf("\nIC: %02x\n",pkg[5]);
			read_pkg_1(pkg+6,len-6,load);	
			iid=pkg_1;
#ifdef VERBOSE			
			printf("---AT_RX----\n");
			printf("Long: %ld\n",((struct payload_pkg1 *) *load)->i);
			printf("Float: %f\n",((struct payload_pkg1 *) *load)->j);
#endif
				break;
		case pkg_2:
#ifdef VERBOSE
			printf("\nIC: %02x\n",pkg[5]);
#endif
			read_pkg_2(pkg+6,len-6,load);	
			iid=pkg_2;
#ifdef VERBOSE
			printf("---AT_RX----\n");
			printf("Double: %f\n",((struct payload_pkg2 *) *load)->i);
			printf("String: %s\n",((struct payload_pkg2 *) *load)->buf);
#endif
				break;	
		case pkg_time:
#ifdef VERBOSE
			printf("\nIC: %02x\n",pkg[5]);
#endif
			read_pkg_time(pkg+6,len-6,load);
			iid = pkg_time;
				break;
		case pkg_rob_command:
#ifdef VERBOSE
			printf("\nIC: %02x\n",pkg[5]);
#endif
			read_pkg_time(pkg+6,len-6,load);
			iid = pkg_rob_command;
				break;
		default:
#ifdef VERBOSE
			//print_pkg(pkg+6,len-6);
#endif
			iid=-1;
	}
	
	return iid;
}

/*  Dispatcher */
int pkg_dispatcher(uint8 *pkg, int len, int * iid, void **load) {
	
	int id;	
	
	*iid=-1;
	
	/* API types */
	switch(pkg[0]){
		case (modem_status):	
#ifdef VERBOSE
			printf("modem_status:\t");
			print_pkg(pkg,len);
#endif
			id = modem_status;			
			break;
		case (at_cmd):	
#ifdef VERBOSE			
			printf("at_cmd:\t");
			print_pkg(pkg,len);
#endif			
			id=at_cmd;			
			break;
		case (at_cmd_queue):	
#ifdef VERBOSE			
			printf("at_cmd queue:\t");
			print_pkg(pkg,len);
#endif			
			id=at_cmd_queue;			
			break;
		case (at_cmd_resp):	
#ifdef VERBOSE			
			printf("at_cmd_response:\t");
			at_cmd_response(pkg,len-1);		
#endif			
			id=at_cmd_resp;
			break;			
		case (tx_req_64):	
#ifdef VERBOSE			
			printf("tx_req_64:\t");
			print_pkg(pkg,len);
#endif			
			id=tx_req_64;			
			break;
		case (tx_req_16):	
#ifdef VERBOSE			
			printf("tx_req_16:\t");
#endif
			//			print_dbg(pkg,len);	
			at_tx_req_16(pkg,len);
			id=tx_req_16;
			break;
		case (tx_status):	
#ifdef VERBOSE			
			printf("tx_status:\t");
#endif			
			//			print_dbg(pkg,len);
			ati_tx_status(pkg);	
			id=tx_status;			
			break;
		case (rx_pack_64):	
#ifdef VERBOSE			
			printf("rx_pack_64:\t");
			print_pkg(pkg+1,len-1);
#endif			
			id=rx_pack_64;			
			break;
		case (rx_pack_16):	
#ifdef VERBOSE			
			printf("rx_pack_16:\t");
#endif			
			*iid=at_rx_pack_16(pkg,len,load);
			
#ifdef VERBOSE
			printf("---DISPATCHER----\n");
			
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
				printf("Command: %c\n",((struct payload_pkg_rob_command *) *load)->command);
				printf("Value: %d\n",((struct payload_pkg_rob_command *) *load)->value);
			} else {
				printf("Unknow Package\n");
			}
#endif
			id=rx_pack_16;
			break;
		default:
#ifdef VERBOSE		
			printf("%2x\n",pkg[0]);
#endif			
			break;	
	}
	return id;
}



/*  */
int pkg_parser(int fd, int *iid, void **load) {
	
	uint8 buf[3];
	uint8 *pkg;
	int len;
	int nRead;
	int id;
	
	*iid = -1; 
	
	/* Look for a command, read start delimiter, MSB and LSB */
	nRead=readport(fd,buf,3);
	
	if ( nRead== 3) {
	
		/* Verify the start delimiter */
		if (buf[0] == 0x7e) {
			
#ifdef VERBOSE
	 		printf("New packet to parser, MSB:%02x\tLSB:%02x\n",buf[1],buf[2]);
#endif			
			
			/* Packet length, API-specific structure length */
			len=MSB_LSB_2_INT(buf+1,2);
			
			if (len>0){
				
#ifdef VERBOSE
				printf("Len pkg: %d\t",len);
#endif		
				pkg = (uint8 *) malloc(sizeof(uint8)*(len+1)); // there is also 
															   // the chksum
							
				/* Read the packet */
				nRead=readport(fd,pkg,len+1);
				
				
				/* Evaluate API type */
				id=pkg_dispatcher(pkg,nRead,iid,load);
				
				if (id==rx_pack_16 && *iid==pkg_1) {
					printf("---PARSER----\n");
					printf("Long: %ld\n",((struct payload_pkg1 *) *load)->i);
					printf("Float: %f\n",((struct payload_pkg1 *) *load)->j);
				}
				
				free(pkg);			
				return id;
			}
			// len <= 0
#ifdef VERBOSE			
			printf("Len: %d\n",len);
#endif
			
		}
		// buf[0] != 0x7e
#ifdef VERBOSE
		printf("Start Delimiter = %02x\t",buf[0]);
		printf("nRead(in): %d\t",nRead);
		print_pkg(buf,nRead);
#endif
		return -1;		
	}
	// nRead != 3
//#ifdef VERBOSE
//	printf("nRead(out): %d\t",nRead);   // Commentare per porta non bloccante
//	print_pkg(buf,nRead);
//#endif
	
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
	uint8* pkg;
	int riid;
	int lenFrame;
	int lenPCMD;
	void *rload;	// Reply load
	int wrote, ver_chks;
	
	/* Length of the data frame  */
	lenFrame = 5+1  + size_load;
	
	/* Length of the whole packet */
	lenPCMD = SIZE_FRAME + lenFrame ;
	
#ifdef VERBOSE
	printf("Packet Len: %d\n",lenPCMD);
#endif
	
	/* Build Packet */
	pkg=(uint8 *)malloc(sizeof(uint8)*lenPCMD);
	
	memset(pkg,'0',lenPCMD);	
	pkg[0]= 0x7e;
	
	INT_2_MSB_LSB(pkg+1,lenFrame);	// MSB + LSB Packet Length (without crc)
#ifdef VERBOSE
	printf("%d\t%d\n",pkg[1],pkg[2]);
#endif
	
	pkg[3]= 0x01;					// API ID
	
	/* Randomly generate the frame id */
	srand(getpid());				// FRAME ID
	
	/* FRAME ID */
	if (ack)
		pkg[4]=  (uint8) 30*(rand()/(double) RAND_MAX);	// Random frame ID
	else
		pkg[4]= 0x00;  // No tx_status ack
	
#ifdef VERBOSE	
	printf("ID: %02x\n",pkg[4]);
#endif
	
	INT_2_MSB_LSB(pkg+5,dest);			// MSB + LSB Destination Address
	
	pkg[7]= 0x01;					// Option
	pkg[8]= iid;					// Internal Protocol ID
	
	memcpy(pkg+9,load,size_load);	// Payload 
	
	pkg[lenPCMD-1]=checksum(pkg);   // Checksum
#ifdef VERBOSE	
	printf("%02x",pkg[lenPCMD-1]);
#endif
	
	/* Send package */	
#ifdef VERBOSE
	printf("\nTo Write: ");
	print_dbg(pkg,lenPCMD);
#endif
	
	wrote = writeport(fd,pkg,lenPCMD);
	ver_chks = verify_checksum(pkg);
#ifdef VERBOSE		
	printf("Wrote: %d\n",wrote);
	printf("Verify pkg: %d\n",ver_chks);
#endif
	
	/* Ack response, if frame id != 0x00 */
	int ack_id = -1;
	
	if (pkg[4])    // If frame Id != 0x00 
		do // Do-while per essere sicuri nel caso di porta non bloccante
			ack_id = pkg_parser(fd,&riid,rload);
		while (ack_id==-1);
	
	free(pkg);
	
	/* DA CAPIRE */
	//if (rload!=NULL)
	// 	free(rload);
	
	//return 1;
	return ack_id;
}

/* Package Size */

/*	Packet Details...
 0x7e									1
 MSB Len								1
 LSB Len								1
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

void read_pkg_1(uint8 * buf, int len, void **load) {
	*load = (void *) malloc(sizeof(struct payload_pkg1));
	memcpy(*load, buf,sizeof(struct payload_pkg1));
//#ifdef VERBOSE
//	printf("\n");
//	printf("Long: %ld\n",((struct payload_pkg1*) *load)->i);
//	printf("Float: %f\n",((struct payload_pkg1*) *load)->j);
//#endif
}

void read_pkg_2(uint8 *buf,int len, void **load){
	//int size;
	*load = (void *) malloc(sizeof(struct payload_pkg2));
	memcpy(*load, buf,sizeof(struct payload_pkg2));
//#ifdef VERBOSE
//	printf("\n");
//	printf("Double: %f\n",((struct payload_pkg2*) *load)->i);
//	printf("String: %s\n",((struct payload_pkg2*) *load)->buf);
//#endif
}

void read_pkg_time(uint8 *buf, int len, void **load) {
	*load = (void *) malloc(sizeof(struct payload_pkg_time));
	memcpy(*load, buf, sizeof(struct payload_pkg_time));
//#ifdef VERBOSE
//	printf("\n");
//	printf("Long: %ld\n",((struct payload_pkg_time*) *load)->i);
//	printf("tv_sec: %ld\n",((struct payload_pkg_time*) *load)->time.tv_sec);
//	printf("tv_usec: %ld\n",((struct payload_pkg_time*) *load)->time.tv_usec);
//#endif
}

void read_pkg_rob_command(uint8 *buf, int len, void **load) {
	*load = (void *) malloc(sizeof(struct payload_pkg_rob_command));
	memcpy(*load, buf, sizeof(struct payload_pkg_rob_command));
}
