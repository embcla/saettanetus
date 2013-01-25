/*! \mainpage Xbee Pro
 *
 * \section Introduction
 *
 * This is the introduction.
 *
 * \section conf_ Configuration
 *
 * \subsection step1 Step 1: Opening the box
 *
 * etc...
 */


/*! \file XbeePro_API.h
    \brief Xbee library header file.
	\date 02/05/2008.
    \author Andrea Gasparri.
*/



#include <stdio.h>		/* Standard input/output definitions */
#include <stdlib.h>		/* Standard library definitions */
#include <string.h>		/* String function definitions */
#include <fcntl.h>		/* File control definitions */
#include <errno.h>		/* Error number definitions */
#include <termios.h>	/* POSIX terminal control definitions */
#include <math.h>		/* Mathematical function definitions */
#include <sys/types.h>
#include <sys/uio.h>
#include <unistd.h>



/*! \def uint8 8 bit unsigned int
    \brief Unsigned char.
*/
#define uint8  unsigned char


/*! \def SIZE_FRAME
 * \brief Start delimiter + MSB + LSB + (PAYLOAD_SIZE) + CRC
 */
#define SIZE_FRAME	4	// 0x7E MSB LSB + (PAYLOAD_SIZE) + CRC


/*! \enum API Types
 * \brief API Types.
 */
enum api_types {
	modem_status=0x8A,	/*  */
	at_cmd=0x08,		/*  */
	at_cmd_queue=0x09,	/*  */
	at_cmd_resp=0x88,	/*  */
	tx_req_64=0x00,		/*  */
	tx_req_16=0x01,		/*  */
	tx_status=0x89,		/*  */
	rx_pack_64=0x80,	/*  */	
	rx_pack_16=0x81		/*  */
};


/*! \enum Internal Commands 
 * \brief internal commands.
 * 
 */
enum i_cmd {
	/*! \var ciao ciao
	 */
	pkg_1=0x01,
	pkg_2=0x02,
	pkg_time=0x03,
	pkg_rob_command=0x04
};


/*! \struct payload_pkg1 
 * \brief Package 1.
 * An example of struct to send.
 */
struct payload_pkg1 {
	long i;
	float j;
};


/*! \struct payload_pkg2 
 * \brief Package 2.
 * An example of struct to send.
 */
struct payload_pkg2 {
	double i;
	char buf[92];
};


/*! \struct payload_pkg_time 
 * \brief Package time.
 * An example of struct to send.
 */
struct payload_pkg_time {
	long i;
	struct timeval time;
};

/*! \struct payload_pkg_command 
 * \brief Package command.
 * An example of struct to send.
 */
struct payload_pkg_rob_command {
	char command;
	int value;
};


/* AT Commands 
enum{
	at_mode='+++',
	at_cn='CN',
	at_dh='DH',
	at_dl='DL',
	at_sh='SH',
	at_sl='SL',		
	at_my='MY',
	at_vr='VR',
	at_vl='VL',
};
*/


/*********************** FILE HANDLER *****************************************/ 
/*! \fn int open_port (char *device) 
 * \brief To open port.
 * \param[in] device Device name.
 * \param[in] blocking 1 set as blocking, 0 set as non blocking.
 * \param[in] vmin Minimum number of characters to read.
 * \param[in] vtime Time to wait for data (tenths of seconds)
 * \return The file descriptor on success or -1 on error.
 */ 
int open_port(char *device, int blocking, int vmin, int vtime);


/*! \fn void close_port (int fd) 
 * \brief To close port.
 * @param[in] fd Descriptor file.
 * \return void.
 */
void close_port(int fd);


/*! \fn int readport (int fd, uint8 *buf, int len)  
 * \brief To read a port.
 * \param[in] fd Descriptor file.
 * \param[out] buf Buffer in which store reading data.
 * \param[in] len Buffer size.
 * \return The number of readed bytes.
 */
int readport(int fd, uint8 *buf, int len);


/*! \fn int writeport (int fd, uint8 *buf, int len)  
 * \brief To write a port.
 * \param[in] fd Descriptor file.
 * \param[in] buf Buffer to write.
 * \param[in] len Buffer size (bytes).
 * \return The number of writed bytes.
 */
int writeport(int fd, uint8 *buf, int len);



/************** SUPPORT FUNCTIONS *********************************************/ 
/*! \fn long hex2dec_char (uint8 *buf, int len) 
 * \brief To convert hexadecimal to integer from characters.
 * \param[in] *buf Buffer to convert.
 * \param[in] len Buffer size (bytes).
 * \return Integer value.
 */
long hex2dec_char(uint8 *buf, int len);


/*! \fn chacksum_char (uint8 *pkg)  
 * \brief To compute checksum from characters.
 * To calculate: Not including frame delimiters and length, 
 * add all bytes keeping only the lowest 8 
 * bits of the result and subtract from 0xFF.
 * \param[in] pkg Buffuer upon calculate checksum.
 * \return Checksum value.
 */
int checksum_char(uint8 *pkg);



/******************* DEBUG FUNCTIONS ******************************************/ 
/*! \fn hex2dec (uint8 *buf, int len) 
 * \brief To convert hexadecimal to integer.
 * \param[in] buf Buffer to convert.
 * \param[in] len Buffer size (bytes).
 * \return 
 */
long hex2dec(uint8 *buf, int len);


/*! \fn dec2hex (uint8 *buf, int len) 
 * \brief To convert integer to hexadecimal.
 * \param[in] buf Buffer to convert.
 * \param[in] len Buffer size (bytes).
 * \return 
 */
long dec2hex(uint8 *buf, int len);


/*! \fn int MSB_LSB_2_INT (uint8 *buf, int len)
 * \brief To compute value from MSB to LSB.
 * \param[in] buf
 * \param[in] len Buffer size (bytes).
 * \return An integer value that rapresents ... 
 */
int MSB_LSB_2_INT(uint8 *buf, int len);


/*! void INT_2_MSB_LSB(uint8 *buf, int val)
 * \brief To compute value from MSB to LSB.
 * \param[in] buf
 * \param[in] val
 * \return void
 */
void INT_2_MSB_LSB(uint8 *buf, int val);


/*! \fn int checksum (uint8 *pkg) 
 * \brief To compute checksum. 
 * \param[in] pkg Package upon calculate checksum.
 * \return Checksum value.
 */
int checksum(uint8 *pkg);


/*! \fn verify_checksum (uint8 *pkg) 
 * \brief To verify checksum.
 * Add all bytes (include checksum, but not the delimiter and length).
 * If the checksum is correct, the sum will equal 0xFF.
 * \param[in] pkg Package upon verify checksum.
 * \return 1 on success, 0 otherwise
 */
int verify_checksum(uint8 *pkg);



/******************** DEBUG FUNCTIONS *****************************************/ 
int process_pkg(int fd);


//void read_pack(uint8 *, int);


/*! \fn int pkg_parser (int fd, int *iid, void **load)
 * \brief To inquire the serial port .
 * To parse a received package.
 * \param[in] fd Descriptor file.
 * \param[in] iid Pointer to internal ID.
 * \param[out] **load 
 * \return The command id code or -1 on failure
 */
int pkg_parser(int fd, int *iid, void **load);


/*! \fn int Send_Data_16 (int fd, int dest, int iid, void* load, int size_load, int ack)
 * \brief To send a package with structured payload.
 * \param[in] fd File descriptor.
 * \param[in] dest Destination.
 * \param[in] iid Internal ID.
 * \param[in] load Pointer to struct that will be send.
 * \param[in] size_load Struct size (bytes).
 * \param[in] ack 0 for no tx_status ack packet, 1 to have a tx_status ack.
 * \return Return code.
 */
int Send_Data_16(int fd, int dest, int iid, void* load, int size_load, int ack);


/*! \fn  void read_pkg_1 (uint8 *buf, int len, void **load)
 * \brief To read packages of type 1.
 * \param[in] buf Buffer in which store reading data.
 * \param[in] len Buffer size (bytes).
 * \param[out] load 
 * \return void
 */
void read_pkg_1(uint8 *buf, int len, void **load);


/*! \fn void read_pkg_2 (uint8 *buf, int len, void **load)
 * \brief To read packages of type 2.
 * \param[in] buf Buffer in which store reading data.
 * \param[in] len Buffer size (bytes).
 * \param[out] load 
 * \return void
 */
void read_pkg_2(uint8 *buf, int len, void **load);


/*! \fn void read_pkg_time(uint8 *buf, int len, void **load)
 * \brief To read packages of type 3.
 * \param[in] *buf Buffer in which store reading data.
 * \param[in] len Buffer size (bytes).
 * \param[out] load 
 * \return void
 */
void read_pkg_time(uint8 *buf, int len, void **load);

/*! \fn void read_pkg_rob_command(uint8 *buf, int len, void **load)
 * \brief To read robot command packages.
 * \param[in] *buf Buffer in which store reading data.
 * \param[in] len Buffer size (bytes).
 * \param[out] load 
 * \return void
 */
void read_pkg_rob_command(uint8 *buf, int len, void **load);
