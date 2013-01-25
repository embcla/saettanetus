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

//********************************************************************************************************************
//*************************************** INCLUDE HEADER *************************************************************
//********************************************************************************************************************
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
#include "console_commands.h"
#include "serial_comm.h"
//********************************************************************************************************************
//********************************************************************************************************************
//********************************************************************************************************************


// Avoiding Including Files Multiple Times
#ifndef XBEE_HEADER
/*! \def XBEE_HEADER
 *  \brief Xbee header
 */
#define XBEE_HEADER

int Frame_id;   // counter, current frame id value

/**Xbee serial port*/
#define XBEE_DEVICE "/dev/ttyS1"

//********************************************************************************************************************
//*************************************** DEFINE *********************************************************************
//******************************************************************************************************************** 

/*! \def VERBOSE_XBEE
 *  \brief To unable the debug prints on the screen
 */
//#define VERBOSE_XBEE

/*! \def VERBOSE_DISPATCHER
 *  \brief To unable the prints relative to the packet load
 */
//#define VERBOSE_DISPATCHER

/*! \def CONSOLE
 *  \brief To merge the old console code in the Xbee library
 */
#define CONSOLE

/*! \def uint8 8 bit unsigned int
 *  \brief Unsigned char.
 */
#define uint8_  unsigned char

/*! \def SIZE_FRAME
 * \brief Start delimiter + MSB + LSB + (PAYLOAD_SIZE) + CRC
 */
#define SIZE_FRAME	4	// 0x7E MSB LSB + (PAYLOAD_SIZE) + CRC

/*! \def MAX_LOAD_SIZE 
 *  \brief Max load size
 */
#define MAX_LOAD_SIZE 100    // Max load size

/*! \def READ_ERROR
 *  \brief Error code, it hasn't been possible to read the whole packet
 */
#define READ_ERROR -99  

/*! \def BLOCKING
 *  \brief Opening port option, the open will be opened as blocking
 */
#define BLOCKING 1

/*! \def NO_BLOCKING
 *  \brief Opening port option, the open will be opened as no blocking
 */
#define NO_BLOCKING 0 

/*! \def V_MIN
 *  \brief Minimum number of characters to read (default to 1)
 */
#define V_MIN 1	        

/*! \def V_TIME
 *  \brief Time to wait for data (tenths of seconds, default to 0)
 */
#define V_TIME 0  

/*! \def ACK
 *  \brief To receive an ack packet
 */
#define ACK 1            

/*! \def NO_ACK
 *  \brief To not receive an ack packet
 */
#define NO_ACK 0

/*! \def MAX_NEIGHBORS
 *  \brief Number of maximum neighbors
 */
#define MAX_NEIGHBORS	15
//********************************************************************************************************************
//********************************************************************************************************************
//********************************************************************************************************************




//********************************************************************************************************************
//****************************************** STRUCT AND ENUM *********************************************************
//********************************************************************************************************************
/*! \enum API Types
 * \brief API Types
 */
enum api_types {
	modem_status=0x8A,	//!\brief RF module status messages are sent from the module in response to specific conditions
	at_cmd=0x08,		//!\brief Allows for module parameters to be queried or set
	at_cmd_queue=0x09,	//!\brief Similar to at_cmd, but the new parameter values are queued and not applied until either the "AT Command API type or the AC command is issued. Register queries are returned immediately.  
	at_cmd_resp=0x88,	//!\brief Response to an AT Command message
	tx_req_64=0x00,		//!\brief Send RF Data as an RF Packet
	tx_req_16=0x01,		//!\brief Send RF Data as an Packet
	tx_status=0x89,		//!\brief When a TX Request is completed, the module sends a TX Status message. This message will indicate if the packet was transmitted successfully or if there was a failure.
	rx_pack_64=0x80,	//!\brief When the module receives an RF packet, it sent out the UART using this message type
	rx_pack_16=0x81		//!\brief When the module receives an RF packet, it sent out the UART using this message type
};


/*! \enum Internal Commands 
 * \brief Internal commands
 */
enum i_cmd {
	pkg_1=0x01,					//!\brief To send an example packet
	pkg_2=0x02,					//!\brief To send an example packet
	pkg_time=0x03,				//!\brief To send a time packet
	pkg_rob_command=0x04,		//!\brief To send a robot command packet
	pkg_ir=0x05,				//!\brief To send a ir packet
	pkg_acc=0x06,				//!\brief To send a acc packet
	pkg_gyro=0x07,				//!\brief To send a gyro packet  
	pkg_magneto=0x08,			//!\brief To send a magneto packet
	pkg_multi=0x09,				//!\brief To send a multi packet
	pkg_state=0x0A,				//!\brief To send a state packet
	pkg_webcam_data=0x0B,	    //!\brief To send a webcam packet
	pkg_sensor=0x0C,
	pkg_trajectory=0x0D
};



/*! \struct payload_pkg1 
 * \brief Package 1.
 * An example of struct to send.
 */
struct payload_pkg1 {
	long  i;    //!\brief A long variable (4 bytes)
	float j;    //!\brief A float variable (4 bytes)
};


/*! \struct payload_pkg2 
 * \brief Package 2.
 * An example of struct to send.
 */
struct payload_pkg2 {
	double i;          //!\brief A double variable (8 bytes)
	char buf[92];      //!\brief An array of char (92 bytes)
};


/*! \struct payload_pkg_time 
 * \brief Package time.
 * An example of struct to send.
 */
struct payload_pkg_time {
	long i;						//!\brief A long variable (4 bytes)
	struct timeval time;        //!\brief A struct timeval variable (8 bytes)
};

/*! \struct payload_pkg_state
 * \brief A robot state
 */
struct payload_pkg_state {
	float x;
	float y;
	float th;
};


/*! \struct payload_pkg_command 
 * \brief A robot command
 */
struct payload_pkg_rob_command {
	char* command;		//!\brief A char which rapresents a robot command (4 bytes, necessary for compatibility in the Fox environment)
	int value;				//!\brief Command value (4 bytes)
};

/*! \struct payload_pkg_ir
 * \brief An ir scan
 */
struct payload_pkg_ir {
	unsigned int num_canali;//!\brief Number of analogic channels
	unsigned int range[5];	//!\brief Raw data
	float converted[5];		//!\brief Converted data
	unsigned int bias[5];	//!\brief Conversion bias
	float d2c;			    //!\brief D/A conversion factor
	unsigned int is_valid;	//!\brief Validation flag
};

/*! \struct payload_pkg_acc
 * \brief An acc scan
 */
struct payload_pkg_acc {
	unsigned int num_canali;
	unsigned int range[3];	
	float converted[3];		
	unsigned int bias[3];	
	float d2c;			    
	unsigned int is_valid;	
};

/*! \struct payload_pkg_gyro
 * \brief A gyro scan
 */
struct payload_pkg_gyro {
	unsigned int num_canali;
	unsigned int range[2];	
	float converted[2];		
	unsigned int bias[2];	
	float d2c;			    
	unsigned int is_valid;	
};

/*! \struct payload_pkg_magneto
 * \brief A magneto scan
 */
struct payload_pkg_magneto {
	unsigned int num_canali;
	unsigned int range[2];	
	float converted[2];		
	unsigned int bias[2];	
	float d2c;			    
	unsigned int is_valid;	
};


struct payload_pkg_sensor {
	unsigned int num_canali;
	float d2c;			    
	unsigned int is_valid;	
	unsigned int *range;	
	float *converted;		
	unsigned int *bias;	
};


/*! \struct payload_pkg_multi
 * \brief Usuful to send a packet that is bigger than maximum load size (100 bytes)
 */
struct payload_pkg_multi {
	int id;				//!\brief Multi package ID
	int total_size;		//!\brief Total size of the packet (> 100 bytes)
	int frame;			//!\brief Current frame number
	int frame_size;		//!\brief Current frame size
	void *data;         //!\brief The data of the current frame 
};

/*! \struct payload_pkg_webcam_data
 * \brief A webcam scan
 */
struct payload_pkg_webcam_data {
	unsigned int n_obj;		//!\brief Number of detected object
	long int time;			//!\brief Detection time
	float *data;			//!\brief Data regarding the detected object
};

/*! \struct payload_pkg_trajectory
 * \brief A vel reference to be tracked
 */
struct payload_pkg_trajectory {
	float  v;		//!\brief vel for the left motor (ticks)
	float  w;	//!\brief vel for the right motor (ticks)
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

//********************************************************************************************************************
//********************************************************************************************************************
//********************************************************************************************************************


//********************************************************************************************************************
//********************************* FUNCTIONS ************************************************************************
//********************************************************************************************************************

/************** FUNCTIONS INVOLVING THE SERIAL PORT ****************************
/*! \fn int open_port (char *device) 
 * \brief To open port.
 * \param[in] device Device name.
 * \param[in] blocking 1 set as blocking, 0 set as non blocking.
 * \param[in] vmin Minimum number of characters to read.
 * \param[in] vtime Time to wait for data (tenths of seconds)
 * \return The file descriptor on success or -1 on error.
 */ 
//int open_port(char *device, int blocking, int vmin, int vtime);
int open_port(char *device, int blocking);
int open_port_read(char *device, int blocking);
int open_port_write(char *device, int blocking);





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
int readport(int fd, uint8_ *buf, int len);


/*! \fn int writeport (int fd, uint8 *buf, int len)  
 * \brief To write a port.
 * \param[in] fd Descriptor file.
 * \param[in] buf Buffer to write.
 * \param[in] len Buffer size (bytes).
 * \return The number of writed bytes.
 */
int writeport(int fd, uint8_ *buf, int len);



/************** SUPPORT FUNCTIONS ********************************************** 
/*! \fn long hex2dec_char (uint8 *buf, int len) 
 * \brief To convert hexadecimal to integer from characters.
 * \param[in] *buf Buffer to convert.
 * \param[in] len Buffer size (bytes).
 * \return Integer value.
 */
long hex2dec_char(uint8_ *buf, int len);


/*! \fn chacksum_char (uint8 *pkg)  
 * \brief To compute checksum from characters.
 * To calculate: Not including frame delimiters and length, 
 * add all bytes keeping only the lowest 8 
 * bits of the result and subtract from 0xFF.
 * \param[in] pkg Buffuer upon calculate checksum.
 * \return Checksum value.
 */
int checksum_char(uint8_ *pkg);



/******************* DEBUG FUNCTIONS *******************************************
/*! \fn hex2dec (uint8 *buf, int len) 
 * \brief To convert hexadecimal to integer.
 * \param[in] buf Buffer to convert.
 * \param[in] len Buffer size (bytes).
 * \return 
 */
long hex2dec(uint8_ *buf, int len);


/*! \fn dec2hex (uint8 *buf, int len) 
 * \brief To convert integer to hexadecimal.
 * \param[in] buf Buffer to convert.
 * \param[in] len Buffer size (bytes).
 * \return 
 */
long dec2hex(uint8_ *buf, int len);


/*! \fn int MSB_LSB_2_INT (uint8 *buf, int len)
 * \brief To compute value from MSB, LSB to int. buf[0] is considered the MSB value, buf[1] the LSB value
 * \param[in] buf A pointer to uint8
 * \param[in] len Buffer size (bytes).
 * \return An integer value that rapresents the quantity described by MSB and LSB
 */
int MSB_LSB_2_INT(uint8_ *buf, int len);


/*! void INT_2_MSB_LSB(uint8 *buf, int val)
 * \brief To compute value from int to MSB, LSB. The MSB value will be put in buf[0], the LSB value in buf[1]
 * \param[in] buf A pointer to uint8
 * \param[in] val The integer value
 * \return void
 */
void INT_2_MSB_LSB (uint8_ *buf, int val);


/*! \fn int checksum (uint8 *pkg) 
 * \brief To compute checksum. 
 * \param[in] pkg Package upon calculate checksum.
 * \return Checksum value.
 */
int checksum(uint8_ *pkg);


/*! \fn verify_checksum (uint8 *pkg) 
 * \brief To verify checksum.
 * Add all bytes (include checksum, but not the delimiter and length).
 * If the checksum is correct, the sum will equal 0xFF.
 * \param[in] pkg Package upon verify checksum.
 * \return 1 on success, 0 otherwise
 */
int verify_checksum(uint8_ *pkg);



/******************** DEBUG FUNCTIONS ******************************************
//int process_pkg(int fd);   



/*! \fn int pkg_parser (int fd, int *iid, void **load)
 * \brief To inquire the serial port .
 * To parse a received package.
 * \param[in] fd Descriptor file.
 * \param[in] iid Pointer to internal ID.
 * \param[out] **load 
 * \return The command id code or -1 on failure
 */
int pkg_parser (int fd, int *iid, void **load, int *sender);


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


/*! \fn int Send_Data_16_mulit (int fd, int dest, void* load, int size_load, int ack)
 * \brief To send a package whose dimension is bigger than 100 bytes
 * \param[in] fd File descriptor.
 * \param[in] dest Destination.
 * \param[in] load Pointer to struct that will be send.
 * \param[in] size_load Struct size (bytes).
 * \param[in] ack 0 for no tx_status ack packet, 1 to have a tx_status ack.
 * \return Return code.
 */
int Send_Data_16_multi(int fd, int dest, void *load, int size_load, int ack);


/*! \fn void read_pkg(uint8 *buf, int len, void **load, int size_load)
 * \brief To read robot command packages.
 * \param[in] *buf Buffer in which store reading data.
 * \param[in] len Buffer size (bytes).
 * \param[out] load 
 * \parami[in] size_load Package size (bytes)
 * \return void
 */
void read_pkg(uint8_ *buf, int len, void **load, int size_load);


/* To parse the TX status pkg */
/*! \fn void ati_tx_status (uint8 *pkg)
 * \brief To parse the TX status packet
 * \param[in] *pkg The buffer which contains the TX status message
 * \return Return the status code
 */
int ati_tx_status (uint8_ * pkg);

//********************************************************************************************************************
//********************************************************************************************************************
//********************************************************************************************************************




#ifdef CONSOLE
///\brief		numero di pacchetti inviati
int 			xbee_packet_sent;					//	num xbee packet sent
///\brief		numero di pacchetti ricevuti
int 			xbee_packet_received;				//	num xbee packet received
///\brief		numero di ack ricevuti
int 			xbee_ack_received;					//	num xbee ack received
///\brief		number of packets received categorized by  sender ID
unsigned int	xbee_packet_neighborhood[MAX_NEIGHBORS];			//	num xbee ack received by a speficied ID


// CONSOLE
///\brief	lunghezza del buffer per comandi da console
#define		CONSOLE_LEN_BUFFER		20

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

///\brief	file descriptor per la console
int 	console_fd;
///\brief	struttura per fli attributi dello standard input
struct termios stdin_saved_attributes;
///\brief	struttura per gli attributi della seriale
struct termios tty_saved_attributes;

///\brief	comando corrente da console
char 		console_current_command;
///\brief	parametro 1 del comando
int 		console_current_value1;
///\brief	parametro 2 del comando
int 		console_current_value2;
///\brief	flag di avvenuta  ricezione del comando
int 		console_flag_cmd_rcvd;
///\brief	buffer di ricezione comando
char		console_buffer[CONSOLE_LEN_BUFFER];
///\brief	OBSOLETO
char		console_first_value[CONSOLE_LEN_BUFFER];
///\brief	OBSOLETO
char		console_second_value[CONSOLE_LEN_BUFFER];


///\brief	Inizializzazione della console
int 		console_init();

/**
Parsing della console
\brief	Parsing della console
@param[in] fd file descriptor da parsare
*/
void 	console_parsing(int fd);
//void 	console_parsing();
unsigned char mau_buf[100];
#endif

#endif
