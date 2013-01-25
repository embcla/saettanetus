#ifndef CONSOLE_H
#define CONSOLE_H

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
#include "console_commands.h"
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
int 	init_console(void);

/**
Parsing della console
\brief	Parsing della console
@param[in] fd file descriptor da parsare
*/
void 	console_parsing(int fd);
//void 	console_parsing();

#endif
