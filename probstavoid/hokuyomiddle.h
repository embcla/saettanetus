/******************************************************************************
*  This program is free software: you can redistribute it and/or modify      *
*  it under the terms of the GNU General Public License as published by      *
*   the Free Software Foundation, either version 3 of the License, or        *
*  (at your option) any later version.                                       *
*                                                                            *
*  This program is distributed in the hope that it will be useful,           *
*  but WITHOUT ANY WARRANTY; without even the implied warranty of            *
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
*  GNU General Public License for more details.                              *
*                                                                            *
*  You should have received a copy of the GNU General Public License         *
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.     *
*****************************************************************************/

/**\file hokuyomiddle.h*/

#ifndef HOKUYO_MIDDLE
#define HOKUYO_MIDDLE

#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include "urg_ctrl.h"
/**
 * @enum urg_capturing_mode_enum this enum is used as a switch between on-demand and continuous mode. 
 */
typedef enum{
  ON_DEMAND=0,
  CONTINUOUS=1,
  HYBRID=2
}urg_capturing_mode_enum;

#define DEVICE "/dev/ttyACM0"
/*Under this constant, the range laser data is not valid*/
const int VALID_LASER_DATA=20;

/**Capturing times to be used in HYBRID mode*/
const int capture_times=100;
/**The maximun amount of receveid data*/
int data_max=-1;

int frontal_index=0;

pthread_mutex_t mutex_laser_read;
long *data_laser=0;
/**Urg-specific parameters structure*/
urg_parameter_t *parameter;

/**Capturing mode*/
int urg_capturing_mode=-1;

/**
*\brief Creates laser data structure
*This function creates the urg laser data structure. This must be called at every laser
*module initialization
*\param [in] urg Urg laser data structure to be allocated
*\param [in] modality Switch between continuous and on-demand mode (cfr. gd_scan and md_scan) 
*\return <0 if an error occurs, >0 otherwise. 
*/
int init_urg_laser(urg_t **urg,int modality);

/**
*\brief Function to read laser data
*This function reads the data from the laser and store them in "data" array.  
* N.B. Programmer is in charge of deallocating data buffer 
*\param[in] urg Urg laser data structure
*\param[in,out] data data buffer
*\return <0 if an error occurs, the number of read data otherwise. 
*/
int read_laser_data(urg_t *urg);

/**
*\brief Function to get the central (frontal) index
*This function returns the frontal index of the data. It is read from the laser specific parameters.
*\param[in] urg Urg laser data structure
*\return <0 if an error occurs, the frontal index otherwise. 
*/
int get_frontal_index();

/**
*\brief Function close the laser module
*This function destroy the laser specific data structures
*\param[in] urg Urg laser data structure
*/
void close_urg_laser(urg_t *urg);

/**
*\brief Maximum number of laser samples
*This function returns the number of maximum data to be stored in data buffer
*\return The maximum number of laser samples
*/
int get_data_max();

#endif
