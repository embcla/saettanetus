//==============================================================================
//
//   \file globals.h
//
//   guarDIAn - Vision system for mobile robots  
//   
//   \brief Global variables for guarDIAn
//
//   \author       Enrico Di Lello   <enr.dilello@gmail.com>
//   \author:	   and many small changes by Giovanni Micheli
//

//   \version      $ Revision 1.2 $  13/04/2010
//
//==============================================================================
//
//  guarDIAn: Vision system for mobile robots
//  Copyright (c) 2009 Enrico Di Lello
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307  USA
//  or obtain a copy from the GNU website at http://www.gnu.org/
//
//==============================================================================

#ifndef globals_h_DEFINED
#define globals_h_DEFINED


//================
//     Includes
//================


#include "v4l2_capture.h"
#include "simple_image.h"
#include "frame.h"  
#include "lists.h" 
#include "labelling.h"
#include "coordinates.h"
#include "log.h"
//#include "my_xbee.h"




//================
//     Macros 
//================




//================
//     Defines 
//================


// Debug stuff

#define VISION_PRINT    	// abilita la stampa a video 
#define SAVE_FRAMES     1   	// save the acquired frame in a ppm file 
#define SAVE_NORMALIZED 0	// save the frame after RGB color normalization.. debug only 
#define SAVE_BINARY     0	// save binary image obtained after color segmentation in a ppm file 
#define SAVE_COLOR_IMG  1	// save a color image obtained after color segmentation in a ppm file

#define SAVE_CENTROIDS 	1	// draw the centoids of the connected components in the image and save it in a ppm file 


// Data logging stuff

#define MAPFILE   "map.txt"     // This file contains mapping from pixel index to fixed frame coordinates*/   


#define DEFAULT_V4L2_FMT  V4L2_PIX_FMT_YUYV 	// Standard capture format for Creative camera (uvcvideo )
#define n_save_frame   5//25 //10		// Save 1 out of x frames in a ppm file 

//Default values

#define DEFAULT_VERT_RESOLUTION 144 //120 
#define DEFAULT_HOR_RESOLUTION 176 //160
#define DEFAULT_FPS 5//15 //20


#define LOGGING 1		// abilita il log su file


//=============================================
//
// 				Global variables 
//
//=============================================



//---------------------------
//		Input parsing
//---------------------------



const char* program_name;			    // The name of this program 


//---------------------------
//		Data logging stuff
//---------------------------



char log_fname[27];
char cam_log_fname[31];										
char time_log_fname[32];

FILE *cam_log_file;
FILE *time_log_file;

//int logging; define now



//---------------------------
//	  Time measuring stuff
//---------------------------



struct timeval vision_start_time, vision_end_time; 
struct timeval step_start_time, step_end_time;
  
long acquisition_time, processing_time, dist_meas_time, total_time;


//---------------------------
//		Camera   stuff
//---------------------------


int input_type;                // if == 1, reading image from file, if == 2 grabbing frame from camera
int frame_counter;

char *size;                    // Aquisition frame resolution 
int frame_width;                                       
int frame_height;									    
int num_frames;                // Number of frames to be acquired 
char *input_fname;
int fps;		       // Acquisition frame rate	

ComponentsList *compList;

FRAME *fr;			// Acquired frame pointer
FRAMEGRABBER_V4L2 *fg_v4l2;	// Framegrabber V4L2 device pointer

char frame_fname[31];		// To contain the filename of the saved frames
char binary_fname[32];
	

//------------------------------
//    Blob extraction stuff
//------------------------------	


	
BWImage *bwIm;

ColorStruct *colorStruct;	// Contiene informazioni relative ai colori utilizzati



//---------------------------------
//	  Distance measurement stuff
//---------------------------------	
	


PixelMap *pixelMap;		// map between pixel coordinates and XY coordinates 

	

//---------------------------------
//			Xbee stuff
//---------------------------------	

int sent_packets;
int transmit;
float *packet_data;

int xbee_fd, ack, dest, size_load;
struct payload_pkg_webcam_data *webcam_data;







#endif 
