//==============================================================================
//
//  Project:        guarDIAn: Vision system for mobile robots equipped with 
//					FOX board Netus G20
//
//  Module:         Distance computation from pixel indexes
//
//  Description:    Definitions of a set of functions for logging experiment 
//					data
//
//  Author:         Enrico Di Lello
//
//------------------------------------------------------------------------------
//  FoxVision: Vision system for mobile robots
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

#ifndef logs_h_DEFINED
#define logs_h_DEFINED

//================
//   Includes
//================

//#include "globals.h"
#include "time.h"


//===============
//   Defines
//===============	

// Camera log file format :

//  Blob X coord	Blob Y coord	Timestamp   Blob number   

// Time log file format :

//  Frame acquisition time  Frame Processing time   Distance Measurement time	Total time   


//==============================
//    Function prototypes
//==============================

//------------------------------------------------------------------------------
///
///	\brief			Main data log initialization function
/// 
//------------------------------------------------------------------------------


int InitLogs();


//------------------------------------------------------------------------------
//
// Creates a string with the current date (Awfull!!!)
//
// \param          char*       the log file name string
//
//------------------------------------------------------------------------------


int GetLogFileName(char *log_fname);


//------------------------------------------------------------------------------
//
// Prints a row of the coordinates log file:
//
// \param			centroidX   X coordinates of the blob in a global ref. sis.
// \param			centroidY   Y coordinates of the blob in a global ref. sis.
// \param		  	timestamp   measurement timestamp
// \param			blob_num    number of connected components found on current
//								frame
//
//------------------------------------------------------------------------------


int LogCoordData(float centroidX, float centroidY, long timestamp, int blob_num);


//------------------------------------------------------------------------------
//
// Prints a row of the coordinates log file:
//
// \param			centroidX   X coordinates of the blob in a global ref. sis.
// \param			centroidY   Y coordinates of the blob in a global ref. sis.
// \param		  	timestamp   measurement timestamp
// \param			blob_num    number of connected components found on current frame
// \param			blob_color  codice colore del blob 
//
//------------------------------------------------------------------------------


int LogCoordDataEvo(float centroidX, float centroidY, long timestamp, int blob_num, int blob_color);


//------------------------------------------------------------------------------
//
// Prints a row of the timing log file
//    
//
//------------------------------------------------------------------------------


int LogTimeData();


//------------------------------------------------------------------------------
///
/// Evaluates elapsed time and prints out values
///
/// \param          char*       Name of step performed (eg. : grabbing, color thresholding, etc)      
/// \param          timeval         start time
///
//------------------------------------------------------------------------------


int GetElapsedTime(char* task, struct timeval* start_time, struct timeval* end_time, long *elapsed_utime);


//------------------------------------------------------------------------------
///
///	\brief			Main data log termination function
/// 
//------------------------------------------------------------------------------


int CloseLogs();



#endif



