//==========================================================================
//
//  Project:        guarDIAn: Vision system for mobile robots equipped with 
//					FOX board Netus G20
//
//  Module:         Distance computation from pixel indexes
//
//  Description:    Definitions of a set of function to manage pixel to 
//					XY coordinates correspondence.
//
//  Author:         Enrico Di Lello
//
//  
//--------------------------------------------------------------------------
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
//==========================================================================

#ifndef distance_h_DEFINED
#define distance_h_DEFINED

//================
//   Includes
//================

#include "globals.h"
#include "lists.h"
//#include <math.h>

//===============
//   Defines
//===============	


//====================
//     Structures
//====================

//------------------------------------------------------------------------------
///
/// \brief Contains mapping information between camera pixel and XY coordinates
/// 
///   This structure contains two array which contain a X and Y coordinates
///   relative to a fixed frame. This information are computed previously
///   during camera calibration
//------------------------------------------------------------------------------

typedef struct
{
  int rows;
  int cols;
  float *x_coord;
  float *y_coord;
  
} PixelMap;

//------------------------------------------------------------------------------
///
/// \brief Contains webcam information of: color blob,distance and deviation angle
/// 
//------------------------------------------------------------------------------


typedef struct
{
  int color;
  float distance;
  float deviation;
}WebCamBlobEstimation;



//==============================
//    Function prototypes
//==============================


//------------------------------------------------------------------------------
///
///	\brief			Main PixelMap initialization function
/// 
//------------------------------------------------------------------------------

int InitPixelMap();

//------------------------------------------------------------------------------
///
///	\brief			Main pixel to coordinates mapping  function
/// 
//------------------------------------------------------------------------------

int EvaluateCoordinates();

//------------------------------------------------------------------------------
///
/// \brief Create a new PixelMap 
///
/// \param  pixelMap   PixelMap structure pointer
/// \param  rows       image rows  (pixels)
/// \param  cols       image columns (pixels)
///
//------------------------------------------------------------------------------

int CreatePixelMap(PixelMap **pixelMap, int rows, int cols);

//------------------------------------------------------------------------------
///
/// \brief read data from a file and use ti to initialize a PixelMap structure
///
/// \param  map_file   text file containing pixel to coordinate mapping
/// \param  pixelMap   pointer to a pixelMap
///
///
//------------------------------------------------------------------------------

int FillPixelMap(char *map_file, PixelMap *pixelMap);

//------------------------------------------------------------------------------
///
/// Print the PixelMap elements
///
//------------------------------------------------------------------------------

int PrintPixelMap(PixelMap *pixelMap);	

//------------------------------------------------------------------------------
///
/// Compute the x,y coordinates respect to a fixed reference system
/// Coordinates for every pixel are stored in a file
//------------------------------------------------------------------------------

int GetCoordinates(ComponentsList *compList, PixelMap *pixelMap, FILE *log_file, int transmit,float *packet_data);

//------------------------------------------------------------------------------
///
/// Compute the x,y coordinates respect to a fixed reference system
/// Coordinates for every pixel are stored in a file
//------------------------------------------------------------------------------

int GetCoordinatesEvo(ComponentsList *compList, PixelMap *pixelMap, FILE *log_file, int transmit,float *packet_data);

//------------------------------------------------------------------------------
/// \brief read x,y estimed coordinates and return estimated distance from fixed point
/// 
/// \param x,y centroid float coordinates fixed 0,0 reference system
/// 
//------------------------------------------------------------------------------

float GetDistCentroidBlob(float x_cent_blob, float y_cent_blob);

//------------------------------------------------------------------------------
///
/// \brief read x,y estimed coordinates and return estimated deviation angle 
/// 
/// \param x,y centroid float coordinates fixed 0,0 reference system
/// 
//------------------------------------------------------------------------------

float GetAngleCentroidBlob(float x_cent_blob, float y_cent_blob);

//------------------------------------------------------------------------------
///
/// \return a info webcam blob estimation structure 
/// 
//------------------------------------------------------------------------------

void GetStructureWebCamBlob(WebCamBlobEstimation** wstr,int *n);

#endif

