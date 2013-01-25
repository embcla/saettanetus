//==============================================================================
//
//  Project:        FoxVision: Vision system for mobile robots equipped with 
//					FOX board Netus G20
//
//  Module:         Component labelling and centroid evaluation
//					interface
//
//  Description:    Provides a C implementation of the labelling algorithm   
//                  referred by Chan,Cheng, Lu paper (1362-F.pdf) and for 
//					computing the blob centroids.
//
//  Author:         Enrico Di Lello
//  Author:         and many small changes by Giovanni Micheli
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

#ifndef labelling_h_DEFINED
#define labelling_h_DEFINED

//================
//   Includes
//================

#include "simple_image.h"
#include "lists.h"
#include "time.h"


//===============
//   Defines
//===============

#define MARK -1
#define BLOB_MIN_SIZE  6//4 //8 //do not compute centroid if the connected component 
			  //contour is smaller than this value 
#define BLOB_MAX_SIZE  300//50

//====================
//     Structures
//====================

//------------------------------------------------------------------------------
//
/// \brief A simple representation of a 2-dimensional array in a one-dimensional 
///        one. 
///
/// Used to store integer values (16 bits per pixel). Contains two ints
/// representing image dimensions and a pointer to one-dimensional array containing 
/// labels values.
///
//------------------------------------------------------------------------------

typedef struct
{
   int   rows;
   int   cols;
   int  *data;

} Labels;


//==============================
//    Function prototypes
//==============================


//------------------------------------------------------------------------------
///
/// \brief Create a new labels matrix and set all the pixels to 0
///
///
/// \param  rows       rows to allocate (pixels)
/// \param  cols       columns to allocate (pixels)
///
/// \return A new allocated grey level image
///
//------------------------------------------------------------------------------

int CreateLabels(Labels **labels, int rows, int cols);

//------------------------------------------------------------------------------
///
/// Releases all memory associated to a labels matrix.
///
/// \param  binIm          The labels matrix to destroy
///
//------------------------------------------------------------------------------

void DestroyLabels(Labels *labels);


//------------------------------------------------------------------------------
///
/// Computes the centroid of the white connected components in the image. 
/// Implements the Chang,Chen,Lu algorithm for component labelling 
///
/// \param  bwIm          The binary image on which blob extraction has to 
///                       performed  
//------------------------------------------------------------------------------

int PerformLabelling(BWImage *bwIm, ComponentsList *compList); 


//------------------------------------------------------------------------------
///
///  \brief Computes the centroid of the color connected components in the image. 
///
///  Implements the Chang,Chen,Lu algorithm for component labelling 
///  Read the paper for more details.
/// 
///  \param  bwIm         The color image on which blob extraction will be 
///                       performed  
///  \param  compList     To store the pixel index of the connected components
///
///  \param  maxColor	  max number of used colors
///						   
///  \return 0 if everything went ok, -1 elsewhere
///
//------------------------------------------------------------------------------


int PerformLabellingColor(BWImage *bwIm, ComponentsList *compList, int maxColor);


//------------------------------------------------------------------------------
///
/// Follow the external or internal contour of a connected component.
///
/// \param  bwIm          The binary image on which blob extraction has to 
///                       performed
/// \param  labels        A copy of the image containing the blobs labels
/// \param  compList      A pointer to a components list to store the components
///                       pixel indexes   
/// \param  startingPixel The index of the starting pixel of the contour
/// \param  contourType   flag to specifying the contour type:
///                       External contour = 1;
///                       Internal contour = 2;
/// \param  label         blob label
///                        
//------------------------------------------------------------------------------

int ContourTracing(BWImage *bwIm, Labels *labels, ComponentsList *compList, int startingPixel, int contourType, int label);


//------------------------------------------------------------------------------
///
///  Follow the external or internal contour of a connected component.
///
///  The first search direction depends on the contour type. 
///
///  \param   bwIm	    The color image on which blob extraction has to 
///                         performed       
///  \param   labels        A copy of the image containing the blobs labels
///  \param   previousPixel Previous contour pixel index
///  \param   lastPixel     Current contour pixel index
///  \param   contourType   Flag to specifying the contour type
///  \param   label         blob label
///  \param   pixelColor    color code of the pixel
///
///
///  \return 0 if everything went ok, -1 elsewhere
///
//------------------------------------------------------------------------------


int ContourTracingColor(BWImage *bwIm, Labels *labels, ComponentsList *compList, int startingPixel, int contourType, int label, int pixelColor);


//------------------------------------------------------------------------------
///
///  Return the next pixel of the contour, the search policy depends on the 
///  contour type.
///
///  \param   bwIm		    The binary image on which blob extraction has to 
///                         performed       
///  \param   labels        A copy of the image containing the blobs labels
///  \param   previousPixel Previous contour pixel index
///  \param   lastPixel     Current contour pixel index
///  \param   contourType   Flag to specifying the contour type
///  \param  label         blob label
///
//------------------------------------------------------------------------------

int Tracer(BWImage *bwIm, Labels *labels , int previousPixel, int currentPixel, int contourType , int label);

//------------------------------------------------------------------------------
///
///  Return the next pixel of the contour, the search policy depends on the 
///  contour type. 
///
///  \param   bwIm	    The color image on which blob extraction has to 
///                         performed       
///  \param   labels        A copy of the image containing the blobs labels
///  \param   previousPixel Previous contour pixel index
///  \param   lastPixel     Current contour pixel index
///  \param   contourType   Flag to specifying the contour type
///  \param   label         blob label
///  \param   pixelColor    color code of the pixel
///
///
///  \return 0 if everything went ok, -1 elsewhere
///
//------------------------------------------------------------------------------


int TracerColor(BWImage *bwIm, Labels *labels , int previousPixel, int currentPixel, int contourType , int label, int pixelColor);


//------------------------------------------------------------------------------
///
///  Return the index of the pixel neighbour in the specified direction.
///  The pixel neighbourhood is indexed as follows:
///
///                5     6    7
///                4   pixel  0
///                3     2    1 
///  \param    pixel              current pixel index
///  \param    searchPosition     search position according previous scheme
///
//------------------------------------------------------------------------------

inline int GetNeighbourCoord(int pixel,int searchPosition, int imageWidth, int imageHeight);

//------------------------------------------------------------------------------
///
///  Return the position of the second pixel respect to the first
///  The pixel neighbourhood is indexed as follows:
///
///                5     6    7
///                4   pixel  0
///                3     2    1 
///
///  \param    previousPixel    previous  pixel index
///  \param    int pixel        current pixel index
///
//------------------------------------------------------------------------------

inline int GetNeighbourIndex(int previousPoint, int point, int imageWidth);


//------------------------------------------------------------------------------
///
/// Print the labelled image on screen, used only in debug with very small images
///
//------------------------------------------------------------------------------

int PrintLabels(Labels *labels);

//------------------------------------------------------------------------------
///
/// Compute the i,j coordinates of the pixel belonging to every connected 
/// components and evaluate the centroid... need to find a smarter way
///
//------------------------------------------------------------------------------

int ComputeCentroids(ComponentsList *compList, BWImage *bwIm);

//------------------------------------------------------------------------------
///
///  \brief Find blobs centroids
/// 
///  Compute the i,j coordinates of the pixel belonging to every connected 
///  components and evaluate the centroid. Save them in the components list and 
///  mark them in the color image with a white cross for visualization.
///
///  \param compList        Where the pixel index of the connected components
///			    are stored after labelling
///  \param   bwIm	    The color image on which blob extraction was 
///                         performed 
///
///  \return 0 if everything went ok, -1 elsewhere
///
//------------------------------------------------------------------------------


int ComputeCentroidsColor(ComponentsList *compList, BWImage *bwIm);


#endif //#ifndef lists_h_DEFINED
