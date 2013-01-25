//==========================================================================
//
//  Project:        FoxVision: Vision system for mobile robots equipped with 
//					FOX board Netus G20
//
//  Module:         Format conversion and simple image handling functions 
//					interface
//
//  Description:    Provides a high-level C interface for controlling frame
//                  grabber. Based on uvc streamer, supports v4l2 and UVC driver
//
//  Author:         Enrico Di Lello
//  Author:  	    and many small changes by Giovanni Micheli
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

#ifndef simple_image_h_DEFINED
#define simple_image_h_DEFINED

//================
//   Includes
//================

#include  <stdio.h>
#include  <stdlib.h>
#include  <string.h>
#include  <sys/time.h>
#include "frame.h"
#include <linux/videodev.h>
#include <linux/videodev2.h>

//===============
//   Defines
//===============

#define MAX_LINE_LEN  4096	//Max length of header lines


// macros for speeding up format conversions

#define YtoY1(x)  ( (1192*(x) >>10 ) ) 
#define CbtoU(x)  ( (1161*(x) >>10 ) )
#define CrtoV(x)  ( (1161*(x) >>10 ) )

#define YUVtoR(y,v) ( y + ( (1436*v)>>10 ) )
#define YUVtoG(y,u,v) ( y - ( (342*v)>>10 ) - ( (731*u)>>10 ) )
#define YUVtoB(y,u) ( y + ( (1814*u)>>10 ) )

/// Macro per la conversione da codice colore a posizione vettore RGB
#define dtR(codColor) colorStruct->dtColorRGB[(codColor-1)*3]
#define dtG(codColor) colorStruct->dtColorRGB[(codColor-1)*3+1]
#define dtB(codColor) colorStruct->dtColorRGB[(codColor-1)*3+2]

/// Macro per la conversione da codice colore a posizione vettore YCbCr
#define dtYmin(codColor) colorStruct->dtColorYCbCr[(codColor-1)*6]
#define dtYmax(codColor) colorStruct->dtColorYCbCr[(codColor-1)*6+1]
#define dtCbmin(codColor) colorStruct->dtColorYCbCr[(codColor-1)*6+2]
#define dtCbmax(codColor) colorStruct->dtColorYCbCr[(codColor-1)*6+3]
#define dtCrmin(codColor) colorStruct->dtColorYCbCr[(codColor-1)*6+4]
#define dtCrmax(codColor) colorStruct->dtColorYCbCr[(codColor-1)*6+5]

/// Codice identificativo croce centroide
#define CROSS  40


/// RGB components codes..

#define RED 10
#define GREEN 20
#define BLUE 30

// Color segmentation stuff

#define RED_Y1_MIN 50  //YUV
#define RED_Y_MIN  75   //YCbCr

#define RED_Y1_MAX 255   //YUV
#define RED_Y_MAX 240    //YCbCr

#define RED_U_MIN -127   //YUV
#define RED_U_MAX 128    //YUV

#define RED_V_MIN 50     //YUV
//#define RED_Cr_MIN 172   //YCbCr
#define RED_Cr_MIN 160   //YCbCr Logitech camera (the one of the robot) seems to be less sensitive...

#define RED_V_MAX 128

#define R_MIN 100        //RGB
#define G_MIN 110        //RGB
#define B_MIN 100		 //RGB

//====================
//     Structures
//====================



//------------------------------------------------------------------------------
///
/// \brief Struttura dati relativa ai colori 
///
/// \struct ColorStruct
///
/// Contiene i dati relativi ai colori utilizzati durante il riconoscimento
///
//------------------------------------------------------------------------------


typedef struct
{
    int    maxColor;	 // Numero massimo dei colori disponibili
    int    numColor;     // Numero dei colori selezionati per l'utilizzo
    int*   useColor; 	 // Vettore con i colori selezionati
    int*   dtColorYCbCr; // Vettore contenente 6 informazioni per ogni colore (YCbCr)
    int*   dtColorRGB;	 // Vettore contenente 3 informazioni per ogni colore (RGB)
    
    

} ColorStruct;


// Different picture formats

typedef struct
{
    char*   name;               // Short name of format
    int     format;             // Constant from V4L2, VAL_PIX_FMT_*

} FMT;

//------------------------------------------------------------------------------
///
/// \brief A simple representation of a 2-dimensional array in a one-dimensional 
///        one. 
///
/// Used to store simple grey level image (8 bits per pixel). Contains two ints
/// representing image dimensions and a pointer to one-dimensional array containing 
/// pixel values.
///
//------------------------------------------------------------------------------

typedef struct
{
   int   rows;
   int   cols;
   unsigned char*  data;

} BWImage;

//------------------------------------------------------------------------------
///
/// A 8-bit greylevel component pixel  
///
//------------------------------------------------------------------------------

typedef struct
{
    int rowIndex;
    int colIndex;
    unsigned char  value;

} BWPixel;


//==============================
//    Function prototypes
//==============================


//------------------------------------------------------------------------------
///
///	 \brief Crea la struttura con i dati relativi ai colori 
///
///	 Crea la struttura con i dati relativi ai colori
///
///      \param  colorStruct         un puntatore alla struttura creata 
/// 
///	 \return 0 if everything went ok
///
//------------------------------------------------------------------------------



int CreateColorStruct(ColorStruct **colorStruct);


//------------------------------------------------------------------------------
///
///	 \brief Distrugge la struttura con i dati relativi ai colori 
///
///	 Distrugge la struttura con i dati relativi ai colori
///
///      \param  colorStruct         un puntatore alla struttura creata
///
//------------------------------------------------------------------------------



void DestroyColorStruct(ColorStruct *colorStruct);


//------------------------------------------------------------------------------
///
///				Frame processing container function
/// 
//------------------------------------------------------------------------------


int ProcessFrame();


//------------------------------------------------------------------------------
///
///	 		Frame processing container function (multi color version)
///
//------------------------------------------------------------------------------


int ProcessFrameEvol();


//------------------------------------------------------------------------------
///
/// \brief Create a new empty grey level image 
///
/// Creates a new grey empty level image, of the given dimensions
///
/// \param  rows       rows to allocate (pixels)
/// \param  cols       columns to allocate (pixels)
///
/// \return A new allocated grey level image
///
//------------------------------------------------------------------------------

int CreateBWImage(BWImage **im, int rows, int cols);

//------------------------------------------------------------------------------
///
/// Receives the freshly aquired frame for the web-cam , and returns a binary 
/// image representing the pixels which satisfy the color values constraint.  
///
//------------------------------------------------------------------------------

int ThresholdFrame(FRAME *fr, BWImage *bwIm, int color);


//------------------------------------------------------------------------------
///
///  \brief Threshold the image colors to obtain a color filtered image
///
///  Receives the freshly aquired frame for the web-cam , and returns a 
///  image representing the pixels which satisfy the color values constraint.  
/// 
///  \param  fr      the acquired frame
///  \param  bwIm    the resulting a color filtered image 
///
///	 \return 0 if everything went ok
///
//------------------------------------------------------------------------------


int ThresholdFrameMultiColor(FRAME *fr, BWImage *bwIm); 


//------------------------------------------------------------------------------
///
/// Releases all memory associated to a grey level image.
///
/// \param  binIm          The binary image to destroy
///
//------------------------------------------------------------------------------

void DestroyBWImage(BWImage* im);

//------------------------------------------------------------------------------
///
/// Saves the binary image to a PNM file for external viewing
///
/// \param          fr          The image to save
/// \param          filename    The output filename (eg. "capture.pnm")
///
//------------------------------------------------------------------------------

int SaveBWImage(BWImage* im, const char* filename);


//------------------------------------------------------------------------------
///
///  \brief Releases all memory associated to a code color image.
///
///  \param  im          The color image to destroy
///
///	 \return 0 if everything went ok
///
//------------------------------------------------------------------------------


void DestroyColorImage(BWImage* im);


//------------------------------------------------------------------------------
///
/// Computes RGB values of a pixel memorized in a YUV 422 stream
///
/// \param          pixelRGB     Structure with RGB values of the pixel 
/// \param          frame          Pointer to the YUV frame
/// \param          pixelIndex       Pixel index
///    
//------------------------------------------------------------------------------

inline int YUV422toRGB(FRAME_RGB* pixelRGB, FRAME* frame, int pixelIndex);

//------------------------------------------------------------------------------
///
/// Computes RGB values of a pixel memorized in a YUV 420P stream
///
/// \param          pixelRGB     Structure with RGB values of the pixel 
/// \param          frame          Pointer to the YUV frame
/// \param          pixelIndex       Pixel index
///    
//------------------------------------------------------------------------------

inline int YUV420PtoRGB(FRAME_RGB* pixelRGB, FRAME* frame, int pixelIndex);

//------------------------------------------------------------------------------
///
/// Return YUV values of a RGB pixel 
///
/// \param          pixelRGB  structure with RGB values of the pixel
///
//------------------------------------------------------------------------------

//inline int RGBtoYUV(FRAME_RGB* pixelRGB);


//------------------------------------------------------------------------------
///
/// Clamp values in the YCbCr valid range 
///
/// \param          Y_p       Y    component of the pixel 
/// \param          Cb_p      Cb   component of the pixel
/// \param          Cr_p      Cr   component of the pixel
///
//------------------------------------------------------------------------------

inline int CLAMP_YCbCr(unsigned char* Y_p, unsigned char* Cb_p, unsigned char* Cr_p);


//------------------------------------------------------------------------------
///
/// Clamp values in the YUV valid range 
///
/// \param          Y_p       Y   component of the pixel 
/// \param          U_p       U   component of the pixel
/// \param          V_p       V   component of the pixel
///
//------------------------------------------------------------------------------

inline int CLAMP_YUV(unsigned char* Y_p, unsigned char* U_p, unsigned char* V_p);


//------------------------------------------------------------------------------
///
/// Clamp values in the RGB valid range 
///
/// \param          R_p      R    component of the pixel 
/// \param          G_p      G    component of the pixel
/// \param          B_p      B    component of the pixel
///
//------------------------------------------------------------------------------

inline int CLAMP_RGB(int* R_p, int* G_p, int* B_p);

//------------------------------------------------------------------------------
///
/// Returns frame width and height according to size selected 
///
/// \param          char*    size resolution identificator (eg. : sif,cif,qcif)      
/// \param          frame_width_pt      G    frame width
/// \param          frame_height_pt     B    frame height
///
//------------------------------------------------------------------------------

int ResolveFrameDimensions(char* size, int* frame_width_pt, int* frame_height_pt);


//------------------------------------------------------------------------------
///
/// \brief load a PPM image file and return the a bwImage after color thresholding
///  performs RGB normalization to reduce light effect
///
/// \param  fname      image filename
/// \param  bwIm       pointer to the bwImage to create
/// \param  color      color to extract
///
/// \return The bw image \resulting from thresholding
///
//------------------------------------------------------------------------------

int ThresholdFile( char *fname, BWImage **im, int color);

//------------------------------------------------------------------------------
///
/// \brief load a PPM image file containing an image already thresholded and 
///  binarized. Return the image content in a bwImage struct
///  performs RGB normalization to reduce light effect
///
/// \param  fname      image filename
/// \param  bwIm       pointer to the bwImage to create
///
///
///
//------------------------------------------------------------------------------

int LoadBWImageFromFile( char *fname, BWImage **bwIm);

//------------------------------------------------------------------------------
///
/// \brief Process image file header, returning image size
///
/// \param file      image file pointer
/// \param width     to return image width
/// \param height    to return image height
///
//------------------------------------------------------------------------------

int ProcessHeaders(FILE *file, int *width, int *height);

#endif //#ifndef simple_image_h_DEFINED

