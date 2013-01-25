//==========================================================================
//
//  Project:        FoxVision: Vision system for mobile robots equipped with 
//					FOX board Netus G20
//
//  Module:         Capture client interface
//
//  Description:    Provides a high-level C interface for controlling frame
//                  grabber. Based on uvc streamer, supports v4l2 and UVC driver
//
//  Author:         Enrico Di Lello
//
//  
//--------------------------------------------------------------------------
//
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

#ifndef __V4L2_CAPTURE_H__
#define __V4L2_CAPTURE_H__

#ifdef __cplusplus__
extern "C" {
#endif /* __cplusplus__ */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <getopt.h>             /* getopt_long() */

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/poll.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/videodev.h>
#include <linux/videodev2.h>
#include <signal.h>

//#include "globals.h"
#include "frame.h"
#include "simple_image.h"

//==========================================================================
//  Definitions
//==========================================================================

// Standard device name
#define DEFAULT_V4L2_DEVICE_NAME       "/dev/video0"    /** Default video input */

// Standard capture window size
#define WINDOW_DEFAULT_WIDTH        320
#define WINDOW_DEFAULT_HEIGHT       240

#define CLEAR(x) memset(&(x), 0, sizeof(x))


//==========================================================================
///
/// Represents all information about a frame grabber device. Same structure 
/// used in uvc-streamer.. some fields are not used here 
///
//==========================================================================

typedef struct {

    int fd;                                       /* device file descriptor */
    char *device_name;
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    struct v4l2_format fmt;
    struct v4l2_requestbuffers req;
    struct v4l2_buffer buf;
	enum v4l2_buf_type type;
    int n_buffers;                              /* image frame buffers number */  
    struct buffer *buffers;                      
    int fps;                      
    //char *status;
    //char *pictName;
    //struct v4l2_capability cap;
    //struct v4l2_format fmt;
    //struct v4l2_buffer buf;
    //struct v4l2_requestbuffers rb;
    //void *mem[NB_BUFFER];
    //unsigned char *tmpbuffer;
    //unsigned char *framebuffer;
    //int isstreaming;
    //int grabmethod;
    //int width;
    //int height;
    //int fps;
    //int formatIn;
    //int formatOut;
    //int framesizeIn;
    //int signalquit;
    //int toggleAvi;
    //int getPict;
    //int rawFrameCapture;
    //unsigned int fileCounter;
    // raw frame stream capture 
    //unsigned int rfsFramesWritten;
    //unsigned int rfsBytesWritten;
    // raw stream capture 
    //FILE *captureFile;
    //unsigned int framesWritten;
    //unsigned int bytesWritten;
    //int framecount;
    //int recordstart;
    //int recordtime;
    
} FRAMEGRABBER_V4L2;

//========================================
/// keeps buffer mapping informations
//========================================

struct buffer {
	void   *start;
	size_t  length;
};

//======================================================================
/// ioctl redefinition with error handling.. from v4l2 capture example
//======================================================================

int xioctl(int fh, int request, void *arg);

//================================================
/// Error handling.. from v4l2 capture example
//================================================

void errno_exit(const char *s);


//------------------------------------------------------------------------------
///
///				Main camera initialization function
/// 
//------------------------------------------------------------------------------


int InitCamera();


//------------------------------------------------------------------------------
///
///				Frame grabbing container function
/// 
//------------------------------------------------------------------------------


int GetFrame();


//------------------------------------------------------------------------------
///
///				Main camera termination function
/// 
//------------------------------------------------------------------------------


int CloseCamera();


//------------------------------------------------------------------------------
///
/// Allocate memory for a FRAMEGRABBER_V4L2 data structure
/// 
//------------------------------------------------------------------------------

int CreateDeviceV4L2(FRAMEGRABBER_V4L2 **fg_v4l2, char* device_name);

//------------------------------------------------------------------------------
///
/// Free memory for a FRAMEGRABBER_V4L2 data structure
/// 
//------------------------------------------------------------------------------

int DestroyDeviceV4L2(FRAMEGRABBER_V4L2 **fg_v4l2, char* device_name);

//------------------------------------------------------------------------------
///
/// Open a UVC-video device (should work with any v4l2 compliant video driver)
/// The default device name is /dev/video0 
///
//------------------------------------------------------------------------------

int OpenDeviceV4L2(FRAMEGRABBER_V4L2 *fg_v4l2);

//------------------------------------------------------------------------------
///
/// Device initialization procedure. Sets format, memory access method...
/// 
//------------------------------------------------------------------------------

int InitDeviceV4L2(FRAMEGRABBER_V4L2 *fg_v4l2, int width, int height, int format, int fps);

//------------------------------------------------------------------------------
///
/// Initializes memory mapped buffers
/// 
//------------------------------------------------------------------------------

int InitMmapV4L2(FRAMEGRABBER_V4L2 *fg_v4l2);

//------------------------------------------------------------------------------
///
/// Clear memory buffers 
/// 
//------------------------------------------------------------------------------

int UninitDeviceV4L2(FRAMEGRABBER_V4L2 *fg_v4l2);

//------------------------------------------------------------------------------
///
/// Close device 
/// 
//------------------------------------------------------------------------------

int CloseDeviceV4L2(FRAMEGRABBER_V4L2 *fg_v4l2);

//------------------------------------------------------------------------------
///
/// Start capturing frames 
/// 
//------------------------------------------------------------------------------

int StartCapturingV4L2(FRAMEGRABBER_V4L2 *fg_v4l2);

//------------------------------------------------------------------------------
///
/// Waits for the camera to complete a frame grabbing
/// 
//------------------------------------------------------------------------------

int AcquisitionLoopV4L2(FRAMEGRABBER_V4L2 *fg_v4l2, FRAME *fr);

//------------------------------------------------------------------------------
///
///   Grab frame 
/// 
//------------------------------------------------------------------------------

int ReadFrameV4L2(FRAMEGRABBER_V4L2 *fg_v4l2, FRAME *fr);

//------------------------------------------------------------------------------
///
///  Copy FRAMEGRABBER_V4L2 buffer into a FRAME, could be fused with ReadFrame
/// 
//------------------------------------------------------------------------------

int ProcessImageV4L2(const void* p, int size, FRAME *fr);

//------------------------------------------------------------------------------
///
/// Polls the camera to discover all supported video formats 
/// 
//------------------------------------------------------------------------------

int PollCameraFormatsV4L2(FRAMEGRABBER_V4L2 *fg_v4l2, int width, int height);


inline void camera_processing();
#ifdef __cplusplus__
}
#endif // __cplusplus__ 

#endif // __V4L2_CAPTURE_H__ 
