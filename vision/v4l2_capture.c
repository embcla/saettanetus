//==============================================================================
//
//  Project:        FoxVision: Vision system for mobile robots equipped with 
//					FOX board Netus G20
//
//  Module:         Capture client implementation
//
//  Description:    Provides a high-level C interface for controlling frame
//                  grabber. Based on uvc streamer, supports v4l2 and UVC driver
//
//  Author:         Enrico Di Lello
//
//  
//------------------------------------------------------------------------------
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
//==============================================================================



#include  <sys/time.h>
#include <string.h>
#include "globals.h"
#include "v4l2_capture.h"
#include "simple_image.h"



//==========================================================================


void errno_exit(const char *s)
{
	fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
	exit(EXIT_FAILURE);
}

//==========================================================================


int xioctl(int fh, int request, void *arg)
{
	int r;

	do {
		r = ioctl(fh, request, arg);
	} while (-1 == r && EINTR == errno);

	return r;
}


//==========================================================================


int InitCamera(){

	
	printf("Initializing framegrabber... \n");
	
	// Create device data structure 
	
	if (CreateDeviceV4L2(&fg_v4l2,DEFAULT_V4L2_DEVICE_NAME) != 0 ) {		
		return -1;
	}
	
	// Open v4l2 standard device 
	
	if ( OpenDeviceV4L2( fg_v4l2 ) != 0 ) {
		printf("Unexpexted error when opening v4l2 device \n");	
		return -1;
	}
	
	// Init device with standard format and requested window size 
	
	if ( InitDeviceV4L2(fg_v4l2,DEFAULT_HOR_RESOLUTION,DEFAULT_VERT_RESOLUTION,DEFAULT_V4L2_FMT,DEFAULT_FPS) !=0 ){
		printf("Unexpexted error when initializing v4l2 device \n");	
		return -1;
	}	
	
	// Start acquisition			
	
	if ( StartCapturingV4L2(fg_v4l2) != 0 ) {	
		printf("Unexpexted error when starting to capture frames \n");	
		return -1;
	}

	// Prepare frame buffer and bwImage buffer
	
	if ( CreateFrame(&fr, DEFAULT_HOR_RESOLUTION, DEFAULT_VERT_RESOLUTION , DEFAULT_V4L2_FMT) != 0 ) {
		return -1;
	}
	
	if ( CreateBWImage(&bwIm,fr->height+1,fr->width) != 0 ) {
		return -1;
	}
	
	
return 0;
}


//=============================================================================

int CloseCamera(){
	
	if ( UninitDeviceV4L2(fg_v4l2) != 0 ) {
		printf(" Unexpected error when uninitializing V4L2 device \n");
		return -1; 
	}
	
	if ( CloseDeviceV4L2(fg_v4l2) != 0 ) {
		printf(" Unexpected error when closing V4L2 device \n");
		return -1; 
	}

	DestroyBWImage (bwIm);
	ReleaseFrame(&fr);
	
	
	free(fg_v4l2);
	
	return 0;
}

//==============================================================================


int GetFrame() {
	

	// Acquire frame
	
	printf("Acquiring from webcam frame number %d\n",frame_counter);
	gettimeofday(&step_start_time,NULL);
	
	if ( AcquisitionLoopV4L2(fg_v4l2,fr) != 0) {
		printf("Unexpexted error when acquiring frame v4l2 device \n");	
		return -1;
	}
	
	gettimeofday(&step_end_time,NULL);
    	GetElapsedTime("Frame acquisition", &step_start_time, &step_end_time, &acquisition_time );
	
	// If required, save one out of n_save_frame frames in a .ppm file
	
	if ( SAVE_FRAMES ) {
	
		if ( (frame_counter%n_save_frame) == 0){
			
			snprintf( frame_fname, sizeof(frame_fname), "frame_%d.ppm",frame_counter );
			SaveFrame(fr, frame_fname);	
			printf("Frame %d saved to file \n",frame_counter);
		
		}
	}	
		
	return 0;
}


//==============================================================================


int CreateDeviceV4L2(FRAMEGRABBER_V4L2 **fg_v4l2, char* device_name)
{
	*fg_v4l2 = (FRAMEGRABBER_V4L2*) malloc ( sizeof( FRAMEGRABBER_V4L2 ) ) ;
	
	if ( *fg_v4l2 == NULL)
	{
		printf("CreateDeviceV4L2: Cannot create device (out of memory?)...\n");
		return -1;
	}
	
	(*fg_v4l2)->device_name = device_name;		
	
	return 0;
}


//==============================================================================


int DestroyDeviceV4L2(FRAMEGRABBER_V4L2 **fg_v4l2, char* device_name)
{
	//TBD
	return 0;
}


//==============================================================================


int OpenDeviceV4L2(FRAMEGRABBER_V4L2 *fg_v4l2)
{
	struct stat st; 
		
	if (-1 == stat(fg_v4l2->device_name, &st)) {
		fprintf(stderr, "Cannot identify '%s': %d, %s\n\n",
			 fg_v4l2->device_name, errno, strerror(errno));
		exit(EXIT_FAILURE);
	}
    
	if (!S_ISCHR(st.st_mode)) {
		fprintf(stderr, "%s is no device\n\n", fg_v4l2->device_name);
		exit(EXIT_FAILURE);
	}

	printf("Opening device %s\n", fg_v4l2->device_name);
	
	fg_v4l2->fd = open(fg_v4l2->device_name, O_RDWR /* required */ | O_NONBLOCK, 0);
	//fg_v4l2->fd = open(fg_v4l2->device_name, O_RDWR /* required */, 0);	
	//fcntl(fg_v4l2->fd,F_SETSIG,SIGUSR1);
	if (-1 == fg_v4l2->fd) {
		fprintf(stderr, "Cannot open '%s': %d, %s\n\n",
			 fg_v4l2->device_name, errno, strerror(errno));
		exit(EXIT_FAILURE);
	}
	
return 0;
}


//==============================================================================


int InitDeviceV4L2(FRAMEGRABBER_V4L2 *fg_v4l2, int width, int height, int format, int fps) {
	
	unsigned int min;
    
	///======================================================================
	///  Query device capabilities, implements only memory mapped buffers
	///======================================================================
	
	CLEAR(fg_v4l2->cap);	
	
	if (-1 == xioctl(fg_v4l2->fd, VIDIOC_QUERYCAP, &fg_v4l2->cap)) {
		if (EINVAL == errno) {
			fprintf(stderr, "%s is no V4L2 device\n",
				 fg_v4l2->device_name);
			exit(EXIT_FAILURE);
		} else {
			errno_exit("VIDIOC_QUERYCAP");
		}
	}

	if (!(fg_v4l2->cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
		fprintf(stderr, "%s is no video capture device\n",
			 fg_v4l2->device_name);
		exit(EXIT_FAILURE);
	}


	//======================================================
	/// Select video input, video standard and tune here. 
	//======================================================	

	CLEAR(fg_v4l2->cropcap);

	fg_v4l2->cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (0 == xioctl(fg_v4l2->fd, VIDIOC_CROPCAP, &fg_v4l2->cropcap)) {
		fg_v4l2->crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		fg_v4l2->crop.c = fg_v4l2->cropcap.defrect;				  /* reset to default */

		if (-1 == xioctl(fg_v4l2->fd, VIDIOC_S_CROP, &fg_v4l2->crop)) {
			switch (errno) {
			case EINVAL:				
				break;							   /* Cropping not supported. */
			default:
				break;									   /* Errors ignored. */
			}
		}
	} else {
		                                                   /* Errors ignored. */
	}

	CLEAR(fg_v4l2->fmt);

	fg_v4l2->fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	
	// setting requested format and resolution
	
		fg_v4l2->fmt.fmt.pix.width       = width;
		fg_v4l2->fmt.fmt.pix.height      = height;
		fg_v4l2->fmt.fmt.pix.pixelformat = format;
		fg_v4l2->fmt.fmt.pix.field	    = V4L2_FIELD_ANY;
	
	if (-1 == xioctl(fg_v4l2->fd, VIDIOC_S_FMT, &fg_v4l2->fmt)){
		
		printf("Request format unavailable, retrieving original settings\n");
		getchar();
	
		if (-1 == xioctl(fg_v4l2->fd, VIDIOC_G_FMT, &fg_v4l2->fmt)) {  /* Preserve original settings*/
			printf("Error when trying to retrieve original settings");
			errno_exit("VIDIOC_G_FMT");
			return -1;
		}
	}
	else {                /* Note VIDIOC_S_FMT may change width and height. */
		
		if (format == V4L2_PIX_FMT_YUYV ) {
		printf("Format YUYV successfully set\n");
		}
		if (format == V4L2_PIX_FMT_MJPEG ){
		printf("Format MJPEG successfully set\n");
		}
		
	}
	
	///=========================
	/// Buggy driver paranoia. 
	///=========================
		
	min = fg_v4l2->fmt.fmt.pix.width * 2;
	if (fg_v4l2->fmt.fmt.pix.bytesperline < min)
		fg_v4l2->fmt.fmt.pix.bytesperline = min;
	min = fg_v4l2->fmt.fmt.pix.bytesperline * fg_v4l2->fmt.fmt.pix.height;
	if (fg_v4l2->fmt.fmt.pix.sizeimage < min)
		fg_v4l2->fmt.fmt.pix.sizeimage = min;
				
	///=============================
	///		Set framerate  
	///=============================		
			
	fg_v4l2->fps = fps;		
	struct v4l2_streamparm *setfps;
	setfps =
		(struct v4l2_streamparm *) calloc(1, sizeof(struct v4l2_streamparm));
	memset(setfps, 0, sizeof(struct v4l2_streamparm));
	setfps->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	setfps->parm.capture.timeperframe.numerator = 1;
	setfps->parm.capture.timeperframe.denominator = fg_v4l2->fps;
	
	if (-1 == xioctl(fg_v4l2->fd, VIDIOC_S_PARM, setfps) ) {
		printf("Error when trying to set frame rate");
		return -1;
	}
	
	//==========================================
	/// Implements only memory mapped buffers
	//==========================================
		
	InitMmapV4L2(fg_v4l2);	
		
return 0;
}


//==============================================================================


int InitMmapV4L2(FRAMEGRABBER_V4L2 *fg_v4l2)
{
	
	CLEAR(fg_v4l2->req);

	fg_v4l2->req.count = 2;
	fg_v4l2->req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fg_v4l2->req.memory = V4L2_MEMORY_MMAP;

	//=========================
	///    Request buffers
	//=========================
		
	if (-1 == ioctl(fg_v4l2->fd, VIDIOC_REQBUFS, &fg_v4l2->req)) {
		if (EINVAL == errno) {
			fprintf(stderr, "%s does not support "
				 "memory mapping\n", fg_v4l2->device_name);
			exit(EXIT_FAILURE);
		} else {
			printf("l'errore e qui\n");
			errno_exit("VIDIOC_REQBUFS");
		}
	}

	if (fg_v4l2->req.count < 2) {
		fprintf(stderr, "Insufficient buffer memory on %s\n",
			 fg_v4l2->device_name);
		exit(EXIT_FAILURE);
	}

	//================================
	///      Map the buffers
	//================================
	
	fg_v4l2->buffers = calloc(fg_v4l2->req.count, sizeof(*fg_v4l2->buffers));     /* keeps track of buffer size and length */

	if (!fg_v4l2->buffers) {
		fprintf(stderr, "Out of memory\n");
		exit(EXIT_FAILURE);
	}

	for (fg_v4l2->n_buffers = 0; fg_v4l2->n_buffers < fg_v4l2->req.count; ++fg_v4l2->n_buffers) {

		CLEAR(fg_v4l2->buf);

		fg_v4l2->buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		fg_v4l2->buf.memory      = V4L2_MEMORY_MMAP;
		fg_v4l2->buf.index       = fg_v4l2->n_buffers;

		if (-1 == xioctl(fg_v4l2->fd, VIDIOC_QUERYBUF, &fg_v4l2->buf))
			errno_exit("VIDIOC_QUERYBUF");

		fg_v4l2->buffers[fg_v4l2->n_buffers].length = fg_v4l2->buf.length;
		fg_v4l2->buffers[fg_v4l2->n_buffers].start =
			mmap(NULL /* start anywhere */,
			      fg_v4l2->buf.length,
			      PROT_READ | PROT_WRITE /* required */,
			      MAP_SHARED /* recommended */,
			      fg_v4l2->fd, fg_v4l2->buf.m.offset);

		if (MAP_FAILED == fg_v4l2->buffers[fg_v4l2->n_buffers].start)
			errno_exit("mmap");
	}
	
return 0;
}


//==============================================================================


int StartCapturingV4L2(FRAMEGRABBER_V4L2 *fg_v4l2)
{
	unsigned int i;
	
	//=================
	// Queue buffers
	//=================
	
	for (i = 0; i < fg_v4l2->n_buffers; ++i) {
		
		CLEAR (fg_v4l2->buf);
		fg_v4l2->buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		fg_v4l2->buf.memory      = V4L2_MEMORY_MMAP;
		fg_v4l2->buf.index       = i;
		if (-1 == xioctl (fg_v4l2->fd, VIDIOC_QBUF, &fg_v4l2->buf))
			errno_exit ("VIDIOC_QBUF");
	}
	
	fg_v4l2->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (-1 == xioctl (fg_v4l2->fd, VIDIOC_STREAMON, &fg_v4l2->type)){
		errno_exit ("VIDIOC_STREAMON");
	}
	
return 0;
}


//==============================================================================


int AcquisitionLoopV4L2(FRAMEGRABBER_V4L2 *fg_v4l2, FRAME *fr){
	
	
	unsigned int count;
	count = 1;
	while (count-- > 0) {
		for (;;) {
			fd_set fds;
			struct timeval tv;
			
			int r;
			FD_ZERO (&fds);
			FD_SET (fg_v4l2->fd, &fds);
			/* Timeout. */
			tv.tv_sec = 0;
			tv.tv_usec = 15000;
			
			// Here is where we can check the camera framerate
			//gettimeofday(&frame_grab_start,NULL);
			
			r = select ((fg_v4l2->fd + 1), &fds, NULL, NULL, &tv);
			//r = poll ((fg_v4l2->fd), &fds, 100);
			//gettimeofday(&frame_grab_end,NULL);
			//GetElapsedTime("Frame acquisition",&frame_grab_start,&frame_grab_end);
			
			if (-1 == r) {
				if (EINTR == errno)
					continue;
				errno_exit ("select");
			}
			if (0 == r) {
				fprintf (stderr, "select timeout\n");
				break; //EDL non uscire quando scade il timeout... continua con i frame successivi
				//exit (EXIT_FAILURE);
			}
			// Here is where the frame transfer time can be checked
			//gettimeofday(&frame_transfer_start,NULL);
			
			if (ReadFrameV4L2(fg_v4l2,fr))
			
			//gettimeofday(&frame_transfer_end,NULL);
			//GetElapsedTime("Frame data transfer",&frame_transfer_start,&frame_transfer_end);			
			break;
			/* EAGAIN - continue select loop. */
		}
	}
	
	return 0;
}


//==============================================================================


int ReadFrameV4L2 (FRAMEGRABBER_V4L2 *fg_v4l2,FRAME *fr)
{
	CLEAR (fg_v4l2->buf);
    fg_v4l2->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fg_v4l2->buf.memory = V4L2_MEMORY_MMAP;
	
	//=========================
	///   Dequeue buffer
	//=========================
	
	if (-1 == xioctl (fg_v4l2->fd, VIDIOC_DQBUF, &fg_v4l2-> buf)) {
		switch (errno) {
			case EAGAIN:
				return 0;
			case EIO:
				/* Could ignore EIO, see spec. */
				/* fall through */
			default:
				errno_exit ("VIDIOC_DQBUF");
		}
	}
	
	
	//======================================================
	/// Copy framegrabber buffer to the FRAME field
	//======================================================
	
	ProcessImageV4L2 (fg_v4l2->buffers[fg_v4l2->buf.index].start, fg_v4l2->buf.bytesused, fr);
			
	if (-1 == xioctl (fg_v4l2->fd, VIDIOC_QBUF, &fg_v4l2->buf)){
		errno_exit ("VIDIOC_QBUF");
	}
	
	return 1;
}

//==============================================================================


int ProcessImageV4L2(const void* p, int size, FRAME *fr)
{
	memcpy(fr->data, p, size);
    return 0;
}


//==============================================================================


int UninitDeviceV4L2(FRAMEGRABBER_V4L2 *fg_v4l2){
	
	unsigned int i;
	
	//=================================
	///  Only memory mapped buffers
	//=================================	
		
		for (i = 0; i < fg_v4l2->n_buffers; ++i){
			if (-1 == munmap (fg_v4l2->buffers[i].start, fg_v4l2->buffers[i].length))
				errno_exit ("munmap");
	       //free(fg_v4l2->buffers[i].start);
		   //free(fg_v4l2->buffers[i].length);
		}
	
	free (fg_v4l2->buffers);

	return 0;	
}


//==============================================================================


int CloseDeviceV4L2(FRAMEGRABBER_V4L2 *fg_v4l2) {
	
	if (-1 == close(fg_v4l2->fd))
		errno_exit("close");
	fg_v4l2->fd = -1;
	
	return 0;
}


//==============================================================================


int PollCameraFormatsV4L2(FRAMEGRABBER_V4L2 *fg_v4l2, int width, int height){
	
	int i;
	int num_formats = 13;
	//int width = WINDOW_DEFAULT_WIDTH;
	//int height = WINDOW_DEFAULT_HEIGHT;
	FMT current_format;
	
	// Test all v4l2 supported formats, when querying device capabilities
	
	FMT fmts[] =
	{ 
		{ "mjpeg",   V4L2_PIX_FMT_MJPEG },
 		{ "yuv422",  V4L2_PIX_FMT_YUYV  },
		{ "yuv411",  V4L2_PIX_FMT_Y41P  },
		{ "yuv422p", V4L2_PIX_FMT_YUV422P  },
		{ "yuv411p", V4L2_PIX_FMT_YUV411P  },
		{ "yuv420p", V4L2_PIX_FMT_YVU420  },
		{ "yuv410p", V4L2_PIX_FMT_YVU410 },
		{ "grey",    V4L2_PIX_FMT_GREY  },
		{ "hi240",   V4L2_PIX_FMT_HI240  },
		{ "rgb565",  V4L2_PIX_FMT_RGB565  },
		{ "rgb555",  V4L2_PIX_FMT_RGB555  },
		{ "rgb24",   V4L2_PIX_FMT_BGR24  },
		{ "uyvy",    V4L2_PIX_FMT_UYVY  },
	
	};
	
	//=====================================================
	///	 Try to open the device with all supported formats
	//=====================================================
	
	for(i=0;i<num_formats;i++){
	
		current_format = fmts[i];
		
		printf("Trying to initialize the device with format(window size) : %s(%d x %d) \n", current_format.name, width, height);
		InitDeviceV4L2 (fg_v4l2,width,height,current_format.format,10);
		UninitDeviceV4L2(fg_v4l2);
	    
	}
	
	return 0;
}


//==============================================================================

//=========================================================================================================

inline void camera_processing() {

    // 			      Vision step
    //Frame Grabbing
    gettimeofday(&vision_start_time, NULL);
    //printf("%s\n",vision_start_time);
    GetFrame(); //blocking
    frame_counter++;
    //Processing Frame
    //ProcessFrame();
    ProcessFrameEvol();
    EvaluateCoordinates(); // commentata la parte che accede alla mappa
    gettimeofday(&vision_end_time, NULL);
    //GetElapsedTime("Frame Processing",&vision_start_time, &vision_end_time, &processing_time);
    //printf("TEMPO: %d\n", processing_time);
    LogTimeData();
}
//=========================================================================================================
