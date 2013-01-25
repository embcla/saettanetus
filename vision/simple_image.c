//==========================================================================
//
//  Project:        FoxVision: Vision system for mobile robots equipped with 
//					FOX board Netus G20
//
//  Module:         Format conversion and simple image handling functions
//					implementations
//
//  Description:    Provides a high-level C interface for controlling frame
//                  grabber. Based on uvc streamer, supports v4l2 and UVC driver
//
//  Author:         Enrico Di Lello
//  Author:  	    and many small changes by Giovanni Micheli
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

//#include "simple_image.h" 
#include "globals.h" 


//=============================================================================

int CreateColorStruct(ColorStruct **colorStruct) {

	// alloco la struttura ColorStruct
	*colorStruct = (ColorStruct *)malloc( sizeof( ColorStruct ) );

	if ( *colorStruct == NULL)
	{
		printf("CreateColorStruct: Non è possibile creare colorStruct (out of memory?)...\n");
		return -1;
	}
	
	// Indico i colori da utilizzare (è possibile specificare solo quelli desiderati)	
	// IMPORTANTE: non utilizzare il bianco, gia' utilizzato per marcare la croce
	// Colori disponibili:
	//	1-red,
	//	2-cyan, 
	//	3-green,
	//	4-yellow	
	(*colorStruct)->maxColor = 4; //numero massimo di colori disponibili
	(*colorStruct)->numColor = 4; //numero di colori da utilizzare
	(*colorStruct)->useColor = (int*)malloc(sizeof(int)*((*colorStruct)->numColor));
	(*colorStruct)->useColor[0] = 1;
	(*colorStruct)->useColor[1] = 2;
	(*colorStruct)->useColor[2] = 4;
	(*colorStruct)->useColor[3] = 3;
	// Calcolo il numero di colori utilizzati
 			

		//Alloco dtColorYCbCr
		//(*colorStruct)->dtColorYCbCr = (int*)malloc(sizeof(int)* 6 * (*colorStruct)->numColor);
		(*colorStruct)->dtColorYCbCr = (int*)malloc(sizeof(int)* 6 * (*colorStruct)->maxColor );
		//riempo dtColorYCbCr con i dati relativi ai colori
		//1-RED YCbCr
		(*colorStruct)->dtColorYCbCr[0] = 42;//52;//75 		//RED_Y_MIN
		(*colorStruct)->dtColorYCbCr[1] = 256;			//RED_Y_MAX
		(*colorStruct)->dtColorYCbCr[2] = 70;			//RED_Cb_MIN
		(*colorStruct)->dtColorYCbCr[3] = 256;			//RED_Cb_MAX
		(*colorStruct)->dtColorYCbCr[4] = 160;//170;//180	//RED_Cr_MIN
		(*colorStruct)->dtColorYCbCr[5] = 256;			//RED_Cr_MAX

		//2-CYAN YCbCr
		(*colorStruct)->dtColorYCbCr[6] = 80;//100;		//CYAN_Y_MIN
		(*colorStruct)->dtColorYCbCr[7] = 220;//256;		//CYAN_Y_MAX
		(*colorStruct)->dtColorYCbCr[8] = 125;//140;		//CYAN_Cb_MIN
		(*colorStruct)->dtColorYCbCr[9] = 155;//195;		//CYAN_Cb_MAX
		(*colorStruct)->dtColorYCbCr[10] = 10;//0;		//CYAN_Cr_MIN
		(*colorStruct)->dtColorYCbCr[11] = 110;			//CYAN_Cr_MAX
				
		//3-GREEN YCbCr
		(*colorStruct)->dtColorYCbCr[12] = 80;//100; 		//RED_Y_MIN
		(*colorStruct)->dtColorYCbCr[13] = 256;			//RED_Y_MAX
		(*colorStruct)->dtColorYCbCr[14] = 0;			//RED_Cb_MIN
		(*colorStruct)->dtColorYCbCr[15] = 110;//120;		//RED_Cb_MAX
		(*colorStruct)->dtColorYCbCr[16] = 80;			//RED_Cr_MIN
		(*colorStruct)->dtColorYCbCr[17] = 110;//120;		//RED_Cr_MAX
		
		//4-YELLOW YCbCr
		(*colorStruct)->dtColorYCbCr[18] = 80;//146; 		//YELLOW_Y_MIN
		(*colorStruct)->dtColorYCbCr[19] = 150;//227;		//YELLOW_Y_MAX
		(*colorStruct)->dtColorYCbCr[20] = 0;			//YELLOW_Cb_MIN
		(*colorStruct)->dtColorYCbCr[21] = 85;//83;		//YELLOW_Cb_MAX
		(*colorStruct)->dtColorYCbCr[22] = 50;//145;		//YELLOW_Cr_MIN
		(*colorStruct)->dtColorYCbCr[23] = 200;//192;		//YELLOW_Cr_MAX

		//Alloco dtColorRGB
		//(*colorStruct)->dtColorRGB = (int*)malloc(sizeof(int)* 3 * (*colorStruct)->numColor);
		(*colorStruct)->dtColorRGB = (int*)malloc(sizeof(int)* 3 * (*colorStruct)->maxColor );
		//riempo dtColorYCbCr con i dati relativi ai colori
		//1-RED RGB
		(*colorStruct)->dtColorRGB[0] = 255; 		//RED_R
		(*colorStruct)->dtColorRGB[1] = 0;		//RED_G
		(*colorStruct)->dtColorRGB[2] = 0;		//RED_B
		//2-CYAN RGB
		(*colorStruct)->dtColorRGB[3] = 0; 		//CYAN_R
		(*colorStruct)->dtColorRGB[4] = 200;		//CYAN_G
		(*colorStruct)->dtColorRGB[5] = 255;		//CYAN_B
		//3-GREEN RGB
		(*colorStruct)->dtColorRGB[6] = 0; 		//GREEN_R
		(*colorStruct)->dtColorRGB[7] = 255;		//GREEN_G
		(*colorStruct)->dtColorRGB[8] = 0;		//GREEN_B
		//4-YELLOW RGB
		(*colorStruct)->dtColorRGB[9] = 255; 		//YELLOW_R
		(*colorStruct)->dtColorRGB[10] = 255;		//YELLOW_G
		(*colorStruct)->dtColorRGB[11] = 0;		//YELLOW_B
		

		return 0;
	}


	//=============================================================================


	void DestroyColorStruct(ColorStruct *colorStruct) {
		free( colorStruct->dtColorYCbCr );
		free( colorStruct->dtColorRGB );
		free( colorStruct );
	}


	//=============================================================================

	int ProcessFrame(){
		
		// Apply a color threshold to the frame and put the result in the 
		// binary image 	
		printf("Thresholding frame number %d\n",frame_counter);
		
		gettimeofday(&step_start_time,NULL);
		
		if ( ThresholdFrame(fr, bwIm, RED) != 0 )  {
			return -1;
		}
		
		// If required, save one out of n_save_frame binary images in a .ppm file
		
		if ( SAVE_BINARY ) {	
			
			if ( (frame_counter%n_save_frame) == 0){
			
			snprintf( binary_fname, sizeof(binary_fname), "binary_%d.ppm",frame_counter );
			SaveBWImage(bwIm, binary_fname);
			printf("Binary image %d saved to file \n",frame_counter);   
			
			}
		}	
		
		// Label the connected components of the image
		printf("Labelling image number %d\n",frame_counter);	

		if ( CreateComponentsList(&compList,2) != 0 ) {
			return -1;
		}
		
		if ( PerformLabelling(bwIm,compList) != 0 )	{
			return -1;
		}
		
		// Compute blob centroids
		
		if ( ComputeCentroids(compList,bwIm) != 0 ) {
			return -1;
		}		
		
		gettimeofday(&step_end_time, NULL);
		GetElapsedTime("Frame Processing",&step_start_time, &step_end_time, &processing_time);
		
		if ( SAVE_CENTROIDS ) {	
			
			if ( (frame_counter%n_save_frame) == 0){
			
			snprintf( binary_fname, sizeof(binary_fname), "binary_%d.ppm",frame_counter );
			SaveBWImage(bwIm, binary_fname);
			printf("Binary image %d saved to file \n",frame_counter);   
			
			}
		}	
		
		
		// Clear dynamic data structures
		
		//DestroyBWImage (bwIm);
		//ReleaseFrame(&fr);
		
		return 0;
		
	}


	//=============================================================================


	int ProcessFrameEvol(){
		
		// Apply a color threshold to the frame and put the result in a 
		// matrix which represent a colors codified image
		
		#ifdef VISION_PRINT	
		printf("Thresholding frame number %d\n",frame_counter);
		#endif	
		gettimeofday(&step_start_time,NULL);
		
		if ( ThresholdFrameMultiColor(fr, bwIm) != 0 )  {
			return -1;
		}
		
		// If required, save one out of n_save_frame colors codified images in a .ppm file
		
		if ( SAVE_COLOR_IMG ) {	
			
			if ( (frame_counter%n_save_frame) == 0){
			
			snprintf( binary_fname, sizeof(binary_fname), "color_%d.ppm",frame_counter );
			SaveColorImage(bwIm, binary_fname);
			printf("Color image %d saved to file \n",frame_counter);   
			//printf("%d\n",frame_counter);
			}
		}	
		
		// Label the connected components of the image
		#ifdef VISION_PRINT
		printf("Labelling image number %d\n",frame_counter);	
		#endif	
		
		if ( CreateComponentsList(&compList,2) != 0 ) {
			return -1;
		}
		
		if ( PerformLabellingColor(bwIm,compList, colorStruct->maxColor) != 0 )	{
			return -1;
		}
		
		// Compute blob centroids
		
		if ( ComputeCentroidsColor(compList,bwIm) != 0 ) {
			return -1;
		}		
		
		gettimeofday(&step_end_time, NULL);
		GetElapsedTime("Frame Processing",&step_start_time, &step_end_time, &processing_time);
		
		if ( SAVE_CENTROIDS ) {	
			
			if ( (frame_counter%n_save_frame) == 0){
			
			snprintf( binary_fname, sizeof(binary_fname), "color_%d.ppm",frame_counter );
			SaveColorImage(bwIm, binary_fname);
			printf("Color image %d saved to file \n",frame_counter);   
			//printf("%d\n",frame_counter);
			}
		}	
		
		// Clear dynamic data structures
		
		//QUI
		//DestroyColorImage (bwIm);
		//ReleaseFrame(&fr);
		
		return 0;
		
	}


	//=============================================================================

	int ThresholdFrame(FRAME *fr, BWImage *bwIm, int color){
		
		int i,imageLength;
		int PixelI,PixelJ;
		unsigned char Y;
		unsigned char Cr;
		
		//Add one line of black pixels at the beginning for centroid extraction step
		for (i=0; i<bwIm->cols;i++){
			bwIm->data[i] = 0;
		}
		
		// Scan the original frame 
		for ( i=0; i < (fr->width * fr->height); i++) {
			
			//=================================================
			///		Color segmentation step					  
			//=================================================
				
				if (fr->format == V4L2_PIX_FMT_YUYV)	{
					
					// Get the Y component of the required pixel
					Y = ((unsigned char*)(fr->data))[i*2];
					
					//Get the Cr component of the required pixel
					
					if (i%2 == 0) {
						Cr = ((unsigned char*)(fr->data))[(i*2)+3];
					}
					else {
						Cr = ((unsigned char*)(fr->data))[(i*2)+1]; 
					}
				}
			
				if (fr->format == VIDEO_PALETTE_YUV420P)	{
					
					imageLength = fr->width*fr->height;
					
					//Get 2 dimensional pixel indexes
					PixelI = i/fr->width;
					PixelJ = i%fr->width;
				
					//Get the Y component of the required pixel
					Y =  ((unsigned char*)(fr->data))[i];
				
					//Get the Cr component of the required pixel
					Cr = ((unsigned char*)(fr->data))[(PixelI/2*(fr->width/2)) + PixelJ/2 + imageLength + imageLength/4]; 
				
				}
			
			if ((Y > RED_Y_MIN) & (Cr > RED_Cr_MIN))
				//write after the black dummy line  
				bwIm->data[(i)+bwIm->cols] = 1;                  /*light the pixel*/
			else
				
				bwIm->data[(i)+bwIm->cols] = 0;               /*turn off the pixel*/
		}
		
		return 0;
	}



	//=============================================================================


	int ThresholdFrameMultiColor(FRAME *fr, BWImage *bwIm){
		
		int i,imageLength; 
		int indexUseColor, codColor;
		int PixelI,PixelJ;
		unsigned char Y;
		unsigned char Cr,Cb;
		
		//Add one line of black pixels at the beginning for centroid extraction step
		for (i=0; i<bwIm->cols;i++){
			bwIm->data[i] = 0;
		}
		
		// Scan the original frame 
		for ( i=0; i < (fr->width * fr->height); i++) {
			
			//=================================================
			///		Color segmentation step					  
			//=================================================
				
				if (fr->format == V4L2_PIX_FMT_YUYV)	{
					
					// Get the Y component of the required pixel
					Y = ((unsigned char*)(fr->data))[i*2];
					
					//Get the Cr component of the required pixel
					//aggiungo la componente Cb
					
					if (i%2 == 0) {
						Cr = ((unsigned char*)(fr->data))[(i*2)+3];
						Cb = ((unsigned char*)(fr->data))[(i*2)+1];
					}
					else {
						Cr = ((unsigned char*)(fr->data))[(i*2)+1];
						Cb = ((unsigned char*)(fr->data))[(i*2)-1]; 
					}
				}
			
				if (fr->format == VIDEO_PALETTE_YUV420P)	{
					
					imageLength = fr->width*fr->height;
					
					//Get 2 dimensional pixel indexes
					PixelI = i/fr->width;
					PixelJ = i%fr->width;
				
					//Get the Y component of the required pixel
					Y =  ((unsigned char*)(fr->data))[i];
				
					//Get the Cr component of the required pixel
					Cr = ((unsigned char*)(fr->data))[(PixelI/2*(fr->width/2)) + PixelJ/2 + imageLength + imageLength/4]; 
				
				}
			/*
			//Controllo componente Rossa 
			if ((Y > RED_Y_MIN) & (Cr > RED_Cr_MIN) & (Cb > RED_Cb_MIN)) {
				//write after the black dummy line  
				bwIm->data[(i)+bwIm->cols] = 1;                  // light the pixel //
			} else if (( Y > CYAN_Y_MIN) & (Cb > CYAN_Cb_MIN) & (Cb < CYAN_Cb_MAX) & (Cr < CYAN_Cr_MAX)) { 
				//Controllo componente ciano 
				bwIm->data[(i)+bwIm->cols] = 2;
			} else if (( Y > GREEN_Y_MIN) & (Cb < GREEN_Cb_MAX) & (Cr > GREEN_Cr_MIN) & (Cr < GREEN_Cr_MAX)) { 
				//Controllo componente verde
				bwIm->data[(i)+bwIm->cols] = 3;		
			} else {
				bwIm->data[(i)+bwIm->cols] = 0;               // turn off the pixel //
			}
			*/
			
			bwIm->data[(i)+bwIm->cols] = 0;
			// Per ogni pixel controllo i valori di soglia specifici di ogni colore di interesse
			//OLD//for (codColor=1; codColor<colorStruct->numColor+1; codColor++) {
			for (indexUseColor=0; indexUseColor < colorStruct->numColor; indexUseColor++) {
				//individuo il codice colore nel vettore dei colori da utilizzare
				codColor = colorStruct->useColor[indexUseColor] ;
				
				//verifico le soglie per ogni codice colore selezionato
				if (( Y > dtYmin(codColor)) & ( Y< dtYmax(codColor)) & ( Cb > dtCbmin(codColor)) & ( Cb < dtCbmax(codColor)) & ( Cr > dtCrmin(codColor)) & ( Cr <dtCrmax(codColor))) {
					bwIm->data[(i)+bwIm->cols] = codColor; 			
				}
			}
			
			
		}
		
		return 0;
	}


	//=============================================================================

	int CreateBWImage( BWImage **im, int rows, int cols )
	{
		*im = (BWImage *)malloc( sizeof( BWImage ) );
		
		if ( *im == NULL)
		{
			printf("CreateBWImage: Cannot create bwIm (out of memory?)...\n");
			return -1;
		}
	    
		(*im)->rows = rows;
		(*im)->cols = cols;
		(*im)->data = (unsigned char*)malloc((rows*cols)*sizeof(unsigned char));
		
		if ( (*im)->data == NULL)
		{
			printf("CreateBWImage: Cannot create data field (out of memory?)...\n");
			return -1;
		}
		
		return 0;
	}

	//------------------------------------------------------------------------------


	void DestroyBWImage( BWImage* im )
	{
		free( im->data );
		free( im );
	}

	//=============================================================================

	int SaveBWImage ( BWImage *bwIm, const char* filename )
	{
		int i = 0;
		FILE* fp = fopen( filename, "w" );
		
		if ( fp == NULL )
		{
			perror( "SaveBWImage: opening file for writing" );
			return -1;
		}
		
		// Write PNM header
		fprintf( fp, "P6\n" );
		fprintf( fp, "# Generated by a herd of rabid fleas\n" );
		fprintf( fp, "%d %d\n", bwIm->cols, bwIm->rows );
		
		// Max val
		fprintf( fp, "255\n" );
		
		for ( i=0; i < (bwIm->rows * bwIm->cols); i++) {
			if(bwIm->data[i] == 1) {
				fprintf(fp,"%c%c%c", 255,255,255);
			}
			else if (bwIm->data[i] == RED){
				//printf("SaveBWImage : Centroid pixel marked \n");
				//getchar();
			fprintf( fp, "%c%c%c",255,0,0 ); 
			}
			else {
				fprintf(fp,"%c%c%c", 0,0,0);
			}
		}
		
		fclose( fp );

	    return 0;
	}

	//=============================================================================

	void DestroyColorImage( BWImage* im )
	{
		free( im->data );
		free( im );
	}

	//=============================================================================

	int SaveColorImage ( BWImage *bwIm, const char* filename )
	{
		int i = 0;
		int codColor = 0;
		FILE* fp = fopen( filename, "w" );
		
		if ( fp == NULL )
		{
			perror( "SaveColorImage: opening file for writing" );
			return -1;
		}
		
		// Write PNM header
		fprintf( fp, "P6\n" );
		fprintf( fp, "# Generated by a herd of rabid fleas\n" );
		fprintf( fp, "%d %d\n", bwIm->cols, bwIm->rows );
		
		// Max val
		fprintf( fp, "255\n" );
		
		/* //vecchia versione
		for ( i=0; i < (bwIm->rows * bwIm->cols); i++) {
			//marco il rosso		
			if(bwIm->data[i] == 1) {
				//fprintf(fp,"%c%c%c", 255,255,255);
				fprintf(fp,"%c%c%c", 255,0,0);
			}
			else if (bwIm->data[i] == 2) {
				fprintf(fp,"%c%c%c", 0,200,255);
			}
			else if (bwIm->data[i] == 3) {
				fprintf(fp,"%c%c%c", 0,255,0);
			}
			else if (bwIm->data[i] == CROSS){
				//printf("SaveBWImage : Centroid pixel marked \n");
				//getchar();
				//fprintf( fp, "%c%c%c",0,255,0 );
				//fprintf( fp, "%c%c%c",200,135,200 );
				fprintf(fp, "%c%c%c",255,255,255); //coloro la croce con il bianco
			}
			else {
				fprintf(fp,"%c%c%c", 0,0,0);
			}
		}
		*/

		
		for ( i=0; i < (bwIm->rows * bwIm->cols); i++) {
			// memorizzo il codice colore relativo al pixel
			codColor = bwIm->data[i];
			// se il codice corrisponde ad uno dei colori 
			if (codColor < colorStruct->maxColor+1 && codColor>0 ) {
				// salvo nell'immagine l'rgb corrispondente al codice colore
				fprintf(fp,"%c%c%c", dtR(codColor),dtG(codColor),dtB(codColor));
			} 
			// se invece corrisponde al codice della croce del centroide
			else if (codColor == CROSS){
				fprintf(fp, "%c%c%c",255,255,255); //coloro la croce con il bianco
			} 
			// altrimenti il pixel corrispondente verra' rappresentato in nero		
			else {
				fprintf(fp,"%c%c%c", 0,0,0);
			}
		}
		


		fclose( fp );

	    return 0;
	}


//=============================================================================

inline int YUV422toRGB(FRAME_RGB* pixelRGB, FRAME* frame, int pixelIndex)
{	
	unsigned char Y;
	unsigned char Cb;
	unsigned char Cr;
	int R,G,B;
	char U;
	char V;

	
	//Get the Y component of the required pixel
	Y =  ((unsigned char*)(frame->data))[(pixelIndex*2)];
	
	//Get the Cb and Cr components of the required pixel
	
	if (pixelIndex%2 == 0) {
		Cb = ((unsigned char*)(frame->data))[(pixelIndex*2)+1];
		Cr = ((unsigned char*)(frame->data))[(pixelIndex*2)+3];
	}
	else {
		Cb = ((unsigned char*)(frame->data))[(pixelIndex*2)-1];
		Cr = ((unsigned char*)(frame->data))[(pixelIndex*2)+1]; 
	}
	
	//Convert YCbCr to YUV
	
	CLAMP_YCbCr(&Y,&Cb,&Cr);        // Clamp the values to their valid range  
	
	Y = YtoY1(Y - 16);
	U = CbtoU(Cb - 128);
	V = CrtoV(Cr - 128);
	
	//CLAMP_YUV(&Y,&U,&V);
	
	//Convert YUV to RGB values     (http://www.thedirks.org/v4l2/v4l2fmt.htm)
	
	R = YUVtoR(Y,V);
	G = YUVtoG(Y,U,V);
	B = YUVtoB(Y,U);
	
	CLAMP_RGB(&R,&G,&B);			// Clamp the values to their valid range 
	
	pixelRGB->red = (unsigned char)(R);
	pixelRGB->green = (unsigned char)(G); 
	pixelRGB->blue = (unsigned char)(B);
	
	return 0;
}

//=============================================================================

inline int YUV420PtoRGB(FRAME_RGB* pixelRGB,FRAME* frame, int pixelIndex) {
	
	unsigned char Y;
	unsigned char Cb;
	unsigned char Cr;
	
	int R,G,B;
	char U;
	char V;
	
	int imageLength;
	imageLength = frame->width*frame->height;
	int PixelI;
	int PixelJ;
	
	//Get 2 dimensional pixel indexes
	PixelI = pixelIndex/frame->width;
	PixelJ = pixelIndex%frame->width;
	
	//Get the Y component of the required pixel
	Y =  ((unsigned char*)(frame->data))[pixelIndex];
	
	//Get the Cb component of the required pixel
	Cb = ((unsigned char*)(frame->data))[(PixelI/2*(frame->width/2)) + PixelJ/2 + imageLength];
	
	//Get the Cr component of the required pixel
	Cr = ((unsigned char*)(frame->data))[(PixelI/2*(frame->width/2)) + PixelJ/2 + imageLength + imageLength/4]; 
	
	//Convert YCbCr to YUV
	
	CLAMP_YCbCr(&Y,&Cb,&Cr);
	
	Y = YtoY1(Y - 16);
	U = CbtoU(Cb - 128);
	V = CrtoV(Cr - 128);
	
	//Convert YUV to RGB values (http://www.thedirks.org/v4l2/v4l2fmt.htm)
	
	R = YUVtoR(Y,V);
	G = YUVtoG(Y,U,V);
	B = YUVtoB(Y,U);
	
	CLAMP_RGB(&R,&G,&B);
	
	pixelRGB->red = (unsigned char)(R);
	pixelRGB->green = (unsigned char)(G); 
	pixelRGB->blue = (unsigned char)(B);
	
	
	
	return 0;
}

/*	
//----------------------------------------------------------------------------------

inline int RGBtoYUV(FRAME_RGB* pixelRGB) 
{
	
	unsigned char Y;
	unsigned char U;
	unsigned char V;
	
	//Convert RGB to YUV values (http://www.thedirks.org/v4l2/v4l2fmt.htm)
	
	Y = RGBtoY(pixelRGB->red,pixelRGB->green,pixelRGB->blue);		
	U = RGBtoU(pixelRGB->red,pixelRGB->green,pixelRGB->blue);
	V = RGBtoV(pixelRGB->red,pixelRGB->green,pixelRGB->blue);
	
	pixelRGB->red = Y;
	pixelRGB->green = U;
	pixelRGB->blue = V;
	
	return 0;
}

*/

//=============================================================================


inline int CLAMP_YCbCr(unsigned char* Y_p, unsigned char* Cb_p, unsigned char* Cr_p){

	if (*Y_p<=16)   *Y_p = 16;
	if (*Y_p>=235)  *Y_p = 235;
	if (*Cb_p<=16)  *Cb_p = 16;
	if (*Cb_p>=240) *Cb_p = 240; 
	if (*Cr_p<=16)  *Cr_p = 16;
	if (*Cr_p>=240) *Cr_p = 240;
	return 0;
} 

//=============================================================================


inline int CLAMP_YUV(unsigned char* Y_p, unsigned char* U_p, unsigned char* V_p){

	if (*Y_p<=0)   *Y_p = 0;
	if (*Y_p>=255) *Y_p = 255;
	if (*U_p<=0)   *U_p = 0;
	if (*U_p>=255) *U_p = 255; 
	if (*V_p<=0)  *V_p = 0;
	if (*V_p>=255) *V_p = 255;
	return 0;
} 


//=============================================================================


inline int CLAMP_RGB(int* R_p, int* G_p, int* B_p){
	
	if (*R_p<=0)   *R_p = 0;
	if (*R_p>=255) *R_p = 255;
	if (*G_p<=0)  *G_p = 0;
	if (*G_p>=255) *G_p = 255; 
	if (*B_p<=0)  *B_p = 0;
	if (*B_p>=255) *B_p = 255;
	return 0;
}


//=============================================================================


int ResolveFrameDimensions(char* size, int* frame_width_pt, int* frame_height_pt)
{
	
	if ( strcmp(size,"sqcif") == 0 )
	{
		*frame_width_pt = 128;
		*frame_height_pt = 96;
		printf("Capture window size = [%d,%d]\n",*frame_width_pt, *frame_height_pt);
	}
	
	else if ( strcmp(size,"qsif") == 0 )
	{
		*frame_width_pt = 160;
		*frame_height_pt = 120;
		printf("Capture window size = [%d,%d]\n",*frame_width_pt, *frame_height_pt);
	}
	
	else if ( strcmp(size,"qcif") == 0 )
	{
		*frame_width_pt = 176;
		*frame_height_pt = 144;
		printf("Capture window size = [%d,%d]\n",*frame_width_pt, *frame_height_pt);
	}

	else if ( strcmp(size,"sif") == 0 )
	{
		*frame_width_pt = 320;
		*frame_height_pt = 240;
		printf("Capture window size = [%d,%d]\n",*frame_width_pt, *frame_height_pt);
	}

	else if ( strcmp(size,"cif") == 0 )
	{
		*frame_width_pt = 352;
		*frame_height_pt = 288;
		printf("Capture window size = [%d,%d]\n",*frame_width_pt, *frame_height_pt);
	}

	else if ( strcmp(size,"vga") == 0 )
	{
		*frame_width_pt = 640;
		*frame_height_pt = 480;
		printf("Capture window size = [%d,%d]\n",*frame_width_pt, *frame_height_pt);
	}
	
	else {
		printf(" Unrecognized frame size");
	}
		
	return 0;
}


//=============================================================================


int ThresholdFile( char *fname, BWImage **bwIm, int color){
	
	FILE* file;
	int width, height;
	int i,num_read;
	unsigned char r,g,b;    /* RGB values of each pixel */   
	float sumPixel;     
	int Bnorm = 100;  	/* Threshold to avoid noise amplification in dark regions */
	
	// Open the file
	file = fopen(fname, "r");
	if (file == NULL)
	{
		printf("ThresholdFile: failed to open file\n");
		return -1;
	}
	
	ProcessHeaders(file, &width, &height);
	CreateBWImage(bwIm, height+1, width);     
	
	//Add one line of black pixels for blob extraction step
	for (i=0; i<(*bwIm)->cols;i++){
		(*bwIm)->data[i] = 0;
	}
	
	//Read data from file
	for(i=0; i<width*height; i++){
		
		//Read one value at a time
		num_read=0;      
		num_read+= fscanf(file,"%c",&r); /*RGB values*/
		num_read+= fscanf(file,"%c",&g);
		num_read+= fscanf(file,"%c",&b);
		
		if (num_read != 3)
		{
			printf("CreateBWImageFromFile: failed to read RGB values from file : numread = %d\n",num_read);
			fclose(file);
			return -1;
		} 
		
		//Normalize RGB values to amplify the strength of the dominant channel 
		
		sumPixel = r+g+b; 
		if(sumPixel >= Bnorm){               
			r =  (255/(double)sumPixel) * r;
			g =  (255/(double)sumPixel) * g;
			b =  (255/(double)sumPixel) * b;
		}
		
		switch (color) {
			case  RED: 
				//printf("Red component of pixel %d after normalization : %d\n",i,r);
				//getchar();
				if (r>=R_MIN){
					(*bwIm)->data[i+width] = 1; //light the pixel 
					//printf("Pixel %d acceso\n",i);
				}  
				else
					(*bwIm)->data[i+width] = 0; //turn the pixel off
				break;   
				/*case  GREEN:  //green
					if (g>=G_MIN){
						(*bwIm)->data[i+width] = 1; //light the pixel
					}
				else
					(*bwIm)->data[i+width] = 0; //turn the pixel off
				break;
				case  BLUE:  //blue
					if (b>=B_MIN){
						(*bwIm)->data[i+width] = 1; //light the pixel
					} 
				else
					(*bwIm)->data[i+width] = 0; //turn the pixel off
				break;*/
			default :
				printf("Unrecognized color\n");
			    getchar();
				break;
		} 
		
	}	
	
	fclose(file);
	
	return 0;
}

//=============================================================================


int ProcessHeaders(FILE *file, int *width, int *height) {
	
  int num_read;
  char line[MAX_LINE_LEN];

  //Ignore first two lines
  fgets(line,MAX_LINE_LEN,file);
  if (line == NULL)
  {
    printf("ProcessHeaders: failed to read from file\n");
    fclose(file);
    return -1;
  }

  fgets(line,MAX_LINE_LEN,file);
  if (line == NULL)
  {
    printf("ProcessHeaders: failed to read from file\n");
    fclose(file);
    return -1;
  }

  //Read number of rows and columns
  fgets(line,MAX_LINE_LEN,file);
  if (line == NULL)
  {
    printf("ProcessHeaders: failed to read from file\n");
    fclose(file);
    return -1;
  }

  num_read = sscanf(line,"%d %d",width,height);
  if (num_read != 2)
  {
    printf("ProcessHeaders: failed to read height and width from file\n");
    fclose(file);
    return -1;
  }
	
  //Read max value line
  fgets(line,MAX_LINE_LEN,file);
  if (line == NULL)
  {
    printf("ProcessHeaders: failed to read max value\n");
    fclose(file);
    return -1;
  }
	
	return 0;
}
	
//=============================================================================


int LoadBWImageFromFile( char *fname, BWImage **bwIm){
	
	FILE* file;
	int width, height;
	int i,num_read;
	unsigned char r,g,b;    /* RGB values of each pixel */
	
	// Open the file
	file = fopen(fname, "r");
	if (file == NULL)
	{
		printf("LoadBWImage: failed to open file\n");
		return -1;
	}
	
	ProcessHeaders(file, &width, &height);
	CreateBWImage(bwIm, height, width);     
	
	//EDL no need for this because the image is already the result of a thresholding
	//Add one line of black pixels for blob extraction step
	//for (i=0; i<(*bwIm)->cols;i++){
	//	(*bwIm)->data[i] = 0;
	//}
	
	//Read data from file
	for(i=0; i<width*height; i++){
		
		//Read one value at a time
		num_read=0;      
		num_read+= fscanf(file,"%c",&r); /*RGB values*/
		num_read+= fscanf(file,"%c",&g);
		num_read+= fscanf(file,"%c",&b);
		
		if (num_read != 3)
		{
			printf("LoadBWImageFromFile: failed to read RGB values from file : numread = %d\n",num_read);
			fclose(file);
			return -1;
		} 
		
		if(r==255 && g==255 && b==255)  //white pixel;
			
			(*bwIm)->data[i+width] = 1; //light the pixel 
		
		if(r==0 && g==0 && b==0)  //black pixel;
			
			(*bwIm)->data[i+width] = 0; //light the pixel 
		
	}
	
	fclose(file);
	
	return 0;
}
	

//=============================================================================
