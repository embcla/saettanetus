//==========================================================================
//
//  Project:        guarDIAn: Vision system for mobile robots equipped with 
//					FOX board Netus G20
//
//  Module:         Distance computation from pixel indexes
//
//  Description:    A set of function to manage pixel to XY coordinates 
//					correspondence.
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


#include "globals.h"
#include "coordinates.h"
#include <math.h>
//==============================================================================


int InitPixelMap() {

	printf("Initializaing Map module ... \n");
	
//	if (!frame_height)
		frame_height=DEFAULT_VERT_RESOLUTION;
//	if (!frame_width)
		frame_width=DEFAULT_HOR_RESOLUTION;

	// Create PixelMap data structure
	
	CreatePixelMap(&pixelMap,frame_height,frame_width); 	
	
	// Fill PixelMap with pixel->coordinates corrispondence
	
	FillPixelMap(MAPFILE, pixelMap);
	
	
	return 0;
}

//==============================================================================

int EvaluateCoordinates() {

	// Prepare Xbee packet data and fill it with values stored in PixelMap
	
	packet_data = (float*)malloc(2*compList->count*sizeof(float));
	//GetCoordinates(compList,pixelMap,cam_log_file,transmit,packet_data);
	
	//GetCoordinates(compList,pixelMap,cam_log_file,0,packet_data);
	GetCoordinatesEvo(compList,pixelMap,cam_log_file,0,packet_data);

	return 0;
	
} 


//==============================================================================


int CreatePixelMap(PixelMap **pixelMap, int rows, int cols){
	 
    int i;
	*pixelMap = (PixelMap *)malloc( sizeof( PixelMap ) );
	
	(*pixelMap)->rows = rows;
	(*pixelMap)->cols = cols;
	(*pixelMap)->x_coord = (float*)malloc((rows*cols)*sizeof(float));
	(*pixelMap)->y_coord = (float*)malloc((rows*cols)*sizeof(float));
	
	for(i=0;i<(*pixelMap)->rows*(*pixelMap)->cols;i++){
		(*pixelMap)->x_coord[i] = 0;
		(*pixelMap)->y_coord[i] = 0;
	}
	return 0;
}


//==============================================================================


int FillPixelMap(char *map_file, PixelMap *pixelMap) {
	
	FILE *map;
//==============================================================================

float GetAngleCentroidBlob(float x_cent_blob, float y_cent_blob)
{
return atan2(y_cent_blob,x_cent_blob);
}

//==============================================================================
	map = fopen(map_file,"r");
	char line[MAX_LINE_LEN];
	int i;
	float indiceRiga;
	float indiceColonna;
	float x_coord;
	float y_coord;
	
	if (map == NULL)
	{
		printf("InitPixelMap: failed to open file\n");
		printf("%s\n",map_file);
	    getchar();
		return -1;
	}
	
	//Read data from file
	for(i=0; i<pixelMap->rows*pixelMap->cols; i++){
		
		//Read one line at a time
		fgets(line,MAX_LINE_LEN,map);
		
		if (line == NULL)
		{
			printf("InitPixelMap: failed to read from file\n");
			fclose(map);
			getchar();
			return -1;
		}
		
		else {
			//printf("Current line of the map file %s",line);
			// getchar();
		    // read the first value
			sscanf(line,"%f\t%f\t%f\t%f",&indiceRiga,&indiceColonna,&x_coord,&y_coord);
			//printf("%f : %f : %f : %f \n",indiceRiga,indiceColonna,x_coord,y_coord);
			pixelMap->x_coord[i] = x_coord;
			pixelMap->y_coord[i] = y_coord;
		}
	}
	
	fclose(map);
	
	return 0;
}

//==============================================================================


int GetCoordinates(ComponentsList *compList, PixelMap *pixelMap, FILE *log_file, int transmit, float *packet_data) {

	struct timeval time;
	long timestamp;
	int i,centroidIndex;
	int imageRows,imageCols;

	imageRows = pixelMap->rows;
	imageCols = pixelMap->cols;
	
	//printf("Col: %d\tRow: %d\n",imageCols,imageRows);
	ComponentsListElement*  compListElement;
	Component* comp;
	
	compListElement = compList->head;
	
	//debug
	//PrintPixelMap(pixelMap);
		
	gettimeofday( &time, NULL );
	timestamp =  ((time.tv_sec) * 1000000 + time.tv_usec);
		
	//for each component
	
	// printf("component count: %d\n",compList->count); // Andrea
	for(i=0;i<compList->count;i++)
	{
		comp = &compListElement->component;
		
		// Get centroid distance only if the blob is big enough
		
		if ( comp->pixelList->count>BLOB_MIN_SIZE) {
	
			centroidIndex = comp->centroidRow*imageCols + comp->centroidCol;
			//QUI//printf("Index: %d\n",centroidIndex);
			comp->centroidX = pixelMap->x_coord[centroidIndex];
			comp->centroidY = pixelMap->y_coord[centroidIndex];
			//printf("Cartesian [coordinates]/pixelIndex of component %d centroid :  [%f , %f]/%d  \n",i,comp->centroidX,comp->centroidY,centroidIndex);

			printf("Cartesian [coordinates]/pixelIndex of component %d centroid :  [%f , %f , %f , %f]/%d  \n",i,comp->centroidX,comp->centroidY,centroidIndex,comp->centroid_distance,comp->centroid_angle);


			
		// } 	// We log only if a significan blob has been found [Andrea]
		
		// If required, save coordinates in text file
		
			if (LOGGING){
			// Andrea 			
				LogCoordData(comp->centroidX,comp->centroidY,timestamp,compList->count );
				//fprintf(cam_log_file,"\t %f \t %f \t %ld \t %d\n",comp->centroidX,comp->centroidY,timestamp,compList->count );
		
			}		
		
			// If required, prepare data for Xbee transmission
		
			if (transmit){
				packet_data[i*2] = comp->centroidX;
				packet_data[i*2+1] = comp->centroidY;
			}
		
		}	
		compListElement = compListElement->next;
		
		//Andrea		
	}
	
	
	// Andrea
	if (compList->count>0)
		ClearComponentsList(compList);
	
	return 0;
}

//==============================================================================


int GetCoordinatesEvo(ComponentsList *compList, PixelMap *pixelMap, FILE *log_file, int transmit, float *packet_data) {
	
	struct timeval time;
	long timestamp;
	int i,centroidIndex;
	int imageRows,imageCols;

	imageRows = pixelMap->rows;
	imageCols = pixelMap->cols;
	
	//printf("Col: %d\tRow: %d\n",imageCols,imageRows);
	ComponentsListElement*  compListElement;
	Component* comp;
	
	compListElement = compList->head;
	
	//debug
	//PrintPixelMap(pixelMap);
		
	gettimeofday( &time, NULL );
	timestamp =  ((time.tv_sec) * 1000000 + time.tv_usec);
		
	//for each component
	
	// printf("component count: %d\n",compList->count); // Andrea
	for(i=0;i<compList->count;i++)
	{
		comp = &compListElement->component;
		
		// Get centroid distance only if the blob is big enough
		
		if ( comp->pixelList->count>BLOB_MIN_SIZE & comp->pixelList->count<BLOB_MAX_SIZE ) {
	
			centroidIndex = comp->centroidRow*imageCols + comp->centroidCol;
			//QUI//printf("Index: %d\n",centroidIndex);
			comp->centroidX = pixelMap->x_coord[centroidIndex];
			comp->centroidY = pixelMap->y_coord[centroidIndex];
			//printf("Cartesian [coordinates]/pixelIndex of component %d centroid :  [%f , %f]/%d  \n",i,comp->centroidX,comp->centroidY,centroidIndex);
		//	printf("component %d centroid with color %d [X,Y] = [%f ,%f] \n",i,comp->color,comp->centroidX,comp->centroidY);
		
			comp->centroid_distance=GetDistCentroidBlob(comp->centroidX,comp->centroidY);
			comp->centroid_angle=GetAngleCentroidBlob(comp->centroidX,comp->centroidY);

	
			printf("[X,%f ,Y,%f , Distance,%f , Angle,%f] \n",comp->centroidX,comp->centroidY,comp->centroid_distance,comp->centroid_angle);

			
		// } 	// We log only if a significan blob has been found [Andrea]
		
		// If required, save coordinates in text file
		
			if (LOGGING){
			// Andrea 			
				LogCoordDataEvo(comp->centroidX,comp->centroidY,timestamp,compList->count,comp->color);
				//fprintf(cam_log_file,"\t %f \t %f \t %ld \t %d\n",comp->centroidX,comp->centroidY,timestamp,compList->count );
		
			}		
		
			// If required, prepare data for Xbee transmission
		
			if (transmit){
				packet_data[i*2] = comp->centroidX;
				packet_data[i*2+1] = comp->centroidY;
			}
		
		}	
		compListElement = compListElement->next;
		
		//Andrea		
	}

	// Andrea 
	/*
	if (compList->count>0)
		ClearComponentsList(compList);
	
	return 0;*/
}

//==============================================================================


int PrintPixelMap(PixelMap *pixelMap){
	
	int i;
	
	printf("PixelMap rows : %d\n",pixelMap->rows);
	printf("PixelMap cols : %d\n",pixelMap->cols);
	printf("PixelMap coordinates : \n");
	
	for(i=0;i<pixelMap->rows*pixelMap->cols;i++){
		printf("Pixel [index]/(coordinates) = [%d]/(%f,%f)\n",i,pixelMap->x_coord[i],pixelMap->y_coord[i]);
	}
	return 0;
}


//==============================================================================

float GetDistCentroidBlob(float x_cent_blob, float y_cent_blob)
{
	if (y_cent_blob==0.0)
	{
		return x_cent_blob;
	} else 
	{
		return (sqrt(pow(x_cent_blob,2.0)+pow(y_cent_blob,2.0)));
	}
}

//==============================================================================

float GetAngleCentroidBlob(float x_cent_blob, float y_cent_blob)
{
return atan2(y_cent_blob,x_cent_blob);
}

//==============================================================================

void GetStructureWebCamBlob(WebCamBlobEstimation** wstr,int *n)
{
	ComponentsListElement*  compListElement;
	Component* comp;
	int i,j=0,valids=0;

	compListElement = compList->head;

	
	for(i=0;i<compList->count;i++)
	{
		comp = &compListElement->component;
		
		// Get centroid distance only if the blob is big enough
		
		if ( comp->pixelList->count>BLOB_MIN_SIZE & comp->pixelList->count<BLOB_MAX_SIZE ) {
	
			valids++;
					
		}	
		compListElement = compListElement->next;
	}

	if(*wstr)
		free(*wstr);
	*wstr=malloc(sizeof(WebCamBlobEstimation)*valids);

	compListElement = compList->head;
	for(i=0;i<compList->count;i++)
	{
		comp = &compListElement->component;
		
		// Get centroid distance only if the blob is big enough
		
		if ( comp->pixelList->count>BLOB_MIN_SIZE & comp->pixelList->count<BLOB_MAX_SIZE ) {
	
			(*wstr)[j].color=comp->color;
			(*wstr)[j].distance=comp->centroid_distance;
			(*wstr)[j].deviation=comp->centroid_angle;
			j++;
					
		}	
		compListElement = compListElement->next;
	}
	if(valids)
	//printf("%lf\n",wstr[0].deviation);
	*n=valids;

	//if(valids==0)
	//	ClearComponentsList(compList);
}

//==============================================================================



