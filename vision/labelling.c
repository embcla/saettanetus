//==========================================================================
//
//  Project:        FoxVision: Vision system for mobile robots equipped with 
//					FOX board Netus G20
//
//  Module:         Format conversion and simple image handling functions
//					implementations
//
//  Description:    Provides a C implementation of the labelling algorithm   
//                  referred by Chan,Cheng, Lu paper (1362-F.pdf) and for 
//					computing the blob centroids.
//
//  Author:         Enrico Di Lello
//  Author:         and many small changes by Giovanni Micheli 
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


//================
//   Includes
//================

#include "labelling.h"
//#define LABELLING_DEBUG

//================================
//    Functions implementations
//================================

//------------------------------------------------------------------------------

int CreateLabels( Labels **labels, int rows, int cols )
{
	int i;
	*labels = (Labels *)malloc( sizeof( Labels ) );
	
	(*labels)->rows = rows;
	(*labels)->cols = cols;
	(*labels)->data = (int*)malloc((rows*cols)*sizeof(int));
	
	for(i=0;i<(*labels)->rows*(*labels)->cols;i++){
		(*labels)->data[i] = 0;
	}
	
	return 0;
}

//------------------------------------------------------------------------------

void DestroyLabels( Labels *labels )
{
	free( labels->data );
	free( labels );
}

//------------------------------------------------------------------------------

int PerformLabelling(BWImage *bwIm, ComponentsList *compList){
	
	int i;
	unsigned char pixel;
	Component *compPt;        
	int labelCounter = 2;       //label counter
	int label;
	Labels *labels;             //copy of the bynary image with different labels for each connected component  
	
	#ifdef LABELLING_DEBUG	
	//debug stuff
	int ImageWidth = bwIm->cols;
	#endif
	
	label = labelCounter;
	
	CreateLabels(&labels,bwIm->rows,bwIm->cols);
		
	// scan the binary image from top left to bottom right, start from the second line because the first is a dummy black one.
	
	for(i = bwIm->cols; i < bwIm->cols*bwIm->rows; i++){
		
		pixel = bwIm->data[i];
		
		#ifdef LABELLING_DEBUG
		//printf("value of pixel %d = %d\n",i,bwIm->data[i]);
		//getchar();
		#endif
		
		// break the flow only when a new white pixel is encountered
		if (pixel == 1){
			
			/* STEP 1: 
			 if the current pixel is not labelled and its north neighbour is black,
			 then it belongs to an external contour of a newly encountered
			 connected component */
			
			if ( (labels->data[i] == 0) && (bwIm->data[i-bwIm->cols] == 0) ) {
				
				label = labelCounter;
				
				//Add empty component to the list 
				CreateComponent(&compPt,label); 
				PushComponent(compList,*compPt);
				
				#ifdef LABELLING_DEBUG
				printf("Step 1 : current pixel  = %d\n",i);       
				printf("Created new component with label: %d\n",label);
				getchar();
				#endif
				
				//Add the pixel to the respective connected component
				AddPixelToComponent(compList,label,i);
				
				#ifdef LABELLING_DEBUG
				printf("Added the pixel %d to the component with label %d\n",i,label);
				PrintComponentsList(compList);
				getchar();
				#endif
				
				//label the pixel
				labels->data[i] = label;
				#ifdef LABELLING_DEBUG
				printf("Labelled pixel [%d,%d] with label %d\n", i/ImageWidth,i%ImageWidth,label);  
				PrintLabels(labels);
				getchar(); 
				#endif
				
				//Follow the freshly encountered external contour,  
				#ifdef LABELLING_DEBUG
				printf("Starting ContourTracing\n");
				getchar(); 
				#endif
				ContourTracing(bwIm,labels,compList,i,1,label);
				
				//Increment connected components counter 
				labelCounter++;
				
			}  
			else {
				
				/* STEP 2: 
				 if the south neighbour of the pixel is unmarked and black,
				 then it belongs to an internal contour*/
				
				if ( ( bwIm->data[i+bwIm->cols] == 0) && (labels->data[i+bwIm->cols] != MARK ) ){
					#ifdef LABELLING_DEBUG
					printf("Step 2: current pixel = %d\n",i);
					getchar();
					#endif
					// if the current pixel is already labelled, then it belongs to an external contour, too.              
					if ( ( labels->data[i] > 0 ) ) {
						label = labels->data[i];
					}
					// if this is not the case, the its west neighbour has to be labelled, so label the current pixel
					// with the same label
					
					else {
						label = labels->data[i-1];  
						labels->data[i] = label;
						
						//Add the pixel to the respective connected component
						AddPixelToComponent(compList,label,i);
						#ifdef LABELLING_DEBUG
						printf("Step 2: added the pixel %d to the component with label %d\n",i,label);
						PrintComponentsList(compList);
						getchar();
						#endif   
						
					}
					// in both cases trace the internal contour
					ContourTracing(bwIm,labels,compList,i,2,label);  
					//printf("label = %d\n",label);       
				}  
				
				/* STEP 3 : if the pixel was not processed during steps 1 and 2, it doesn't belong to a contour,
				 so just label it with is west neighbour label*/
				else {
					if (labels->data[i] == 0){
						label = labels->data[i-1];
						labels->data[i] = label;
						
						//Add the pixel to the respective connected component
						AddPixelToComponent(compList,label,i);
						#ifdef LABELLING_DEBUG
						printf("Step 3: Added the pixel %d to the component with label %d\n",i,label);
						PrintComponentsList(compList);
						getchar(); 
						#endif 
					}
					else {
						#ifdef LABELLING_DEBUG
						printf("Pixel %d ignored because already labelled with label %d\n",i,label);
						PrintLabels(labels);
						getchar();
						#endif     
						// ignore the pixel
						continue;
					}
				}  
			}
		}
	}
	
	#ifdef LABELLING_DEBUG
	PrintLabels(labels);
	PrintComponentsList(compList);
	getchar();
	#endif
	
	DestroyLabels(labels);
	return 0;
}

//------------------------------------------------------------------------------

int PerformLabellingColor(BWImage *bwIm, ComponentsList *compList, int maxColor){
	
	int i;
	unsigned char pixel;
	Component *compPt;        
	int labelCounter = 2;       //label counter
	int label;
	Labels *labels;             //copy of the color image with different labels for each connected component  
	
	#ifdef LABELLING_DEBUG	
	//debug stuff
	int ImageWidth = bwIm->cols;
	#endif
	
	label = labelCounter;
	
	CreateLabels(&labels,bwIm->rows,bwIm->cols);
		
	// scan the color image from top left to bottom right, start from the second line because the first is a dummy black one.
	
	for(i = bwIm->cols; i < bwIm->cols*bwIm->rows; i++){
		
		//memorizzo il codice colore del pixel corrispondente
		pixel = bwIm->data[i];
		
		#ifdef LABELLING_DEBUG
		//printf("value of pixel %d = %d\n",i,bwIm->data[i]);
		//getchar();
		#endif
		

		// break the flow only when a new color pixel is encountered
		//if (pixel == 1 || pixel == 2 || pixel == 3){
		if (pixel > 0 && pixel < maxColor+1){
			
			/* STEP 1: 
			 if the current pixel is not labelled and its north neighbour is black,
			 then it belongs to an external contour of a newly encountered
			 connected component */
			
			if ( (labels->data[i] == 0) && (bwIm->data[i-bwIm->cols] == 0) ) {
				
				label = labelCounter;
								
				//Add empty component to the list 
				CreateComponent(&compPt,label); 

				//segno il colore della componente
				compPt->color = pixel;			
	
				PushComponent(compList,*compPt);
				
				#ifdef LABELLING_DEBUG
				printf("Step 1 : current pixel  = %d\n",i);       
				printf("Created new component with label: %d\n",label);
				getchar();
				#endif
				
				//Add the pixel to the respective connected component
				AddPixelToComponent(compList,label,i);
				
				#ifdef LABELLING_DEBUG
				printf("Added the pixel %d to the component with label %d\n",i,label);
				PrintComponentsList(compList);
				getchar();
				#endif
				
				//label the pixel
				labels->data[i] = label;
				#ifdef LABELLING_DEBUG
				printf("Labelled pixel [%d,%d] with label %d\n", i/ImageWidth,i%ImageWidth,label);  
				PrintLabels(labels);
				getchar(); 
				#endif
				
				//Follow the freshly encountered external contour,  
				#ifdef LABELLING_DEBUG
				printf("Starting ContourTracing\n");
				getchar(); 
				#endif
				ContourTracingColor(bwIm,labels,compList,i,1,label,pixel);
				
				//Increment connected components counter 
				labelCounter++;
				
			}  
			else {
				
				/* STEP 2: 
				 if the south neighbour of the pixel is unmarked and black,
				 then it belongs to an internal contour*/
				
				if ( ( bwIm->data[i+bwIm->cols] == 0) && (labels->data[i+bwIm->cols] != MARK ) ){
					#ifdef LABELLING_DEBUG
					printf("Step 2: current pixel = %d\n",i);
					getchar();
					#endif
					// if the current pixel is already labelled, then it belongs to an external contour, too.              
					if ( ( labels->data[i] > 0 ) ) {
						label = labels->data[i];
					}
					// if this is not the case, the its west neighbour has to be labelled, so label the current pixel
					// with the same label
					
					else {
						label = labels->data[i-1];  
						labels->data[i] = label;
						
						//Add the pixel to the respective connected component
						AddPixelToComponent(compList,label,i);
						#ifdef LABELLING_DEBUG
						printf("Step 2: added the pixel %d to the component with label %d\n",i,label);
						PrintComponentsList(compList);
						getchar();
						#endif   
						
					}
					// in both cases trace the internal contour
					ContourTracingColor(bwIm,labels,compList,i,2,label,pixel);  
					//printf("label = %d\n",label);       
				}  
				
				/* STEP 3 : if the pixel was not processed during steps 1 and 2, it doesn't belong to a contour,
				 so just label it with is west neighbour label*/
				else {
					if (labels->data[i] == 0){
						label = labels->data[i-1];
						labels->data[i] = label;
						
						//Add the pixel to the respective connected component
						AddPixelToComponent(compList,label,i);
						#ifdef LABELLING_DEBUG
						printf("Step 3: Added the pixel %d to the component with label %d\n",i,label);
						PrintComponentsList(compList);
						getchar(); 
						#endif 
					}
					else {
						#ifdef LABELLING_DEBUG
						printf("Pixel %d ignored because already labelled with label %d\n",i,label);
						PrintLabels(labels);
						getchar();
						#endif     
						// ignore the pixel
						continue;
					}
				}  
			}
		}
	}
	
	#ifdef LABELLING_DEBUG
	PrintLabels(labels);
	PrintComponentsList(compList);
	getchar();
	#endif
	
	DestroyLabels(labels);
	return 0;
}

//------------------------------------------------------------------------------

int ContourTracing(BWImage *bwIm, Labels *labels, ComponentsList *compList, int startingPixel, int contourType, int label){

 int next, secondPixel, previousPixel, lastPixel, secondNext;
 
 #ifdef LABELLING_DEBUG
 int ImageWidth = bwIm->cols;
 #endif	
 
 #ifdef LABELLING_DEBUG
 printf("ContourTracing :  launching tracer from pixel [%d,%d],(%d) \n",startingPixel/ImageWidth,startingPixel%ImageWidth,startingPixel);
 getchar(); 
 #endif 
       
 // Find the next pixel in the contour
 next = Tracer(bwIm,labels,startingPixel,startingPixel,contourType,label);

 //Add the pixel to the respective connected component
 AddPixelToComponent(compList,label,next);
 
 #ifdef LABELLING_DEBUG
 printf("Added the pixel %d to the component with label %d\n",next,label);
 PrintComponentsList(compList);
 getchar();
 #endif
 
 secondPixel = next;
 secondNext = -1;

 // the pixel after the first are not starting point of any kind of contour
 contourType = 0;

 // if the connected component is composed of only one pixel, return
 if (next == startingPixel) {
 #ifdef LABELLING_DEBUG 
 printf("The connected component is composed of only one pixel\n");
 getchar();
 return 0;
 #endif 
 }
 else {

     previousPixel = startingPixel;
     lastPixel = next;

     //follow the contour until the last two points of the contour are the first two ones
     
    do  {

       #ifdef LABELLING_DEBUG
       printf("ContourTracing :  launching tracer from pixel [%d,%d](%d) \n",lastPixel/ImageWidth,lastPixel%ImageWidth,lastPixel);
       getchar(); 
       #endif 
       
       next = Tracer(bwIm,labels,previousPixel,lastPixel,contourType,label);
      
       #ifdef LABELLING_DEBUG
       printf("ContourTracing :  next pixel of the contour = [%d,%d](%d) \n",next/ImageWidth,next/ImageWidth,next );
       getchar(); 
       #endif 
    
       //Add the pixel to the respective connected component
       AddPixelToComponent(compList,label,next); 

       #ifdef LABELLING_DEBUG
       printf("ContourTracing: Added the pixel [%d,%d](%d) to the component with label %d\n",next/ImageWidth,next/ImageWidth,next,label);
       PrintComponentsList(compList);
       getchar();
       #endif
		
	   #ifdef LABELLING_DEBUG
		printf("next = %d\n", next);
		printf("startingPixel = %d\n", startingPixel);
		printf("secondNext = %d\n",secondNext);
		printf("secondPixel = %d\n",secondPixel);
	   #endif 
      
	   if	( (next == secondPixel ) && (secondNext == startingPixel) ){
	  	   break;
	   }
       
       #ifdef LABELLING_DEBUG
       printf("ContourTracing :  launching tracer from pixel [%d,%d],(%d) \n",next/ImageWidth,next%ImageWidth,next);
       getchar(); 
       #endif 
       
       secondNext = Tracer(bwIm,labels,lastPixel,next,contourType,label);
       
	   #ifdef LABELLING_DEBUG
		printf("next = %d\n", next);
		printf("startingPixel = %d\n", startingPixel);
		printf("secondNext = %d\n",secondNext);
		printf("secondPixel = %d\n",secondPixel);
	   #endif  
       
       #ifdef LABELLING_DEBUG
       printf("ContourTracing :  second next pixel of the contour = [%d,%d](%d) \n",secondNext/ImageWidth,secondNext/ImageWidth,secondNext);
      //Add the pixel to the respective connected component
       AddPixelToComponent(compList,label,secondNext); 
       #endif 

       #ifdef LABELLING_DEBUG
       printf("ContourTracing: Added the pixel [%d,%d](%d) to the component with label %d\n",secondNext/ImageWidth,secondNext/   ImageWidth,secondNext,label);
       PrintComponentsList(compList);
       getchar();
       #endif

	   if	( (next == startingPixel ) && (secondNext == secondPixel) ){
	    break;
	   }
		
       previousPixel = next;
       lastPixel = secondNext;
       
	 

    
	}		
	while(1);		
    //while ( (next != startingPixel ) && (secondNext != secondPixel));
	
}
 #ifdef LABELLING_DEBUG
 printf("ContourTracing terminated\n");
 PrintComponentsList(compList);
 getchar();
 #endif


return 0;
}


//------------------------------------------------------------------------------


int ContourTracingColor(BWImage *bwIm, Labels *labels, ComponentsList *compList, int startingPixel, int contourType, int label, int pixelColor){

 int next, secondPixel, previousPixel, lastPixel, secondNext;
 
 #ifdef LABELLING_DEBUG
 int ImageWidth = bwIm->cols;
 #endif	
 
 #ifdef LABELLING_DEBUG
 printf("ContourTracing :  launching tracer from pixel [%d,%d],(%d) \n",startingPixel/ImageWidth,startingPixel%ImageWidth,startingPixel);
 getchar(); 
 #endif 
       
 // Find the next pixel in the contour
 next = TracerColor(bwIm,labels,startingPixel,startingPixel,contourType,label,pixelColor);

 //Add the pixel to the respective connected component
 AddPixelToComponent(compList,label,next);
 
 #ifdef LABELLING_DEBUG
 printf("Added the pixel %d to the component with label %d\n",next,label);
 PrintComponentsList(compList);
 getchar();
 #endif
 
 secondPixel = next;
 secondNext = -1;

 // the pixel after the first are not starting point of any kind of contour
 contourType = 0;

 // if the connected component is composed of only one pixel, return
 if (next == startingPixel) {
 #ifdef LABELLING_DEBUG 
 printf("The connected component is composed of only one pixel\n");
 getchar();
 return 0;
 #endif 
 }
 else {

     previousPixel = startingPixel;
     lastPixel = next;

     //follow the contour until the last two points of the contour are the first two ones
     
    do  {

       #ifdef LABELLING_DEBUG
       printf("ContourTracing :  launching tracer from pixel [%d,%d](%d) \n",lastPixel/ImageWidth,lastPixel%ImageWidth,lastPixel);
       getchar(); 
       #endif 
       
       next = TracerColor(bwIm,labels,previousPixel,lastPixel,contourType,label,pixelColor);
      
       #ifdef LABELLING_DEBUG
       printf("ContourTracing :  next pixel of the contour = [%d,%d](%d) \n",next/ImageWidth,next/ImageWidth,next );
       getchar(); 
       #endif 
    
       //Add the pixel to the respective connected component
       AddPixelToComponent(compList,label,next); 

       #ifdef LABELLING_DEBUG
       printf("ContourTracing: Added the pixel [%d,%d](%d) to the component with label %d\n",next/ImageWidth,next/ImageWidth,next,label);
       PrintComponentsList(compList);
       getchar();
       #endif
		
	   #ifdef LABELLING_DEBUG
		printf("next = %d\n", next);
		printf("startingPixel = %d\n", startingPixel);
		printf("secondNext = %d\n",secondNext);
		printf("secondPixel = %d\n",secondPixel);
	   #endif 
      
	   if	( (next == secondPixel ) && (secondNext == startingPixel) ){
	  	   break;
	   }
       
       #ifdef LABELLING_DEBUG
       printf("ContourTracing :  launching tracer from pixel [%d,%d],(%d) \n",next/ImageWidth,next%ImageWidth,next);
       getchar(); 
       #endif 
       
       secondNext = TracerColor(bwIm,labels,lastPixel,next,contourType,label,pixelColor);
       
	   #ifdef LABELLING_DEBUG
		printf("next = %d\n", next);
		printf("startingPixel = %d\n", startingPixel);
		printf("secondNext = %d\n",secondNext);
		printf("secondPixel = %d\n",secondPixel);
	   #endif  
       
       #ifdef LABELLING_DEBUG
       printf("ContourTracing :  second next pixel of the contour = [%d,%d](%d) \n",secondNext/ImageWidth,secondNext/ImageWidth,secondNext);
      //Add the pixel to the respective connected component
       AddPixelToComponent(compList,label,secondNext); 
       #endif 

       #ifdef LABELLING_DEBUG
       printf("ContourTracing: Added the pixel [%d,%d](%d) to the component with label %d\n",secondNext/ImageWidth,secondNext/   ImageWidth,secondNext,label);
       PrintComponentsList(compList);
       getchar();
       #endif

	   if	( (next == startingPixel ) && (secondNext == secondPixel) ){
	    break;
	   }
		
       previousPixel = next;
       lastPixel = secondNext;
       
	 

    
	}		
	while(1);		
    //while ( (next != startingPixel ) && (secondNext != secondPixel));
	
}
 #ifdef LABELLING_DEBUG
 printf("ContourTracing terminated\n");
 PrintComponentsList(compList);
 getchar();
 #endif


return 0;
}

//------------------------------------------------------------------------------


int Tracer(BWImage *bwIm, Labels *labels , int previousPixel, int currentPixel, int contourType , int label){

int next;
int previousDirection;
int firstSearchDirection;
int searchDirection;
int nextFound = 0;
int neighbour;
int ImageWidth = bwIm->cols;	
int ImageHeight = bwIm->rows;
	
 //Decide first search direction

 switch (contourType) {

 case 1:  
    firstSearchDirection = 7;
    break;
 case 2:
    firstSearchDirection = 3;
    break;
 default:  
    previousDirection =  GetNeighbourIndex(previousPixel,currentPixel,ImageWidth
										   ); //Da eliminare
    firstSearchDirection = (previousDirection+2)%8;
    break;
 }

  //Look for the next white pixel
  searchDirection = firstSearchDirection;
  #ifdef LABELLING_DEBUG
  printf("Current pixel = [%d,%d](%d)\n",currentPixel/ImageWidth,currentPixel%ImageWidth,currentPixel);
  printf("Previous pixel = [%d,%d](%d)\n",previousPixel/ImageWidth,previousPixel%ImageWidth,previousPixel);
  printf("Previous pixel direction = %d\n",previousDirection);
  printf("First search Direction = %d\n",searchDirection);
  #endif 

  while ( ( ((searchDirection+1)%8) != firstSearchDirection ) && (nextFound <= 0) ){
    
     neighbour = GetNeighbourCoord(currentPixel, searchDirection, ImageWidth, ImageHeight );    //da eliminare
     #ifdef LABELLING_DEBUG
     printf("Pixel in direction %d = [%d,%d]\n",searchDirection,neighbour/ImageWidth,neighbour%ImageWidth);
     #endif  

     // if the neighbour is still inside the image
     if (neighbour != -1){
        nextFound = bwIm->data[neighbour];
     #ifdef LABELLING_DEBUG
     printf("Value found in direction %d = %d\n",searchDirection,nextFound);
     #endif  

     } 
     else{ 
        nextFound = -1;
     #ifdef LABELLING_DEBUG
     printf("Neigbour outside the image\n");
     #endif
  
     }  
    
     // if the found pixel is black, mark it as explored
     if (nextFound == 0){
        labels->data[neighbour] = MARK;
        #ifdef LABELLING_DEBUG
        printf("Pixel [%d,%d](%d) marked as explored\n",neighbour/ImageWidth,neighbour%ImageWidth,neighbour);
        PrintLabels(labels);
        getchar();
        #endif
     }
     
     searchDirection = (searchDirection + 1)%8;    
  }


if (nextFound == 0){
   next = currentPixel;
}
else{
   next = neighbour;
   //label the pixel
   labels->data[next] = label;
   #ifdef LABELLING_DEBUG
   printf("tracer: labelled pixel [%d,%d](%d) with label %d\n", next/ImageWidth,next%ImageWidth,next,label);  
   PrintLabels(labels);
   getchar(); 
   #endif
}

return next;
}

//------------------------------------------------------------------------------

int TracerColor(BWImage *bwIm, Labels *labels , int previousPixel, int currentPixel, int contourType , int label, int pixelColor){

int next;
int previousDirection;
int firstSearchDirection;
int searchDirection;
int nextFound = 0;
int neighbour;
int ImageWidth = bwIm->cols;	
int ImageHeight = bwIm->rows;
	
 //Decide first search direction

 switch (contourType) {

 case 1:  
    firstSearchDirection = 7;
    break;
 case 2:
    firstSearchDirection = 3;
    break;
 default:  
    previousDirection =  GetNeighbourIndex(previousPixel,currentPixel,ImageWidth
										   ); //Da eliminare
    firstSearchDirection = (previousDirection+2)%8;
    break;
 }

  //Look for the next white pixel
  searchDirection = firstSearchDirection;
  #ifdef LABELLING_DEBUG
  printf("Current pixel = [%d,%d](%d)\n",currentPixel/ImageWidth,currentPixel%ImageWidth,currentPixel);
  printf("Previous pixel = [%d,%d](%d)\n",previousPixel/ImageWidth,previousPixel%ImageWidth,previousPixel);
  printf("Previous pixel direction = %d\n",previousDirection);
  printf("First search Direction = %d\n",searchDirection);
  #endif 

  while ( ( ((searchDirection+1)%8) != firstSearchDirection ) && (nextFound <= 0) ){
    
     neighbour = GetNeighbourCoord(currentPixel, searchDirection, ImageWidth, ImageHeight);    //da eliminare
     #ifdef LABELLING_DEBUG
     printf("Pixel in direction %d = [%d,%d]\n",searchDirection,neighbour/ImageWidth,neighbour%ImageWidth);
     #endif  

     // if the neighbour is still inside the image
     if (neighbour != -1){
        nextFound = bwIm->data[neighbour];
	//se il pixel è di un colore diverso da quello esplorato
	/*if ( nextFound != color) {
		nextFound = -1;	
	}*/
     #ifdef LABELLING_DEBUG
     printf("Value found in direction %d = %d\n",searchDirection,nextFound);
     #endif  

     } 
     else{ 
        nextFound = -1;
     #ifdef LABELLING_DEBUG
     printf("Neigbour outside the image\n");
     #endif
  
     }  
    
     // if the found pixel is black, mark it as explored
     if (nextFound == 0 || (nextFound != pixelColor && nextFound>0)){
	nextFound = 0;
        labels->data[neighbour] = MARK;
        #ifdef LABELLING_DEBUG
        printf("Pixel [%d,%d](%d) marked as explored\n",neighbour/ImageWidth,neighbour%ImageWidth,neighbour);
        PrintLabels(labels);
        getchar();
        #endif
     }
     
     /*if ( nextFound != color) {
		nextFound = 0;	
     }*/
     
     searchDirection = (searchDirection + 1)%8;    
  }

//printf("color: %d\n",color);

//se ci sono solo pixel neri intorno o di colore diverso o già esplorati
if (nextFound == 0){
   next = currentPixel;
}
else{
   next = neighbour;
   //label the pixel
   labels->data[next] = label;
   #ifdef LABELLING_DEBUG
   printf("tracer: labelled pixel [%d,%d](%d) with label %d\n", next/ImageWidth,next%ImageWidth,next,label);  
   PrintLabels(labels);
   getchar(); 
   #endif
}

return next;
}

//------------------------------------------------------------------------------


inline int GetNeighbourCoord(pixel,searchPosition, imageWidth, imageHeight){

int neighbour;

 switch (searchPosition){

    case 0:
        neighbour =  pixel+1 ;    //E
    break;

    case 1:
        neighbour = pixel+imageWidth +1  ;  // SE
    break;

    case 2:
        neighbour = pixel+imageWidth  ;     // S
    break;

    case 3:
        neighbour = pixel+imageWidth - 1  ; // SO
    break;

    case 4:
        neighbour = pixel-1 ;    // O
    break;
 
    case 5:
        neighbour =  pixel-imageWidth -1;  // NO
    break;

    case 6:
        neighbour = pixel-imageWidth  ;    // N
    break;

    case 7:
        neighbour = pixel-imageWidth + 1 ;  // NE
    break;
    default: 
    break;
 } 
    // Be sure that the neighbour is still inside the image
    if( (neighbour<0) || (neighbour > ( imageWidth*(imageHeight)-1) ) ){
       neighbour = -1;
       //printf("Neighbour out of image\n");
       //getchar();
    }
return neighbour;
}


//------------------------------------------------------------------------------

inline int GetNeighbourIndex(int previousPoint, int point, int imageWidth){

int index;

if  (previousPoint == point+1)  index = 0; //E

if  (previousPoint == point+imageWidth +1) index = 1; //SE

if  (previousPoint == point+imageWidth)   index = 2; //S

if  (previousPoint == point+imageWidth -1) index = 3; //SO

if  (previousPoint == point-1) index = 4; //O

if  (previousPoint == point-imageWidth -1) index = 5; //NO

if  (previousPoint == point-imageWidth) index = 6; // N
  
if  (previousPoint == point-imageWidth +1) index = 7; // NE
 
return index;

}

//------------------------------------------------------------------------------

int PrintLabels(Labels *labels){
	
int i;

 for(i=0;i<labels->cols*labels->rows;i++){
   printf(" %3d ",labels->data[i]);
   if ((i+1)%labels->cols == 0){
      printf("\n"); 
   } 
  }
  printf("\n");
 return 0;
}

//------------------------------------------------------------------------------

int ComputeCentroids( ComponentsList *compList, BWImage *bwIm ){
	
	int i,j;
	ComponentsListElement*  compListElement;
	IntListElement* intListElement;
	Component* comp;
	int pixelRow,pixelCol;
	int SumRow=0;
	int SumCol = 0;
	
	int ImageWidth = bwIm->cols;
	compListElement = compList->head;
	
	//debug
	//PrintComponentsList(compList);
	
	//for each component
	for(i=0;i<compList->count;i++)
	{
		comp = &compListElement->component;
		
		intListElement = comp->pixelList->head;
		
		// Compute the centroid only if the blob is bigger than a min value
		
		if (comp->pixelList->count>BLOB_MIN_SIZE) {
			
			for(j=0;j<comp->pixelList->count;j++){
				
				// dummy line compensation
				pixelRow = intListElement->value/ImageWidth - 1;
				pixelCol = intListElement->value%ImageWidth;
				SumRow = SumRow + pixelRow;
				SumCol = SumCol + pixelCol; 
				
				intListElement = intListElement->next;
			}
			
			if (SumRow > 0){
				comp->centroidRow = SumRow/comp->pixelList->count;
			}
			else{
				comp->centroidRow = 0;
			}
			if (SumCol > 0){ 
				comp->centroidCol = SumCol/comp->pixelList->count;
			}
			else{
				comp->centroidCol = 0;
			}
			
			printf("Component %d : [Centroid Row, Centroid Col] = : [%d,%d]\n",i,comp->centroidRow,comp->centroidCol);
			
			//Draw a red X in the binaryImage in corrispondence of the centrois 
			
			bwIm->data[comp->centroidRow*ImageWidth + comp->centroidCol] = RED ;
			bwIm->data[comp->centroidRow*ImageWidth + comp->centroidCol - 1] = RED ;
			bwIm->data[comp->centroidRow*ImageWidth + comp->centroidCol + 1] = RED ;
			bwIm->data[comp->centroidRow*ImageWidth + comp->centroidCol - 2] = RED ;
			bwIm->data[comp->centroidRow*ImageWidth + comp->centroidCol + 2] = RED ;
			
			bwIm->data[comp->centroidRow*ImageWidth + comp->centroidCol + ImageWidth] = RED ;
			bwIm->data[comp->centroidRow*ImageWidth + comp->centroidCol - ImageWidth] = RED ;
			
			bwIm->data[comp->centroidRow*ImageWidth + comp->centroidCol + 2*ImageWidth] = RED ;
			bwIm->data[comp->centroidRow*ImageWidth + comp->centroidCol - 2*ImageWidth] = RED ;
			
		}
		
		else{
			
			//printf("Component with label %d ignored because too small\n", i);
			//printf(" Pixel count of the ignored component = %d\n",comp->pixelList->count);
		}
		 
		compListElement = compListElement->next;
		SumRow = 0;
		SumCol = 0;
		
	}
	return 0;
}


//------------------------------------------------------------------------------

int ComputeCentroidsColor( ComponentsList *compList, BWImage *bwIm ){
	int i,j;
	ComponentsListElement*  compListElement;
	IntListElement* intListElement;
	Component* comp;
	int pixelRow,pixelCol;
	int SumRow=0;
	int SumCol = 0;
	
	int ImageWidth = bwIm->cols;
	compListElement = compList->head;
	
	//debug
	//PrintComponentsList(compList);
	
	//for each component
	for(i=0;i<compList->count;i++)
	{
		comp = &compListElement->component;
		
		intListElement = comp->pixelList->head;
		
		// Compute the centroid only if the blob is bigger than a min value
		// and smaller than a max value
		if (comp->pixelList->count>BLOB_MIN_SIZE & comp->pixelList->count<BLOB_MAX_SIZE) {
			
			for(j=0;j<comp->pixelList->count;j++){
				
				// dummy line compensation
				//GIO//pixelRow = intListElement->value/ImageWidth - 1;
				pixelRow = intListElement->value/ImageWidth;
				pixelCol = intListElement->value%ImageWidth;
				SumRow = SumRow + pixelRow;
				SumCol = SumCol + pixelCol; 
				
				intListElement = intListElement->next;
			}
			
			if (SumRow > 0){
				comp->centroidRow = SumRow/comp->pixelList->count;
			}
			else{
				comp->centroidRow = 0;
			}
			if (SumCol > 0){ 
				comp->centroidCol = SumCol/comp->pixelList->count;
			}
			else{
				comp->centroidCol = 0;
			}
			
			
			printf("Component %d : color %d [Centroid Row, Centroid Col] = : [%d,%d]\n",i,comp->color,comp->centroidRow,comp->centroidCol);
			
			
			//Mark a code of X in the binaryImage in corrispondence of the centrois 
			
			bwIm->data[comp->centroidRow*ImageWidth + comp->centroidCol] = CROSS ;
			bwIm->data[comp->centroidRow*ImageWidth + comp->centroidCol - 1] = CROSS ;
			bwIm->data[comp->centroidRow*ImageWidth + comp->centroidCol + 1] = CROSS ;
			bwIm->data[comp->centroidRow*ImageWidth + comp->centroidCol - 2] = CROSS ;
			bwIm->data[comp->centroidRow*ImageWidth + comp->centroidCol + 2] = CROSS ;
			
			bwIm->data[comp->centroidRow*ImageWidth + comp->centroidCol + ImageWidth] = CROSS ;
			bwIm->data[comp->centroidRow*ImageWidth + comp->centroidCol - ImageWidth] = CROSS ;
			
			bwIm->data[comp->centroidRow*ImageWidth + comp->centroidCol + 2*ImageWidth] = CROSS ;
			bwIm->data[comp->centroidRow*ImageWidth + comp->centroidCol - 2*ImageWidth] = CROSS ;
			
		}
		
		else{
			
			//printf("Component with label %d ignored because too small\n", i);
			//printf(" Pixel count of the ignored component = %d\n",comp->pixelList->count);
		}
		 
		compListElement = compListElement->next;
		SumRow = 0;
		SumCol = 0;
		
	}
	return 0;
}



