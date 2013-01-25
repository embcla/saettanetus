//==========================================================================
//
//  Project:        guarDIAn: Vision system for mobile robots equipped with 
//					FOX board Netus G20
//
//  Module:         Distance computation from pixel indexes
//
//  Description:    A set of function for LOGGING experiment data
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
#include "log.h"

//===============================================================================


int InitLogs(){
	
	// If LOGGING has been activated trough command-line parameter, enable it
	
	if(LOGGING){
		
		//printf("%s\n\n",log_fname);
		snprintf(cam_log_fname,sizeof(cam_log_fname),"cam_%s",log_fname);
		snprintf(time_log_fname,sizeof(time_log_fname),"time_%s",log_fname);
		//printf("%s\n\n",cam_log_fname);
		//printf("%s\n\n",time_log_fname);

		if (fopen(cam_log_fname,"wr") != NULL && fopen(time_log_fname,"wr") != NULL) {
			printf("LOGGING webcam experiment data in file : %s ... \n", cam_log_fname);
			cam_log_file = fopen(cam_log_fname,"wr");
			printf("LOGGING experiment step duration times  data in file : %s ... \n", time_log_fname);
			time_log_file = fopen(time_log_fname,"wr");
		}
		
		else{
			printf("InitLog: cannot create log files.. Out of memory? \n");
			return -1;
		}
	}
	
	return 0;
}

//===============================================================================


int GetLogFileName(char *log_fname){
  
int i;
char *date;

	time_t timer;
	time(&timer);
	date = ctime(&timer); 
        //date has the following format:
	    //"%.3s %.3s%3d %.2d:%.2d:%.2d %d\n"
        //printf("Currente dat %s ", date);

        for (i=0;i<=2;i++) {
 		log_fname[i] = date[i]; 
 		//printf("carattere %d : %c \n", i,log_fname[i]);
		//getchar();         
	}

        log_fname[i] = '_';

        for (i=4;i<=6;i++) {
 		log_fname[i] = date[i]; 
 		//printf("carattere %d : %c \n", i,log_fname[i]);
		//getchar();         
	}

        log_fname[i] = '_';

        for (i=8;i<=9;i++) {
 		log_fname[i] = date[i]; 
 		//printf("carattere %d : %c \n", i,log_fname[i]);
		//getchar();         
	}

        log_fname[i] = '_';

        for (i=11;i<=12;i++) {
 		log_fname[i] = date[i]; 
 		//printf("carattere %d : %c \n", i,log_fname[i]);        
	}

        log_fname[i] = '_';

        for (i=14;i<=15;i++) {
 		log_fname[i] = date[i]; 
 		//printf("carattere %d : %c \n", i,log_fname[i]);         
	}
        log_fname[i]='.';
        i++;
	log_fname[i]='t';
	i++;
	log_fname[i]='x';
	i++;
	log_fname[i]='t';
	i++;
	log_fname[i] = '\0';

return 0;
		
}


//===============================================================================


int LogCoordData(float centroidX, float centroidY, long timestamp, int blob_num){
	
	if (LOGGING) {
		if (fprintf(cam_log_file,"\t %f \t %f \t %ld \t %d \n",centroidX,centroidY,timestamp,blob_num ) <= 0) {
			
			printf("Error when LOGGING coordinates values \n");
			return -1;
			
		}
	}
	return 0;

}


//===============================================================================


int LogCoordDataEvo(float centroidX, float centroidY, long timestamp, int blob_num, int blob_color){
	
	if (LOGGING) {
		if (fprintf(cam_log_file,"\t %f \t %f \t %ld \t %d \t %d \n",centroidX,centroidY,timestamp,blob_num, blob_color ) <= 0) {
			
			printf("Error when LOGGING coordinates values \n");
			return -1;
			
		}
	}
	return 0;

}


//===============================================================================



int LogTimeData() {
	
	if (LOGGING) {
		
	    GetElapsedTime("Total iteration time",&vision_start_time, &vision_end_time, &total_time);
                //printf(time_log_file,"\t %ld \t %ld \t %ld \t %ld\n", acquisition_time, processing_time, dist_meas_time, total_time );
		if ( fprintf(time_log_file,"\t %ld \t %ld \t %ld \t %ld\n", acquisition_time, processing_time, dist_meas_time, total_time ) <= 0) {
			
			printf("Error when LOGGING elapsed time values \n");
			return -1;
			
		}
	}		
	return 0;
	
}


//===============================================================================


int GetElapsedTime(char *step_name, struct timeval *start_time_pt, struct timeval *end_time_pt, long *elapsed_utime)
{
	
	//long elapsed_utime;    /* elapsed time in microseconds */
	//long elapsed_mtime;    /* elapsed time in milliseconds */
	long elapsed_seconds;	 /* diff between seconds counter */
	long elapsed_useconds;	 /* diff between microseconds counter */
	
	elapsed_seconds  = end_time_pt->tv_sec  - start_time_pt->tv_sec;
	elapsed_useconds = end_time_pt->tv_usec - start_time_pt->tv_usec;
	
	*elapsed_utime = (elapsed_seconds) * 1000000 + elapsed_useconds;
	//elapsed_mtime = ((elapsed_seconds) * 1000 + elapsed_useconds/1000.0) + 0.5;
	
	//printf("%s : Elapsed time = %ld microseconds\n", step_name, *elapsed_utime);
	//printf("%s : Elapsed time = %ld milliseconds\n", step_name, elapsed_mtime);
	
	return 0;	
}


//===============================================================================

int CloseLogs(){
	
	if (LOGGING) {
		
		/*printf("Closing camera log file ...\n");
		if ( fclose(cam_log_file) != 0 ){
			printf("Error when trying to close camera log file");
			return -1;
		}*/
		
		printf("Closing time log file ...\n");
		
		if ( fclose(time_log_file) != 0 ) {
			printf("Error when trying to time log file");
			return -1;
		}
		
	}
	
	return 0;
}





