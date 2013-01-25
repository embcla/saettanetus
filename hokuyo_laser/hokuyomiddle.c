#include "hokuyomiddle.h"

/*-----------------------------------------------------------*/
int init_urg_laser(urg_t **urg,int modality)
{
  int ret;
  
  /*Switching capturing mode*/
  urg_capturing_mode=modality;
  
  /*Memory structure allocation*/
  *urg=malloc(sizeof(urg_t));
  
  /*Printing and signalling allocation error*/
  if (!(*urg)) {
    fprintf(stderr, "Error in urg memory allocation\n");
    return -1;
  }
  /*Urg laser connection (structure, file descriptor, baud rate)*/
  ret = urg_connect(*urg, DEVICE, 115200);
   if (ret < 0) {
    return ret;
  }
  /*Switching on the laser scanner*/
  urg_laserOn(*urg); 
 
  pthread_mutex_init(&mutex_laser_read, NULL);
  /*Getting parameters*/
  if(!parameter)
	parameter=malloc(sizeof(urg_parameter_t));
  urg_parameters(*urg, parameter);
  frontal_index=get_frontal_index(*urg);
  /*Switch to select the laser range finder acquiring mode*/
  switch (urg_capturing_mode)
  {
    case CONTINUOUS:
      /*keep capturing*/
      urg_setCaptureTimes(*urg, UrgInfinityTimes);
      /*request all the data*/
      urg_requestData(*urg, URG_MD, URG_FIRST, URG_LAST);
      break;
    case HYBRID:
      /*keep capturing capture_times*/
      urg_setCaptureTimes(*urg, capture_times);
      /*request data*/
      urg_requestData(*urg, URG_MD, URG_FIRST, URG_LAST);
      break;
    case ON_DEMAND:
      urg_requestData(*urg, URG_GD, URG_FIRST, URG_LAST);
      break;
    default:
      break;
  } 
  return ret;
}
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/
int read_laser_data(urg_t *urg)
{
  int n;
  /*If there was an error...*/
  /*Getting the maximum amount of data*/
  data_max = urg_dataMax(urg);
  
  if (data_max<0)
    return -1;
  pthread_mutex_lock(&mutex_laser_read);
    if(!data_laser)
      data_laser=malloc(sizeof(long)*data_max);
      //free(data);
    /*Data buffer allocation (deallocation is demanded to library user)*/
    /*Printing and signalling allocation error*/
    if (!(data_laser)) {
      fprintf(stderr, "Error in data memory allocation\n");
      return -1;
    }
  
  /*Check if capturing mode is HYBRID. If it is true, check the remaining capture times,
   clean the serial and request data again.*/
  else
  {
    if(urg_capturing_mode==HYBRID)
    {
      /*Escamotage to avoid the laser stuck bug*/
      if(urg_remainCaptureTimes(urg)<10)
      {
	urg_disconnect(urg);
	init_urg_laser(&urg,HYBRID);
      }
    }
  }
  /*Receive the data in the buffer*/
  n = urg_receiveData(urg,data_laser,data_max);
  pthread_mutex_unlock(&mutex_laser_read);
    /*When the on_demand mode is on (it is necessary to explicitly request data)*/
  if(urg_capturing_mode==ON_DEMAND)
  {
  int ret = urg_requestData(urg, URG_GD, URG_FIRST, URG_LAST);
  if (ret < 0) {
    return ret;
  }
  }
  
  /*If there was an error...*/
  if (n < 0) {
    return -1;
  }  
  return n;
}
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/
int get_frontal_index()
{
  /*Get it!*/
  if(!parameter)
    return -1;
  return parameter->area_front_;
}
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/
void close_urg_laser(urg_t *urg)
{
  pthread_mutex_destroy(&mutex_laser_read);
  urg_disconnect(urg);
}
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/
int get_data_max()
{
  return data_max;
}
/*-----------------------------------------------------------*/
