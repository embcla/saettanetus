#include "hokuyomiddle.h"
#include <stdio.h>
int main()
{
  //int i;
  int n;
  urg_t *urg;
  long *data;

init_urg_laser(&urg,&data);
read_laser_data(urg,&data);

int data_max=get_data_max();
int j=0;

float v_risk[36]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1};


for(n=0;n<data_max;n=n+32)
{
	//printf("%ld\n",data[n]);
	v_risk[j]=(15.0/(((float)data[n])/10.0));
	if (v_risk[j]<0.0)
		v_risk[j]=0.0;

	if (v_risk[j]>1.0)
		v_risk[j]=1.0;		

	printf("v_risk[%d]=%f \n",j,v_risk[j]);
	j++;	
}

int i=get_frontal_index(urg);
int front=i/32;
printf("valore frontale:%d\n",front);
close_urg_laser(urg);
free(data);
}
