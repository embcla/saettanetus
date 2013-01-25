#include "probstavoid.h"
int main()
{
	int i=0;
	float horizon[HORIZON_ELEMENTS];
	for(i=0;i<HORIZON_ELEMENTS;i++)
		horizon[i]=0.0;
	create_triangle(20,0,1,horizon);
	for(i=0;i<HORIZON_ELEMENTS;i++)
		printf("%f\n",horizon[i]);
	return 1;
}
