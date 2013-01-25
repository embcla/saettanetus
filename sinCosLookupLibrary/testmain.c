#include "lookupsincos.h"
#include <stdio.h>
#include <math.h>
void main()
{
	init_sincos_library();
	printf("sine: %f\n",fsin(M_PI));
	printf("cosine: %f\n",fcos(M_PI));
	close_sincos_library();
}
