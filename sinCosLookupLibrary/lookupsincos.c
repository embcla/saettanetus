#include "lookupsincos.h"

   // inline void float_to_int(INTORFLOAT* intorfloat)
   // {
   //     /*Initializing the bias structure for float to int conversion*/
   //     bias.i=((23+127)<<23) +(1<<22) ;
   //     /*Adding and subtracting the bias (ref Game programming gems 2)*/
   //     intorfloat->f+=bias.f;
   //     intorfloat->i-=bias.i;
   // }


    void init_sincos_library(){
        int i;
        pthread_mutex_init (&lookup_table_mutex, NULL);
        /*Malloc of the lookup table*/
        if(!lookup_table)
            lookup_table=malloc(sizeof(float)*elements);
        /*Filling up*/
        pthread_mutex_lock (&lookup_table_mutex);
        for(i=0;i<elements;i++)
            lookup_table[i]=(float)sin((double)i*2.0*M_PI/((float)elements));
        pthread_mutex_unlock (&lookup_table_mutex);
    }

 

    float fsin(float theta)
    {
        /*If the user was neglectful*/
        if(!lookup_table)
            init_sincos_library();
        int i;
        /*Union needed to obtain the index*/
        INTORFLOAT ftmp;
        ftmp.f=theta*((float)elements/(2.0f*M_PI))+FTOIBIAS;
        /*Masking*/
        i=(ftmp.i)&(elements-1);
        return lookup_table[i];
    }

    float fcos(float theta)
    {
        /*If the user was neglectful*/
        if(!lookup_table)
            init_sincos_library();
        int i;
        float result;
        /*Union needed to obtain the index*/
        INTORFLOAT ftmp;
        /*Here, the phase shift of the cosine function is added w.r.t. the index*/
        ftmp.f=theta*((float)elements/(2.0f*M_PI))+FTOIBIAS+(float)elements/4.0f;
        /*Masking*/
        i=(ftmp.i)&(elements-1);
        return lookup_table[i];
    }

    void close_sincos_library()
    {
        pthread_mutex_destroy(&lookup_table_mutex);
        free(lookup_table);
    }
