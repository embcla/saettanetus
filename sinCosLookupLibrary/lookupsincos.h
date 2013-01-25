/*
 * This library aims to implement a very fast lookup table for sin and cos *
 * trigonometric functions. The author tries to start a very simple project*
 * and hopes someone will take over                                        *
 * Copyright (C) 2010  Attilio Priolo                                      *
 *                                                                         *
 * This code is totally based on the code shown in the book: "Game         *
 *  Programming Gems 2"                                                    *
 *                                                                         *
 * This program is free software: you can redistribute it and/or modify    *
 * it under the terms of the GNU General Public License as published by    *
 * the Free Software Foundation, either version 3 of the License, or       *
 * (at your option) any later version.                                     *
 *                                                                         *
 * This program is distributed in the hope that it will be useful,         *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 * GNU General Public License for more details.                            *
 *                                                                         *
 * You should have received a copy of the GNU General Public License       *
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.   *
 */

//Define Guards
#ifndef _LOOKUPSINCOS_H
#define	_LOOKUPSINCOS_H
//-------------

//Library Inclusion
#include <math.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <pthread.h>
//-----------------

#ifdef	__cplusplus
extern "C" {
#endif

/*!
 * This function implements a very fast conversion from float to int (5 cycles on
 * a pentium II architecture vs. 60 cycles)
 * @param[in,out] intorfloat union that contains float and int value. The union has
 * to contain the float value before execution.
*/
 //   inline void float_to_int(INTORFLOAT* intorfloat);

/*! \var typedef uint_fast16_t uint
    \brief This datatype represents an unsigned 16 bit integer 
*/
typedef uint_fast16_t uint_16;

#define FTOIBIAS 12582912.0f

const uint elements=1024;
/*! \var typedef union INTORFLOAT
    \brief This datatype represents an union containing either float or int
 * value. Ref. Game programming gems 2
*/

typedef union{
    int i;
    float f;
}INTORFLOAT;
/*!
 * \brief Lookup table
 */
static float *lookup_table=0;

/*!
*  Mutex to manage concurrent access to the lookup table
*/
pthread_mutex_t lookup_table_mutex;
/*!
 * Bias for conversion from float to int
 */
 //   INTORFLOAT bias;
/*!
 * This function initialize the library. It constructs the lookup table and
 * the mutexes able to control the concurrent accesses to the lookup table.
 * 
*/
    void init_sincos_library();

/*!
 * This function implements a very fast trigonometric sin function
 * @param[in] theta The value in radians.
 * @param[out] the sin of the specified theta.
*/
    float fsin(float theta);

/*!
 * This function implements a very fast trigonometric cos function
 * @param[in] theta The value in radians.
 * @param[out] the cos of the specified theta.
*/
    float fcos(float theta);
/*!
 * This function clears all the allocated memory.
*/    
   void close_sincos_library();

#ifdef	__cplusplus
}
#endif

#endif	/* _LOOKUPSINCOS_H */

