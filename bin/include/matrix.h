/*
 *  matrix.h
 *  
 *
 *  Created by andrea on 1/7/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

/* 
 * This is a simple matrix library to implement the basic matrix operations
 */

#include <stdlib.h>
#include <stdio.h>
#include<math.h>	// fabs

/***** MATRIX *****/
// Create Empty Matrix
void mBuild(float ***m, int row, int col);
// Create Random Matrix
void mRand(float ***m,int row, int col, float max, int seed);
// Create Diagonal Matrix
void mDiag(float ***m, int row, float *diag);
// Create Identity Matrix
void mEye(float ***m, int row);
// Destroy Matrix
void mFree(float **m, int row);

// Print Matrix
void mPrint(float **m, int row, int col);

/***** These functions allocate the memory to store the result of the operation in a NEW matrix */
// Perform Matrix Transposition
void mTranspB(float ***m1, int *row1, int *col1, 
			 float **m2, int row2, int col2);

// Perform Matrix Multiplication
int mMultB(float ***m1, int *row1, int *col1, 
		  float **m2, int row2, int col2, 
		  float **m3, int row3, int col3);

// Perform Matrix Inversion
int mInvB(float ***m1, int *row1, int *col1, 
		 float **m2, int row2, int col2);

// Perform Matrix Addition
int mAddB(float ***m1, int *row1, int *col1, 
		 float **m2, int row2, int col2, 
		 float **m3, int row3, int col3);

// Perform Matrix Substraction
int mSubB(float ***m1, int *row1, int *col1, 
		 float **m2, int row2, int col2, 
		 float **m3, int row3, int col3);


/***** These functions store the result of the operation in a matrix ALREADY allocated */
// Perform Matrix Transposition
int mTransp(float **m1, int row1, int col1, 
			  float **m2, int row2, int col2);

// Perform Matrix Multiplication
int mMult(float **m1, int row1, int col1, 
		   float **m2, int row2, int col2, 
		   float **m3, int row3, int col3);


// Perform Matrix Multiplication by transposing the first matrix
//int mMultTr1(float **m1, int row1, int col1, 
//			 float **m2, int row2, int col2, 
//			 float **m3, int row3, int col3);

// Perform Matrix Multiplication by transposing the second matrix
int mMultTr2(float **m1, int row1, int col1, 
		  float **m2, int row2, int col2, 
		  float **m3, int row3, int col3);


// Perform Matrix Inversion
int mInv(float **m1, int row1, int col1, 
		  float **m2, int row2, int col2);

// Perform Matrix Addition
int mAdd(float **m1, int row1, int col1, 
		  float **m2, int row2, int col2, 
		  float **m3, int row3, int col3);

// Perform Matrix Addition (the result is stored in
// the first of the two matrices)
int mAdd2(float **m1, int row1, int col1, 
		 float **m2, int row2, int col2);


// Perform Matrix Substraction (the result is stored 
// in the first of the two matrices)
int mSub(float **m1, int row1, int col1, 
		  float **m2, int row2, int col2, 
		  float **m3, int row3, int col3);

// Perform Matrix Substraction
int mSub2(float **m1, int row1, int col1, 
		 float **m2, int row2, int col2);

// Perform Matrix Copy
int mCopy(float **m1, int row1, int col1, 
		  float **m2, int row2, int col2);



// Workaround for the Kalman Filter
// Note: So far it has been implementened only 
// for squared matrix!

// To copy data from an array to a pointer matrix
void mCopyA2M(float **m1, int n, float m2[n][n]);
// To copy data to an array from a pointer matrix
void mCopyM2A(int n, float m1[n][n], float **m2 );



/***** VECTORS *****/
// Create Empty Matrix
void vBuild(float **v, int row);
// Create Random Matrix
void vRand(float **v,int row, float max, int seed);
// Destroy Matrix
void vFree(float *v);
// Print Matrix
void vPrint(float *m, int row);


