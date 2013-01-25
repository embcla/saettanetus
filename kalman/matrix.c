/*
 *  matrix.c
 *  
 *
 *  Created by andrea on 1/7/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */
#include "matrix.h"

void mBuild(float ***m,int row, int col) {
	int i;
	
	*m=(float **) malloc(sizeof(float *)*row);
	for (i=0; i< row; i++)
		*(*m+i)= (float *) calloc(1,sizeof(float)*col);

}

void mRand(float ***m,int row, int col, float max, int seed) {
	int i,j;
	
	srand(seed);
	
	*m=(float **) malloc(sizeof(float *)*row);
	for (i=0; i< row; i++) {
		*(*m+i)= (float *) calloc(1,sizeof(float)*col);
		for (j=0; j<col; j++) {
			*(*(*m+i)+j)=(max*rand()/(RAND_MAX+1.0)); 
		}
	}
	
}

void mDiag(float ***m, int row, float *diag) {
	int i;
	
	*m=(float **) malloc(sizeof(float *)*row);	
	for (i=0; i< row; i++) {
		*(*m+i)= (float *) calloc(1,sizeof(float)*row);
		*(*(*m+i)+i)=diag[i]; 
	}
	
}

void mEye(float ***m, int row) {
	int i;
	
	*m=(float **) malloc(sizeof(float *)*row);	
	for (i=0; i< row; i++) {
		*(*m+i)= (float *) calloc(1,sizeof(float)*row);
		*(*(*m+i)+i)=1; 
	}
	
}

void mFree(float **m, int row){
	int i;
	
	for (i=0; i<row; i++)
		free(*(m+i));
	free(m);
	

}


void mPrint(float **m, int row, int col) {
	int i,j;
	
	printf("Matrix %dx%d\n",row,col);
	for (i=0; i<row; i++) {
		for (j=0; j<col; j++) {
			printf("%.4f  ",m[i][j]);
		}
		printf("\n");
	}
	printf("\n");
}

/***** These functions store the result of the operation in a NEW matrix */

void mTranspB(float ***m1, int *row1, int *col1, 
			 float **m2, int row2, int col2) {
	int i,j;

	mBuild(m1,col2,row2);
	*row1=col2;
	*col1=row2;
	
	for (i=0; i<row2; i++)
		for (j=0; j<col2; j++)
			(*m1)[j][i]=m2[i][j];

}


int mMultB(float ***m1, int *row1, int *col1, 
		  float **m2, int row2, int col2, 
		  float **m3, int row3, int col3) {
	int i,j,k;
	float sum;
	
	if (col2!=row3)
		return -1;
	
	mBuild(m1,row2,col3);
	*row1=row2;
	*col1=col3;
	
	for ( i=0; i<row2; i++ )
		for  ( j=0; j<col3; j++ ) {
			sum = 0.0;
			for ( k=0; k<col2; k++ )
				sum += m2[i][k]*m3[k][j];
			(*m1)[i][j] = sum;
		}
}


/* Function to perform the partial-pivoting Gaussian elimination.
 a[][] is the original matrix in the input and transformed
 matrix plus the pivoting element ratios below the diagonal
 in the output. indx[] records the pivoting order.
 Copyright (c) Tao Pang 2001. */

void elgs (float **a, int n, int *indx)
{
	int i, j, k, itmp;
	float c1, pi, pi1, pj;
	float c[n];
	
	/* Initialize the index */	
	for (i = 0; i < n; ++i)
		indx[i] = i;
	
	
	/* Find the rescaling factors, one from each row */	
	for (i = 0; i < n; ++i)	{
		c1 = 0;
		for (j = 0; j < n; ++j) {
			if (fabs(a[i][j]) > c1) c1 = fabs(a[i][j]);
		}
		c[i] = c1;
	}
	
	/* Search the pivoting (largest) element from each column */ 	
	for (j = 0; j < n-1; ++j) {
		pi1 = 0;
		for (i = j; i < n; ++i) {
			pi = fabs(a[indx[i]][j])/c[indx[i]];
			if (pi > pi1) {
				pi1 = pi;
				k = i;
			}
		}
		
		/* Interchange the rows via indx[] to record pivoting order */		
		itmp = indx[j];
		indx[j] = indx[k];
		indx[k] = itmp;
		for (i = j+1; i < n; ++i) {
			pj = a[indx[i]][j]/a[indx[j]][j];
			
			/* Record pivoting ratios below the diagonal */			
			a[indx[i]][j] = pj;
			
			/* Modify other elements accordingly */			
			for (k = j+1; k < n; ++k)
				a[indx[i]][k] = a[indx[i]][k]-pj*a[indx[j]][k];
			
		}
	}
}


/* Function to invert matrix a[][] with the inverse stored
 in x[][] in the output. Copyright (c) Tao Pang 2001. */
void migs(float **a, int n, float **x)
{
	int i,j,k;
	float b[n][n];
	int indx[n];
	
	
	for (i = 0; i < n; ++i)
		for (j = 0; j < n; ++j)
			b[i][j] = 0;
	
	for (i = 0; i < n; ++i)
		b[i][i] = 1;
	
	elgs (a,n,indx);
	
	for (i = 0; i < n-1; ++i) 
		for (j = i+1; j < n; ++j) 
			for (k = 0; k < n; ++k) 
				b[indx[j]][k] = b[indx[j]][k]-a[indx[j]][i]*b[indx[i]][k];
	
	for (i = 0; i < n; ++i)	{
		x[n-1][i] = b[indx[n-1]][i]/a[indx[n-1]][n-1];
		for (j = n-2; j >= 0; j = j-1)	{
			x[j][i] = b[indx[j]][i];
			for (k = j+1; k < n; ++k)
				x[j][i] = x[j][i]-a[indx[j]][k]*x[k][i];
			
			x[j][i] = x[j][i]/a[indx[j]][j];
		}
	}
}

int mInvB(float ***m1, int *row1, int *col1, 
		 float **m2, int row2, int col2) {
	int i,j,k;
	
	if (row2!=col2) {
		printf("Error mInvB\n");
		return -1;
	}
	
	mBuild(m1,row2,col2);
	*row1=row2;
	*col1=col2;
	
	migs(m2,row2,*m1);

}


int mAddB(float ***m1, int *row1, int *col1, 
		 float **m2, int row2, int col2, 
		 float **m3, int row3, int col3) {
	int i,j;
	
	if(row2!=row3 || col2!=col3) {
		printf("Error mAddB\n");
		return -1;
	}
	
	mBuild(m1,row2,col2);
	*row1=row2;
	*col1=col2;
	
	for ( i=0; i<row2; i++ )
		for  ( j=0; j<col3; j++ )
			(*m1)[i][j]=m2[i][j]+m3[i][j];
}

int mSubB(float ***m1, int *row1, int *col1, 
		 float **m2, int row2, int col2, 
		 float **m3, int row3, int col3) {
	int i,j;
	
	if(row2!=row3 || col2!=col3) {
		printf("Error mSubB\n");		
		return -1;
	}
	
	mBuild(m1,row2,col2);
	*row1=row2;
	*col1=col2;
	
	for ( i=0; i<row2; i++ )
		for  ( j=0; j<col3; j++ )
			(*m1)[i][j]=m2[i][j]-m3[i][j];
}


/***** These functions store the result of the operation in a matrix ALREADY allocated */

int mTransp(float **m1, int row1, int col1, 
			 float **m2, int row2, int col2) {
	int i,j;
	
	if (row1!=col2 || col1!=row2) {
		printf("Error mTransp\n");		
		return -1;
	}
	
	for (i=0; i<row2; i++)
		for (j=0; j<col2; j++)
			m1[j][i]=m2[i][j];
	
}


int mMult(float **m1, int row1, int col1, 
		  float **m2, int row2, int col2, 
		  float **m3, int row3, int col3) {
	int i,j,k;
	float sum;
	
	if (col2!=row3) {
		printf("Error mMult\n");		
		return -1;
	}
	
	if (row1!=row2 || col1!=col3) {
		printf("Error mMult\n");		
		return -1;
	}

	
	
	for ( i=0; i<row2; i++ )
		for  ( j=0; j<col3; j++ ) {
			sum = 0.0;
			for ( k=0; k<col2; k++ )
				sum += m2[i][k]*m3[k][j];
			m1[i][j] = sum;
		}
}


// To double check!
int mMultTr1(float **m1, int row1, int col1, 
			 float **m2, int row2, int col2, 
			 float **m3, int row3, int col3) {
	int i,j,k;
	float sum;

	if (row2!=row3) {
		printf("Error mMult\n");		
		return -1;
	}
	
	if (row1!=col2 || col1!=col3) {
		printf("Error mMult\n");		
		return -1;
	}
	
	
	
	for ( i=0; i<row2; i++ )
		for  ( j=0; j<col3; j++ ) {
			sum = 0.0;
			for ( k=0; k<col2; k++ )
				sum += m2[k][i]*m3[k][j];
			m1[i][j] = sum;
		}
	
}

int mMultTr2(float **m1, int row1, int col1, 
			 float **m2, int row2, int col2, 
			 float **m3, int row3, int col3) {


	int i,j,k;
	float sum;
	
	if (col2!=col3) {
		printf("Error mMultTr2\n");		
		return -1;
	}

	
	if (row1!=row2 || col1!=row3) {
		printf("Error mMultTr2\n");		
		return -1;
	}
	
	
	for ( i=0; i<row2; i++ )
		for  ( j=0; j<row3; j++ ) {
			sum = 0.0;
			for ( k=0; k<col2; k++ )
				sum += m2[i][k]*m3[j][k];
			m1[i][j] = sum;
		}
	
}


int mInv(float **m1, int row1, int col1, 
		 float **m2, int row2, int col2) {
	int i,j,k;
	
	if (row2!=col2) {
		printf("Error mInv\n");		
		return -1;
	}
	
	if (row1!=row2 || col1!= col2) {
		printf("Error mInv\n");		
		return -1;
	}
	
	migs(m2,row2,m1);
	
}


int mAdd(float **m1, int row1, int col1, 
		 float **m2, int row2, int col2, 
		 float **m3, int row3, int col3) {
	int i,j;
	
	if(row2!=row3 || col2!=col3) {
		printf("Error mAdd\n");		
		return -1;
	}

	if(row1!=row2 || col1!=col2) {
		printf("Error mAdd\n");		
		return -1;
	}
	
	for ( i=0; i<row2; i++ )
		for  ( j=0; j<col3; j++ )
			m1[i][j]=m2[i][j]+m3[i][j];
}

int mAdd2(float **m1, int row1, int col1, 
		  float **m2, int row2, int col2) {
	int i,j;
	
	if(row1!=row2 || col1!=col2) {
		printf("Error mAdd2\n");		
		return -1;
	}
	
	for ( i=0; i<row2; i++ )
		for  ( j=0; j<col2; j++ )
			m1[i][j]+=m2[i][j];
	

}


int mSub(float **m1, int row1, int col1, 
		 float **m2, int row2, int col2, 
		 float **m3, int row3, int col3) {
	int i,j;
	
	if(row2!=row3 || col2!=col3) {
		printf("Error mSub\n");		
		return -1;
	}

	if(row1!=row2 || col1!=col2) {
		printf("Error mSub\n");		
		return -1;
	}

	for ( i=0; i<row2; i++ )
		for  ( j=0; j<col3; j++ )
			m1[i][j]=m2[i][j]-m3[i][j];
}

int mSub2(float **m1, int row1, int col1, 
		 float **m2, int row2, int col2) {
	int i,j;
		
	if(row1!=row2 || col1!=col2) {
		printf("Error mSub2\n");		
		return -1;
	}
	
	for ( i=0; i<row2; i++ )
		for  ( j=0; j<col2; j++ )
			m1[i][j]-=m2[i][j];
}

int mCopy(float **m1, int row1, int col1, 
		  float **m2, int row2, int col2) {

	int i,j;
	
	if(row1!=row2 || col1!=col2) {
		printf("Error mCopyf\n");		
		return -1;
	}
	
	for ( i=0; i<row2; i++ )
		for  ( j=0; j<col2; j++ )
			m1[i][j]=m2[i][j];
	
}


void mCopyA2M(float **m1, int n, float m2[n][n]) {
	int i,j;

	for ( i=0; i<n; i++ )
		for  ( j=0; j<n; j++ )
			m1[i][j]=m2[i][j];	
	
}

void mCopyM2A(int n, float m1[n][n], float **m2 ) {
	int i,j;
	
	for ( i=0; i<n; i++ )
		for  ( j=0; j<n; j++ )
			m1[i][j]=m2[i][j];	
	
}



/******** VECTORS *********/

void vBuild(float **v, int row) {

	*v = (float *) calloc(1,sizeof(float)*row);

}


void vRand(float **v,int row, float max, int seed){
	int i;
	
	srand(seed);
	
	*v=(float *) malloc(sizeof(float )*row);
	for (i=0; i< row; i++)
		*(*v+i)=(max*rand()/(RAND_MAX+1.0)); 

}


void vFree(float *v) {

	free(v);
}


void vPrint(float *m, int row) {
	int i;
	
	for (i=0; i<row; i++) {
		printf("%f\n",m[i]);
	}


}
