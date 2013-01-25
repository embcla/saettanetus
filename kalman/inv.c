#include <stdlib.h>
#include <stdio.h>
#include <math.h>


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


void mBuild(float ***m,int row, int col) {
	int i;
	
	*m=(float **) malloc(sizeof(float *)*row);
	for (i=0; i< row; i++)
		*(*m+i)= (float *) calloc(1,sizeof(float)*col);
	
}


void mFree(float **m, int row){
	int i;
	
	for (i=0; i<row; i++)
		free(*(m+i));
	free(m);
	
	
}

void mRand(float **m,int row, int col, float max, int seed) {
	int i,j;
	
	srand(seed);
	for (i=0; i< row; i++) {
		for (j=0; j<col; j++) {
			m[i][j]=(max*rand()/(RAND_MAX+1.0)); 
		}
	}
	
}


void mPrint(float **m, int row, int col) {
	int i,j;
	
	printf("\nMatrix %dx%d\n",row,col);
	for (i=0; i<row; i++) {
		for (j=0; j<col; j++) {
			printf("%2.4f\t",m[i][j]);
		}
		printf("\n");
	}
}

int main(int argc, char *argv[]){

	float **m, **iM;
	int *indx;	
	int n;
	
	if (argc<3) {
		printf("Usage: %s mSize randSeed\n",argv[0]);
		return -1;
	}
		
	n=atoi(argv[1]);
	
	
	mBuild(&m,n,n);
	indx = (int*) malloc(sizeof(int)*n);
	
	mRand(m,n,n,10,atoi(argv[2]));
	mPrint(m,n,n);

	mBuild(&iM,n,n);
	
	migs(m,n,iM);
	mPrint(iM,n,n);
	
	mFree(m,n);
	mFree(iM,n);
	free(indx);
}
