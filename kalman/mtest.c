#include "matrix.h"

#define ROW	5
#define COL	6


int main(int argc, char *argv[]){
	int i,j;
	float **m1;
	float **m2;
	float **m3;	
	float **m4;
	float **m5;	
	float *diag4;
	float **m1T;
	float **M1;
	float M2[5][5];
	
	int row1, col1;		// n x m
	int row1T,col1T;	// m x n
	int row2, col2;		// n x n
	int row3, col3;		// n x n
	int row4, col4;		// n x n
	int row5, col5;		// n x n	
	
	if (argc<2) {
		printf("Usage: %s randSeed\n",argv[0]);
		return -1;
	}
	
	// Test of functions that allocate the memory required to store the resulting matrix 
	row1=ROW;
	col1=COL;
	mRand(&m1,row1,col1,10,atoi(argv[1]));	
	mPrint(m1,row1,col1);

	// Transposition
	mTranspB(&m1T,&row1T,&col1T,m1,row1,col1);
	mPrint(m1T,row1T,col1T);
	
	// Multiplication
	mMultB(&m2,&row2,&col2,m1,row1,col1,m1T,row1T,col1T);
	mPrint(m2,row2,col2);	
	
	// Inversion
	mInvB(&m3,&row3,&col3,m2,row2,col2);	
	mPrint(m3,row3,col3);	
	
	// Diagonal matrix
	row4=ROW;
	col4=ROW;
	diag4= (float *) malloc(sizeof(float)*row4);
	for (i=0; i<row4; i++) 
		diag4[i]=i+1;

	mDiag(&m4,row4,diag4);
	mPrint(m4,row4,col4);	
	
	mFree(m1,row1);
	mFree(m1T,row1T);	
	mFree(m2,row2);	
	mFree(m3,row3);	
	mFree(m4,row4);	
	
	// Test of functions that assume the memory required to store the 
	// resulting matrix to be already allocated
	//	row1=row2=row3=row4=row1T=0;
	//	col1=col2=col3=col4=col1T=0;	

	// Random Creation
	mRand(&m1,row1,col1,10,atoi(argv[1]));	
	mPrint(m1,row1,col1);

	mBuild(&m1T,row1T,col1T);
	mBuild(&m2,row2,col2);
	mBuild(&m3,row3,col3);
	mBuild(&m4,row4,col4);	
	
	// Transposition
	mTransp(m1T,row1T,col1T,m1,row1,col1);
	mPrint(m1T,row1T,col1T);
	
	// Multiplication
	mMult(m2,row2,col2,m1,row1,col1,m1T,row1T,col1T);
	mPrint(m2,row2,col2);	
	
	// Inversion
	mInv(m3,row3,col3,m2,row2,col2);	
	mPrint(m3,row3,col3);	
	
	// Diagonal matrix
	row4=ROW;
	col4=ROW;
	diag4= (float *) malloc(sizeof(float)*row4);
	for (i=0; i<row4; i++) 
		diag4[i]=i+1;
	
//	mDiag(&m4,row4,diag4);
//	mPrint(m4,row4,col4);	
	
	// Mult
	mPrint(m1T,row1T,col1T);
	mFree(m2,row2);	
	mRand(&m2,row2,col2,10,atoi(argv[1])*5);	
	mPrint(m2,row2,col2);
	mFree(m4,row4);	
	row4=row2;
	col4=row1T;
	mBuild(&m4,row4,col4);
	mMultTr2(m4,row4,col4,m2,row2,col2,m1T,row1T,col1T);
	mPrint(m4,row4,col4);

	// Copy
	row5=row4;
	col5=col4;
	
	printf("\ncopy\n");
	mBuild(&m5,row5,col5);
	mPrint(m5,row5,col5);
	mCopy(m5,row5,col5,m4,row4,col4);
	mPrint(m4,row4,col4);	
	mPrint(m5,row5,col5);
	
	mFree(m1,row1);
	mFree(m1T,row1T);	
	mFree(m2,row2);	
	mFree(m3,row3);	
	mFree(m4,row4);	
	
	
	// To check copy to and from float[r][c]
	// So far only squared matrix are copied!
	printf("\nLast\n");	
	mRand(&M1,5,5,10,atoi(argv[1])*5);		
	mPrint(M1,5,5);
	
	mCopyM2A(5,M2,M1);
	
	for(i=0;i<5;i++)
		for(j=0;j<5;j++)
			M2[i][j]*=100;
	
	mCopyA2M(M1,5,M2);
	mPrint(M1,5,5);
		
	
	
	
	return 0;
}

