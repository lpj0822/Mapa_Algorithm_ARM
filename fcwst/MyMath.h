#ifndef MYMATH_H
#define MYMATH_H


////////////////Gauss-Jordan Matrix inversion algorithm
////////////////if success return 1, else return 0
//////////////// a is the input and output
int brinv(double a[], int n);
/***************  void MatMul(int m,int n,int k,double a[],double b[],double c[]) ***************
   This function is used to multiplication a[m,n]*b[n,k]=c[m,k]
/*****************************************************************************************/
void MatMul(double a[],double b[],int m,int n,int k,double c[]);

/***************  void mul(int m,int n,int k,double a[],double b[],double c[]) ***************
   This function is used to multiplication a[m,n]*b[n,k]=c[m,k]
/*****************************************************************************************/

void mul(double a[],double b[],int m,int n,int k,double c[]);
/*********************  void Trans(double a[],int m,int n,double b[]) ********************
   This function is used to transform a[m,n] into b[n,m]
/*****************************************************************************************/
void Trans(double a[],int m,int n,double b[]) ;

/*********************  void TransScale(double a[],int m,int n,double s,double b[]) ********************
   This function is used to transform a[m,n] into b[n,m]
/*****************************************************************************************/
void TransScale(double a[],int m,int n,double s,double b[]) ;


//c=a*b*a'  where b is a symmetric matrix, a is p by n, b is n by n, c is p by p
void SymmetricMultiply(double a[],double b[],int p,int n,double c[]);


/***************  void add(int m,int n,int k,double a[],double b[],double c[]) ************
   This function is used to add a[m,n] and b[m,n] get c[m,n]
/*****************************************************************************************/

void add(double a[],double b[],int m,int n,double c[]);

/***************  void sub(int m,int n,int k,double a[],double b[],double c[]) ************
   This function is used to sub a[m,n] and b[m,n] get c[m,n]
/*****************************************************************************************/

void sub(double a[],double b[],int m,int n,double c[]);

void EYEsub(double a[],int m);

/////////////////////////
// dst = src1.*src2//
////////////////////////
void inner_mul_float(float src1[], float src2[],float dst[], int width, int  height);

void sMat(float src1[], float dst[], float scaler, int width, int  height);
void sub_float(float a[],float b[],int m,int n,float c[]);
void add_float(float a[],float b[],int m,int n,float c[]);
float MAX_Matrix(float src1[], int width, int  height);
void absMat(float src1[], float dst[],int width, int  height);
#endif
