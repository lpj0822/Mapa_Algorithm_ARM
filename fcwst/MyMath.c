//#include "main.h"
#include "MyMath.h"
#include <stdlib.h>
#include <math.h>
#include <stdio.h>


////////////////Gauss-Jordan Matrix inversion algorithm
////////////////if success return 1, else return 0
//////////////// a is the input and output
  int brinv(double a[], int n)
   { int *is,*js,i,j,k,l,u,v;
    double d,p;
    is=(int*)malloc(n*sizeof(int));
    js=(int*)malloc(n*sizeof(int));
    for (k=0; k<=n-1; k++)
      { d=0.0;
        for (i=k; i<=n-1; i++)
        for (j=k; j<=n-1; j++)
          { l=i*n+j; p=fabs(a[l]);
            if (p>d) { d=p; is[k]=i; js[k]=j;}
          }
        if (d+1.0==1.0)
          { free(is); free(js); printf("err**not inv\n");
            return(0);
          }
        if (is[k]!=k)
          for (j=0; j<=n-1; j++)
            { u=k*n+j; v=is[k]*n+j;
              p=a[u]; a[u]=a[v]; a[v]=p;
            }
        if (js[k]!=k)
          for (i=0; i<=n-1; i++)
            { u=i*n+k; v=i*n+js[k];
              p=a[u]; a[u]=a[v]; a[v]=p;
            }
        l=k*n+k;
        a[l]=1.0/a[l];
        for (j=0; j<=n-1; j++)
          if (j!=k)
            { u=k*n+j; a[u]=a[u]*a[l];}
        for (i=0; i<=n-1; i++)
          if (i!=k)
            for (j=0; j<=n-1; j++)
              if (j!=k)
                { u=i*n+j;
                  a[u]=a[u]-a[i*n+k]*a[k*n+j];
                }
        for (i=0; i<=n-1; i++)
          if (i!=k)
            { u=i*n+k; a[u]=-a[u]*a[l];}
      }
    for (k=n-1; k>=0; k--)
      { if (js[k]!=k)
          for (j=0; j<=n-1; j++)
            { u=k*n+j; v=js[k]*n+j;
              p=a[u]; a[u]=a[v]; a[v]=p;
            }
        if (is[k]!=k)
          for (i=0; i<=n-1; i++)
            { u=i*n+k; v=i*n+is[k];
              p=a[u]; a[u]=a[v]; a[v]=p;
            }
      }
    free(is); free(js);
    return(1);
  }

/***************  void MatMul(int m,int n,int k,double a[],double b[],double c[]) ***************
   This function is used to multiplication a[m,n]*b[n,k]=c[m,k]
/*****************************************************************************************/

void MatMul(double a[],double b[],int m,int n,int k,double c[]) 
{ int i,j,l,u;
for (i=0; i<=m-1; i++)
for (j=0; j<=k-1; j++)
  { u=i*k+j; c[u]=0.0;
    for (l=0; l<=n-1; l++)
      c[u]=c[u]+a[i*n+l]*b[l*k+j];
  }
return;
}

/***************  void mul(int m,int n,int k,double a[],double b[],double c[]) ***************
   This function is used to multiplication a[m,n]*b[n,k]=c[m,k]
/*****************************************************************************************/

void mul(double a[],double b[],int m,int n,int k,double c[])
 
  { int i,j,l,u;
    for (i=0; i<=m-1; i++)
    for (j=0; j<=k-1; j++)
      { u=i*k+j; c[u]=0.0;
        for (l=0; l<=n-1; l++)
          c[u]=c[u]+a[i*n+l]*b[l*k+j];
      }
    return;
  }
/*********************  void Trans(double a[],int m,int n,double b[]) ********************
   This function is used to transform a[m,n] into b[n,m]
/*****************************************************************************************/
void Trans(double a[],int m,int n,double b[]) 
{ 
	int i,j,u,v;
	for (i=0; i<m; i++)
	for (j=0; j<n; j++)
	  { 
		u=i*n+j; 
		v=j*m+i;
		b[v]=a[u];
	  }
	return;
}
/*********************  void TransScale(double a[],int m,int n,double s,double b[]) ********************
   This function is used to transform a[m,n] into b[n,m] and multiply by s
/*****************************************************************************************/
void TransScale(double a[],int m,int n,double s,double b[])
{
    int i,j,u,v;
    for (i=0; i<m; i++)
    for (j=0; j<n; j++)
      {
        u=i*n+j;
        v=j*m+i;
        b[v]=s*a[u];
      }
    return;
}

//c=a*b*a'  where b is a symmetric matrix, a is p by n, b is n by n, c is p by p
void SymmetricMultiply(double a[],double b[],int p,int n,double c[])
{ 
	int i,l,j,k;
	for(i=0;i<p;i++)
	{
		for(l=0;l<p;l++)
		{
			c[i*p+l]=0;
			for(j=0;j<n;j++)
			{
				for(k=0;k<n;k++)
				{
					c[i*p+l]=c[i*p+l]+a[i*n+j]*b[j*n+k]*a[l*n+k];
				}
			}
			//double see=c[i*p+l];
		}
	}
	double mean;

	for(i=0;i<p;i++)
		for(l=i;l<p;l++)
		{

			mean=(c[i*p+l]+c[l*p+i])*0.5;
			c[i*p+l]=mean;
			c[l*p+i]=mean;
		}


}


/***************  void add(int m,int n,int k,double a[],double b[],double c[]) ************
   This function is used to add a[m,n] and b[m,n] get c[m,n]
/*****************************************************************************************/

void add(double a[],double b[],int m,int n,double c[])
 
  { int i,j,u;
    for (i=0; i<m; i++)
    for (j=0; j<n; j++)
      { 
		u=i*n+j; 
	    c[u]=a[u]+b[u];
      }
    return;
  }

void add_float(float a[],float b[],int m,int n,float c[])
 
  { int i,j,u;
    for (i=0; i<m; i++)
    for (j=0; j<n; j++)
      { 
		u=i*n+j; 
	    c[u]=a[u]+b[u];
      }
    return;
  }

/***************  void sub(int m,int n,int k,double a[],double b[],double c[]) ************
   This function is used to sub a[m,n] and b[m,n] get c[m,n]
/*****************************************************************************************/

void sub(double a[],double b[],int m,int n,double c[])
 
  { int i,j,u;
    for (i=0; i<m; i++)
    for (j=0; j<n; j++)
      { 
		u=i*n+j; 
	    c[u]=a[u]-b[u];
      }
    return;
  }

void sub_float(float a[],float b[],int m,int n,float c[])
 
  { int i,j,u;
    for (i=0; i<m; i++)
    for (j=0; j<n; j++)
      { 
		u=i*n+j; 
	    c[u]=a[u]-b[u];
      }
    return;
  }

void EYEsub(double a[],int m)
{
	for(int i=0;i<m;i++)
	{
		for(int j=0;j<m;j++)
		{
			if(i==j)
			{
				a[i*m+j]=1-a[i*m+j];
			}
			else
			{
				a[i*m+j]=-a[i*m+j];
			}
		}
		
	}
}

/////////////////////////
// dst = src1.*src2//
////////////////////////
void inner_mul_float(float src1[], float src2[],float dst[], int width, int  height)
{ 
	int i,j,l,u;
	float temp = 0.0;
    for (i = 0; i < height; i++)
	{
		for (j = 0; j < width; j++)
        { 
			u = i*width + j; 
			temp = src1[u]*src2[u];
            dst[u] = temp;
        }
	}
}

void absMat(float src1[], float dst[],int width, int  height)
{
	int i,j,l,u;
	float temp = 0.0;
    for (i = 0; i < height; i++)
	{
		for (j = 0; j < width; j++)
        { 
			u = i*width + j; 
			
			if (dst[u]<0)
				dst[u] = -dst[u];

        }
	}
}
void sMat(float src1[], float dst[], float scaler, int width, int  height)
{ 
	int i,j,l,u;
	float temp = 0.0;
    for (i = 0; i < height; i++)
	{
		for (j = 0; j < width; j++)
        { 
			u = i*width + j; 
			
			if (dst[u]<0)
				dst[u] = 0;
			else
			{
				dst[u] = scaler*src1[u];

			}
			

        }
	}
}


float MAX_Matrix(float src1[], int width, int  height)
{
	int i;
	float temp = 0.0;
    for (i = 0; i < width*height; i++)
	{
		if (src1[i]>temp)
			temp = src1[i];
	}
	return temp;

}
