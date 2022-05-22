/**************************************************************************************************************
Copyright © Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: Matrice.h
Version: 1.0		Date: 2017-04-28		Author: Yanming Wang		ID: 1047930

Description:
	The functions in this file are defined to realize the matrix operations.
	The following function types are included:
	+ M_identite(): .
	
Deviation:

History:
	+ Version: 1.0		Date: 2017-04-28		Author: Yanming Wang	ID: 1047930
	  Modification: Coding Standards are added.
**************************************************************************************************************/

#ifndef _MATRICE_H_
#define _MATRICE_H_

extern void M_identite(int l, double *C);

extern void SommeAB(double *A, double *B, int l, int c, double *C);

extern void DifferenceAB(double *A, double *B, int l, int c, double *C);

extern void ProduitAB(double *A, double *B, int lA, int cA, int cB, double *C);

extern void TransposeA(double *A, int lA, int cA, double *At);

extern void InverseA(double *A, int lA);

extern void AfficheMatrice(double *M, int nbl, int nbc, char *titre);

#endif
