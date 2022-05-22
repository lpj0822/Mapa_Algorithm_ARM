/**************************************************************************************************************
Copyright © Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: AMF.h
Version: 1.0		Date: 2017-04-28		Author: Yanming Wang		ID: 1047930

Description:
	The functions in this file are defined to realize the Adaptive median filter (AMF).
	The following function types are included:
	+ AMF_Copy(): If line detected Copy M_I_est to M_Filter; else init M_Filter.
	+ AMF_Pro(): Adaptive median filter(AMF) for M_I_est->X.
	+ AMF_Param(): Adaptive median filter(AMF) for Param.

Deviation:

History:
	+ Version: 1.0		Date: 2017-04-28		Author: Yanming Wang	ID: 1047930
	  Modification: Coding Standards are added.
**************************************************************************************************************/

#ifndef _AMF_H_
#define _AMF_H_

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in]	    M_I_est		          Modele_Image*	      Estimated Parameters of 2D modele of road (Location in 2D image).
[in/out]	M_Filter		      Modele_Filter*	  Median filter parameters model.
[in]	    Donnees		          Fichier*	          Global parameters to be used in LDWS .

Realized function:
    + If line detected Copy M_I_est to M_Filter; else init M_Filter
*/
void AMF_Copy(const Modele_Image *M_I_est, Modele_Filter *M_Filter,const Fichier *Donnees);

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	M_I_est		          Modele_Image*	      Estimated Parameters of 2D modele of road (Location in 2D image).
[in/out]	M_Filter		      Modele_Filter*	  Median filter parameters model.
[in/out]	Donnees		          Fichier*	          Global parameters to be used in LDWS .

Realized function:
    + Adaptive median filter(AMF) for M_I_est->X
*/
void AMF_Pro(Modele_Image *M_I_est, Modele_Filter *M_Filter, Fichier *Donnees);

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	diffX		          double*		      differents between ref and cur.
[in]	    ref		              double	          Input value.
[in/out]	cur		              double*	          Filtered value.
[in]        ratio                 double              ratio value
[in]	    Donnees		          Fichier*	          Global parameters to be used in LDWS.
[in]	    i		              int	              index of param

Realized function:
    + Adaptive median filter(AMF) for Param
*/
void AMF_Param(double *diffX, double ref, double *cur, double ratio, Fichier * Donnees, int i);

#endif	/* _AMF_H_ */
