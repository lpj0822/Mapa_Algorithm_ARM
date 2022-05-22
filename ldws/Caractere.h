/**************************************************************************************************************
Copyright © Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: Caractere.h
Version: 1.0		Date: 2017-04-28		Author: Yanming Wang		ID: 1047930

Description:
	The functions in this file are defined to find the int or double paramters in the file.
	The following function types are included:
	+ LectureParametresdouble(): Find the double paramter in the file.
	+ LectureParametresInt(): Find the int paramter in the file

Deviation:

History:
	+ Version: 1.0		Date: 2017-04-28		Author: Yanming Wang	ID: 1047930
	  Modification: Coding Standards are added.
**************************************************************************************************************/

#ifndef _CARAC_H_
#define _CARAC_H_

/*
I/O:	    Name		    Type	         Content
					    						  
[in]	    acces		    const char*	     input file name.
[in]	    chaine		    const char*		 chars wants to found.
[in]	    type		    const char*		 The type of paramter to given.
[in/out]	ptr		        double*		     The found double value.

[out]	    return value    int		         If double paramter is found in File return 1; else return 0.

Realized function:
    + Find the double paramter in the file
*/
int LectureParametresdouble(const char *acces, const char *chaine, const char *type, double *ptr);

/*
I/O:	    Name		    Type	         Content
					    						  
[in]	    acces		    const char*	     input file name.
[in]	    chaine		    const char*		 chars wants to found.
[in]	    type		    const char*		 The type of paramter to given.
[in/out]	ptr		        double*		     The found int value.

[out]	    return value    int		         If int paramter is found in File return 1; else return 0.

Realized function:
    + Find the int paramter in the file
*/
int LectureParametresInt(const char *acces, const char *chaine, const char *type, int *ptr);

#endif
