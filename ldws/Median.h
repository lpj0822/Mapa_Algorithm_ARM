/**************************************************************************************************************
Copyright © Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: Median.h
Version: 1.0		Date: 2017-04-28		Author: Yanming Wang		ID: 1047930

Description:
	The functions in this file are defined to do the line fitting in detected zone.
	The following function types are included:
	+ Median(): line fitting based on the detected points, and update Zone_detect->X and Zone_detect->CX.
	
Deviation:

History:
	+ Version: 1.0		Date: 2017-04-28		Author: Yanming Wang	ID: 1047930
	  Modification: Coding Standards are added.
**************************************************************************************************************/

#ifndef _MEDIAN_H_
#define _MEDIAN_H_

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	tab_x		          int*		          The detected x coord of points.
[in/out]	tab_y		          int*		          The detected y coord of points.
[in/out]	nb_pts	              int*	              The detected points num
[in/out]	Zone_detect		      Zone*		          The detected zone.
[in/out]	Donnees		          Fichier*	          Global parameters to be used in LDWS .

[out]	    returned	          int	              if 0 failed

Realized function:
    + line fitting based on the detected points, and update Zone_detect->X and Zone_detect->CX
*/
int Median(int *tab_x, int *tab_y, int *nb_pts, Zone *Zone_detect, Fichier *Donnees);

#endif
