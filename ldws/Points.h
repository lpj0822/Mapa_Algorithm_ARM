/**************************************************************************************************************
Copyright © Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: Points.h
Version: 1.0		Date: 2017-04-28		Author: Yanming Wang		ID: 1047930

Description:
	The functions in this file are defined to find the points in detected zone.
	The following function types are included:
	+ PointsCandidats(): Extract the points of zone.

	
Deviation:

History:
	+ Version: 1.0		Date: 2017-04-28		Author: Yanming Wang	ID: 1047930
	  Modification: Coding Standards are added.
**************************************************************************************************************/

#ifndef _POINTS_H_
#define _POINTS_H_

#define _USEPEAKS_

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in]	    ugsup		          const int		      x coord of left-up point.
[in]	    udsup		          const int		      x coord of right-up point.
[in]	    vsup		          const int		      y coord of up.
[in]	    uginf		          const int		      x coord of left-down point.
[in]	    udinf		          const int		      x coord of right-down point.
[in]	    vinf		          const int		      y coord of down.
[in/out]	tab_x		          int*		          The detected x coord of points.
[in/out]	tab_y		          int*		          The detected y coord of points.
[in/out]	Zone_detect		      Zone*		          The detected zone.
[in]	    Donnees		          Fichier*	          Global parameters to be used in LDWS .
[in]	    Tab_Image	     const unsigned char*	  The input image buffer.

[out]	    returned	          int	              The detected points num

Realized function:
    + Extract the points of zone
*/
int PointsCandidats(const int ugsup, const int udsup, const int vsup, const int uginf, const int udinf, const int vinf, 
					int *tab_x, int *tab_y, Zone *Zone_detect, const Fichier *Donnees, const unsigned char *Tab_Image);

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in]	    ugsup		          const int		      x coord of left-up point.
[in]	    udsup		          const int		      x coord of right-up point.
[in]	    vsup		          const int		      y coord of up.
[in]	    uginf		          const int		      x coord of left-down point.
[in]	    udinf		          const int		      x coord of right-down point.
[in]	    vinf		          const int		      y coord of down.
[in/out]	tab_x		          int*		          The detected x coord of points.
[in/out]	tab_y		          int*		          The detected y coord of points.
[in/out]	Zone_detect		      Zone*		          The detected zone.
[in]	    Donnees		          Fichier*	          Global parameters to be used in LDWS .
[in]	    Tab_Image	     const unsigned char*	  The input image buffer.

[out]	    returned	          int	              The detected points num

Realized function:
    + Extract the points of zone
*/
int PointsCandidats_LR(const int ugsup, const int udsup, const int vsup, const int uginf, const int udinf,const int vinf,
					   int *tab_x, int *tab_y, Zone *Zone_detect, const Fichier *Donnees, const unsigned char *Tab_Image);

#endif
