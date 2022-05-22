/**************************************************************************************************************
Copyright © Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: Recherche.h
Version: 1.0		Date: 2017-04-28		Author: Yanming Wang		ID: 1047930

Description:
	The functions in this file are defined to Search the lines of LDWS.
	The following function types are included:
	+ Recherche(): Search the lines of LDWS.

	
Deviation:

History:
	+ Version: 1.0		Date: 2017-04-28		Author: Yanming Wang	ID: 1047930
	  Modification: Coding Standards are added.
**************************************************************************************************************/

#ifndef _RECHERCHE_H_
#define _RECHERCHE_H_

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	M_I_init		      Modele_Image*	      Init Parameters of 2D modele of road (Location in 2D image).
[in/out]	M_I_est		          Modele_Image*	      Estimated Parameters of 2D modele of road (Location in 2D image).
[in/out]	Donnees		          Fichier*	          Global parameters to be used in LDWS .
[in]	    Tab_Image	          const const char*	  The input image buffer.

Realized function:
    + Search the lines of LDWS
*/
int Recherche(Modele_Image *M_I_init, Modele_Image *M_I_est, Fichier *Donnees, const unsigned char *Tab_Image);

extern int Recherche_LR(Modele_Image *M_I_init, Modele_Image *M_I_est, Fichier *Donnees, const unsigned char *Tab_Image);
#endif
