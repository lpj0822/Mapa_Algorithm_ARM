/**************************************************************************************************************
Copyright © Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: Bspline.h
Version: 1.0		Date: 2017-04-28		Author: Yanming Wang		ID: 1047930

Description:
	The functions in this file are defined to realize CBspline Filter.
	The following function types are included:
	+ CBsplineFilter(): Do the CBspline Filter for L_output->pPoint.

Deviation:

History:
	+ Version: 1.0		Date: 2017-04-28		Author: Yanming Wang	ID: 1047930
	  Modification: Coding Standards are added.
**************************************************************************************************************/

#ifndef _BSPLINE_H_
#define _BSPLINE_H_

void CBsplineFilter(const Modele_Image *M_I_est, LDWS_Output *L_output, Fichier *Donnees);

#endif	/* _BSPLINE_H_ */
