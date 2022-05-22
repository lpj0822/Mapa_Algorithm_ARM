#ifndef CLUSTERING_RECT_H
#define CLUSTERING_RECT_H
#include "vehicle_taillight.h"

void clusteringRect(const TailLightResult *srcRects, const float eps, int *treeNode, int *labelsNode, TailLightResult *dstRects);

#endif // CLUSTERING_RECT_H

