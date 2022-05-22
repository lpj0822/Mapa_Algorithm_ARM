#include "clustering_rect.h"

/*
Function process:
+ If rect1 is in the neighbourhood of rect2, return 1
*/
static int  predicateRect(const float eps, const TailLightRect r1, const TailLightRect r2)
{
    int ret = 0;

    float delta = (float)(eps * (WS_MIN(r1.width, r2.width) + WS_MIN(r1.height, r2.height)) * 0.5);

    ret = WS_ABS(r1.x - r2.x) <= delta &&
        WS_ABS(r1.y - r2.y) <= delta &&
        WS_ABS(r1.x + r1.width - r2.x - r2.width) <= delta &&
        WS_ABS(r1.y + r1.height - r2.y - r2.height) <= delta;

    return ret;
}

/*
Function process:
+ Clustered the list of rects.
*/
static int clusteredRects(const TailLightRect *srcRect, const int count, const float eps, int *treeNode, int *labelsNode)
{
    int i, j;
    int N = count;
	const TailLightRect *vec = srcRect;
    const int RANK = 1;

    int *nodes = treeNode;
    int *labels = labelsNode;
    int k, root, root2, parent;
    int nclasses = 0;

    /* Build the queue element as the root node of the forest */
    for (i = 0; i < (N << 1); i++)
    {
        nodes[i++] = -1;
        nodes[i] = 0;
    }

    /* loop through all the elements */
    for (i = 0; i < N; i++)
    {
        root = i;

        /* find the root of each elements */
        while (nodes[(root << 1)] >= 0)
        {
            root = nodes[(root << 1)];
        }

        for (j = 0; j < N; j++)
        {
            /*
            * if i, j elements are not the same one, and vec[i] in the neighbourhood of vec[j], merged rects
            */
            if (i == j || !predicateRect(eps, vec[i], vec[j]))
            {
                continue;
            }

            root2 = j;

            /* find the root of this elements */
            while (nodes[(root2 << 1)] >= 0)
            {
                root2 = nodes[(root2 << 1)];
            }

            /* not in the same tree can be merged */
            if (root2 != root)
            {
                int rank = nodes[(root << 1) + RANK], rank2 = nodes[(root2 << 1) + RANK];

                if (rank > rank2)
                {
                    nodes[(root2 << 1)] = root;
                }
                else
                {
                    nodes[(root << 1)] = root2;
                    nodes[(root2 << 1) + RANK] += (rank == rank2);
                    root = root2;
                }

                k = j;
                /* compress the path from node2 to root */
                while ((parent = nodes[(k << 1)]) >= 0)
                {
                    nodes[(k << 1)] = root;
                    k = parent;
                }

                /* compress the path from node to root */
                k = i;
                while ((parent = nodes[(k << 1)]) >= 0)
                {
                    nodes[(k << 1)] = root;
                    k = parent;
                }
            }
        }
    }

    /* Final O(N) pass: enumerate classes */
    for (i = 0; i < N; i++)
    {
        root = i;
        while (nodes[(root << 1)] >= 0)
        {
            root = nodes[(root << 1)];
        }

        if (nodes[(root << 1) + RANK] >= 0)
        {
            nodes[(root << 1) + RANK] = ~nclasses++;
        }
        labels[i] = ~nodes[(root << 1) + RANK];
    }

    return nclasses;
}

/*
Function process:
+ Clustered and merge the proposals list of rects.
*/
void clusteringRect(const TailLightResult *srcRects, const float eps, int *treeNode, int *labelsNode, TailLightResult *dstRects)
{
    const int objectsCount = srcRects->count;
    int index = 0;
    int cls = 0;
    int classesNum = 0;
    float factor = 0;
	TailLightRect resultRect;

    classesNum = clusteredRects(srcRects->lightRect, srcRects->count, eps, treeNode, labelsNode);

    dstRects->count = classesNum;
	memset(dstRects->lightRect, 0, classesNum * sizeof(TailLightRect));

    for (index = 0; index < objectsCount; index++)
    {
        cls = labelsNode[index];
        dstRects->lightRect[cls].x += srcRects->lightRect[index].x;
        dstRects->lightRect[cls].y += srcRects->lightRect[index].y;
        dstRects->lightRect[cls].width += srcRects->lightRect[index].width;
        dstRects->lightRect[cls].height += srcRects->lightRect[index].height;
        dstRects->lightRect[cls].confidence++;
    }
    for (index = 0; index < classesNum; index++)
    {
        resultRect = dstRects->lightRect[index];
        factor = 1.0f / dstRects->lightRect[index].confidence;
        dstRects->lightRect[index].x = (int)(resultRect.x * factor + 0.5f);
        dstRects->lightRect[index].y = (int)(resultRect.y * factor + 0.5f);
        dstRects->lightRect[index].width = (int)(resultRect.width * factor + 0.5f);
        dstRects->lightRect[index].height = (int)(resultRect.height * factor + 0.5f);
    }
}
