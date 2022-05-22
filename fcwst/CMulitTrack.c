#include "CMulitTrack.h"
#include "fast.h"
#include "harriscroner.h"
#include "declare.h"
#include "surf.h"
#include "Geometry.h"
#include "preprocess.h"
#include "OBJVERF_Interface.h"
#include "vehicle_taillight.h"

//#define RET_DS   //switch for if reserve discard target for 3 frame
#define NEW_DET_PORT
//#define OBJVERF //switch for verify target
//#define OBJRLCT  //switch for relocation
//#ifdef TIME_TEST  //switch for cost time print

//#define TAIL_LIGHT_DEBGU
//#define USE_TAIL_LIGHT

#ifdef __cplusplus
extern "C"
{
#endif

#include "utils.h"
#include <LDWS_Interface.h>

#ifdef  DETCOR_STAR

#ifdef  NEW_DET_PORT
#include <FCWSD_Interface.h>
static objectSetsCar *g_pDetobjsets;
#else
#include "FCW_Detect.h"
extern ObjectSets_car *g_pDetobjsets;
#endif

#endif

#ifdef __cplusplus
}
#endif

#ifdef TAIL_LIGHT_DEBGU
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#endif // TAIL_LIGHT_DEBGU

static adas_fcw_inner_global_param m_globlparam[16];
static MuliTracker g_MuliTracker[2];
static s32 g_OutIndex = -1;
static LDWS_Point g_pLDWSVPoint;
static int histGroundVal[MAX_MOTION_NUM] = {0};
static int histGroundValNum = 0;
static LDWS_InitGuid *pLDWSInit = NULL;
static DarkTailLightGlobalPara gl_tailnightpara = { 0 };


/*
	I/O:	  Name		    Type	   Size			   Content

	[in]	  index		    int		   4-Byte	       Index num of detector to be freed.

	[out]	returned value  int        4-Byte	       If 0, free failed.

	Realized function:
+ Declaration of function
	+ Free the memory space of variables.
*/
static void mvfindGroundValue(PortInput *pInPutParam);


/*
 I/O:	    Name		          Type	     		  Content

 [in]	    pInPutParam		      const PortInput*	  input stract of multi-tracking.
 [in]	    nId		              const s32		      Scale index.

 Realized function:
 + Calculate the ground line of every target
 */
static void mvGroundLineDet(const PortInput *pInPutParam, const s32 nId);


#ifdef DETCOR_STAR


/*
 I/O:	    Name		          Type	     		  Content

 [in]	    pInPutParam		      const PortInput*	  input stract of multi-tracking.
 [in]	    nId		              const s32		      Scale index.

 Realized function:
 + verify the history target in m_globlparam[nId].m_pGroupSets
 */
static void DetcorBytrain(const PortInput *pInPutParam, const s32 nId);

/*
 I/O:	    Name		          Type	     		  Content

 [in]	    pInPutParam		      const PortInput*	  input stract of multi-tracking.
 [in/out]	pGroup		          obj_group*	      tracking target group.
 [in/out]	pDetRec		          AdasRect*		      the detected rect.
 [in]	    bSync		          const uint8_t		  if 1, check the dist between pDetRec and pGroup->.
 [in]	    nId		              const s32		      Scale index.

 Realized function:
 + Do the re-det around the pGroup->rtContour and get the new location as pDetRec; if new location found return 1, else return 0;
 */
static uint8_t mvDetcorBytrainByGroup(const PortInput *pInPutParam,
		obj_group * pGroup, AdasRect *pDetRec, uint8_t bSync, s32 nId);

/*
 I/O:	    Name		          Type	     		  Content

 [in/out]	pgroup		          obj_group*		  Target group.

 Realized function:
 + decide if pgroup can be verified based on pgroup->nDetState and pgroup->nStateNum
 */
static void mvSureTrueGroup(obj_group * pgroup);

#endif

/*
 I/O:	    Name		    Type	     		  Content

 [in/out]	pInPutParam	    PortInput*		      input stract of multi-tracking.

 Realized function:
 + Remove the overlapped rects in pInPutParam.
 */
static void mvReInput(PortInput *pInPutParam);

/*
 I/O:	    Name		          Type	     		  Content

 [in]	    pSrcImg		          const imgage*		  input image buffer.
 [in/out]	pDstImg		          imgage*		      output resized image buffer.
 [in]	    nId		              const s32		      Scale index.

 Realized function:
 + Get the resized image buffer;
 */
static void mvResizeImg(const imgage *pSrcImg, imgage *pDstImg, const s32 nId);

/*
 I/O:	    Name		    Type	     		  Content

 [in]    	pInPutParam	    const PortInput*	  input stract of multi-tracking.
 [in]	    nId		        const s32		      Scale index.

 Realized function:
 + Get the tracking tragets for single-scale tracking, Given the value of m_globlparam[nId].scaleInput.
 */
static void mvGetScaleInPut(const PortInput *pInPutParam, const s32 nId);

/*
 I/O:	    Name		    Type	     		  Content

 [in]	    nId		        const s32		      Scale index.

 Realized function:
 + find m_globlparam[nId].m_pGroupSets->nGroupId != -1; change m_globlparam[nId].m_GroupIndexNum
 and m_globlparam[nId].m_pGroupIndex
 */
static void mvGetCurrenGroupIndex(const s32 nId);

/*
 I/O:	    Name		    Type	     		  Content

 [in]        pInPutParam	    PortInput*	          input stract of multi-tracking.
 [in]	    nId		        const s32		      Scale index.

 Realized function:
 + Add new tracking target based on the detected result. Update m_globlparam.m_PNewRec
 */
static void mvAddNewObjRec(const s32 nId);

/*
 I/O:	    Name		    Type	     		  Content

 [in]        ptPredict	    const AdasPoint*	  input predicted point.
 [in]	    GroupRec		const AdasRect		  target size in last frame.
 [in]	    nMatchLeng		const s32		      length of track, pTrajec->nTrackLen.

 [out]	    returned		AdasRect		      predicted the possible region of target targect.

 Realized function:
 + predict the possible region of target targect based on points and target size in last frame.
 */
static AdasRect mvMatchPreditRect(const AdasPoint *ptPredict,
		const AdasRect GroupRec, const s32 nMatchLeng);

/*
 I/O:	    Name		    Type	     		  Content

 [in]	    nId		        const s32		      Scale index.

 Realized function:
 + update pgroup->PreditRec
 */
static void mvGoupPreditRec(const s32 nId);

/*
 I/O:	    Name		    Type	     		  Content

 [in]        pInPutParam	    PortInput*	          input stract of multi-tracking.
 [in]	    nId		        const s32		      Scale index.

 Realized function:
 + update m_globlparam[nId].m_pMask based on pInPutParam->objRec and m_globlparam[nId].m_pGroupSets
 */
static void mvGenerateRioGrayByMask(const s32 nId);


/*
 I/O:	    Name		    Type	     		  Content

 [in]        pGray	        const uint8_t*	      input image buffer
 [in]	    pMask		    const uint8_t*		  Mask of roi, same size with pGray
 [in]        width	        const s32	          image width
 [in]	    height		    const s32		      image height
 [in]	    Barrier		    const s32		      Corner Thresh
 [in]	    nId		        const s32		      Scale index.

 Realized function:
 + extract the corner points in mask region, the result saved in m_globlparam.m_nCornerPassNum and m_globlparam.m_nFastCornerNum
 */
static void mvCornerDetct(const uint8_t *pGray, const uint8_t *pMask,
		const s32 width, const s32 height, const s32 Barrier, const s32 nId);

/*
 I/O:	    Name		    Type	     		  Content

 [in]        pGray	        const uint8_t*	      input image buffer
 [in]	    size		    AdaSize		          input image size
 [in]	    nId		        const s32		      Scale index.

 Realized function:
 + extract the corner points in mask region, the result saved in m_globlparam.m_nCornerPassNum and m_globlparam.m_nFastCornerNum
 */
static void mvFeatrueDescribe(uint8_t *pGray, AdaSize size, s32 nId);

/*
 I/O:	    Name		    Type	     		  Content

 [in]	    nId		        const s32		      Scale index.

 Realized function:
 + match the traject point with the fast points and update m_globlparam[nId].m_pGroupSets->pObjtr->pTrackPoint
 */
static void mvTrajectorymatch(s32 nId);

/*
 I/O:	    Name		    Type	     		  Content

 [in]        RioRec	        AdasRect	          ROI region of target
 [in/out]	pCarimg		    imgage*		          output car image buffer
 [in]	    nId		        const s32		      Scale index.

 Realized function:
 + Get the car temple img based on ROI region of RioRec
 */
static void mvSetUpdataCarImg(AdasRect RioRec, imgage *pCarimg, s32 nId);

/*
 I/O:	    Name		    Type	     		  Content

 [in]        srcimg	        const imgage*	      input image buffer
 [in/out]	IntelImg		integral imgage*	  output integral image
 [in]	    pointRio		AdasPoint		      Left and up ROI point.

 Realized function:
 +  caculate the integral image of ROI of srcimg
 */
static void mvInterlimg(const imgage *srcimg, imgage*IntelImg,
		AdasPoint pointRio);

/*
 I/O:	    Name		    Type	     		  Content

 [in]	    IntelImg		integral imgage*	  input integral image
 [in]	    lt		        AdasPoint		      Left and up point of ROI.
 [in]	    br		        AdasPoint		      Right and bottom point of ROI.

 Realized function:
 +  Get the inteval value of ROI based on the interval image
 */
static s32 mvSumImgRio(imgage *IntelImg, AdasPoint lt, AdasPoint br);

/*
 I/O:	    Name		    Type	     		  Content

 [in]	    img		        const imgage	      input searched image
 [in]	    Tempimg		    const imgage		  Temple image.
 [in]	    SearcRec		AdasRect		      search region.
 [in/out]	pMatchRec		AdasRect		      Matched result.
 [in]	    nPubliBuffOff	s32		              public buffer offset.
 [in]	    bComparSecd	    uint8_t		          if 1 Caculate the second matched region, else just caculate the best one.
 [in]	    nId		        s32		              scale factor.
 [in]	    fMatchScore		float32_t*		      matching score.

 [out]       returned        uint8_t               if 1 find the matched result; else, return 0

 Realized function:
 +  Do the Temple matching
 */
static uint8_t mvTemplatMatch(const imgage img, const imgage Tempimg,
		AdasRect SearcRec, AdasRect *pMatchRec, s32 nPubliBuffOff,
		uint8_t bComparSecd, s32 nId, float32_t *fMatchScore);

/*
 I/O:	    Name		    Type	     		  Content

 [in]	    pGroup		    const obj_group*	  target group
 [in]	    pLapCarRec		AdasRect*		      detected car region.
 [in]	    nId		        s32		              scale factor.

 [out]       returned        s32                   if 1 use the detected rsult to update group; else, return 0

 Realized function:
 +  match the detect region in pGroup if matched return 1; else return 0
 */
static s32 mvMatchDetRec(const obj_group *pGroup, AdasRect *pLapCarRec, s32 nId);

/*
 I/O:	    Name		    Type	     		  Content

 [in/out]	pGroup		    obj_group*	          target group
 [in]	    RioRec		    AdasRect		      detected rect.
 [in]	    nId		        s32		              scale factor.

 Realized function:
 +  update the temple of target(pGroup->UpdatedImg) based on the detected region
 */
static void mvUpdataNormalCarImg(obj_group * pGroup, AdasRect RioRec, s32 nId);

/*
 I/O:	    Name		    Type	     		  Content

 [in/out]	pGroup		    obj_group*	          target group
 [in]	    nId		        s32		              scale factor.

 Realized function:
 +  update pgroup->pOrInitGray based on pgroup->InitContour
 */
static void mvSetOriInitGray(obj_group *pgroup, s32 nId);

/*
 I/O:	    Name		    Type	     		  Content

 [in/out]	pGroup		    obj_group*	          target group
 [in]	    nId		        s32		              scale factor.

 Realized function:
 +  add traject to pgroup->pObjtr by the matched fast points
 */
static void mvAddTrajecToGroup(obj_group *pgroup, s32 nId);

/*
 I/O:	    Name		    Type	     		  Content

 [in/out]	RioImg		    imgage*	              ROI image buffer
 [in]	    RioRec		    AdasRect	          ROI rect.
 [in]	    bPresent		uint8_t	              if 1 get ROI of m_globlparam[nId].m_pGrayData，else get from m_globlparam[nId].m_preGrayData.
 [in]	    nId		        s32		              scale factor.

 Realized function:
 +  get the ROI image buffer.
 */
static void mvSelcImg(imgage * RioImg, AdasRect RioRec, uint8_t bPresent,
		s32 nId);

/*
 I/O:	    Name		    Type	     		  Content

 [in/out]	pGroup		    obj_group*	          target group
 [in]	    pDetrec		    AdasRect*	          detected rect.
 [in]	    nId		        s32		              scale factor.

 Realized function:
 + Update pGroup by detected result
 */
static void mvUpdataGroupByDet(obj_group *pGroup, AdasRect *pDetrec, s32 nId);

/*
 I/O:	    Name		    Type	     		  Content

 [in/out]	pGroup		    obj_group*	          target group
 [in]	    pInPutParam		const PortInput*	  detetcted result.
 [in]	    nId		        s32		              scale factor.

 Realized function:
 + Update pGroup by detected result
 */
static uint8_t mvUpdataByDetctor(obj_group *pgroup, s32 nId);

/*
 I/O:	    Name		       Type	     		  Content

 [in]	    SelfGroupIndex	   uint8_t	          input group index
 [in/out]	OccluedRec		   AdasRect *         overlopped region.
 [in/out]	pFindInxdex		   s32*		          overlopped group index.
 [in]	    nId		           s32		          scale factor.

 [out]       returned           uint8_t            if there is overlopped return 1; else return 0

 Realized function:
 + decide if there is overlapped in differnet scales
 */
static uint8_t mvOccludedbyGroup(uint8_t SelfGroupIndex, AdasRect *OccluedRec,
		s32 *pFindInxdex, s32 nId);

/*
 I/O:	    Name		    Type	     		  Content

 [in/out]	pGroup		    obj_group*	          target group
 [in/out]	MatchResultRec	AdasRect*	          matched result
 [in]	    nId		        s32		              scale factor.
 [in/out]	pfMatchScore	float32_t*	          matching score

 [out]	    returned    	uint8_t	              if 1 find the matched rect; else return 0

 Realized function:
 + do the temple matching in pgroup->rtContour by pgroup->Templat
 */
static uint8_t mvMatchByGroupTemplate(obj_group *pgroup,
		AdasRect *MatchResultRec, s32 nId, float32_t *pfMatchScore);

/*
 I/O:	    Name		    Type	     		  Content

 [in/out]	pGroup		    obj_group*	          target group
 [in]	    bUpdata	        uint8_t	              if 1 using the filter
 [in]	    nId		        s32		              scale factor.

 Realized function:
 + Do the midle filter for pGroup->rtContour based on pGroup->histoyRec
 */
static void mvGroupFilter(obj_group *pgroup, uint8_t bUpdata, s32 nId);

/*    
 I/O:	    Name		        Type	     		  Content

 [in/out]	pGroup		        obj_group*	          target group
 [in]	    MidVoteCen	        AdasPoint	          middle vote center
 [in]	    pVoteTrajecIndex	s32 *	              vote traject index
 [in]	    nVoteNum	        s32 	              vote traject Num
 [in]	    nDisTrd	            s32 	              traject vote bias threshold
 [in]	    nId		            s32		              scale factor.

 Realized function:
 + return the Num of traject that has similar vote with MidVoteCen
 */
static s32 mvDelFarCen(obj_group *pGroup, AdasPoint MidVoteCen,
		s32 * pVoteTrajecIndex, s32 nVoteNum, s32 nDisTrd, s32 nId);

/*    
 I/O:	    Name		        Type	     		  Content

 [in/out]	pGroup		        obj_group*	          target group
 [in]	    nId		            s32		              scale factor.

 [out]	    returned		    uint8_t		          the similar tarject Num.

 Realized function:
 + caculate group sacles and location based on pgroup->InitContour and pTrajecy->InitPoint, update pgroup->rtContour
 */
static uint8_t mvInitConsensVote(obj_group *pgroup, s32 nId);

/*    
 I/O:	    Name		        Type	     		  Content

 [in/out]	pGroup		        obj_group*	          target group
 [in]	    nId		            s32		              scale factor.

 Realized function:
 + update pGroup->ProcessContour，pGroup->nPreProssVotFram，pTrajecy->bProcessTracked，pTrajecy->ProcessVote，pTrajecy->processfeature
 */
static void mvUpdataProcessVote(obj_group *pGroup, s32 nId);

/*    
 I/O:	    Name		        Type	     		  Content

 [in]	    pGroup		        obj_group*	          target group
 [in/out]	pResImg		        uint8_t*	          output image buffer
 [in]	    nId		            s32		              scale factor.

 Realized function:
 + copy and resize the image of pgroup->OriInitContour to pResImg, pResImg has the same size of pgroup->rtContour
 */
static void mvOriObjResizeToDstImg(obj_group *pgroup, uint8_t *pResImg, s32 nId);

/*    
 I/O:	    Name		        Type	     		  Content

 [in]	    img		            const imgage	      input image
 [in/out]	pTempimg		    imgage*	              Temple image
 [in]	    pTempRec		    AdasRect*	          Temple rect acorrding to img.

 Realized function:
 + Get the temple image from img
 */
static void mvSelcTemplatByCen(const imgage img, imgage *pTempimg,
		AdasRect *pTempRec);

/*    
 I/O:	    Name		        Type	     		  Content

 [in/out]	pgroup		        obj_group*	          Tracking target group
 [in/out]	pMatchRec		    AdasRect*	          temple matched result
 [in]	    nId		            s32		              scale factor.
 [in]	    fMatchScore		    float32_t*		      matched score.

 [out]	    returned		    uint8_t		          if 1 matching succees,else return 0.

 Realized function:
 + Do the temple matching for group if vote tarcking failed
 */
static uint8_t mvTempleMatchByOri(obj_group *pgroup, AdasRect *pMatchRec,
		s32 nId, float32_t *fMatchScore);

/*    
 I/O:	    Name		        Type	     		  Content

 [in/out]	pGroup		        obj_group*	          target group
 [in]	    nId		            s32		              scale factor.

 [out]	    returned		    uint8_t		          the similar tarject Num.

 Realized function:
 + caculate group sacles and location based on pTrajecy->ProcessPoint and pTrajecy->InitPoint, update pgroup->rtContour
 */
static uint8_t mvProcessConsensVote(obj_group *pgroup, s32 nId);

/*    
 I/O:	    Name		        Type	     		  Content

 [in]	    pInPutParam		    const PortInput*      target group
 [in]	    nId		            s32		              scale factor.

 Realized function:
 + Tracking the groups in m_globlparam[].m_pGroupSets
 */
static void mvPreditGroup(const PortInput *pInPutParam, s32 nId);

/*    
 I/O:	    Name		        Type	     		  Content

 [in]	    nId		            s32		              scale factor.

 Realized function:
 + add new group to m_globlparam.m_pGroupSets from m_globlparam[nId].m_PNewRec
 */
static void mvGroupGenerate(s32 nId);

/*    
 I/O:	    Name		        Type	     		  Content

 [in]	    nId		            s32		              scale factor.

 Realized function:
 + Do the Kalman filter for pGroup->rtContour
 */
static void mvKalmanFiter(s32 nId);

/*
 I/O:	    Name		    Type	     		  Content

 [in/out]	pInPutParam	    PortInput*		      input stract of multi-tracking.

 Realized function:
 + Do the multi-sacle tracking for pInPutParam.
 */
static void mvMultileScaleTrack(PortInput *pInPutParam);

/*
 I/O:	    Name		    Type	     		  Content

 [in/out]	pInPutParam	    PortInput*		      input stract of multi-tracking.
 [in]	    nId	            const s32		      Scale index.

 Realized function:
 + Do the single-sacle tracking for pInPutParam in sacle index of nId.
 */
static void mvSingledScaleTrack(PortInput *pInPutParam, const s32 nId);

/*					    						  
 Realized function:
 + do the fushion for each search scale index
 */
static void mvfushionObj(void);

/*    
 I/O:	    Name		        Type	     		  Content

 [in/out]	pGroup		        obj_group*	          target group
 [in]	    nId		            s32		              scale factor.
 [in]	    fZDis		        float32_t		      Z distance of group.
 [in]	    fXDis		        float32_t		      X distance of group.
 [in]	    fDelVanish		    float32_t	          vanish line of group.

 Realized function:
 + set the value of pgroup->pMotion
 */
static void mvSetGroupMotion(obj_group *pgroup, s32 nId, float32_t fZDis,
		float32_t fXDis, float32_t fDelVanish);

/*    
 Realized function:
 + set the value of pgroup->pMotion of each sacle in m_globlparam[nId].m_pGroupSets
 */
static void mvMotionState(void);

/*    
 I/O:	    Name		        Type	     		  Content

 [in]	    pA		            const systime*        time A
 [in]	    pB		            const systime*        time B
 [in/out]	pC		            systime*		      time C

 Realized function:
 + caculate the time difference
 */
static void mvSubTime(const systime *pA, const systime *pB, systime *pC);

/*    
 I/O:	    Name		        Type	     		  Content

 [in/out]	pGroup		        obj_group*	          target group
 [in]	    nId		            s32		              scale factor.

 Realized function:
 + Caculate the TTC and collison path for pgroup
 */
static void mvGroupTimeToCollison(obj_group *pgroup, s32 nId);

/*    
 Realized function:
 + caculate TTC
 */
static void mvTTC(void);

/*    
 Realized function:
 + Set the tracking result to g_MuliTracker
 */
static void mvSetTrackerReult(void);

/*    
 I/O:	    Name		        Type	     		  Content

 [in]	    DetRec		        AdasRect	          search region of car bottom
 [in]	    bottomRec		    AdasRect		      bottom rect.
 [in]	    nId		            s32		              scale factor.
 [in/out]	pSide	            s32*		          The detected left and right side.

 Realized function:
 + search the left and right edge of vehicle, return pSide[0] and pSide[1]. If found return 1, else return 0
 */
static s32 mvFindCarSide(AdasRect DetRec, AdasRect bottomRec, s32 nId,
		s32 *pSide);

/*
 I/O:	    Name		        Type	     		  Content

 [in]	    pgroup		        obj_group*	          Tracking object group
 [in/out]	pBottomRec		    AdasRect*		      returned bottom rect.
 [in]	    nId		            s32		              scale factor.

 Realized function:
 + caculate pgroup->CarBottom
 */
static uint8_t GetVerticlBotoomLine(obj_group *pgroup, AdasRect *pBottomRec,
		s32 nId);

#ifdef DETCOR_STAR

#if 0
/*
 Function process:
 + change pInPutParam to RioImg, RioImg according to pInPutParam->pOriGrayfram.ptr
 Fan-in :
 + mvReLoaclDetRec()
 Fan-out:
 + N/A
 ATTENTION: __________
 */
static void mvSelcInputImg(const PortInput *pInPutParam, imgage * RioImg, AdasRect RioRec)
{
	s32 j;
	uint8_t *pDstr = 0;
	uint8_t *pSrctr = 0;
	s32 orgWidth;

	orgWidth = pInPutParam->imgSize.width * pInPutParam->fzoom;

	RioRec.x = RioRec.x * pInPutParam->fzoom;
	RioRec.y = RioRec.y * pInPutParam->fzoom;
	RioRec.width = RioRec.width * pInPutParam->fzoom;
	RioRec.height = RioRec.height * pInPutParam->fzoom;

	RioImg->nWid = RioRec.width;
	RioImg->nHig = RioRec.height;

	for( j = 0;j < RioImg->nHig; j++)
	{
		pDstr = RioImg->ptr + RioImg->nWid * j;

		pSrctr = pInPutParam->pOriGrayfram.ptr + orgWidth * (j + RioRec.y);

		memcpy(pDstr, pSrctr + RioRec.x, RioRec.width);

	}

}
#endif

/*
	I/O:	  Name		    Type	   Size			   Content

	[in]	  index		    int		   4-Byte	       Index num of detector to be freed.

	[out]	returned value  int        4-Byte	       If 0, free failed.

	Realized function:
+ Declaration of function
	+ Free the memory space of variables.
*/
static void mvfindGroundValue(PortInput *pInPutParam)
{
	unsigned char* prow;
	int groundHist[256] = {0};
	int i, j, k, maxGrayValue, nId, num = 0;
	float32_t groundVar = 0, groundMean = 0, varThresh = 20;
	uint8_t bFilte = 0;

	LDWS_Getinit(&pLDWSInit);
	for (i = pLDWSInit->pBoundPoint[1].y; i < pLDWSInit->pBoundPoint[2].y; i += 2)
	{
		prow = pInPutParam->pOriGrayfram.ptr + i * pInPutParam->pOriGrayfram.nWid;
 		for (j = pLDWSInit->pBoundPoint[0].x; j < pLDWSInit->pBoundPoint[1].x; j += 2)
		{
			k = prow[j];
			groundMean += k;
			groundHist[k]++;  
			num++;
		}
	}
	groundMean /= num;

	for (i = 0; i < 256; i++)
	{
		groundVar += (float)WS_ABS(i - groundMean) * groundHist[i];
	}
	groundVar /= num;


	for (nId = scale_shink_1_id; nId <= scale_shink_4_id; nId++)
	{
		for (i = 0; i < MAX_LAYER_OBJ_GROUP_NUMS; i++)
		{
			AdasRect srcRect, dstRect;
			obj_group *pGroup = m_globlparam[nId].m_pGroupSets + i;
			if ( -1 == pGroup->nGroupId ) 
			{
				continue;
			}

			srcRect.x = pGroup->rtContour.x << nId;
			srcRect.y = pGroup->rtContour.y << nId;
			srcRect.width = pGroup->rtContour.width << nId;
			srcRect.height = pGroup->rtContour.height << nId;

			dstRect.x = pLDWSInit->pBoundPoint[0].x;
			dstRect.y = pLDWSInit->pBoundPoint[1].y;
			dstRect.width = pLDWSInit->pBoundPoint[1].x - pLDWSInit->pBoundPoint[0].x;
			dstRect.height = pLDWSInit->pBoundPoint[2].y - pLDWSInit->pBoundPoint[1].y;

			bFilte = (mvLapTrd(&srcRect, &dstRect, 0.2f)
				|| mvLapTrd(&dstRect, &srcRect, 0.2f));

			if (bFilte)
			{
				break;
			}
		}
	}

	k = histGroundValNum % MAX_MOTION_NUM;
	if (groundVar < varThresh && !bFilte )
	{
		maxGrayValue = -256;
		for (i = 0; i < 256; i++)
		{
			if (groundHist[i] > maxGrayValue)
			{
				maxGrayValue = groundHist[i];
				histGroundVal[k] = i;
			}
		}
	}

	binSort_INT(histGroundVal, MAX_MOTION_NUM);
	pInPutParam->pOriGrayfram.groundValue = histGroundVal[(MAX_MOTION_NUM >> 1)];
    histGroundValNum++;
}


/*
 I/O:	    Name		          Type	     		  Content

 [in]	    pInPutParam		      const PortInput*	  input stract of multi-tracking.
 [in]	    nId		              const s32		      Scale index.

 Realized function:
 + Calculate the ground line of every target
 */
static void mvGroundLineDet(const PortInput *pInPutParam, const s32 nId)
{
	s32 i;
	obj_group *pGroup = 0;
	uint8_t *ptr, *ptr0;
	uint8_t localGroundValue = -1, groundValue = 70;// pInPutParam->pOriGrayfram.groundValue;
	float32_t coeff = 0.1;
	s32 deltCols = 0;
	AdasRect Detect, rltRect;
	s32 groundHist[256] = { 0 };

	for (i = 0; i < MAX_LAYER_OBJ_GROUP_NUMS; i++)
	{
		u16 hist[400] = { 0 };
		s32 x, y, z, objHeight, groundRow, groundRow2, rows, temp0, temp1, maxGrayValue;
		pGroup = m_globlparam[nId].m_pGroupSets + i;
		if (-1 == pGroup->nGroupId || 1 != pGroup->nTruelyObj )
		{
			continue;
		}

		// local ground value
		objHeight = WS_MIN(6 * pGroup->rtContour.height / 4, m_globlparam[nId].m_ImgHeight - pGroup->rtContour.y);
		ptr = m_globlparam[nId].m_pGrayData + (pGroup->rtContour.y + objHeight - 1) * m_globlparam[nId].m_ImgWidth;
		for (y = 0; y < objHeight * 1 / 6; y++)
		{
			for (x = pGroup->rtContour.x; x < pGroup->rtContour.x + pGroup->rtContour.width; x++)
			{
				z = ptr[x];
				groundHist[z]++;

			}
			ptr -= m_globlparam[nId].m_ImgWidth;
		}

		maxGrayValue = -256;
		for (y = 0; y < 256; y++)
		{
			if (groundHist[y] > maxGrayValue)
			{
				maxGrayValue = groundHist[y];
				localGroundValue = y;
			}
		}
		//cv::Mat src2(720, 1280, CV_8UC1, pInPutParam->pOriGrayfram.ptr);
		//cv::line(src2, cv::Point(0, pLDWSInit->pBoundPoint[2].y), cv::Point(1280, pLDWSInit->pBoundPoint[2].y), cv::Scalar(0, 0, 255), 2);
		//cv::rectangle(src2, cv::Point(pGroup->rtContour.x << nId, pGroup->rtContour.y << nId), cv::Point( ((pGroup->rtContour.x + pGroup->rtContour.width) << nId), ((pGroup->rtContour.y + pGroup->rtContour.height) << nId) ), cv::Scalar(0, 0, 255), 2);
		//namedWindow("src2", CV_WINDOW_NORMAL);
		//imshow("src2", src2);

		LDWS_Getinit(&pLDWSInit);
		if ( ((pGroup->rtContour.y + objHeight - 1) << nId) < pLDWSInit->pBoundPoint[2].y && localGroundValue <= groundValue)
		{
			groundValue = localGroundValue;
		}  

		//can use the diff of gray value and the diff of rows as weight
		//objHeight = WS_MIN(5 * pGroup->rtContour.height / 4, m_globlparam[nId].m_ImgHeight - pGroup->rtContour.y);
		ptr = m_globlparam[nId].m_pGrayData + (pGroup->rtContour.y + 5 * objHeight / 6) * m_globlparam[nId].m_ImgWidth;
		temp0 = 0;
		rows = 0;
		for (y = 0; y < 3 * objHeight / 6; y++)
		{
			for (x = pGroup->rtContour.x; x < pGroup->rtContour.x + pGroup->rtContour.width; x++)
			{
				if (ptr[x] < groundValue * 0.9)
				{
					hist[rows]++;
				}
			}
			ptr -= m_globlparam[nId].m_ImgWidth;
			temp0 += hist[rows];
			hist[rows] = temp0;
			rows++;
		}

		// calculate the ground value segmentation line
		z = 0;
		groundRow = -1;
		for (y = 5 * objHeight / 6; y > 3 * objHeight / 6; y--)
		{
			if (hist[z] > temp0 * coeff)
			{
				groundRow = y;
				break;
			}
			z++;
		}

		//can use the diff of gray value and the diff of rows as weight
		//objHeight = WS_MIN(5 * pGroup->rtContour.height / 4, m_globlparam[nId].m_ImgHeight - pGroup->rtContour.y);
		//ptr = m_globlparam[nId].m_pGrayData + (pGroup->rtContour.y + objHeight - 1) * m_globlparam[nId].m_ImgWidth;
		//temp1 = 0;
		//rows = 0;
		//memset(hist, 0, sizeof(u16)* 400);
		//for (y = 0; y < objHeight * 2 / 5; y++)
		//{
		//	for (x = pGroup->rtContour.x; x < pGroup->rtContour.x + pGroup->rtContour.width; x++)
		//	{
		//		if (ptr[x] < groundValue * 0.5)
		//		{
		//			hist[rows]++;
		//		}
		//	}
		//	ptr -= m_globlparam[nId].m_ImgWidth;
		//	temp1 += hist[rows];
		//	hist[rows] = temp1;
		//	rows++;
		//}

		//// calculate the ground value segmentation line
		//z = 0;
		//groundRow2 = -1;
		//for (y = objHeight - 1; y > objHeight * 2 / 5 - 1; y--)
		//{
		//	if (hist[z] > temp1 * coeff)
		//	{
		//		groundRow2 = y;
		//		break;
		//	}
		//	z++;
		//}

		// updata the target rect
		rltRect = pGroup->rtContour;
		//if ( WS_ABS(temp0 - temp1) )
			//groundRow = groundRow2;

		if ((WS_ABS(groundRow - pGroup->rtContour.height) > pGroup->rtContour.height / 4))
		{
			AdasRect DetRect;
#ifdef DETCOR_STAR	
			uint8_t bDetCarByRio = mvDetcorBytrainByGroup(pInPutParam, pGroup,
				&DetRect, 1, nId);
#else
			bDetCarByRio = 0;
#endif
			if (!bDetCarByRio)
			{
				my_printf(
					"***pgroup ID: %d is not Tracked and not bottom shadow detected!\n",
					pGroup->nGroupId);
				mvClearGroupIfo(pGroup, 1);
			}
			else
			{
				pGroup->nDisCardNum = 0;
				mvUpdataGroupByDet(pGroup, &DetRect, nId);
				mvUpdataGroupCenTraj(pGroup);
			}
		}
		else
		{
//			if (pGroup->sysKalmState.x[5] > 3 || pGroup->sysKalmState.x[5] < 0.5)
//			{
//				AdasRect DetRect;
//#ifdef DETCOR_STAR	
//				uint8_t bDetCarByRio = mvDetcorBytrainByGroup(pInPutParam, pGroup,
//					&DetRect, 1, nId);
//#else
//				bDetCarByRio = 0;
//#endif
//				if (!bDetCarByRio)
//				{
//					my_printf(
//						"***pgroup ID: %d is not Tracked and not bottom shadow detected!\n",
//						pGroup->nGroupId);
//					mvClearGroupIfo(pGroup, 1);
//				}
//				else
//				{
//					pGroup->nDisCardNum = 0;
//					mvUpdataGroupByDet(pGroup, &DetRect, nId);
//					mvUpdataGroupCenTraj(pGroup);
//				}
//			}
//			else
//			{
				//groundRow = WS_MIN(groundRow, m_globlparam[nId].m_ImgHeight - pGroup->rtContour.y);
				//groundRow = (groundRow + pGroup->rtContour.height) / 2;
		        //deltCols = (groundRow - pGroup->rtContour.width) / 2;
				//rltRect.height = groundRow;
				//rltRect.x -= WS_MIN(deltCols, pGroup->rtContour.x);
				//rltRect.width += WS_MIN(deltCols, m_globlparam[nId].m_ImgWidth - pGroup->rtContour.x) * 2;

			    groundRow = (groundRow + pGroup->rtContour.height) / 2;
				deltCols = groundRow - pGroup->rtContour.height;
				rltRect.y = WS_MAX(rltRect.y, deltCols);
				rltRect.height = WS_MIN(pGroup->rtContour.height + 2 * deltCols, m_globlparam[nId].m_ImgHeight - rltRect.y);
				rltRect.x = WS_MAX(rltRect.x, deltCols);
				rltRect.width = WS_MIN(pGroup->rtContour.width + 2 * deltCols, m_globlparam[nId].m_ImgWidth - rltRect.x);
				pGroup->rtContour = rltRect;
		}
	}
}


static int findSymmetryAxisX(unsigned char *src, int width, int height) 
{
	int axisX = -1; 
	int secaxisX = -1;
	int half_width, x;
	float minHs = 100000;
	float secMinHS = 100000;

	for(x = width * 1 / 4; x < width * 3 / 4; x++) 
	{
		float HS = 0;
		int count = 0;
		int step, y;
		half_width =  WS_MIN(x, width - x);
		for(step = 1; step < half_width; step++)
		{
			for(y = height / 4; y < height; y += 2) 
			{
				unsigned char* ptr = src + y * width;
				int neg = x - step;
				int pos = x + step;
				unsigned char Gneg = ptr[neg];
				unsigned char Gpos = ptr[pos];
				HS += abs(Gneg - Gpos);
				count++;
			}
		}
		HS /= (count + 1e-10f);

		if (HS < minHs )
		{
			secMinHS = minHs;
			minHs = HS;
			secaxisX = axisX;
			axisX = x;
		}
	}

	if ( secMinHS >= 1.2 * minHs )
	{
		return axisX;
	}
	else
	{
		if (abs(secaxisX - axisX) < width * 0.1)
		{
			return axisX;
		}
		else
		{
			return 0;
		}
	}
}

#ifdef TAIL_LIGHT_DEBGU
void cvtUcharToMat(uint8_t *image, int width, int height, cv::Mat &dst)
{
	int index = 0;
	for (int row = 0; row < height; row++)
	{
		for (int col = 0; col < width; col++)
		{
			if (image[index] != 0)
			{
				dst.at<uchar>(row, col) = 255;
			}
			else
			{
				dst.at<uchar>(row, col) = 0;
			}
			index++;
		}
	}
}
#endif //TAIL_LIGHT_DEBGU

static void  mvTailNightDetct(const PortInput *pInPutParam, const s32 nId)
{
	s32 i, j, k, syAxis;
	obj_group *pGroup = 0;
	unsigned char* src;

	for (i = 0; i < MAX_LAYER_OBJ_GROUP_NUMS; i++)
	{
		imgage img = { 0 };
		AdasRect obj;
		Motion *pNearestMotion = 0;

		pGroup = m_globlparam[nId].m_pGroupSets + i;
		pNearestMotion = pGroup->pMotion
			+ ((pGroup->nMotionLeng - 1) & MAX_MOTION_BIT);
		if ( -1 == pGroup->nGroupId || 1 != pGroup->nTruelyObj || pNearestMotion->bInCollishionPath != 1 ) 
		{
			continue;
		}

		img.nHig = pInPutParam->pOriGrayfram.nHig;
		img.nWid = pInPutParam->pOriGrayfram.nWid;
		img.ptr = pInPutParam->pOriGrayfram.ptr;
		//img.putr = pInPutParam->pOriGrayfram.putr;
		img.pvtr = pInPutParam->pOriGrayfram.pvtr;

		obj.x = pGroup->rtContour.x << nId;
		obj.y = (pGroup->rtContour.y << nId);
		obj.width = (pGroup->rtContour.width << nId);
		obj.height = (pGroup->rtContour.height << nId);

		// calculate the symmetry
		src = (unsigned char* )my_malloc(obj.height * obj.width);
		k = 0;
		for( i = obj.y; i < obj.y + obj.height; i++)
		{
			for( j = obj.x; j < obj.x + obj.width; j++) 
			{
				src[k] = img.pvtr[i * obj.width + j];
				k++;
			}
		}
		syAxis = findSymmetryAxisX(src, obj.width, obj.height);
		syAxis += obj.x;

		//obj.x = WS_MAX(0, obj.x - obj.width * 0.4);
		obj.x = obj.x - (int16_t)(obj.width * 0.2);

		if ( obj.x + obj.width + obj.width * 0.4 < img.nWid)
		{
			obj.width = obj.width + (int16_t)(obj.width * 0.4);
		}

		// find the tail light
		computeTailLight(&img, obj, pGroup->nGroupId, &gl_tailnightpara);


#ifdef TAIL_LIGHT_DEBGU
		// show the result
		cv::Mat src0(img.nHig, img.nWid, CV_8UC1, img.ptr);
		cv::Mat dst;
		cv::cvtColor(src0, dst, CV_GRAY2BGR);
		cv::Mat darkImage = cv::Mat::zeros(img.nHig, img.nWid, CV_8UC1);
		cv::Mat tailLight = cv::Mat::zeros(img.nHig, img.nWid, CV_8UC1);

		cvtUcharToMat(gl_tailnightpara.lightImage, img.nWid, img.nHig, darkImage);
		cvtUcharToMat(gl_tailnightpara.tailLightImage, img.nWid, img.nHig, tailLight);

		cv::rectangle(dst, cv::Point((pGroup->rtContour.x << nId), pGroup->rtContour.y << nId), \
			cv::Point((pGroup->rtContour.x << nId) + (pGroup->rtContour.width << nId), (pGroup->rtContour.y << nId) + (pGroup->rtContour.width << nId)), \
			cv::Scalar(255, 0, 0), 2);


		cv::rectangle(dst, cv::Point(obj.x, obj.y), \
			cv::Point(obj.x + obj.width, obj.y + obj.height), \
			cv::Scalar(0, 255, 0), 2);


		TailLightRect temp;
		for (int i = 0; i < gl_tailnightpara.finalResult.count; i++)
		{
			temp = gl_tailnightpara.finalResult.lightRect[i];

			cv::rectangle(dst, cv::Point(temp.x, temp.y), \
				cv::Point(temp.x + temp.width, temp.y + temp.height), \
				cv::Scalar(0, 0, 255), 2);
		}
		std::string winName = std::to_string (nId);
		cv::imshow(winName, dst);
		cv::imshow("light", darkImage);
		cv::imshow("tail", tailLight);
#endif //TAIL_LIGHT_DEBGU

		// use the tailNight information to adjust the object
		if ( gl_tailnightpara.finalResult.count != 0  )
		{
			int objlx =  gl_tailnightpara.finalResult.lightRect[0].x +  gl_tailnightpara.finalResult.lightRect[0].width / 2;
			int objrx =  gl_tailnightpara.finalResult.lightRect[1].x +  gl_tailnightpara.finalResult.lightRect[1].width / 2;
			int objw = (objrx - objlx) / 8;

			if ( abs( (objlx + objrx) / 2 - ( (pGroup->rtContour.x + pGroup->rtContour.width / 2 ) << nId) ) < pGroup->rtContour.width / 8 )
			{
				objlx = WS_MAX( objlx - objw, 0);
				objrx = WS_MIN( objrx + objw, img.nWid );
				objlx = objlx >> nId;
				objrx = objrx >> nId;

				pGroup->rtContour.x =  objlx;
				pGroup->rtContour.y -= (objrx - objlx - pGroup->rtContour.width) / 2;
				pGroup->rtContour.width =  objrx - objlx;
				pGroup->rtContour.height = pGroup->rtContour.width;
			}
		}

		my_free(src);
	}
}


/*
 Function process:
 + Do the re-det around the pGroup->rtContour and get the new location as pDetRec
 Fan-in :
 + mvPreditGroup()
 Fan-out:
 + FCWSD_Processor_ROI()
 ATTENTION: __________
 */
static uint8_t mvDetcorBytrainByGroup(const PortInput *pInPutParam,
		obj_group * pGroup, AdasRect *pDetRec, uint8_t bSync, s32 nId) {
	AdasRect OriRioRec;
	s32 i;
	s32 nDistance;
	s32 nExtend_x, nExtend_y;

#ifdef NEW_DET_PORT
	FCWSDSize fcwsDetMinSize;
	FCWSDSize fcwsDetMaxSize;
	FCWSDImage imgROITemp;
	FCWSDRect roi;
#endif

	if (Car != pGroup->ntype || !pGroup->nTruelyObj) {
		return 0;
	}

	if (pGroup->nMinCarWid > 0) {

#ifndef NEW_DET_PORT
		nExtend = nExtend * 0.4f;
		OriRioRec.x = (pGroup->rtContour.x - nExtend);
		OriRioRec.y = ( pGroup->rtContour.y - nExtend);
		OriRioRec.width = (pGroup->rtContour.width + (nExtend<<1));
		OriRioRec.height = (pGroup->rtContour.height+ (nExtend<<1));

		mvScopeImg(&OriRioRec,adasrect(0,0,m_globlparam.m_ImgWidth- 1,m_globlparam.m_ImgHeight -1));

		imgRio.ptr = (uint8_t*)m_globlparam.m_PublacSape;

		mvSelcInputImg(pInPutParam,&imgRio,OriRioRec);
#endif

#ifdef NEW_DET_PORT

		nExtend_x = (s32)(pGroup->rtContour.width * 0.2f);
		nExtend_y = (s32)(pGroup->rtContour.height * 0.2f);
		OriRioRec.x = (pGroup->rtContour.x - nExtend_x);
		OriRioRec.y = (pGroup->rtContour.y - nExtend_y);
		OriRioRec.width = (pGroup->rtContour.width + (nExtend_x << 1));
		OriRioRec.height = (pGroup->rtContour.height + (nExtend_y << 1));

		mvScopeImg(&OriRioRec,
				adasrect(0, 0, m_globlparam[nId].m_ImgWidth - 1,
						m_globlparam[nId].m_ImgHeight - 1));
		

		imgROITemp.ptr = pInPutParam->pOriGrayfram.ptr;
		imgROITemp.nWid = (int)(m_globlparam[nId].m_ImgWidth * m_globlparam[nId].scaleInput.fzoom);
		imgROITemp.nHig = (int)(m_globlparam[nId].m_ImgHeight * m_globlparam[nId].scaleInput.fzoom);

		fcwsDetMinSize.width = (s32) (pGroup->rtContour.width
				* m_globlparam[nId].scaleInput.fzoom / 1.2f - 1);
		fcwsDetMinSize.height = (s32) (pGroup->rtContour.height
				* m_globlparam[nId].scaleInput.fzoom / 1.2f - 1);

		fcwsDetMaxSize.width = (s32) (pGroup->rtContour.width
				* m_globlparam[nId].scaleInput.fzoom * 1.2f + 1);
		fcwsDetMaxSize.height = (s32) (pGroup->rtContour.height
				* m_globlparam[nId].scaleInput.fzoom * 1.2f + 1);

		roi.point.x = (int)(OriRioRec.x * m_globlparam[nId].scaleInput.fzoom);
		roi.point.y = (int)(OriRioRec.y * m_globlparam[nId].scaleInput.fzoom);
		roi.size.width = (int)(OriRioRec.width * m_globlparam[nId].scaleInput.fzoom);
		roi.size.height = (int)(OriRioRec.height * m_globlparam[nId].scaleInput.fzoom);

		if (m_globlparam[nId].pOriInPutParam->pOriGrayfram.dayOrNight == 0)
		{
			FCWSD_Processor_ROI(3, 0, &imgROITemp, &roi,
				&fcwsDetMinSize, &fcwsDetMaxSize, 1);
			FCWSD_GetResult(3, &g_pDetobjsets);
		}
		else if (m_globlparam[nId].pOriInPutParam->pOriGrayfram.dayOrNight == 1)
		{
			FCWSD_Processor_ROI(7, 0, &imgROITemp, &roi,
				&fcwsDetMinSize, &fcwsDetMaxSize, 1);
			FCWSD_GetResult(7, &g_pDetobjsets);
		}

		for (i = 0; i < g_pDetobjsets->nObjectNum; ++i) {
			g_pDetobjsets->objects[i].objectRect.point.x = (int)(g_pDetobjsets->objects[i].objectRect.point.x / m_globlparam[nId].scaleInput.fzoom);
			g_pDetobjsets->objects[i].objectRect.point.y = (int)(g_pDetobjsets->objects[i].objectRect.point.y / m_globlparam[nId].scaleInput.fzoom);
			g_pDetobjsets->objects[i].objectRect.size.width =
					(int)(g_pDetobjsets->objects[i].objectRect.size.width / m_globlparam[nId].scaleInput.fzoom);
			g_pDetobjsets->objects[i].objectRect.size.height =
					(int)(g_pDetobjsets->objects[i].objectRect.size.height / m_globlparam[nId].scaleInput.fzoom);
		}
#else

		FCW_DETCOR_VehicleDetprocess_Rio(&imgRio, nn, adasize( (s32)(pGroup->rtContour.width * 0.6f), (s32)(pGroup->rtContour.width * 0.6f) ),
				adasize((s32)(OriRioRec.width),(s32)(OriRioRec.width)),0);
#endif


		if (1 == g_pDetobjsets->nObjectNum) {

#ifdef  NEW_DET_PORT

			pDetRec->x = (g_pDetobjsets->objects[0].objectRect.point.x);
			pDetRec->y = (g_pDetobjsets->objects[0].objectRect.point.y);
			pDetRec->width = (g_pDetobjsets->objects[0].objectRect.size.width);
			pDetRec->height =
					(g_pDetobjsets->objects[0].objectRect.size.height);

#else
			pDetRec->x = ( g_pDetobjsets->objects[0].x + OriRioRec.x);
			pDetRec->y = ( g_pDetobjsets->objects[0].y + OriRioRec.y);
			pDetRec->width = ( g_pDetobjsets->objects[0].width);
			pDetRec->height = ( g_pDetobjsets->objects[0].height);
#endif

			mvScopeImg(pDetRec,
					adasrect(0, 0, m_globlparam[nId].m_ImgWidth,
							m_globlparam[nId].m_ImgHeight));
			g_pDetobjsets->nObjectNum = 0;
			pDetRec->nType = Car;

			pDetRec->confidence = pGroup->rtContour.confidence;

			if (bSync) {
				nDistance =
						WS_ABS( pDetRec->x + (pDetRec->width >>1) - ( pGroup->rtContour.x + (pGroup->rtContour.width>>1)) )
								+\
 WS_ABS( pDetRec->y + (pDetRec->height >>1) - (pGroup->rtContour.y + (pGroup->rtContour.height >>1 ) ) );

				if (nDistance > 0.3f * pGroup->rtContour.width) {
					return 0;
				}
			}
			return 1;
		}
		return 0;
	}

	return 0;
}

/*
 Function process:
 + verify the history target in m_globlparam[nId].m_pGroupSets
 Fan-in :
 + mvSingledScaleTrack()
 Fan-out:
 + FCWSD_Processor_ROI()
 + mvSureTrueGroup()
 ATTENTION: __________
 */
static void DetcorBytrain(const PortInput *pInPutParam, const s32 nId) {
 
#ifdef OBJVERF
	//float32_t scale = 1.15f;
	//s32  nn = 8;
	s32 i;
	obj_group *pgroup;
	uint8_t uInterFramTrd = 2;
	//imgage imgRio;
	//imgage GrayImg;
	AdasRect RioRec;
	//float32_t fScalerio = 0.3f;
	//s32 nDis,nDelNum =0;
	int nDetCarNum = 0;

#ifdef NEW_DET_PORT	
	FCWSDSize fcwsDetMinSize;
	FCWSDSize fcwsDetMaxSize;
	FCWSDRect roi;
	FCWSDImage imgRioTemp;
#endif

	for (i = 0; i < MAX_LAYER_OBJ_GROUP_NUMS; i++) {
		pgroup = m_globlparam[nId].m_pGroupSets + i;

		if (Car != pgroup->ntype) {
			pgroup->nTruelyObj = 1;
			continue;
		}

		if (-1 != pgroup->nGroupId && !pgroup->nTruelyObj
			&& m_globlparam[nId].scaleInput.nFramSeq - pgroup->nPerLastDetFramseq
	> uInterFramTrd) {
		RioRec = pgroup->PreditRec;

		if (RioRec.width * RioRec.height < MAX_PUBLIC_SPACE_SIZE) {

#ifdef NEW_DET_PORT
			RioRec.x -= RioRec.width * 0.2f;
			RioRec.y -= RioRec.height * 0.2f;
			RioRec.width += RioRec.width * 0.4f;
			RioRec.height += RioRec.height * 0.4f;
			mvScopeImg(&RioRec,
				adasrect(0, 0, m_globlparam[nId].m_ImgWidth - 1,
				m_globlparam[nId].m_ImgHeight - 1));

			fcwsDetMinSize.width = (s32) (pgroup->rtContour.width
				* pInPutParam->fzoom / 1.2f - 1);
			fcwsDetMinSize.height = (s32) (pgroup->rtContour.height
				* pInPutParam->fzoom / 1.2f - 1);

			fcwsDetMaxSize.width = (s32) (pgroup->rtContour.width
				* pInPutParam->fzoom * 1.2f + 1);

			fcwsDetMaxSize.height = (s32) (pgroup->rtContour.height
				* pInPutParam->fzoom * 1.2f + 1);


			roi.point.x = RioRec.x * pInPutParam->fzoom;
			roi.point.y = RioRec.y * pInPutParam->fzoom;
			roi.size.width = RioRec.width * pInPutParam->fzoom;
			roi.size.height = RioRec.height * pInPutParam->fzoom;

			imgRioTemp.ptr = pInPutParam->pOriGrayfram.ptr;
			imgRioTemp.nWid = m_globlparam[nId].m_ImgWidth
				* pInPutParam->fzoom;

			imgRioTemp.nHig = m_globlparam[nId].m_ImgHeight
				* pInPutParam->fzoom;

			nDetCarNum = FCWSD_Processor_ROI(1, 0, &imgRioTemp, &roi,
				&fcwsDetMinSize, &fcwsDetMaxSize, 1);

#else

			mvCopyToObjMat(&GrayImg,RioRec,&imgRio );

			FCW_DETCOR_VehicleDetprocess_Rio(&imgRio, nn, adasize( (s32)(pgroup->rtContour.width * 0.5f), (s32)(pgroup->rtContour.width*0.5f) ),
				adasize(pgroup->PreditRec.width,pgroup->PreditRec.width),0);
#endif

		} else {
			nDetCarNum = 0;
		}

		pgroup->nDetState[pgroup->nStateNum++] = nDetCarNum ? 1 : 0;
		mvSureTrueGroup(pgroup);

		pgroup->nPerLastDetFramseq = m_globlparam[nId].scaleInput.nFramSeq;

		}
	}
#else

	s32 i, j;
	obj_group *pgroup;
	uint8_t uInterFramTrd = 2;
	AdasRect RioRec;
	for (i = 0; i < MAX_LAYER_OBJ_GROUP_NUMS; i++)
	{
		pgroup = m_globlparam[nId].m_pGroupSets + i;

		if (Car != pgroup->ntype)
		{
			pgroup->nTruelyObj = 1;
			continue;
		}

		if (-1 != pgroup->nGroupId && !pgroup->nTruelyObj
				&& m_globlparam[nId].scaleInput.nFramSeq - pgroup->nPerLastDetFramseq
						> uInterFramTrd) 
		{
			RioRec = pgroup->rtContour;

			for (j = 0; j < pVerfobjOutput.nObjNum; j++) 
			{
				if ( pgroup->nGroupId == pVerfobjOutput.objInput[j].nGroupID )
				{
					pgroup->nDetState[pgroup->nStateNum++] = pVerfobjOutput.objInput[i].nObjState;
					break;
				}
			}

			mvSureTrueGroup(pgroup);
			pgroup->nPerLastDetFramseq = m_globlparam[nId].scaleInput.nFramSeq;
		}
	}
#endif

}

static void mvCopyToObjMat(const imgage *pGray, AdasRect Rio, imgage *RioMat) {
	s32 j;
	uint8_t *pSrctr;
	uint8_t *pDstr;

	RioMat->nWid = Rio.width;
	RioMat->nHig = Rio.height;
	//my_printf("%d,%d\n",pGray->nWid,pGray->nHig);
	//my_printf("%d,%d,%d,%d\n",Rio.x,Rio.y,Rio.width,Rio.height);

	for (j = Rio.y; j < Rio.y + Rio.height; j++) {
		pDstr = RioMat->ptr + RioMat->nWid * (j - Rio.y);
		pSrctr = pGray->ptr + pGray->nWid * j;

		memcpy(pDstr, pSrctr + Rio.x, Rio.width);
	}
}

/*
 Function process:
 + decide if pgroup can be verified based on pgroup->nDetState and pgroup->nStateNum
 Fan-in :
 + DetcorBytrain()
 Fan-out:
 + mvClearGroupIfo()
 ATTENTION: __________
 */
static void mvSureTrueGroup(obj_group * pgroup) {
	s32 i;
	s32 nDetNum = 0;

	/*if (1 == pgroup->nStateNum && pgroup->nDetState[0])
	{
	pgroup->nTruelyObj = 1;
	}
	else
	{
	mvClearGroupIfo(pgroup, 1);
	}*/

	
	if (GROUP_DETER_STATE_NUM == pgroup->nStateNum) {
		for (i = 0; i < GROUP_DETER_STATE_NUM; i++) {

			if (pgroup->nDetState[i]) {
				nDetNum++;
			}
		}

		if (nDetNum >= GROUP_DETER_STATE_NUM * 0.6) //3
				{
			pgroup->nTruelyObj = 1;
		} else {
			mvClearGroupIfo(pgroup, 1);
		}
	} else if (3 == pgroup->nStateNum) {
		for (i = 0; i < pgroup->nStateNum; i++) {

			if (pgroup->nDetState[i]) {
				nDetNum++;
			}
		}

		if (!nDetNum) {
			mvClearGroupIfo(pgroup, 1);
		}
	}
}

#endif

/*
Function process:
+ Remove the overlapped rects in pInPutParam.
Fan-in :
+ FCW_TRACK_MultieTrack()
Fan-out:
+ mvLapTrd()
ATTENTION: __________
*/
static void mvReInput(PortInput *pInPutParam) {
	s32 i, j;
	AdasRect *pSrcRec = 0;
	AdasRect *pDstRec = 0;

	for (i = 0; i < pInPutParam->nRecNum - 1; i++)
	{
		pSrcRec = pInPutParam->objRec + i;

		if (pSrcRec->width == 0)
			continue;

		for (j = i + 1; j < pInPutParam->nRecNum; j++)
		{
			pDstRec = pInPutParam->objRec + j;

			if (pDstRec->width == 0)
				continue;

			if ( mvLapTrd(pSrcRec, pDstRec, 0.7f) || mvLapTrd(pDstRec, pSrcRec, 0.7f) )
			{
				if ( pSrcRec->confidence < pDstRec->confidence )
				{
                    pSrcRec->width = 0;
				}
				else
				{
					pDstRec->width = 0;
				}
			}
		}
	}
}

/*
 Function process:
 + Get the resized image
 Fan-in :
 + mvGetScaleInPut()
 Fan-out:
 + N/A
 ATTENTION: __________
 */
static void mvResizeImg(const imgage *pSrcImg, imgage *pDstImg, const s32 nId) {
	s32 i, j;
	float32_t fScale;
	float32_t fSum;
	uint8_t *Ptr = 0;
	uint8_t *PSrc = 0;
	s32 width, height;
	s32 *arr_y = 0;
	s32 *arr_x = 0;

	width = pDstImg->nWid;
	height = pDstImg->nHig;

	arr_y = (s32 *) m_globlparam[nId].m_pXYCorners;

	arr_x = (s32 *) m_globlparam[nId].m_pXYNoMax;

	fScale = pSrcImg->nWid / (width + 0.001f);

	fSum = -fScale;
	for (j = 0; j < height; j++) {
		fSum += fScale;

		arr_y[j] = (s32) (fSum + 0.5f);

		if (j < width) {
			arr_x[j] = arr_y[j];
		}
	}

	if (width > height) {
		for (i = height; i < width; i++) {
			fSum += fScale;

			arr_x[i] = (s32) (fSum + 0.5f);
		}
	}

	for (j = 0; j < height; j++) {
		Ptr = pDstImg->ptr + width * j;

		PSrc = pSrcImg->ptr + pSrcImg->nWid * arr_y[j];

		for (i = 0; i < width; i++) {
			Ptr[i] = PSrc[arr_x[i]];
		}
	}
}

/*
 Function process:
 + Get the tracking tragets for single-scale tracking; Given the value of m_globlparam[nId].scaleInput
 Fan-in :
 + mvSingledScaleTrack()
 Fan-out:
 + mvResizeImg()
 ATTENTION: __________
 */
static void mvGetScaleInPut(const PortInput *pInPutParam, const s32 nId) {
	s32 nObjNum = 0;
	s32 i;
	imgage Srcgray, Dstgray;
	s32 nYBoothThresh = (s32)((pInPutParam->pOriGrayfram.nHig >> 1) * 0.75f);
	s32 nscale_shink_1_id_W = scale_shink_1_id_W;
	s32 nscale_shink_2_id_W = scale_shink_2_id_W;

	/* the minimum size target */
	if (nId == scale_shink_1_id) {
		m_globlparam[nId].scaleInput.pOriGrayfram = pInPutParam->pOriGrayfram;
		m_globlparam[nId].scaleInput.fzoom = 1;
		m_globlparam[nId].scaleInput.pGrayfram = pInPutParam->pOriGrayfram.ptr;

		for (i = 0; i < pInPutParam->nRecNum; i++) {
			if (pInPutParam->objRec[i].width < nscale_shink_1_id_W
					&& pInPutParam->objRec[i].width) {
				m_globlparam[nId].scaleInput.objRec[nObjNum] =
						pInPutParam->objRec[i];
				m_globlparam[nId].scaleInput.objRec[nObjNum].x =
						pInPutParam->objRec[i].x << 1;
				m_globlparam[nId].scaleInput.objRec[nObjNum].y =
						pInPutParam->objRec[i].y << 1;
				m_globlparam[nId].scaleInput.objRec[nObjNum].width =
						pInPutParam->objRec[i].width << 1;
				m_globlparam[nId].scaleInput.objRec[nObjNum].height =
						pInPutParam->objRec[i].height << 1;
				nObjNum++;
			}
		}
		m_globlparam[nId].scaleInput.nRecNum = nObjNum;
		m_globlparam[nId].scaleInput.nFramSeq = pInPutParam->nFramSeq;
		m_globlparam[nId].scaleInput.imgSize.width =
				pInPutParam->pOriGrayfram.nWid;
		m_globlparam[nId].scaleInput.imgSize.height =
				pInPutParam->pOriGrayfram.nHig;
		m_globlparam[nId].scaleInput.objTime = pInPutParam->objTime;
	} else if (nId == scale_shink_2_id)
	{
		m_globlparam[nId].scaleInput.pOriGrayfram = pInPutParam->pOriGrayfram;
		m_globlparam[nId].scaleInput.fzoom = 2;
		m_globlparam[nId].scaleInput.pGrayfram = pInPutParam->pGrayfram;

		for (i = 0; i < pInPutParam->nRecNum; i++) {
			if ((pInPutParam->objRec[i].width >= nscale_shink_1_id_W
					&& pInPutParam->objRec[i].width < nscale_shink_2_id_W)
					&& (pInPutParam->objRec[i].y + pInPutParam->objRec[i].height)
							< nYBoothThresh) {

				m_globlparam[nId].scaleInput.objRec[nObjNum] =
						pInPutParam->objRec[i];
				nObjNum++;
			}
		}
		m_globlparam[nId].scaleInput.nRecNum = nObjNum;
		m_globlparam[nId].scaleInput.nFramSeq = pInPutParam->nFramSeq;
		m_globlparam[nId].scaleInput.imgSize.width =
				pInPutParam->pOriGrayfram.nWid >> 1;
		m_globlparam[nId].scaleInput.imgSize.height =
				pInPutParam->pOriGrayfram.nHig >> 1;
		m_globlparam[nId].scaleInput.objTime = pInPutParam->objTime;

	} else if (nId == scale_shink_4_id) {
		Srcgray.nWid = pInPutParam->pOriGrayfram.nWid >> 1;
		Srcgray.nHig = pInPutParam->pOriGrayfram.nHig >> 1;
		Srcgray.ptr = pInPutParam->pGrayfram;

		Dstgray.nWid = pInPutParam->pOriGrayfram.nWid >> 2;
		Dstgray.nHig = pInPutParam->pOriGrayfram.nHig >> 2;
		Dstgray.ptr = m_globlparam[nId].scaleInput.pGrayfram;

		m_globlparam[nId].scaleInput.pOriGrayfram = pInPutParam->pOriGrayfram;
		m_globlparam[nId].scaleInput.fzoom = 4;

		for (i = 0; i < pInPutParam->nRecNum; i++) {
			if (pInPutParam->objRec[i].width >= nscale_shink_2_id_W
					|| ((pInPutParam->objRec[i].y
							+ pInPutParam->objRec[i].height) >= nYBoothThresh
							&& pInPutParam->objRec[i].width)) {
				m_globlparam[nId].scaleInput.objRec[nObjNum] =
						pInPutParam->objRec[i];
				m_globlparam[nId].scaleInput.objRec[nObjNum].x =
						pInPutParam->objRec[i].x >> 1;
				m_globlparam[nId].scaleInput.objRec[nObjNum].y =
						pInPutParam->objRec[i].y >> 1;
				m_globlparam[nId].scaleInput.objRec[nObjNum].width =
						pInPutParam->objRec[i].width >> 1;
				m_globlparam[nId].scaleInput.objRec[nObjNum].height =
						pInPutParam->objRec[i].height >> 1;
				nObjNum++;
			}
		}

		if (nObjNum) {
			mvResizeImg(&Srcgray, &Dstgray, nId);
		} else {
			for (i = 0; i < MAX_LAYER_OBJ_GROUP_NUMS; i++) {
				if (-1 != m_globlparam[nId].m_pGroupSets[i].nGroupId) {
					mvResizeImg(&Srcgray, &Dstgray, nId);
					break;
				}
			}
		}

		m_globlparam[nId].scaleInput.nRecNum = nObjNum;
		m_globlparam[nId].scaleInput.nFramSeq = pInPutParam->nFramSeq;
		m_globlparam[nId].scaleInput.imgSize.width = Dstgray.nWid;
		m_globlparam[nId].scaleInput.imgSize.height = Dstgray.nHig;
		m_globlparam[nId].scaleInput.objTime = pInPutParam->objTime;
	}
	m_globlparam[nId].m_pGrayData = m_globlparam[nId].scaleInput.pGrayfram;
}


/*
 Function process:
 + find m_globlparam[nId].m_pGroupSets->nGroupId != -1; change m_globlparam[nId].m_GroupIndexNum
 and m_globlparam[nId].m_pGroupIndex
 Fan-in :
 + mvSingledScaleTrack()
 Fan-out:
 + N/A
 ATTENTION: __________
 */
static void mvGetCurrenGroupIndex(const s32 nId) {
	s32 i;
	obj_group *pgroup = 0;
	m_globlparam[nId].m_GroupIndexNum = 0;

	for (i = 0; i < MAX_LAYER_OBJ_GROUP_NUMS; i++) {
		pgroup = m_globlparam[nId].m_pGroupSets + i;

		if (-1 != pgroup->nGroupId) {
			m_globlparam[nId].m_pGroupIndex[m_globlparam[nId].m_GroupIndexNum++] =
					i;
		}
	}
}

/*
 Function process:
 + Add new tracking target based on the detected result.update m_globlparam.m_PNewRec
 Fan-in :
 + mvSingledScaleTrack()
 Fan-out:
 + mvLapTrd()
 ATTENTION: __________
 */
static void mvAddNewObjRec(const s32 nId) {

	s32 i, j;
	uint8_t bLap = 0;
	obj_group *pGroup = 0;
	s32 nDetNum = m_globlparam[nId].scaleInput.nRecNum;
	m_globlparam[nId].m_NewRecNum = 0;

	for (i = 0; i < nDetNum; i++) {
		for (j = 0; j < m_globlparam[nId].m_GroupIndexNum; j++) {
			pGroup = m_globlparam[nId].m_pGroupSets
					+ m_globlparam[nId].m_pGroupIndex[j];

			if (mvLapTrd(m_globlparam[nId].scaleInput.objRec + i, &pGroup->rtContour, 0.7f)
					|| mvLapTrd(&pGroup->rtContour, m_globlparam[nId].scaleInput.objRec + i,
							0.7f)) {
				bLap = 1;
				pGroup->nTruelyObj = 1;
				break;
			} else {
				bLap = 0;
			}
		}

		if (!bLap) {
			m_globlparam[nId].m_PNewRec[m_globlparam[nId].m_NewRecNum++] =
					m_globlparam[nId].scaleInput.objRec[i];
		}
	}
}

/*
 Function process:
 + predict the possible region of target targect based on points and target size in last frame.
 Fan-in :
 + mvGoupPreditRec()
 Fan-out:
 + N/A
 ATTENTION: __________
 */
static AdasRect mvMatchPreditRect(const AdasPoint *ptPredict,
		const AdasRect GroupRec, const int nMatchLeng) {
	s32 nExtandX;
	s32 nExtandY;
	AdasRect rt;
	const s32 nMinExTend = 8;
	s32 nMaxExTend = 15;

	if (nMatchLeng < 3) {
		nMaxExTend = 20;
	}

	nExtandX = GroupRec.width >> 2;
	nExtandY = GroupRec.height >> 2;

	if (nExtandX < nMinExTend) {
		nExtandX = nMinExTend;
	}

	nExtandX = WS_MIN(nExtandX, nMaxExTend);

	if (nExtandY < nMinExTend) {
		nExtandY = nMinExTend;
	}

	nExtandY = WS_MIN(nExtandY,nMaxExTend);

	rt.x = ptPredict->x - (nExtandX >> 1);
	rt.width = nExtandX;

	rt.y = ptPredict->y - (nExtandY >> 1);
	rt.height = nExtandY;

	return rt;

}

/*
 Function process:
 + update pgroup->PreditRec
 Fan-in :
 + mvSingledScaleTrack()
 Fan-out:
 + mvPredictTrack()
 + mvMatchPreditRect()
 ATTENTION: __________
 */
static void mvGoupPreditRec(const s32 nId) {
	s32 i, j;
	obj_group *pgroup = 0;
	trajecy *pTrajec = 0;
	AdasPoint TopLef, BotmRig;
	float fRio = 0.45f;

	if (scale_shink_2_id == nId) {
		fRio = 0.4f;
	} else if (scale_shink_4_id == nId) {
		fRio = 0.45f;
	}

	for (i = 0; i < m_globlparam[nId].m_GroupIndexNum; i++) {
		pgroup = m_globlparam[nId].m_pGroupSets
				+ m_globlparam[nId].m_pGroupIndex[i];

		TopLef.x = m_globlparam[nId].m_ImgWidth;
		TopLef.y = m_globlparam[nId].m_ImgHeight;
		BotmRig.x = 0;
		BotmRig.y = 0;

		for (j = 0; j < pgroup->ntrajecyNum; j++) {
			pTrajec = pgroup->pObjtr + j;

			mvPredictTrack(pTrajec, m_globlparam[nId].m_ImgSize,
					m_globlparam[nId].scaleInput.nFramSeq);

			pTrajec->pRectPredict = mvMatchPreditRect(&pTrajec->ptPredict,
					pgroup->rtContour, pTrajec->nTrackLen);

			TopLef.x = WS_MIN(TopLef.x, pTrajec->pRectPredict.x);
			TopLef.y = WS_MIN(TopLef.y, pTrajec->pRectPredict.y);
			BotmRig.x =
					WS_MAX(BotmRig.x, pTrajec->pRectPredict.x + pTrajec->pRectPredict.width);
			BotmRig.y =
					WS_MAX(BotmRig.y, pTrajec->pRectPredict.y + pTrajec->pRectPredict.height);

		}

		mvPredictTrack(&pgroup->Centr, m_globlparam[nId].m_ImgSize,
   				m_globlparam[nId].scaleInput.nFramSeq);

		TopLef.x =
				(s32) WS_MIN(TopLef.x, pgroup->Centr.ptPredict.x - fRio * pgroup->rtContour.width);
		TopLef.y =
				(s32) WS_MIN(TopLef.y, pgroup->Centr.ptPredict.y - fRio * pgroup->rtContour.height);
		BotmRig.x =
				(s32) WS_MAX(BotmRig.x, pgroup->Centr.ptPredict.x + fRio * pgroup->rtContour.width);
		BotmRig.y =
				(s32) WS_MAX(BotmRig.y, pgroup->Centr.ptPredict.y + fRio * pgroup->rtContour.height);

		pgroup->PreditRec = adasrect(TopLef.x, TopLef.y, BotmRig.x - TopLef.x,
				BotmRig.y - TopLef.y);

		mvScopeImg(&pgroup->PreditRec,
				adasrect(0, 0, m_globlparam[nId].m_ImgWidth,
						m_globlparam[nId].m_ImgHeight));

	}
}



/*
 Function process:
 + update m_globlparam[nId].m_pMask based on pInPutParam->objRec and m_globlparam[nId].m_pGroupSets
 Fan-in :
 + mvSingledScaleTrack()
 Fan-out:
 + GetVerticlBotoomLine()
 ATTENTION: __________
 */
static void mvGenerateRioGrayByMask(const s32 nId) {
	s32 i, j;
	s32 m;
	AdasRect *prec = 0;
	uint8_t *pData = 0;
	obj_group *pGroup = 0;
	s32 nLength;
	s32 nRigTrd = m_globlparam[nId].m_ImgWidth - 1;
	m_globlparam[nId].nscope_start_y = m_globlparam[nId].m_ImgHeight;
	m_globlparam[nId].nscope_end_y = 0;
	m_globlparam[nId].nscope_start_x = m_globlparam[nId].m_ImgWidth;
	m_globlparam[nId].nscope_end_x = 0;

	if (m_globlparam[nId].scaleInput.nRecNum == 0 && m_globlparam[nId].m_GroupIndexNum == 0)
		return;

	for (i = 0; i < m_globlparam[nId].scaleInput.nRecNum; i++) {
		prec = m_globlparam[nId].scaleInput.objRec + i;
		m_globlparam[nId].nscope_start_y =
			WS_MIN(prec->y - (prec->height >> 2) ,m_globlparam[nId].nscope_start_y);
		m_globlparam[nId].nscope_end_y =
			WS_MAX( prec->y + prec->height + (prec->height >> 2) ,m_globlparam[nId].nscope_end_y);

		m_globlparam[nId].nscope_start_x =
			WS_MIN(prec->x - (prec->width >> 2),m_globlparam[nId].nscope_start_x);
		m_globlparam[nId].nscope_end_x =
			WS_MAX( prec->x + prec->width + (prec->width >> 2) ,m_globlparam[nId].nscope_end_x);
	}

	for (i = 0; i < m_globlparam[nId].m_GroupIndexNum; i++) {
		pGroup = m_globlparam[nId].m_pGroupSets
			+ m_globlparam[nId].m_pGroupIndex[i];
		prec = &(pGroup->rtContour);

		m_globlparam[nId].nscope_start_y =
			WS_MIN(prec->y - (prec->height >> 2),m_globlparam[nId].nscope_start_y);
		m_globlparam[nId].nscope_end_y =
			WS_MAX(prec->y + prec->height + (prec->height >> 2) ,m_globlparam[nId].nscope_end_y);

		m_globlparam[nId].nscope_start_x =
			WS_MIN(prec->x - (prec->width >> 2) ,m_globlparam[nId].nscope_start_x);
		m_globlparam[nId].nscope_end_x =
			WS_MAX( prec->x + prec->width + (prec->width >> 2) ,m_globlparam[nId].nscope_end_x);

	}

	m_globlparam[nId].nscope_start_x =
		WS_MAX(0, m_globlparam[nId].nscope_start_x );
	m_globlparam[nId].nscope_end_x =
		WS_MIN(m_globlparam[nId].m_ImgWidth - 1,m_globlparam[nId].nscope_end_x);

	m_globlparam[nId].nscope_start_y =
		WS_MAX(0, m_globlparam[nId].nscope_start_y );
	m_globlparam[nId].nscope_end_y =
		WS_MIN(m_globlparam[nId].m_ImgHeight - 1,m_globlparam[nId].nscope_end_y);

	if ( m_globlparam[nId].nscope_end_x < m_globlparam[nId].nscope_start_x || m_globlparam[nId].nscope_end_y < m_globlparam[nId].nscope_start_y )
	{
		return;
	}

	//
	memset(
		m_globlparam[nId].m_pMask
		+ m_globlparam[nId].nscope_start_y
		* m_globlparam[nId].m_ImgWidth, 0,
		m_globlparam[nId].m_ImgWidth
		* (m_globlparam[nId].nscope_end_y
		- m_globlparam[nId].nscope_start_y + 1));

	// initialize PreditRec area value to be 255.
	if (scale_shink_4_id == nId || scale_shink_2_id == nId) {
		for (i = 0; i < m_globlparam[nId].scaleInput.nRecNum; i++) {
			prec = m_globlparam[nId].scaleInput.objRec + i;
			nLength = prec->width;

			for (m = WS_MAX(0, prec->y - (prec->height >> 3)); m < WS_MIN(m_globlparam[nId].m_ImgHeight - 1, prec->y + prec->height + (prec->height >> 3));
				m++) {
					int memSize;
					j = WS_MAX( 0,  ADAS_ALIGN_16BYTE_SIZE(prec->x - (prec->width >> 3) - 16) );

					memSize = WS_MAX(0, WS_MIN( m_globlparam[nId].m_ImgWidth - 1,  (prec->x + prec->width + (prec->width >> 3)) ) - j );
					pData = m_globlparam[nId].m_pMask
						+ m_globlparam[nId].m_ImgWidth * m + j;

					memset(pData, 255, ADAS_ALIGN_16BYTE_SIZE(memSize));
			}
		}

	} else {
		for (m = WS_MAX(0, prec->y - (prec->height >> 2)); m < WS_MIN(m_globlparam[nId].m_ImgHeight - 1, prec->y + prec->height + (prec->height >> 2));
			m++) {
				int memSize;
				j = WS_MAX( 0,  ADAS_ALIGN_16BYTE_SIZE(prec->x - (prec->width >> 2) - 16) );

				memSize = WS_MAX(0, WS_MIN( m_globlparam[nId].m_ImgWidth - 1,  (prec->x + prec->width + (prec->width >> 2)) ) - j );
				pData = m_globlparam[nId].m_pMask
					+ m_globlparam[nId].m_ImgWidth * m + j;

				memset(pData, 255, ADAS_ALIGN_16BYTE_SIZE(memSize));
		}
	}

	for (i = 0; i < m_globlparam[nId].m_GroupIndexNum; i++) {
		pGroup = m_globlparam[nId].m_pGroupSets
			+ m_globlparam[nId].m_pGroupIndex[i];
		prec = &(pGroup->PreditRec);

		if (scale_shink_4_id == nId || scale_shink_2_id == nId) {
			for (m = WS_MAX(0, prec->y - (prec->height >> 3)); m < WS_MIN(m_globlparam[nId].m_ImgHeight - 1, prec->y + prec->height + (prec->height >> 3));
				m++) {
					int memSize;
					j = WS_MAX( 0,  ADAS_ALIGN_16BYTE_SIZE(prec->x - (prec->width >> 3) - 16) );

					memSize = WS_MAX(0, WS_MIN( m_globlparam[nId].m_ImgWidth - 1,  (prec->x + prec->width + (prec->width >> 3)) ) - j );
					pData = m_globlparam[nId].m_pMask
						+ m_globlparam[nId].m_ImgWidth * m + j;

					memset(pData, 255, ADAS_ALIGN_16BYTE_SIZE(memSize));
			}
		} else {
			for (m = WS_MAX(0, prec->y - (prec->height >> 2)); m < WS_MIN(m_globlparam[nId].m_ImgHeight - 1, prec->y + prec->height + (prec->height >> 2));
				m++) {
					int memSize;
					j = WS_MAX( 0,  ADAS_ALIGN_16BYTE_SIZE(prec->x - (prec->width >> 2) - 16) );

					memSize = WS_MAX(0, WS_MIN( m_globlparam[nId].m_ImgWidth - 1,  (prec->x + prec->width + (prec->width >> 2)) ) - j );
					pData = m_globlparam[nId].m_pMask
						+ m_globlparam[nId].m_ImgWidth * m + j;

					memset(pData, 255, ADAS_ALIGN_16BYTE_SIZE(memSize));
			}
		}
	}
}

/*
 Function process:
 + extract the corner points in mask region, the result saved in m_globlparam.m_nCornerPassNum and m_globlparam.m_nFastCornerNum
 Fan-in :
 + mvSingledScaleTrack()
 Fan-out:
 + oast9_16()
 + fast_nonmax()
 + CalculateHarrisResponse()
 + CornerResponseRestrain()
 ATTENTION: __________
 */
static void mvCornerDetct(const uint8_t *pGray, const uint8_t *pMask,
		const s32 width, const s32 height, const s32 Barrier, const s32 nId) {
	xy *corners = 0;
	xy *corners_nonmax = 0;
	s32 num_corners = 0;
	s32 num_nonmax = 0;
	AdaSize size;
	s32 i;

	m_globlparam[nId].m_nCornerPassNum = 0;
	m_globlparam[nId].m_nFastCornerNum = 0;

#ifdef TIME_TEST
	double ts = cvGetTickCount();
#endif

#ifdef MV_ADAS_USE_FAST_STATIC

#ifdef  FAST_OAST9_16
	oast9_16(pGray, pMask, width, height, Barrier, &num_corners,
			m_globlparam[nId].m_pXYCorners, nId,
			m_globlparam[nId].nscope_start_y, m_globlparam[nId].nscope_end_y,
			m_globlparam[nId].nscope_start_x, m_globlparam[nId].nscope_end_x);
#else
	corners = fast_corner_detect_9(pGray,width, height, Barrier, &num_corners,m_globlparam.m_pXYCorners);

#endif
	corners = m_globlparam[nId].m_pXYCorners;

#else
	corners = fast_corner_detect_9(pGray,width, height, Barrier, &num_corners,0);

#endif

#ifdef TIME_TEST
	my_printf("fast_corner_detect_9:%0.3fms\n",(cvGetTickCount()-ts)/cvGetTickFrequency() /1000.0f);
	ts = cvGetTickCount();
#endif

#ifdef MV_ADAS_USE_FAST_STATIC
	corners_nonmax = fast_nonmax(pGray, width, height, corners, num_corners,
			Barrier, &num_nonmax, m_globlparam[nId].m_pRowStart,
			m_globlparam[nId].m_pScore, m_globlparam[nId].m_pXYNoMax);
#else
	corners_nonmax = fast_nonmax(pGray, width,height, corners, num_corners,Barrier, &num_nonmax,0,0,0);
#endif

#ifdef TIME_TEST
	my_printf("fast_nonmax:%0.3fms\n",(cvGetTickCount()-ts)/cvGetTickFrequency() /1000.0f);
	ts = cvGetTickCount();
#endif

	if (num_nonmax < MAX_CORNERS_PER_FRAME) {
		for (i = 0; i < num_nonmax; ++i) {
			m_globlparam[nId].m_pFastCorner[m_globlparam[nId].m_nFastCornerNum].x =
					corners_nonmax[i].x;
			m_globlparam[nId].m_pFastCorner[m_globlparam[nId].m_nFastCornerNum].y =
					corners_nonmax[i].y;
			m_globlparam[nId].m_pFastCorner[m_globlparam[nId].m_nFastCornerNum].State.nMacthNum =
					0;
			m_globlparam[nId].m_pFastCorner[m_globlparam[nId].m_nFastCornerNum++].State.nMatchDis =
					0;
		}

		return;
	}

	for (i = 0; i < num_nonmax; ++i) {
		if (m_globlparam[nId].m_nCornerPassNum >= MAX_CORNERS_OF_NONMAX - 1) {
			break;
		}
		m_globlparam[nId].m_pCornerPass[m_globlparam[nId].m_nCornerPassNum].x =
				corners_nonmax[i].x;
		m_globlparam[nId].m_pCornerPass[m_globlparam[nId].m_nCornerPassNum].y =
				corners_nonmax[i].y;
		m_globlparam[nId].m_pCornerPass[m_globlparam[nId].m_nCornerPassNum].State.nMacthNum =
				0;
		m_globlparam[nId].m_pCornerPass[m_globlparam[nId].m_nCornerPassNum++].State.nMatchDis =
				0;

	}

#ifndef MV_ADAS_USE_FAST_STATIC
	my_free(corners_nonmax);
	my_free(corners);
#endif

	size.width = width;
	size.height = height;
	CalculateHarrisResponse(pGray, size, m_globlparam[nId].m_pCornerPass,
			m_globlparam[nId].m_nCornerPassNum);

#ifdef TIME_TEST
	my_printf("CalculateHarrisResponse:%0.3fms\n",(cvGetTickCount()-ts)/cvGetTickFrequency() /1000.0f);
	ts = cvGetTickCount();
#endif

	CornerResponseRestrain(m_globlparam[nId].m_pCornerPass,
			m_globlparam[nId].m_pFastCorner, m_globlparam[nId].m_nCornerPassNum,
			&m_globlparam[nId].m_nFastCornerNum, MAX_CORNERS_PER_FRAME);

#ifdef TIME_TEST
	my_printf("CornerResponseRestrain:%0.3fms\n",(cvGetTickCount()-ts)/cvGetTickFrequency() /1000.0f);
	ts = cvGetTickCount();
#endif

}

/*
 Function process:
 + caculate the surf features (m_globlparam[nId].m_pfeature) of fast points(m_globlparam[nId].m_pFastCorner)
 Fan-in :
 + mvSingledScaleTrack()
 Fan-out:
 + mvComputeSurfDescriptor()
 ATTENTION: __________
 */
static void mvFeatrueDescribe(uint8_t *pGray, AdaSize size, s32 nId) {
	mvComputeSurfDescriptor(pGray, size, m_globlparam[nId].m_pFastCorner,
			m_globlparam[nId].m_nFastCornerNum, m_globlparam[nId].m_pfeature);

}

/*************************************************************
 Function process:
 + match the traject point with the fast points and update m_globlparam[nId].m_pGroupSets->pObjtr->pTrackPoint
 Fan-in :
 + mvSingledScaleTrack()
 Fan-out:
 + mvMatchTrackPoint()
 + mvDelUnReasonTrack()
 ATTENTION: __________
 */
static void mvTrajectorymatch(s32 nId) {
	s32 i, j;
	obj_group *pgroup = 0;
	trajecy *pTrajec = 0;
	TrackPoint *pMacthTrackPoint = 0;
	//trajecy *pPreMacthTrack = 0;
	AdasCorner *pMacthCorner = 0;
	s32 bMatch, nMathcMinDis;
	s32 nMatchCornIndex = -1;
	//const s32 nPushNum = 45;
	//TrackPoint *pCurPoin = 0;
	//TrackPoint *pPrePoin =0;

	for (i = 0; i < m_globlparam[nId].m_GroupIndexNum; i++) {
		pgroup = m_globlparam[nId].m_pGroupSets
				+ m_globlparam[nId].m_pGroupIndex[i];

#ifdef  RET_DS

		if (pgroup->nDetDiscardFam == m_globlparam[nId].scaleInput.nFramSeq - 1
			&& pgroup->nDisCardNum < 4 )
		{
			continue;
		}
#endif

		for (j = 0; j < pgroup->ntrajecyNum; j++) {

			pTrajec = pgroup->pObjtr + j;

			bMatch = mvMatchTrackPoint(pTrajec, m_globlparam[nId].m_pfeature,
					m_globlparam[nId].m_pFastCorner,
					m_globlparam[nId].m_nFastCornerNum, &nMatchCornIndex,
					&nMathcMinDis, nId);

			if (bMatch) {
				pMacthCorner = m_globlparam[nId].m_pFastCorner
						+ nMatchCornIndex;

				if (!pMacthCorner->State.nMacthNum) {
					pMacthCorner->State.nGroupIndex[0] =
							m_globlparam[nId].m_pGroupIndex[i];
					pMacthCorner->State.nTrjIndexofGroup[0] = j;
					pMacthCorner->State.nMatchDis = nMathcMinDis;
					pMacthCorner->State.nMacthNum++;
				} else {
					if (nMathcMinDis < pMacthCorner->State.nMatchDis) {
						pMacthCorner->State.nGroupIndex[0] =
								m_globlparam[nId].m_pGroupIndex[i];
						pMacthCorner->State.nTrjIndexofGroup[0] = j;
						pMacthCorner->State.nMatchDis = nMathcMinDis;

					}
				}

			}

			pMacthTrackPoint = pTrajec->pTrackPoint
					+ (pTrajec->nTrackLen & MAX_CORNER_OF_TRACK_BIT);
			pMacthTrackPoint->nFramseq = m_globlparam[nId].scaleInput.nFramSeq;
			pMacthTrackPoint->nTime = m_globlparam[nId].scaleInput.objTime;
			pMacthTrackPoint->nMatchStatus = 0;
			pMacthTrackPoint->point = pTrajec->ptPredict;

			pTrajec->nTrackLen++;
			pTrajec->nEstTimes++;

		}

	}

	for (i = 0; i < m_globlparam[nId].m_nFastCornerNum; i++) {
		pMacthCorner = m_globlparam[nId].m_pFastCorner + i;

		if (1 == pMacthCorner->State.nMacthNum) {
			pgroup = m_globlparam[nId].m_pGroupSets
					+ pMacthCorner->State.nGroupIndex[0];
#ifdef  RET_DS
			if (pgroup->nDetDiscardFam == m_globlparam[nId].scaleInput.nFramSeq - 1
				&& pgroup->nDisCardNum < 4 )
			{
				continue;
			}
#endif


			pTrajec = pgroup->pObjtr + pMacthCorner->State.nTrjIndexofGroup[0];

			pMacthTrackPoint = pTrajec->pTrackPoint
					+ ((pTrajec->nTrackLen - 1) & MAX_CORNER_OF_TRACK_BIT);
			pMacthTrackPoint->nMatchStatus = 1;
			pMacthTrackPoint->point = adaspoint(pMacthCorner->x,
					pMacthCorner->y);
			pTrajec->nEstTimes = 0;
			//memcpy(pTrajec->pfeature,m_globlparam.m_pfeature + SURF_DESC_DIMENTION * i,SURF_DESC_DIMENTION);
			memcpy(pTrajec->pfeature, m_globlparam[nId].m_pfeature + (i << 6),
					SURF_DESC_DIMENTION);

		}
	}

	for (i = 0; i < m_globlparam[nId].m_GroupIndexNum; i++) {
		pgroup = m_globlparam[nId].m_pGroupSets
				+ m_globlparam[nId].m_pGroupIndex[i];

		mvDelUnReasonTrack(pgroup);

	}
}

/*
 Function process:
 + Get the car temple img based on ROI region of RioRec
 Fan-in :
 + mvMatchDetRec()
 Fan-out:
 + N/A
 ATTENTION: __________
 */
static void mvSetUpdataCarImg(AdasRect RioRec, imgage *pCarimg, s32 nId) {
	s32 i, j;
	//uint8_t *pDstr = 0;
	//uint8_t *pSrctr = 0;
	float32_t fx_zoom, fy_zoom;
	uint8_t *pDst = 0;
	s32 nWidth = m_globlparam[nId].scaleInput.pOriGrayfram.nWid >> nId;

	fx_zoom = ((float32_t) RioRec.width) / UPDATA_IMAGE_SIZE;
	fy_zoom = ((float32_t) RioRec.height) / UPDATA_IMAGE_SIZE;

	for (j = 0; j < UPDATA_IMAGE_SIZE; j++) {
		pDst = pCarimg->ptr + j * UPDATA_IMAGE_SIZE;

		for (i = 0; i < UPDATA_IMAGE_SIZE; i++) {
			pDst[i] = *(m_globlparam[nId].scaleInput.pGrayfram
					+ ((s32) (j * fy_zoom) + RioRec.y) * nWidth
					+ ((s32) (i * fx_zoom) + RioRec.x));
		}
	}
}

/*
 Function process:
 + caculate the integral image of ROI of srcimg
 Fan-in :
 + mvTemplatMatch()
 Fan-out:
 + N/A
 ATTENTION: __________
 */
static void mvInterlimg(const imgage *srcimg, imgage*IntelImg,
		AdasPoint pointRio) {
	s32 i, j;
	s32 nColSum;
	s32 * pInterlTr = 0;
	uint8_t * pTr = 0;

	for (j = 0; j < IntelImg->nHig - 1; j++) {
		nColSum = 0;

		pTr = srcimg->ptr + srcimg->nWid * (j + pointRio.y);

		pInterlTr = (s32*) IntelImg->ptr + IntelImg->nWid * (j + 1);

		for (i = 0; i < IntelImg->nWid - 1; i++) {
			nColSum += pTr[i + pointRio.x];
			pInterlTr[i + 1] = nColSum;
		}
	}

	for (i = 1; i < IntelImg->nWid; i++) {
		for (j = 2; j < IntelImg->nHig; j++) {
			pInterlTr = (s32*) IntelImg->ptr + IntelImg->nWid * j;
			pInterlTr[i] += pInterlTr[i - IntelImg->nWid];
		}
	}
}

/*
 Function process:
 + Get the inteval value of ROI based on the interval image
 Fan-in :
 + mvTemplatMatch()
 Fan-out:
 + N/A
 ATTENTION: __________
 */
static s32 mvSumImgRio(imgage *IntelImg, AdasPoint lt, AdasPoint br) {
	s32 * pInterlTr = (s32*) IntelImg->ptr;

	s32 nSumVal = *(pInterlTr + IntelImg->nWid * lt.y + lt.x)
			+ *(pInterlTr + IntelImg->nWid * br.y + br.x)
			- *(pInterlTr + IntelImg->nWid * lt.y + br.x)
			- *(pInterlTr + IntelImg->nWid * br.y + lt.x);

	return nSumVal;
}

/*
 *************************************************************
 Function process:
 + Do the Temple matching
 Fan-in :
 + mvMatchDetRec()
 Fan-out:
 + mvInterlimg()
 + mvSumImgRio()
 ATTENTION: __________
 */
static uint8_t mvTemplatMatch(const imgage img, const imgage Tempimg,
		AdasRect SearcRec, AdasRect *pMatchRec, s32 nPubliBuffOff,
		uint8_t bComparSecd, s32 nId, float32_t *fMatchScore) {

#ifdef SHOW_TEMPLE
	IplImage *serchImg = Trans_Imgage_TO_cvIplImage(img);
	IplImage *TemplateImg = Trans_Imgage_TO_cvIplImage(Tempimg);
	cvShowImage("serchImg",serchImg);
	cvShowImage("TemplateImg",TemplateImg);
	cvWaitKey(10);
#endif

	imgage IntelImg;
	s32 m, n, i, j;
	s32 addresoff = 0;
	uint8_t* pTemplateptr = 0;
	s32* pdata = 0;
	float32_t* pSimlarTr = 0;
	imgage tempSubAvg;
	imgage MathchSimilar;
	s32 nTemplateAvg = 0;
	uint8_t* ptr = (uint8_t*) m_globlparam[nId].m_PublacSape + nPubliBuffOff;
	s32 powT = 0;
	s32 ntemplateSum = Tempimg.nHig * Tempimg.nWid;
	AdasPoint lt, br;
	s32 RioSubAvgSum = 0;
	float32_t fMatchVal = 0.0f;
	s32 nAvg;
	uint8_t *pMatchRio = 0;
	float32_t fMaxMatchSim = 0.0f;
	float32_t fSecdMaxMatchSim = 0.0f;
	//float32_t fMinMatchSim = 1.0f;
	AdasPoint point;
	s32 nMax_PublicSpace_byte = MAX_PUBLIC_SPACE_SIZE * sizeof(s32);
	*fMatchScore = 0;

	IntelImg.ptr = ptr;

	if (SearcRec.width < Tempimg.nWid || SearcRec.height < Tempimg.nHig
			|| ntemplateSum < 10) {
		return 0;
	}

	IntelImg.nWid = SearcRec.width + 1;
	IntelImg.nHig = SearcRec.height + 1;
	memset(IntelImg.ptr, 0, IntelImg.nWid * IntelImg.nHig * sizeof(s32));

	addresoff +=
			ADAS_ALIGN_16BYTE_SIZE(IntelImg.nWid * IntelImg.nHig * sizeof(s32));
	ptr += addresoff;

	if (addresoff + nPubliBuffOff > nMax_PublicSpace_byte) {
		return 0;
	}

	mvInterlimg(&img, &IntelImg, adaspoint(SearcRec.x, SearcRec.y));

	tempSubAvg.ptr = ptr;
	tempSubAvg.nWid = Tempimg.nWid;
	tempSubAvg.nHig = Tempimg.nHig;
	addresoff +=
			ADAS_ALIGN_16BYTE_SIZE(Tempimg.nWid * Tempimg.nHig * sizeof(s32));
	ptr += addresoff;

	MathchSimilar.ptr = ptr;
	MathchSimilar.nWid = SearcRec.width - Tempimg.nWid;
	MathchSimilar.nHig = SearcRec.height - Tempimg.nHig;
	addresoff +=
			ADAS_ALIGN_16BYTE_SIZE(MathchSimilar.nWid * MathchSimilar.nHig * sizeof(float32_t));

	if (addresoff + nPubliBuffOff > nMax_PublicSpace_byte) {
		return 0;
	}

	for (m = 0; m < Tempimg.nHig; m++) {
		pTemplateptr = Tempimg.ptr + Tempimg.nWid * m;

		for (n = 0; n < Tempimg.nWid; n++) {
			nTemplateAvg += pTemplateptr[n];
		}
	}

	nTemplateAvg /= ntemplateSum;

	for (m = 0; m < tempSubAvg.nHig; m++) {
		pdata = (s32*) tempSubAvg.ptr + tempSubAvg.nWid * m;

		pTemplateptr = Tempimg.ptr + Tempimg.nWid * m;

		for (n = 0; n < tempSubAvg.nWid; n++) {
			pdata[n] = pTemplateptr[n] - nTemplateAvg;
			powT += pdata[n] * pdata[n];
		}
	}

	pMatchRec->width = Tempimg.nWid;
	pMatchRec->height = Tempimg.nHig;

	for (m = SearcRec.y; m <= SearcRec.y + SearcRec.height - Tempimg.nHig;
			m++) {
		for (n = SearcRec.x; n <= SearcRec.x + SearcRec.width - Tempimg.nWid;
				n++) {
			lt = adaspoint(n - SearcRec.x, m - SearcRec.y);

			br = adaspoint(n - SearcRec.x + Tempimg.nWid /*-1 + 1*/,
					m - SearcRec.y + Tempimg.nHig);

			nAvg = mvSumImgRio(&IntelImg, lt, br);

			nAvg = nAvg / ntemplateSum;

			RioSubAvgSum = 0;
			fMatchVal = 0.0f;

			for (j = 0; j < tempSubAvg.nHig; j++) {
				pdata = (s32*) tempSubAvg.ptr + tempSubAvg.nWid * j;

				pMatchRio = img.ptr + img.nWid * (m + j);

				for (i = 0; i < tempSubAvg.nWid; i++) {
					RioSubAvgSum += (pMatchRio[i + n] - nAvg)
							* (pMatchRio[i + n] - nAvg);
					fMatchVal += pdata[i] * (pMatchRio[i + n] - nAvg);
				}
			}

			fMatchVal = fMatchVal / sqrtf((float32_t) powT * RioSubAvgSum);

			pSimlarTr = (float32_t*) MathchSimilar.ptr
					+ MathchSimilar.nWid * (s32) (lt.y);
			pSimlarTr[(s32) (lt.x)] = fMatchVal;

			if (fMatchVal > fMaxMatchSim) {
				pMatchRec->x = n;
				pMatchRec->y = m;
				fMaxMatchSim = fMatchVal;

			}

		}
	}

	*fMatchScore = fMaxMatchSim;

	if (!bComparSecd) {
		if (fMaxMatchSim > 0.8f) {
			return 1;
		}

		return 0;
	}

	for (m = 0; m < MathchSimilar.nHig; m++) {
		pSimlarTr = (float32_t*) MathchSimilar.ptr + MathchSimilar.nWid * m;

		for (n = 0; n < MathchSimilar.nWid; n++) {
			point = adaspoint(n + pMatchRec->x, m + pMatchRec->y);

			if (mvAdaspointOutRect(&point, pMatchRec)) {
				if (pSimlarTr[n] > fSecdMaxMatchSim) {
					fSecdMaxMatchSim = pSimlarTr[n];
				}
			}
		}
	}


	if (fMaxMatchSim > 0.8f && (fMaxMatchSim - fSecdMaxMatchSim) > 0.1f) {
		return 1;
	}

	return 0;

}

/*
 Function process:
 + match the detect region in pGroup if matched return 1; else return 0
 Fan-in :
 + mvUpdataByDetctor()
 Fan-out:
 + mvSetUpdataCarImg()
 + mvTemplatMatch()
 ATTENTION: __________
 */
static s32 mvMatchDetRec(const obj_group *pGroup, AdasRect *pLapCarRec, s32 nId) {

	s32 i, j;
	s32 nMatchIndex = -1;
	AdasRect *pCarRec;
	AdasRect GroupRec, CarRec;
	s32 nDisTance;
	PortInput *oriPut = m_globlparam[nId].pOriInPutParam; //d都与1/4压缩图像上信息进行融合
	imgage Carimg;
	float32_t fMatchScore;
	float32_t fAvgScore = 0.0f;
	AdasRect MatchRec;
	s32 noff = UPDATA_IMAGE_SIZE * UPDATA_IMAGE_SIZE;
	Carimg.nWid = UPDATA_IMAGE_SIZE;
	Carimg.nHig = UPDATA_IMAGE_SIZE;
	Carimg.ptr = (uint8_t*) m_globlparam[nId].m_PublacSape;

	if (scale_shink_1_id == nId)
	{
		for (i = 0; i < oriPut->nRecNum; i++) 
		{
			pCarRec = oriPut->objRec + i;
			if (pCarRec->width == 0)
				continue;

			CarRec.x = pCarRec->x << 1;
			CarRec.y = pCarRec->y << 1;
			CarRec.width = pCarRec->width << 1;
			CarRec.height = pCarRec->height << 1;
			CarRec.confidence = pCarRec->confidence;
			pCarRec = &CarRec;
			GroupRec = (pGroup->rtContour);

			nDisTance = WS_ABS(GroupRec.x + (GroupRec.width>>1) - \
				pCarRec->x - (pCarRec->width>>1) ) +\
				WS_ABS(GroupRec.y + (GroupRec.height>>1) - \
				pCarRec->y - (pCarRec->height>>1) );

			if ( (mvLapTrd(pCarRec, &GroupRec, 0.7f)
				|| mvLapTrd(&GroupRec, pCarRec, 0.7f)) 
				&& pCarRec->confidence > GroupRec.confidence) 
			{
				if ( nDisTance < 0.4f * GroupRec.width  && ( pCarRec->width < 1.2f * GroupRec.width &&  pCarRec->width > 0.8f * GroupRec.width ) )
				{
					*pLapCarRec = CarRec;
					nMatchIndex = i;
					break;
				}
				else
				{
					mvSetUpdataCarImg(*pCarRec, &Carimg, nId);
					for (j = 0; j < WS_MIN(pGroup->nUpdataIndex, UPDATA_IMAGE_NUM); j++) 
					{

						mvTemplatMatch(pGroup->UpdatedImg[j], Carimg,
							adasrect(0, 0, UPDATA_IMAGE_SIZE,
							UPDATA_IMAGE_SIZE), &MatchRec, noff, 0,
							nId, &fMatchScore);
						fAvgScore += fMatchScore;
					}
					fAvgScore /= 
						WS_MIN(pGroup->nUpdataIndex,UPDATA_IMAGE_NUM);

					if (fAvgScore > 0.5f)
					{
						*pLapCarRec = CarRec;
						nMatchIndex = i;
						break;
					}

				}
			}
		}
	}
	else if (scale_shink_2_id == nId)
	{
		for (i = 0; i < oriPut->nRecNum; i++)
		{
			pCarRec = oriPut->objRec + i;
			if (pCarRec->width == 0)
				continue;
			GroupRec = (pGroup->rtContour);

			nDisTance = WS_ABS(GroupRec.x + (GroupRec.width>>1) - \
				pCarRec->x - (pCarRec->width>>1) ) +\
				WS_ABS(GroupRec.y + (GroupRec.height>>1) - \
				pCarRec->y - (pCarRec->height>>1) );

			if ( (mvLapTrd(pCarRec, &GroupRec, 0.7f)
				|| mvLapTrd(&GroupRec, pCarRec, 0.7f)) 
				&& pCarRec->confidence > GroupRec.confidence)
			{

				if ( nDisTance < 0.4f * GroupRec.width  && ( pCarRec->width < 1.2f * GroupRec.width &&  pCarRec->width > 0.8f * GroupRec.width ) )
				{
					*pLapCarRec = *pCarRec;
					nMatchIndex = i;
					break;
				}
				else
				{
					mvSetUpdataCarImg(*pCarRec, &Carimg, nId);
					for (j = 0; j < WS_MIN(pGroup->nUpdataIndex, UPDATA_IMAGE_NUM); j++) 
					{
						mvTemplatMatch(pGroup->UpdatedImg[j], Carimg,
							adasrect(0, 0, UPDATA_IMAGE_SIZE,
							UPDATA_IMAGE_SIZE), &MatchRec, noff, 0,
							nId, &fMatchScore);
						fAvgScore += fMatchScore;
					}
					fAvgScore /= 
						WS_MIN(pGroup->nUpdataIndex,UPDATA_IMAGE_NUM);

					if (fAvgScore > 0.5f)
					{
						*pLapCarRec = *pCarRec;
						nMatchIndex = i;
						break;
					}

				}
			}
		}
	} 
	else if (scale_shink_4_id == nId) 
	{
		for (i = 0; i < oriPut->nRecNum; i++)
		{
			pCarRec = oriPut->objRec + i;
			if (pCarRec->width == 0)
				continue;
			CarRec.x = pCarRec->x >> 1;
			CarRec.y = pCarRec->y >> 1;
			CarRec.width = pCarRec->width >> 1;
			CarRec.height = pCarRec->height >> 1;
			CarRec.confidence = pCarRec->confidence;
			pCarRec = &CarRec;
			GroupRec = (pGroup->rtContour);

			nDisTance = WS_ABS(GroupRec.x + (GroupRec.width>>1) - \
				pCarRec->x - (pCarRec->width>>1) ) +\
				WS_ABS(GroupRec.y + (GroupRec.height>>1) - \
				pCarRec->y - (pCarRec->height>>1) );

			if ( (mvLapTrd(pCarRec, &GroupRec, 0.7f)
				|| mvLapTrd(&GroupRec, pCarRec, 0.7f)) 
				&& pCarRec->confidence > GroupRec.confidence) 
			{

				if ( nDisTance < 0.4f * GroupRec.width  && ( pCarRec->width < 1.2f * GroupRec.width &&  pCarRec->width > 0.8f * GroupRec.width ) )
				{
					*pLapCarRec = CarRec;
					nMatchIndex = i;
					break;
				}
				else
				{
					mvSetUpdataCarImg(*pCarRec, &Carimg, nId);
					for (j = 0; j < WS_MIN(pGroup->nUpdataIndex, UPDATA_IMAGE_NUM); j++) 
					{
						mvTemplatMatch(pGroup->UpdatedImg[j], Carimg,
							adasrect(0, 0, UPDATA_IMAGE_SIZE,
							UPDATA_IMAGE_SIZE), &MatchRec, noff, 0,
							nId, &fMatchScore);
						fAvgScore += fMatchScore;
					}
					fAvgScore /= 
						WS_MIN(pGroup->nUpdataIndex,UPDATA_IMAGE_NUM);

					if (fAvgScore > 0.5f)
					{
						*pLapCarRec = CarRec;
						nMatchIndex = i;
						break;
					}

				}
			}
		}
	}

	return nMatchIndex;
}

/*
 Function process:
 + update the temple of target(pGroup->UpdatedImg) based on the detected region
 Fan-in :
 + mvUpdataByDetctor()
 Fan-out:
 + N/A
 ATTENTION: __________
 */
static void mvUpdataNormalCarImg(obj_group * pGroup, AdasRect RioRec, s32 nId) {
	s32 i, j;
	//uint8_t *pDstr = 0;
	//uint8_t *pSrctr = 0;
	float32_t fx_zoom, fy_zoom;
	imgage *pupdataimage;
	uint8_t *pDst = 0;
	s32 nWidth = m_globlparam[nId].scaleInput.pOriGrayfram.nWid >> nId;
	pupdataimage = pGroup->UpdatedImg + pGroup->nUpdataIndex % UPDATA_IMAGE_NUM;

	fx_zoom = ((float32_t) RioRec.width) / UPDATA_IMAGE_SIZE;
	fy_zoom = ((float32_t) RioRec.height) / UPDATA_IMAGE_SIZE;

	for (j = 0; j < UPDATA_IMAGE_SIZE; j++) {
		pDst = pupdataimage->ptr + j * UPDATA_IMAGE_SIZE;

		for (i = 0; i < UPDATA_IMAGE_SIZE; i++) {
			pDst[i] = *(m_globlparam[nId].scaleInput.pGrayfram
					+ ((s32) (j * fy_zoom) + RioRec.y) * nWidth
					+ ((s32) (i * fx_zoom) + RioRec.x));
		}
	}

	pGroup->nUpdataIndex++;
}

/*
 *************************************************************
 Function process:
 + update pgroup->pOrInitGray based on pgroup->InitContour
 Fan-in :
 + mvUpdataGroupByDet()
 Fan-out:
 + N/A
 ATTENTION: __________
 */
static void mvSetOriInitGray(obj_group *pgroup, s32 nId) {
	s32 j;
	uint8_t *pDst = 0;
	uint8_t *pSrc = 0;
	s32 nRow = 0;
	mvScopeImg(&pgroup->InitContour,
			adasrect(0, 0, m_globlparam[nId].m_ImgWidth,
					m_globlparam[nId].m_ImgHeight));

	if (Car
			== pgroup->ntype&& WS_MAX(pgroup->InitContour.width,pgroup->InitContour.height) >= MAX_TEMPLAT_TRACK_SIZE) {
		pgroup->InitContour.width = 0;
		pgroup->InitContour.height = 0;
		return;
	}

	for (j = pgroup->InitContour.y;
			j < pgroup->InitContour.y + pgroup->InitContour.height; j++) {
		pDst = pgroup->pOrInitGray + pgroup->InitContour.width * nRow;
		pSrc = m_globlparam[nId].m_pGrayData + m_globlparam[nId].m_ImgWidth * j
				+ pgroup->InitContour.x;
		memcpy(pDst, pSrc, pgroup->InitContour.width);
		nRow++;
	}

}

/*
 *************************************************************
 Function process:
 + add traject to pgroup->pObjtr by the detected fast points
 Fan-in :
 + mvUpdataGroupByDet()
 Fan-out:
 + N/A
 ATTENTION: __________
 */
static void mvAddTrajecToGroup(obj_group *pgroup, s32 nId) {

	s32 m;
	s32 nInRecNum = 0;
	AdasCorner *pCorner = 0;
	trajecy *pTrajecy = 0;
	AdasRect shrinkRec;


	shrinkRec.x = pgroup->ProcessContour.x
			+ (s32) (pgroup->ProcessContour.width * 0.15f);
	shrinkRec.y = pgroup->ProcessContour.y
			+ (s32) (pgroup->ProcessContour.height * 0.15f);
	shrinkRec.width = (s32) (pgroup->ProcessContour.width * 0.7f);
	shrinkRec.height = (s32) (pgroup->ProcessContour.height * 0.7f);

	for (m = 0; m < m_globlparam[nId].m_nFastCornerNum; m++) {

		pCorner = m_globlparam[nId].m_pFastCorner + m;

		if (mvAdasCornerInRect(pCorner, &shrinkRec)
				&& !pCorner->State.nMacthNum) {
			if (pgroup->ntrajecyNum >= MAX_TRACKS_NUM_OF_GROUP) {
				break;
			}

			pTrajecy = pgroup->pObjtr + pgroup->ntrajecyNum;

			pTrajecy->nMapGroupId = m_globlparam[nId].m_GroupId;
			pTrajecy->nEstTimes = 0;

			pTrajecy->pTrackPoint->point = adaspoint(pCorner->x, pCorner->y);
			pTrajecy->pTrackPoint->nFramseq = m_globlparam[nId].scaleInput.nFramSeq;
			pTrajecy->pTrackPoint->nTime = m_globlparam[nId].scaleInput.objTime;
			pTrajecy->pTrackPoint->nMatchStatus = 1;
			pTrajecy->nTrackLen++;

			//memcpy(pTrajecy->pfeature,m_globlparam.m_pfeature + m * SURF_DESC_DIMENTION,SURF_DESC_DIMENTION);
			memcpy(pTrajecy->pfeature, m_globlparam[nId].m_pfeature + (m << 6),
					SURF_DESC_DIMENTION);

			pgroup->ntrajecyNum++;
		} else if (mvAdasCornerInRect(pCorner, &pgroup->rtContour)
				&& !pCorner->State.nMacthNum) {
			if (nInRecNum < MAX_PUBLIC_SPACE_SIZE) {
				m_globlparam[nId].m_PublacSape[nInRecNum++] = m;
			}

		}
	}

	for (m = 0; m < nInRecNum; m++) {

		if (pgroup->ntrajecyNum >= MAX_TRACKS_NUM_OF_GROUP) {
			break;
		}

		pCorner = m_globlparam[nId].m_pFastCorner
				+ m_globlparam[nId].m_PublacSape[m];

		pTrajecy = pgroup->pObjtr + pgroup->ntrajecyNum;

		pTrajecy->nMapGroupId = m_globlparam[nId].m_GroupId;
		pTrajecy->nEstTimes = 0;

		pTrajecy->pTrackPoint->point = adaspoint(pCorner->x, pCorner->y);
		pTrajecy->pTrackPoint->nFramseq = m_globlparam[nId].scaleInput.nFramSeq;
		pTrajecy->pTrackPoint->nTime = m_globlparam[nId].scaleInput.objTime;
		pTrajecy->pTrackPoint->nMatchStatus = 1;
		pTrajecy->bProcessTracked = 0;
		pTrajecy->bInitTracked = 0;
		pTrajecy->nTrackLen++;

		memcpy(pTrajecy->pfeature,
				m_globlparam[nId].m_pfeature + m * SURF_DESC_DIMENTION,
				SURF_DESC_DIMENTION);

		pgroup->ntrajecyNum++;

	}

}

/*
 *************************************************************
 Function process:
 + get the ROI image buffer.
 Fan-in :
 + mvUpdataGroupByDet()
 Fan-out:
 + N/A
 ATTENTION: __________
 */
static void mvSelcImg(imgage * RioImg, AdasRect RioRec, uint8_t bPresent,
		s32 nId) {

	s32 j;
	uint8_t *pDstr = 0;
	uint8_t *pSrctr = 0;

	RioImg->nWid = RioRec.width;
	RioImg->nHig = RioRec.height;

	if (bPresent) {
		for (j = RioRec.y; j < RioRec.y + RioRec.height; j++) {
			pDstr = RioImg->ptr + RioImg->nWid * (j - RioRec.y);

			pSrctr = m_globlparam[nId].m_pGrayData
					+ m_globlparam[nId].m_ImgWidth * j;

			memcpy(pDstr, pSrctr + RioRec.x, RioRec.width);

		}
	} else {
		for (j = RioRec.y; j < RioRec.y + RioRec.height; j++) {
			pDstr = RioImg->ptr + RioImg->nWid * (j - RioRec.y);

			pSrctr = m_globlparam[nId].m_preGrayData
					+ m_globlparam[nId].m_ImgWidth * j;

			memcpy(pDstr, pSrctr + RioRec.x, RioRec.width);

		}

	}

}

/*
 Function process:
 + Update pGroup by detected result
 Fan-in :
 + mvUpdataByDetctor()
 Fan-out:
 + mvSetOriInitGray()
 + mvDelUnReasonTrack()
 + mvAddTrajecToGroup()
 + mvEstablInitVote()
 + mvSelcImg()
 ATTENTION: __________
 */
static void mvUpdataGroupByDet(obj_group *pGroup, AdasRect *pDetrec, s32 nId) {
	AdasRect ShrinkTempRec;
	float fzoom = (float) GROUP_TEMPLATE_SHINK_RATE;
	s32 nValidCar = 0;
	uint8_t bFilte = (mvLapTrd(&pGroup->rtContour, pDetrec, 0.7f)
		|| mvLapTrd(pDetrec, &pGroup->rtContour, 0.7f));

	if ( bFilte ) {
		//
		pDetrec->x = (pDetrec->x + (pDetrec->width >> 1))
				- (pGroup->rtContour.width >> 1);
		pDetrec->y = (pDetrec->y + (pDetrec->height >> 1))
				- (pGroup->rtContour.height >> 1);
		pDetrec->width = pGroup->rtContour.width;
		pDetrec->height = pGroup->rtContour.height;
		nValidCar = 1;
	}

	mvScopeImg(pDetrec,
			adasrect(0, 0, m_globlparam[nId].m_ImgWidth,
					m_globlparam[nId].m_ImgHeight));

	pGroup->rtContour = *pDetrec;
	pGroup->InitContour = *pDetrec;

	if (nValidCar) {
		mvSetOriInitGray(pGroup, nId);
	}

	mvDelUnReasonTrack(pGroup);

	mvAddTrajecToGroup(pGroup, nId);

	mvEstablInitVote(pGroup, 0, nId, m_globlparam[nId].scaleInput.nFramSeq);

	pGroup->OriInitContour = *pDetrec;
	pGroup->histoyRec.nSizNum = 0;
	pGroup->histoyRec.pGroupSiz[0] = pDetrec->width;
	pGroup->histoyRec.pGroupSiz[MAX_HISRORY_GROUP_REC_NUM] = pDetrec->height;
	pGroup->histoyRec.nSizNum++;
	pGroup->updataFrambyCar = m_globlparam[nId].scaleInput.nFramSeq;
	pGroup->SerpreditNum = 0;
	pGroup->nMinCarWid = WS_MIN(pGroup->nMinCarWid, pGroup->rtContour.width);
	pGroup->CarBottom.nDetBottomNum = 0;

	mvSetOriInitGray(pGroup, nId);

	if (WS_MAX(pGroup->rtContour.width,pGroup->rtContour.height)
			< MAX_TEMPLAT_NCC_TRACK_SIZE) {
		mvShinkRect(pGroup->rtContour, &ShrinkTempRec, fzoom);

		mvSelcImg(&pGroup->Templat, ShrinkTempRec, 1, nId);
	}

}

/*
 Function process:
 + Update pGroup by detected result
 Fan-in :
 + mvUpdataByDetctor()
 Fan-out:
 + mvSetOriInitGray()
 + mvDelUnReasonTrack()
 + mvAddTrajecToGroup()
 + mvEstablInitVote()
 + mvSelcImg()
 ATTENTION: __________
 */
static uint8_t mvUpdataByDetctor(obj_group *pgroup, s32 nId) {
	//size_t  SmallObjTrd = 32;
	AdasRect ShrinkTempRec;
	s32 nMatchIndex;
	uint8_t bUpadata = 0;
	AdasRect Rec;
	float fzoom = (float) GROUP_TEMPLATE_SHINK_RATE;

	if (Car != pgroup->ntype) {
		return 0;
	}

	nMatchIndex = mvMatchDetRec(pgroup, &Rec, nId);

	if (pgroup->nTruelyObj && -1 != nMatchIndex) {

		pgroup->nLastupdataCarWidth = Rec.width;

		mvUpdataNormalCarImg(pgroup, Rec, nId);

		// if find object overlap with the target which has been verified, so update the rect.
		mvScopeImg(&Rec,
			adasrect(0, 0, m_globlparam[nId].m_ImgWidth,
			m_globlparam[nId].m_ImgHeight));


		pgroup->rtContour = Rec;
		pgroup->InitContour = Rec;

		mvDelUnReasonTrack(pgroup);

		mvAddTrajecToGroup(pgroup, nId);

		mvEstablInitVote(pgroup, 0, nId, m_globlparam[nId].scaleInput.nFramSeq);

		pgroup->OriInitContour = Rec;
		pgroup->histoyRec.nSizNum = 0;
		pgroup->histoyRec.pGroupSiz[0] = Rec.width;
		pgroup->histoyRec.pGroupSiz[MAX_HISRORY_GROUP_REC_NUM] = Rec.height;
		pgroup->histoyRec.nSizNum++;
		pgroup->updataFrambyCar = m_globlparam[nId].scaleInput.nFramSeq;
		pgroup->SerpreditNum = 0;
		pgroup->nMinCarWid = WS_MIN(pgroup->nMinCarWid,pgroup->rtContour.width);
		pgroup->CarBottom.nDetBottomNum = 0;

		mvSetOriInitGray(pgroup, nId);

		if (WS_MAX(pgroup->rtContour.width,pgroup->rtContour.height)
			< MAX_TEMPLAT_NCC_TRACK_SIZE) {
				mvShinkRect(pgroup->rtContour, &ShrinkTempRec, fzoom);

				mvSelcImg(&pgroup->Templat, ShrinkTempRec, 1, nId);
		}

		mvUpdataGroupCenTraj(pgroup);

		if (WS_MAX(pgroup->rtContour.width,pgroup->rtContour.height)
				< MAX_TEMPLAT_NCC_TRACK_SIZE) {

			mvShinkRect(pgroup->rtContour, &ShrinkTempRec, fzoom);

			mvSelcImg(&pgroup->Templat, ShrinkTempRec, 1, nId);

#ifdef SHOW_TEMPLE
			IplImage *TemplateImg = Trans_Imgage_TO_cvIplImage(pgroup->Templat);
			cvShowImage("MAKE_TemplateImg",TemplateImg);
			cvWaitKey(0);
			cvReleaseImage(&TemplateImg);
#endif

		}

		bUpadata = 1;
	}

	return bUpadata;
}

/*
 Function process:
 + decide if there is overlapped in differnet scales
 Fan-in :
 + mvPreditGroup()
 Fan-out:
 + mvLapDetbyGoup()
 ATTENTION: __________
 */
static uint8_t mvOccludedbyGroup(uint8_t SelfGroupIndex, AdasRect *OccluedRec,
		s32 *pFindInxdex, s32 nId) {
	s32 i;
	s32 nIndex;
	uint8_t bFilte = 0;
	obj_group *pDstGroup = m_globlparam[nId].m_pGroupSets + SelfGroupIndex;
	obj_group *pGroup = 0;

	for (i = 0; i < m_globlparam[nId].m_GroupIndexNum; i++) {
		nIndex = m_globlparam[nId].m_pGroupIndex[i];

		if (SelfGroupIndex != nIndex) {
			pGroup = m_globlparam[nId].m_pGroupSets + nIndex;

			bFilte = (mvLapTrd(&pGroup->rtContour, &pDstGroup->rtContour, 0.7f)
				|| mvLapTrd(&pDstGroup->rtContour, &pGroup->rtContour, 0.7f));

			if (pGroup->nTruelyObj
				&& bFilte ) {
                mvLapDetbyGoup( &pGroup->rtContour, &pDstGroup->rtContour, OccluedRec );
				*pFindInxdex = nIndex;
				return 1;
			}
		}
	}

	return 0;
}

/*
 Function process:
 + do the temple matching in pgroup->rtContour by pgroup->Templat
 Fan-in :
 + mvPreditGroup()
 Fan-out:
 + mvTemplatMatch()
 ATTENTION: __________
 */
static uint8_t mvMatchByGroupTemplate(obj_group *pgroup,
		AdasRect *MatchResultRec, s32 nId, float32_t *pfMatchScore) {
	imgage Curnimg;
	s32 nAddress0ff = 0;
	uint8_t *ptr = 0;
	AdasRect SearcRect;
	uint8_t bTempltMatch = 0;

	nAddress0ff = 0;
	ptr = (uint8_t *) m_globlparam[nId].m_PublacSape;

	SearcRect.width = (s32) (pgroup->rtContour.width * 1.5f);
	SearcRect.height = (s32) (pgroup->rtContour.height * 1.5f);
	SearcRect.x = pgroup->rtContour.x
			- ((SearcRect.width - pgroup->rtContour.width) >> 1);
	SearcRect.y = pgroup->rtContour.y
			- ((SearcRect.height - pgroup->rtContour.height) >> 1);
	mvScopeImg(&SearcRect,
			adasrect(0, 0, m_globlparam[nId].m_ImgWidth,
					m_globlparam[nId].m_ImgHeight));

	Curnimg.ptr = ptr;
	Curnimg.nWid = SearcRect.width;
	Curnimg.nHig = SearcRect.height;
	nAddress0ff += ADAS_ALIGN_16BYTE_SIZE( Curnimg.nWid * Curnimg.nHig);
	ptr = (uint8_t *) m_globlparam[nId].m_PublacSape + nAddress0ff;

	mvSelcImg(&Curnimg, SearcRect, 1, nId);

	bTempltMatch = mvTemplatMatch(Curnimg, pgroup->Templat,
			adasrect(0, 0, Curnimg.nWid, Curnimg.nHig), MatchResultRec,
			nAddress0ff, 0, nId, pfMatchScore);

	MatchResultRec->x -= (s32) (MatchResultRec->width
			* GROUP_TEMPLATE_SHINK_RATE);
	MatchResultRec->y -= (s32) (MatchResultRec->height
			* GROUP_TEMPLATE_SHINK_RATE);
	MatchResultRec->width += (s32) ((MatchResultRec->width << 1)
			* GROUP_TEMPLATE_SHINK_RATE);
	MatchResultRec->height += (s32) ((MatchResultRec->height << 1)
			* GROUP_TEMPLATE_SHINK_RATE);

	MatchResultRec->x += SearcRect.x;
	MatchResultRec->y += SearcRect.y;

	if (bTempltMatch) {
		mvScopeImg(MatchResultRec,
				adasrect(0, 0, m_globlparam[nId].m_ImgWidth,
						m_globlparam[nId].m_ImgHeight));
	}

	MatchResultRec->confidence = pgroup->rtContour.confidence;

	return bTempltMatch;

}

/*
 Function process:
 + Do the midle filter for pGroup->rtContour based on pGroup->histoyRec
 Fan-in :
 + mvPreditGroup()
 Fan-out:
 + mvMidGroupRec()
 ATTENTION: __________
 */
static void mvGroupFilter(obj_group *pgroup, uint8_t bUpdata, s32 nId) {
	//s32 nBottomWith;

	if (!bUpdata) {

#ifdef RET_DS
	
		pgroup->nDetDiscardFam =  m_globlparam[nId].scaleInput.nFramSeq;
		pgroup->nDisCardNum++;

#else
       mvClearGroupIfo(pgroup, 1);
#endif

	} else {
		pgroup->nDisCardNum = 0;

		if ((pgroup->rtContour.x < 5
				&& pgroup->rtContour.y + pgroup->rtContour.height + 5
						> m_globlparam[nId].m_ImgHeight)
				|| (pgroup->rtContour.width > m_globlparam[nId].m_ImgWidth / 2
						|| pgroup->rtContour.width < 5)
				/*|| ( pgroup->rtContour.x > 30 && pgroup->rtContour.x +  pgroup->rtContour.width  < m_globlparam[nId].m_ImgWidth - 30
				 && pgroup->rtContour.y + pgroup->rtContour.height + 10 > m_globlparam[nId].m_ImgHeight)*/) {
#ifdef RET_DS

					 pgroup->nDetDiscardFam =  m_globlparam[nId].scaleInput.nFramSeq;
					 pgroup->nDisCardNum++;

#else
					 mvClearGroupIfo(pgroup, 1);
#endif
		} else {

			pgroup->nDisCardNum = 0;
			pgroup->nFramseq = m_globlparam[nId].scaleInput.nFramSeq;
			pgroup->nTime = m_globlparam[nId].scaleInput.objTime;

			pgroup->histoyRec.pGroupSiz[pgroup->histoyRec.nSizNum
					& MAX_HISRORY_GROUP_REC_NUM_BIT] = pgroup->rtContour.width;
			pgroup->histoyRec.pGroupSiz[(pgroup->histoyRec.nSizNum
					& MAX_HISRORY_GROUP_REC_NUM_BIT) + MAX_HISRORY_GROUP_REC_NUM] =
					pgroup->rtContour.height;
			pgroup->histoyRec.nSizNum++;

			mvMidGroupRec(pgroup);

			mvScopeImg(&pgroup->rtContour,
					adasrect(0, 0, m_globlparam[nId].m_ImgWidth,
							m_globlparam[nId].m_ImgHeight));

			mvUpdataGroupCenTraj(pgroup);

		}

	}
}

/*
 *************************************************************
 Function process:
 + filter out the traject that has large center vote bise
 Fan-in :
 + mvInitConsensVote()
 Fan-out:
 + N/A
 ATTENTION: __________
 */
static s32 mvDelFarCen(obj_group *pGroup, AdasPoint MidVoteCen,
		s32 * pVoteTrajecIndex, s32 nVoteNum, s32 nDisTrd, s32 nId) {
	s32 i;
	AdasPoint votePoin;

	trajecy *ptrajecy = 0;
	s32 nDis = nDisTrd;
	s32 nsimilary = 0;
	s32 votDis;

	for (i = 0; i < nVoteNum; i++) {
		votePoin = adaspoint(m_globlparam[nId].m_ndx[i],
				m_globlparam[nId].m_ndy[i]);

		votDis = mvDisPow(&votePoin, &MidVoteCen);

		if (votDis > nDis) {
			ptrajecy = pGroup->pObjtr + pVoteTrajecIndex[i];

			ptrajecy->nEstTimes = 10;
		} else if (votDis < 25) {

			nsimilary++;
		}

	}


	return nsimilary;
}

/*
 *************************************************************
 Function process:
 + caculate group sacles and location based on pgroup->InitContour and pTrajecy->InitPoint, update pgroup->rtContour
 Fan-in :
 + mvPreditGroup()
 Fan-out:
 + mvScaleVal()
 + mvTrajecyVote()
 + mvDelFarCen()
 ATTENTION: __________
 */
static uint8_t mvInitConsensVote(obj_group *pgroup, s32 nId) {
	uint8_t bfsalcSucces;
	float32_t fscale;
	s32 j;
	trajecy *pTrajecy = 0;
//	Vote CenVote,AvgVotec;
	Vote CenVote;
	s32 nCenVotNum = 0;
	s32 nSimlarVote = 0;
	s32 nVoteMidx, nVoteMidy;
	AdasPoint Votec;
	s32 nVotTrd = VOTE_CONSENSE_TRD;
	s32 *ptr = m_globlparam[nId].m_PublacSape;

	bfsalcSucces = mvScaleVal(pgroup, m_globlparam[nId].m_fsclare, &fscale,
			InitVote, nId);

	//if( fscale > 1.15f ||  fscale < 0.85f)
	if (fscale > 1.2f || fscale < 0.8f) {
		//my_printf("pgroupID:%d Scale change too mutch!\n", pgroup->nGroupId);
		return 0;
	}

	if (bfsalcSucces) {
		memset(m_globlparam[nId].m_PublacSape, 0,
				sizeof(s32) * pgroup->ntrajecyNum);

		for (j = 0; j < pgroup->ntrajecyNum; j++) {
			pTrajecy = pgroup->pObjtr + j;

			if (pTrajecy->bInitTracked && !pTrajecy->nEstTimes) {

				CenVote = mvTrajecyVote(pTrajecy, fscale, InitVote);
				m_globlparam[nId].m_ndx[nCenVotNum] = CenVote.x;
				m_globlparam[nId].m_ndy[nCenVotNum] = CenVote.y;
				m_globlparam[nId].m_PublacSape[nCenVotNum] = j;
				nCenVotNum++;
			}
		}

		memcpy(ptr + nCenVotNum, m_globlparam[nId].m_ndx,
				sizeof(s32) * nCenVotNum);
		memcpy(ptr + (nCenVotNum << 1), m_globlparam[nId].m_ndy,
				sizeof(s32) * nCenVotNum);

		binSort_INT(ptr + nCenVotNum, nCenVotNum);
		binSort_INT(ptr + (nCenVotNum << 1), nCenVotNum);

		Votec.x = ptr[nCenVotNum + (nCenVotNum >> 1)]; //nCenVotNum + (nCenVotNum>>1)
		Votec.y = ptr[(nCenVotNum << 1) + (nCenVotNum >> 1)]; //
		nVoteMidx = Votec.x;
		nVoteMidy = Votec.y;

		nSimlarVote = mvDelFarCen(pgroup, Votec, m_globlparam[nId].m_PublacSape,
				nCenVotNum,
				pgroup->rtContour.width * pgroup->rtContour.width / 25, nId);

		if (nSimlarVote < nVotTrd) {
			return 0;
		}

		pgroup->rtContour.width = (s32) (pgroup->InitContour.width * fscale);
		pgroup->rtContour.height = (s32) (pgroup->InitContour.height * fscale);
		pgroup->rtContour.x = nVoteMidx - (pgroup->rtContour.width >> 1);
		pgroup->rtContour.y = nVoteMidy - (pgroup->rtContour.height >> 1);

	}

	return nSimlarVote;
}

/*
 Function process:
 + update pGroup->ProcessContour，pGroup->nPreProssVotFram，pTrajecy->bProcessTracked，pTrajecy->ProcessVote，pTrajecy->processfeature
 Fan-in :
 + mvPreditGroup()
 Fan-out:
 + mvAdaspointOutRect()
 + mvProcessVote()
 ATTENTION: __________
 */
static void mvUpdataProcessVote(obj_group *pGroup, s32 nId) {
	s32 m;
	trajecy *pTrajecy = 0;
	TrackPoint *Trackpin = 0;
	AdasRect shrinkRec;
	//const uint8_t DispoweMovTrd = 25;
	//uint8_t  bFieled = 0;

	if (m_globlparam[nId].scaleInput.nFramSeq - pGroup->nPreProssVotFram < 3
			&& pGroup->nPreProssVotFram) {
		return;
	}

	//if(pGroup->nGroupId ==3)
	//my_printf("----------------------------------------------mvUpdataProcessVote:x:%d,%d\n",pGroup->ProcessContour.x,pGroup->ProcessContour.y);

	pGroup->ProcessContour = pGroup->rtContour;
	shrinkRec.x = pGroup->ProcessContour.x
			+ (s32) (pGroup->ProcessContour.width * 0.15f);
	shrinkRec.y = pGroup->ProcessContour.y
			+ (s32) (pGroup->ProcessContour.height * 0.15f);
	shrinkRec.width = (s32) (pGroup->ProcessContour.width * 0.7f);
	shrinkRec.height = (s32) (pGroup->ProcessContour.height * 0.7f);

	pGroup->nPreProssVotFram = m_globlparam[nId].scaleInput.nFramSeq;

	for (m = 0; m < pGroup->ntrajecyNum; m++) {
		pTrajecy = pGroup->pObjtr + m;
		Trackpin = pTrajecy->pTrackPoint
				+ ((pTrajecy->nTrackLen - 1) & MAX_CORNER_OF_TRACK_BIT);

		if (mvAdaspointOutRect(&Trackpin->point, &shrinkRec)) {
			pTrajecy->bProcessTracked = 0;

		} else {
			pTrajecy->bProcessTracked = 1;
			mvProcessVote(pTrajecy, pGroup->ProcessContour);
			memcpy(pTrajecy->processfeature, pTrajecy->pfeature,
					SURF_DESC_DIMENTION);
		}
	}

}

/*
 *************************************************************
 Function process:
 + copy and resize the image of pgroup->OriInitContour to pResImg, pResImg has the same size of pgroup->rtContour
 Fan-in :
 + mvTempleMatchByOri()
 Fan-out:
 + N/A
 ATTENTION: __________
 */
static void mvOriObjResizeToDstImg(obj_group *pgroup, uint8_t *pResImg, s32 nId) {

	s32 i, j;
	float32_t fScale;
	float32_t fSum;
	uint8_t *Ptr = 0;
	uint8_t *PSrc = 0;
	s32 width, height;
	s32 *arr_y = 0;
	s32 *arr_x = 0;

	width = pgroup->rtContour.width;
	height = pgroup->rtContour.height;

	arr_y = (s32 *) m_globlparam[nId].m_pXYCorners;
	arr_x = (s32 *) m_globlparam[nId].m_pXYNoMax;

	fScale = pgroup->OriInitContour.width / (width + 0.001f);

	fSum = -fScale;
	for (j = 0; j < height; j++) {
		fSum += fScale;

		arr_y[j] = (s32) (fSum + 0.5);

		if (j < width) {
			arr_x[j] = arr_y[j];
		}
	}

	if (width > height) {
		for (i = height; i < width; i++) {
			fSum += fScale;

			arr_x[i] = (s32) (fSum + 0.5);
		}
	}

	for (j = 0; j < height; j++) {
		Ptr = pResImg + width * j;

		PSrc = pgroup->pOrInitGray + pgroup->OriInitContour.width * arr_y[j];

		for (i = 0; i < width; i++) {
			Ptr[i] = PSrc[arr_x[i]];
		}
	}
}

/*
 *************************************************************
 Function process:
 + Get the temple image from img
 Fan-in :
 + mvTempleMatchByOri()
 Fan-out:
 + N/A
 ATTENTION: __________
 */
static void mvSelcTemplatByCen(const imgage img, imgage *pTempimg,
		AdasRect *pTempRec) {
	s32 j;
	AdasPoint lt;
	AdasPoint imgCen;
	uint8_t *pTr = 0;
	uint8_t *pSrcTr = 0;

	pTempimg->nWid = WS_MAX( 2, (s32)( 0.4 * WS_MIN(img.nWid,img.nHig) ) );
	pTempimg->nHig = pTempimg->nWid;

	imgCen.x = img.nWid >> 1;
	imgCen.y = img.nHig >> 1;

	lt = adaspoint(imgCen.x - (pTempimg->nWid >> 1),
			imgCen.y - (pTempimg->nWid >> 1));

	pTempRec->x = lt.x;
	pTempRec->y = lt.y;
	pTempRec->width = pTempimg->nWid;
	pTempRec->height = pTempimg->nHig;

	for (j = 0; j < pTempimg->nHig; j++) {
		pTr = pTempimg->ptr + pTempimg->nWid * j;

		pSrcTr = img.ptr + img.nWid * (j + lt.y);

		memcpy(pTr, pSrcTr + lt.x, pTempimg->nWid);

	}
}

/*
 Function process:
 + Do the temple matching for group if vote tarcking failed
 Fan-in :
 + mvPreditGroup()
 Fan-out:
 + mvOriObjResizeToDstImg()
 + mvSelcTemplatByCen()
 + mvTemplatMatch()
 ATTENTION: __________
 */
static uint8_t mvTempleMatchByOri(obj_group *pgroup, AdasRect *pMatchRec,
		s32 nId, float32_t *fMatchScore) {
	//int i;
	int nAddress0ff = 0;
	imgage ResizeOrimg;
	imgage Tempimg, Curnimg;
	AdasRect SearcRe, MatchRec, TempRec;
	float32_t fShunkRio;
	uint8_t bMatchSucces;
	//VoteErorrVal VoteCorreVal;
	//trajecy *pTrajecy = 0;

	uint8_t * ptr = (uint8_t *) m_globlparam[nId].m_PublacSape;
	ResizeOrimg.ptr = ptr;
	ResizeOrimg.nWid = pgroup->rtContour.width;
	ResizeOrimg.nHig = pgroup->rtContour.height;

	if (WS_MAX(ResizeOrimg.nWid ,ResizeOrimg.nHig) > MAX_TEMPLAT_TRACK_SIZE
			|| (!pgroup->InitContour.width)
			|| pgroup->rtContour.width * pgroup->rtContour.height
					>= MAX_GROUP_IMG_BYTE) {
		return 0;
	}

	mvOriObjResizeToDstImg(pgroup, ResizeOrimg.ptr, nId);

	nAddress0ff += ADAS_ALIGN_16BYTE_SIZE( ResizeOrimg.nWid * ResizeOrimg.nHig);
	ptr = (uint8_t *) m_globlparam[nId].m_PublacSape + nAddress0ff;

	Tempimg.ptr = ptr;
	mvSelcTemplatByCen(ResizeOrimg, &Tempimg, &TempRec);
	nAddress0ff += ADAS_ALIGN_16BYTE_SIZE( Tempimg.nWid * Tempimg.nHig);
	ptr = (uint8_t *) m_globlparam[nId].m_PublacSape + nAddress0ff;

	Curnimg.ptr = ptr;
	Curnimg.nWid = pgroup->rtContour.width;
	Curnimg.nHig = pgroup->rtContour.height;
	nAddress0ff += ADAS_ALIGN_16BYTE_SIZE( Curnimg.nWid * Curnimg.nHig);
	ptr = (uint8_t *) m_globlparam[nId].m_PublacSape + nAddress0ff;

	mvSelcImg(&Curnimg, pgroup->rtContour, 1, nId);

	fShunkRio = 1 / 10.0f;
	SearcRe.x = (s32) (fShunkRio * Curnimg.nWid);
	SearcRe.y = (s32) (fShunkRio * Curnimg.nWid);
	SearcRe.width = (s32) ((1 - fShunkRio * 2) * Curnimg.nWid);
	SearcRe.height = (s32) ((1 - fShunkRio * 2) * Curnimg.nHig);

	bMatchSucces = mvTemplatMatch(Curnimg, Tempimg, SearcRe, &MatchRec,
			nAddress0ff, 1, nId, fMatchScore);

	if (bMatchSucces) {
		pMatchRec->x = MatchRec.x - TempRec.x + pgroup->rtContour.x; // ProcessContour
		pMatchRec->y = MatchRec.y - TempRec.y + pgroup->rtContour.y; //ProcessContour
		pMatchRec->width = ResizeOrimg.nWid;
		pMatchRec->height = ResizeOrimg.nHig;
		pMatchRec->confidence = pgroup->rtContour.confidence;
	}

	return bMatchSucces;
}

/*
 *************************************************************
 Function process:
 + caculate group sacles and location based on pTrajecy->ProcessPoint and pTrajecy->InitPoint, update pgroup->rtContour
 Fan-in :
 + mvScaleVal()
 + mvTrajecyVote()
 + mvDelFarCen()
 Fan-out:
 + N/A
 ATTENTION: __________
 */
static uint8_t mvProcessConsensVote(obj_group *pgroup, s32 nId) {

	uint8_t bfsalcSucces;
	float32_t fscale = 2.0f;
	s32 j;
	trajecy *pTrajecy = 0;
	Vote CenVote;
	s32 nCenVotNum = 0;
	s32 nSimlarVote = 0;
	AdasPoint Votec;
	s32 nDisPowTrd;
	s32 nVotTrd = (scale_shink_4_id == nId) ? 9 : VOTE_CONSENSE_TRD;

	bfsalcSucces = mvScaleVal(pgroup, m_globlparam[nId].m_fsclare, &fscale,
			ProcessVote, nId);

	if ((fscale > 1.15f && pgroup->nMinCarWid < 60) || fscale > 1.3f) {
		return 0;
	}

	//nDisPowTrd = pgroup->rtContour.width * pgroup->rtContour.width/25;
	nDisPowTrd = ((pgroup->nMinCarWid * pgroup->nMinCarWid) >> 6);

	//nDisPowTrd = WS_MIN(225,nDisPowTrd);
	nDisPowTrd = WS_MAX(25,nDisPowTrd);

	if (bfsalcSucces) {
		memset(m_globlparam[nId].m_PublacSape, 0,
				sizeof(s32) * pgroup->ntrajecyNum);

		for (j = 0; j < pgroup->ntrajecyNum; j++) {
			pTrajecy = pgroup->pObjtr + j;

			if (pTrajecy->bProcessTracked) {
				CenVote = mvTrajecyVote(pTrajecy, fscale, ProcessVote);

				m_globlparam[nId].m_ndx[nCenVotNum] = CenVote.x;
				m_globlparam[nId].m_ndy[nCenVotNum] = CenVote.y;
				m_globlparam[nId].m_PublacSape[nCenVotNum] = j;
				nCenVotNum++;

			}
		}

		if (!nCenVotNum) {
			return 0;
		}

		memcpy(m_globlparam[nId].m_PublacSape + nCenVotNum,
				m_globlparam[nId].m_ndx, sizeof(s32) * nCenVotNum);
		memcpy(m_globlparam[nId].m_PublacSape + (nCenVotNum << 1),
				m_globlparam[nId].m_ndy, sizeof(s32) * nCenVotNum);

		binSort_INT(m_globlparam[nId].m_PublacSape + nCenVotNum, nCenVotNum);
		binSort_INT(m_globlparam[nId].m_PublacSape + (nCenVotNum << 1),
				nCenVotNum);

		Votec.x =
				m_globlparam[nId].m_PublacSape[nCenVotNum + (nCenVotNum >> 1)];
		Votec.y = m_globlparam[nId].m_PublacSape[(nCenVotNum << 1)
				+ (nCenVotNum >> 1)];

		nSimlarVote = mvDelFarCen(pgroup, Votec, m_globlparam[nId].m_PublacSape,
				nCenVotNum, nDisPowTrd, nId);

		if (nSimlarVote < nVotTrd) {
			return 0;
		}

		pgroup->rtContour.width = (s32) (pgroup->ProcessContour.width * fscale);
		pgroup->rtContour.height =
				(s32) (pgroup->ProcessContour.height * fscale);
		pgroup->rtContour.x = Votec.x - (pgroup->rtContour.width >> 1);
		pgroup->rtContour.y = Votec.y - (pgroup->rtContour.height >> 1);

	}

	return nSimlarVote;

}

/*
 Function process:
 + Tracking the groups in m_globlparam[].m_pGroupSets
 Fan-in :
 + mvSingledScaleTrack()
 Fan-out:
 + mvUpdataByDetctor()
 + mvUpdataGroupCenTraj()
 + mvOccludedbyGroup()
 + mvDetcorBytrainByGroup()
 + mvMatchByGroupTemplate()
 + mvGroupFilter()
 + mvInitConsensVote()
 + mvUpdataProcessVote()
 + mvclearProcesState()
 + mvTempleMatchByOri()
 + mvProcessConsensVote()
 ATTENTION: __________
 */
static void mvPreditGroup(const PortInput *pInPutParam, s32 nId) {
	s32 i;
	obj_group *pgroup = 0;
	//const uint8_t nSimlarTrd = 4;
	uint8_t nSimlarVote;
	AdasRect OccluedRec;
	uint8_t bocclued = 0;
	uint8_t bTempltMatch;
	AdasRect MatchResultRec;
	size_t SmallObjTrd = 32;
	AdasRect ShrinkTempRec;
	uint8_t bDetCarByRio = 0;
	AdasRect DetRect;
	float fzoom = (float) GROUP_TEMPLATE_SHINK_RATE;
	uint8_t bUpadataByCar;
	uint8_t bGetNewLocal;
	s32 nFindLapIndex;
	float32_t fMatchScore;
	int64_t unFramSeq = m_globlparam[nId].scaleInput.nFramSeq;

	for (i = 0; i < m_globlparam[nId].m_GroupIndexNum; i++) {
		pgroup = m_globlparam[nId].m_pGroupSets
				+ m_globlparam[nId].m_pGroupIndex[i];
		pgroup->nTime = m_globlparam[nId].scaleInput.objTime;
		pgroup->nFramseq = m_globlparam[nId].scaleInput.nFramSeq;
		bGetNewLocal = 0;

		// reserve discard target for 4 frame
#ifdef RET_DS
		if (4 == pgroup->nDisCardNum ) {

			mvClearGroupIfo(pgroup, 1);
		}
#endif

		// update target by detect
		bUpadataByCar = mvUpdataByDetctor(pgroup, nId);
		if (bUpadataByCar) {
			pgroup->SerpreditNum = 0;
			mvUpdataGroupCenTraj(pgroup);
			my_printf("***pgroup ID: %d is updated by detect!\n",
					pgroup->nGroupId);
			continue;
		}

		// delete the dead target
		if (pgroup->nFramseq - pgroup->updataFrambyCar > 30 && pgroup->rtContour.confidence == 0)
		{
			my_printf("***pgroup ID: %d don't get the update for a long time and confidence is zero!\n",
				pgroup->nGroupId);
			mvClearGroupIfo(pgroup, 1);
			continue;
		}

		// target close to edge of image
		if ( pgroup->rtContour.x < 10
			|| pgroup->rtContour.x + pgroup->rtContour.width + 10
		> m_globlparam[nId].m_ImgWidth
		|| pgroup->rtContour.y < 10
		|| pgroup->rtContour.y + pgroup->rtContour.height + 10
		> m_globlparam[nId].m_ImgHeight) {
			my_printf("***pgroup ID: %d is too large and be dropped!\n",
				pgroup->nGroupId);
			mvClearGroupIfo(pgroup, 1);
			continue;
		}

		// Occlusion process
		bocclued = mvOccludedbyGroup(m_globlparam[nId].m_pGroupIndex[i],
			&OccluedRec, &nFindLapIndex, nId);
		if (bocclued)
		{
			obj_group *pComparedgroup = m_globlparam[nId].m_pGroupSets
				+ m_globlparam[nId].m_pGroupIndex[nFindLapIndex];

			if ( pgroup->nTruelyObj && pComparedgroup->nTruelyObj )
			{
				if (pComparedgroup->rtContour.width > pgroup->rtContour.width)
				{
					mvClearGroupIfo(pgroup, 1);
				} 
				else 
				{
					mvClearGroupIfo(pComparedgroup, 1);
				}
			}
			else  if( pgroup->nTruelyObj && !pComparedgroup->nTruelyObj )
			{
				mvClearGroupIfo(pComparedgroup, 1);
			}
			else if( !pgroup->nTruelyObj && pComparedgroup->nTruelyObj )
			{
				mvClearGroupIfo(pgroup, 1);
			}
			mvclearInitState(pgroup, OccluedRec);
		}
	
		// small target use NCC temp match
		if (WS_MAX(pgroup->rtContour.width ,pgroup->rtContour.height)
				< MAX_TEMPLAT_NCC_TRACK_SIZE) 
		{

			bTempltMatch = mvMatchByGroupTemplate(pgroup, &MatchResultRec, nId,
					&fMatchScore);

			if (bTempltMatch) 
			{
				bGetNewLocal = 1;
				pgroup->rtContour = MatchResultRec; 

				if (scale_shink_4_id == nId) {
					mvEstablInitVote(pgroup, 0.25f, nId, unFramSeq);
				} else {
					mvEstablInitVote(pgroup, 0.15f, nId, unFramSeq);
				}

				pgroup->nPreProssVotFram = 0;

			} 
			else 
			{

#ifdef DETCOR_STAR	
				bDetCarByRio = mvDetcorBytrainByGroup(pInPutParam, pgroup,
						&DetRect, 1, nId);
				if (bDetCarByRio && !bocclued) {
					bGetNewLocal = 1;
					mvUpdataGroupByDet(pgroup, &DetRect, nId);
				}
#endif	
			}

			if (bGetNewLocal == 0) {
				my_printf(
						"***pgroup ID: %d is Small and not Template Matched and not Detected!\n",
						pgroup->nGroupId);
			}

			mvGroupFilter(pgroup, bGetNewLocal, nId);
			continue;
		}

		// big target use CMT vote
		nSimlarVote = mvInitConsensVote(pgroup, nId);
		if (nSimlarVote)
		{
			mvDelUnReasonTrack(pgroup);

			mvAddTrajecToGroup(pgroup, nId);

			mvUpdataProcessVote(pgroup, nId);

			if (bocclued) {
				mvclearProcesState(pgroup, OccluedRec);
			}

			bGetNewLocal = 1;

		} 
		else if (pgroup->nPreProssVotFram)
		{
			if (bocclued) {
				mvclearProcesState(pgroup, OccluedRec);
			}

			bTempltMatch = mvTempleMatchByOri(pgroup, &MatchResultRec, nId,
					&fMatchScore);

			if (bTempltMatch) {
				bGetNewLocal = 1;

				pgroup->rtContour = MatchResultRec; //

				if (scale_shink_4_id == nId) {
					mvEstablInitVote(pgroup, 0.25f, nId, unFramSeq);
				} else {
					mvEstablInitVote(pgroup, 0.15f, nId, unFramSeq);
				}

				pgroup->nPreProssVotFram = 0;

				if (bGetNewLocal == 0) {
					my_printf(
							"pgroup ID: %d is not Tracked and not Template Matched!\n",
							pgroup->nGroupId);
				}

				mvGroupFilter(pgroup, bGetNewLocal, nId);

				continue;
			}

			if (fMatchScore < 0.5f) 
			{
#ifdef DETCOR_STAR	
				bDetCarByRio = mvDetcorBytrainByGroup(pInPutParam, pgroup,
						&DetRect, 1, nId);
#else
				bDetCarByRio = 0;
#endif
				if (!bDetCarByRio) {
					my_printf(
							"***pgroup ID: %d is not Tracked and not Template Matched and not detect!\n",
							pgroup->nGroupId);

#ifdef RET_DS

					pgroup->nDetDiscardFam =  m_globlparam[nId].scaleInput.nFramSeq;
					pgroup->nDisCardNum++;

#else
					mvClearGroupIfo(pgroup, 1);
#endif
				} else {
					pgroup->nDisCardNum = 0;
					mvUpdataGroupByDet(pgroup, &DetRect, nId);
					mvUpdataGroupCenTraj(pgroup);
				}

				continue;
			}

			nSimlarVote = mvProcessConsensVote(pgroup, nId);

			if (nSimlarVote)
			{
				mvDelUnReasonTrack(pgroup);

				mvAddTrajecToGroup(pgroup, nId);

				mvScopeImg(&pgroup->rtContour,
						adasrect(0, 0, m_globlparam[nId].m_ImgWidth,
								m_globlparam[nId].m_ImgHeight));
				pgroup->InitContour = pgroup->rtContour;

				mvEstablInitVote(pgroup, 0.15f, nId, unFramSeq);

				bocclued = mvOccludedbyGroup(m_globlparam[nId].m_pGroupIndex[i],
						&OccluedRec, &nFindLapIndex, nId);

				if (bocclued) {
					mvclearInitState(pgroup, OccluedRec);
				} else if (m_globlparam[nId].scaleInput.nFramSeq
						- pgroup->updataOriFramSeq > 30) {
					mvSetOriInitGray(pgroup, nId);
					pgroup->OriInitContour = pgroup->InitContour;
					pgroup->updataOriFramSeq = m_globlparam[nId].scaleInput.nFramSeq;
				}

				bGetNewLocal = 1;
			}

		}

		if (!bGetNewLocal)
		{
			if (WS_MAX(pgroup->rtContour.width,pgroup->rtContour.height)
				< MAX_PREDIT_TRACK_SIZE) {
					if (pgroup->SerpreditNum < 3) {
						bGetNewLocal = 1;
						pgroup->rtContour = pgroup->rtContour;
						pgroup->SerpreditNum++;

					mvDelUnReasonTrack(pgroup);
					if (bocclued) {
						mvclearProcesState(pgroup, OccluedRec);
					}

				} else {
#ifdef DETCOR_STAR	

					bDetCarByRio = mvDetcorBytrainByGroup(pInPutParam, pgroup,
							&DetRect, 1, nId);
					if (bDetCarByRio && !bocclued) {
						bGetNewLocal = 1;
						mvUpdataGroupByDet(pgroup, &DetRect, nId);
					}

					if (bGetNewLocal == 0) {
						my_printf(
								"***pgroup ID: %d size is < 80 and not updated before and already pretected for 3 frames but not detected!\n",
								pgroup->nGroupId);
					}
#endif			
				}

			} else {

#ifdef DETCOR_STAR	
				bDetCarByRio = mvDetcorBytrainByGroup(pInPutParam, pgroup,
						&DetRect, 1, nId);
				if (bDetCarByRio && !bocclued) {
					bGetNewLocal = 1;
					mvUpdataGroupByDet(pgroup, &DetRect, nId);
				}
				if (bGetNewLocal == 0) {
					my_printf(
							"***pgroup ID: %d size is > 80 and not updated before and not detected!\n",
							pgroup->nGroupId);
				}
#endif	
			}

		}

		mvGroupFilter(pgroup, bGetNewLocal, nId);

		if ((Car == pgroup->ntype
				&& (pgroup->rtContour.height
						> pgroup->rtContour.width
								+ (pgroup->rtContour.width >> 1)))
				|| (pgroup->rtContour.width
						> pgroup->rtContour.height
								+ (pgroup->rtContour.height >> 1))) {
			my_printf("***pgroup ID: %d width and height is not a rect!\n",
					pgroup->nGroupId);
			mvClearGroupIfo(pgroup, 1);
			continue;
		}

		if (WS_MAX(pgroup->rtContour.width, pgroup->rtContour.height)
				< MAX_TEMPLAT_NCC_TRACK_SIZE)
		{
			mvShinkRect(pgroup->rtContour, &ShrinkTempRec, fzoom);
			mvSelcImg(&pgroup->Templat, ShrinkTempRec, 1, nId);
		}

	}
}

/*
 *************************************************************
 Function process:
 + add new group to m_globlparam.m_pGroupSets from m_globlparam[nId].m_PNewRec
 Fan-in :
 + mvSingledScaleTrack()
 Fan-out:
 + kalman_rect_init()
 + mvShinkRect()
 + mvSelcImg()
 ATTENTION: __________
 */
static void mvGroupGenerate(s32 nId) {
	s32 i, j;
	s32 nNextGroup = 0;
	s32 nUseGroup = 0;
	obj_group *pNewgroup = 0;
	AdasCorner *pCorner = 0;
	trajecy *pTrajec = 0;
	AdasCorner *pRectCorer = (AdasCorner*) m_globlparam[nId].m_PublacSape;
	s32 RectCorenerNum;
	//s32 nExtend = 20;
	AdasCorner ReCen;
	AdasRect ShrinkTempRec;
	float32_t fzoom = (float32_t) GROUP_TEMPLATE_SHINK_RATE;
	float32_t init_p[4] = { 5e2, 5e2, 5e2, 5e2 };

	for (i = 0; i < m_globlparam[nId].m_NewRecNum; i++) {
		nUseGroup = 0;

		for (j = nNextGroup; j < MAX_LAYER_OBJ_GROUP_NUMS; j++) {
			pNewgroup = m_globlparam[nId].m_pGroupSets + j;

			if (-1 == pNewgroup->nGroupId) {
				nNextGroup = j + 1;
				nUseGroup = 1;
				break;
			}
		}

		//
		if (!nUseGroup) {
			break;
		}

		m_globlparam[0].m_GroupId++;
		m_globlparam[nId].m_GroupId = m_globlparam[0].m_GroupId;

		pNewgroup->nGroupId = m_globlparam[nId].m_GroupId;
		pNewgroup->rtContour = m_globlparam[nId].m_PNewRec[i];
		pNewgroup->InitContour = m_globlparam[nId].m_PNewRec[i];
		pNewgroup->nFramseq = m_globlparam[nId].scaleInput.nFramSeq;
		pNewgroup->nTime = m_globlparam[nId].scaleInput.objTime;
		pNewgroup->ntype = m_globlparam[nId].m_PNewRec[i].nType;
		pNewgroup->nLastupdataCarWidth = pNewgroup->rtContour.width;

		pNewgroup->histoyRec.pGroupSiz[0] = pNewgroup->rtContour.width;
		pNewgroup->histoyRec.pGroupSiz[MAX_HISRORY_GROUP_REC_NUM] =
				pNewgroup->rtContour.height;
		pNewgroup->histoyRec.nSizNum++;

		RectCorenerNum = 0;

		ReCen.x = pNewgroup->rtContour.x + (pNewgroup->rtContour.width >> 1);
		ReCen.y = pNewgroup->rtContour.y + (pNewgroup->rtContour.height >> 1);

		for (j = 0; j < m_globlparam[nId].m_nFastCornerNum; j++) {
			pCorner = m_globlparam[nId].m_pFastCorner + j;

			if (mvAdasCornerInRect(pCorner, m_globlparam[nId].m_PNewRec + i)
					&& !pCorner->State.nMacthNum) {
				pRectCorer[RectCorenerNum] = *pCorner;
				pRectCorer[RectCorenerNum].val = -(WS_ABS(pCorner->x - ReCen.x)
						+ WS_ABS(pCorner->y - ReCen.y));
				pRectCorer[RectCorenerNum].nCornerIndex = j;
				RectCorenerNum++;

			}

		}

		if (RectCorenerNum > MAX_TRACKS_NUM_OF_GROUP) {
			binSort_Corner(pRectCorer, RectCorenerNum);
			RectCorenerNum = MAX_TRACKS_NUM_OF_GROUP;
		}

		for (j = 0; j < WS_MIN(RectCorenerNum,MAX_TRACKS_NUM_OF_GROUP); j++) {
			pCorner = pRectCorer + j;

			pTrajec = pNewgroup->pObjtr + pNewgroup->ntrajecyNum;

			pTrajec->nMapGroupId = m_globlparam[nId].m_GroupId;
			pTrajec->nEstTimes = 0;

			pTrajec->pTrackPoint->point = adaspoint(pCorner->x, pCorner->y);

			pTrajec->pTrackPoint->nFramseq = m_globlparam[nId].scaleInput.nFramSeq;
			pTrajec->pTrackPoint->nTime = m_globlparam[nId].scaleInput.objTime;
			pTrajec->pTrackPoint->nMatchStatus = 1;
			pTrajec->nTrackLen++;
			pTrajec->bInitTracked = 1;

			memcpy(pTrajec->pfeature,
					m_globlparam[nId].m_pfeature
							+ (pRectCorer[j].nCornerIndex << 6),
					SURF_DESC_DIMENTION);

			mvInitVote(pTrajec, pNewgroup->rtContour, 0);

			pNewgroup->ntrajecyNum++;
		}

		mvSetOriInitGray(pNewgroup, nId);
		pNewgroup->OriInitContour = pNewgroup->InitContour;
		pNewgroup->updataOriFramSeq = m_globlparam[nId].scaleInput.nFramSeq;
		pNewgroup->nTruelyObj = 0;
		pNewgroup->nPerLastDetFramseq = m_globlparam[nId].scaleInput.nFramSeq;
		pNewgroup->nStateNum = 0;
		pNewgroup->nMinCarWid = pNewgroup->InitContour.width;
		pNewgroup->CarBottom.nDetBottomNum = 0;
		pNewgroup->CarBottom.nDetYPos = pNewgroup->rtContour.y
				+ pNewgroup->rtContour.height;


		//initial discard target 
		pNewgroup->nDisCardNum = 0;

		kalman_rect_init(&pNewgroup->KalmState, pNewgroup->rtContour, init_p);

		AdasRect tempRec;
		tempRec.x = (pNewgroup->rtContour.x << nId);
		tempRec.y = (pNewgroup->rtContour.y << nId) + (pNewgroup->rtContour.height << nId);
		tempRec.width = (pNewgroup->rtContour.width << nId);
		tempRec.height = (pNewgroup->rtContour.height << nId);
		sysKalmanInit(&pNewgroup->sysKalmState, tempRec);

#ifdef DETCOR_STAR	
		pNewgroup->updataFrambyCar = m_globlparam[nId].scaleInput.nFramSeq;

		if (m_globlparam[nId].pOriInPutParam->pOriGrayfram.dayOrNight == 0)
		{
			if (m_globlparam[nId].m_PNewRec[i].confidence > 20) {
				pNewgroup->nTruelyObj = 1;
			}
		}
		else if (m_globlparam[nId].pOriInPutParam->pOriGrayfram.dayOrNight == 1)
		{
			if (m_globlparam[nId].m_PNewRec[i].confidence > 60) {
				pNewgroup->nTruelyObj = 1;
			}
		}
	
#endif
		if (WS_MAX(pNewgroup->rtContour.width,pNewgroup->rtContour.height)
  				< MAX_TEMPLAT_NCC_TRACK_SIZE) {
			mvShinkRect(pNewgroup->rtContour, &ShrinkTempRec, fzoom);

			mvSelcImg(&pNewgroup->Templat, ShrinkTempRec, 1, nId);

		}
		mvUpdataGroupCenTraj(pNewgroup);

	}

}

/*
Function process:
+ Do the Kalman filter for pGroup->rtContour
Fan-in :
+ mvSingledScaleTrack()
Fan-out:
+ kalman_rect_filter()
ATTENTION: __________
*/
static void mvKalmanFiter(s32 nId) {
	s32 i;
	obj_group *pGroup = 0;

	for (i = 0; i < MAX_LAYER_OBJ_GROUP_NUMS; i++) {
		pGroup = m_globlparam[nId].m_pGroupSets + i;
		if (-1 != pGroup->nGroupId) {
			pGroup->rtContour = kalman_rect_filter(&pGroup->KalmState,
				pGroup->rtContour);
		}
	}

}

/*
 Function process:
 + Do the singe-sacle tracking for pInPutParam in sacle index of nId.
 Fan-in :
 + mvMultileScaleTrack()
 Fan-out:
 + mvGetScaleInPut()
 + mvReLoaclDetRec()
 + mvGetCurrenGroupIndex()
 + mvAddNewObjRec()
 + mvGoupPreditRec()
 + DetcorBytrain()
 + mvGenerateRioGrayByMask()
 + mvCornerDetct()
 + mvFeatrueDescribe()
 + mvTrajectorymatch()
 + mvPreditGroup()
 + mvGroupGenerate()
 + mvKalmanFiter()

 ATTENTION: __________
 */
static void mvSingledScaleTrack(PortInput *pInPutParam, const s32 nId) {

	uint8_t *pTemptr = 0;
	mvGetScaleInPut(pInPutParam, nId);

#ifdef OBJRLCT 
	mvReLoaclDetRec(pInPutParam, nId);

#ifdef TIME_TEST
	TIME_TAKE(cvGetTickCount(),ts,"mvReLoaclDetRec");
	ts = cvGetTickCount();
#endif

#endif

	mvGetCurrenGroupIndex(nId);
	if (!m_globlparam[nId].scaleInput.nRecNum && !m_globlparam[nId].m_GroupIndexNum) 
	{
		return;
	}

#ifdef TIME_TEST
	TIME_TAKE(cvGetTickCount(),ts,"mvGetCurrenGroupIndex");
	ts = cvGetTickCount();
#endif

	mvAddNewObjRec(nId);

#ifdef TIME_TEST
	TIME_TAKE(cvGetTickCount(),ts,"mvAddNewObjRec");
	ts = cvGetTickCount();
#endif

	mvGoupPreditRec(nId);

#ifdef TIME_TEST
	TIME_TAKE(cvGetTickCount(),ts,"mvGoupPreditRec");
	ts = cvGetTickCount();
#endif

	// confirm the target which has low level confidence whether or not a vehicle 
#ifdef DETCOR_STAR
	DetcorBytrain(pInPutParam, nId);

#ifdef TIME_TAKE
	TIME_TAKE(cvGetTickCount(),ts,"DetcorBytrain");
	ts = cvGetTickCount();
#endif

#endif

	mvGetCurrenGroupIndex(nId);

	mvGenerateRioGrayByMask(nId);


#ifdef TIME_TEST
	TIME_TAKE(cvGetTickCount(),ts,"mvGenerateRioGrayByMask");
	ts = cvGetTickCount();
#endif


#ifdef TIME_TEST
	TIME_TAKE(cvGetTickCount(),ts,"mvImgEnhance");
	ts = cvGetTickCount();
#endif

	//fast角点检测
	mvCornerDetct(m_globlparam[nId].m_pGrayData, m_globlparam[nId].m_pMask,
			m_globlparam[nId].m_ImgWidth, m_globlparam[nId].m_ImgHeight,
			m_globlparam[nId].m_nCornerThresh, nId);

#ifdef TIME_TEST
	TIME_TAKE(cvGetTickCount(),ts,"mvCornerDetct");
	ts = cvGetTickCount();
#endif

	//角点描述
	mvFeatrueDescribe(m_globlparam[nId].m_pGrayData,
			adasize(m_globlparam[nId].m_ImgWidth,
					m_globlparam[nId].m_ImgHeight), nId);

#ifdef TIME_TEST
	TIME_TAKE(cvGetTickCount(),ts,"角点描述");
	ts = cvGetTickCount();
#endif

	mvTrajectorymatch(nId);

#ifdef TIME_TEST
	TIME_TAKE(cvGetTickCount(),ts,"mvTrajectorymatch");
	ts = cvGetTickCount();
#endif

	mvPreditGroup(pInPutParam, nId);

#ifdef TIME_TEST
	TIME_TAKE(cvGetTickCount(),ts,"mvPreditGroup");
	ts = cvGetTickCount();
#endif

	mvGroupGenerate(nId);

	if (m_globlparam[nId].pOriInPutParam->pOriGrayfram.dayOrNight == 0)
	{
		mvGroundLineDet(pInPutParam,nId);
	}
	else if (m_globlparam[nId].pOriInPutParam->pOriGrayfram.dayOrNight == 1)
	{
		mvGroundLineDet(pInPutParam, nId);
#ifdef USE_TAIL_LIGHT
		mvTailNightDetct(pInPutParam, nId);
#endif // USE_TAIL_LIGHT
	}

	mvKalmanFiter(nId);

#ifdef TIME_TEST
	TIME_TAKE(cvGetTickCount(),ts,"mvKalmanFiter");
	ts = cvGetTickCount();
#endif


	SWAP(m_globlparam[nId].m_preGrayData, m_globlparam[nId].m_pGrayData,
			pTemptr);

#ifdef TIME_TEST
	TIME_TAKE(cvGetTickCount(),ts,"SWAP");
	ts = cvGetTickCount();
#endif

}

/*
 Function process:
 + Do the multi-sacle tracking for pInPutParam.
 Fan-in :
 + FCW_TRACK_MultieTrack()
 Fan-out:
 + mvSingledScaleTrack()
 ATTENTION: __________
 */
static void mvMultileScaleTrack(PortInput *pInPutParam) {
	s32 nScaleId;
	for (nScaleId = scale_shink_1_id; nScaleId <= scale_shink_4_id; nScaleId++) 
	{
		m_globlparam[nScaleId].pOriInPutParam = pInPutParam;
		mvSingledScaleTrack(pInPutParam, nScaleId);
	}

}

/*
 *************************************************************
 Function process:
 + do the fushion for each search scale index
 Fan-in :
 + FCW_TRACK_MultieTrack()
 Fan-out:
 + N/A
 ATTENTION: __________
 */
static void mvfushionObj() {

	s32 i, j;
	obj_group *pgroup, *pComparedgroup;
	AdasRect srcRec;
	uint8_t bFilte;

	for (i = 0; i < MAX_LAYER_OBJ_GROUP_NUMS; i++) {
		pgroup = m_globlparam[scale_shink_1_id].m_pGroupSets + i;
		if (-1 == pgroup->nGroupId)
			continue;

 		srcRec.x = pgroup->rtContour.x >> scale_shink_2_id;
		srcRec.y = pgroup->rtContour.y >> scale_shink_2_id;
		srcRec.width = pgroup->rtContour.width >> scale_shink_2_id;
		srcRec.height = pgroup->rtContour.height >> scale_shink_2_id;
		srcRec.confidence = pgroup->rtContour.confidence;

		for (j = 0; j < MAX_LAYER_OBJ_GROUP_NUMS; j++) {
			pComparedgroup = m_globlparam[scale_shink_2_id].m_pGroupSets + j;
			if (-1 == pComparedgroup->nGroupId)
				continue;

			bFilte = (mvLapTrd(&srcRec, &pComparedgroup->rtContour, 0.7f)
				|| mvLapTrd(&pComparedgroup->rtContour, &srcRec, 0.7f));
			if (!bFilte)
				continue;

			if (pgroup->nTruelyObj && pComparedgroup->nTruelyObj)
			{
				if (pgroup->nPerLastDetFramseq == m_globlparam[scale_shink_1_id].scaleInput.nFramSeq)
				{
					if ( srcRec.confidence < pComparedgroup->rtContour.confidence )
					{
						mvClearGroupIfo(pgroup, 1);
					}
					else
					{
						// decide the two target if are the same
						s32 nDisTance = WS_ABS(pComparedgroup->rtContour.x + (pComparedgroup->rtContour.width >> 1) - \
							srcRec.x - (srcRec.width >> 1)) + \
							WS_ABS(pComparedgroup->rtContour.y + (pComparedgroup->rtContour.height >> 1) - \
							srcRec.y - (srcRec.height >> 1));

						if (WS_ABS(srcRec.width - pComparedgroup->rtContour.width) < 0.2 * srcRec.width || nDisTance < 0.4f * srcRec.width)
						{
							pComparedgroup->rtContour.confidence = pgroup->rtContour.confidence;
							mvClearGroupIfo(pgroup, 1);
						}
						else
						{
							imgage Carimg;
							float32_t fMatchScore;
							float32_t fAvgScore = 0.0f;
							AdasRect MatchRec;
							s32 noff = UPDATA_IMAGE_SIZE * UPDATA_IMAGE_SIZE;
							Carimg.nWid = UPDATA_IMAGE_SIZE;
							Carimg.nHig = UPDATA_IMAGE_SIZE;
							Carimg.ptr = (uint8_t*)m_globlparam[scale_shink_1_id].m_PublacSape;

							mvSetUpdataCarImg(pgroup->rtContour, &Carimg, scale_shink_1_id);
							for (j = 0; j < WS_MIN(pComparedgroup->nUpdataIndex, UPDATA_IMAGE_NUM); j++)
							{
								mvTemplatMatch(pComparedgroup->UpdatedImg[j], Carimg,
									adasrect(0, 0, UPDATA_IMAGE_SIZE,
									UPDATA_IMAGE_SIZE), &MatchRec, noff, 0,
									scale_shink_2_id, &fMatchScore);
								fAvgScore += fMatchScore;
							}
							fAvgScore /=
								WS_MIN(pComparedgroup->nUpdataIndex, UPDATA_IMAGE_NUM);

							if (fAvgScore > 0.5f)
							{
								pComparedgroup->rtContour.confidence = pgroup->rtContour.confidence;
								mvClearGroupIfo(pgroup, 1);
							}
							else
							{
								mvClearGroupIfo(pComparedgroup, 1);
							}
						}
					}

				}
				else if (pComparedgroup->nPerLastDetFramseq == m_globlparam[scale_shink_2_id].scaleInput.nFramSeq)
				{
					if (srcRec.confidence > pComparedgroup->rtContour.confidence)
					{
						mvClearGroupIfo(pComparedgroup, 1);
					}
					else
					{
						// decide the two target if are the same
						s32 nDisTance = WS_ABS(pComparedgroup->rtContour.x + (pComparedgroup->rtContour.width >> 1) - \
							srcRec.x - (srcRec.width >> 1)) + \
							WS_ABS(pComparedgroup->rtContour.y + (pComparedgroup->rtContour.height >> 1) - \
							srcRec.y - (srcRec.height >> 1));

						if ( WS_ABS(srcRec.width - pComparedgroup->rtContour.width) < 0.2 * srcRec.width ||  nDisTance < 0.4f * srcRec.width )
						{
							pgroup->rtContour.confidence = pComparedgroup->rtContour.confidence;
							mvClearGroupIfo(pComparedgroup, 1);
						}
						else
						{
							imgage Carimg;
							float32_t fMatchScore;
							float32_t fAvgScore = 0.0f;
							AdasRect MatchRec;
							s32 noff = UPDATA_IMAGE_SIZE * UPDATA_IMAGE_SIZE;
							Carimg.nWid = UPDATA_IMAGE_SIZE;
							Carimg.nHig = UPDATA_IMAGE_SIZE;
							Carimg.ptr = (uint8_t*)m_globlparam[scale_shink_2_id].m_PublacSape;

							mvSetUpdataCarImg(pComparedgroup->rtContour, &Carimg, scale_shink_2_id);
							for (j = 0; j < WS_MIN(pgroup->nUpdataIndex, UPDATA_IMAGE_NUM); j++)
							{
								mvTemplatMatch(pgroup->UpdatedImg[j], Carimg,
									adasrect(0, 0, UPDATA_IMAGE_SIZE,
									UPDATA_IMAGE_SIZE), &MatchRec, noff, 0,
									scale_shink_1_id, &fMatchScore);
								fAvgScore += fMatchScore;
							}
							fAvgScore /=
								WS_MIN(pgroup->nUpdataIndex, UPDATA_IMAGE_NUM);

							if (fAvgScore > 0.5f)
							{
								pgroup->rtContour.confidence = pComparedgroup->rtContour.confidence;
								mvClearGroupIfo(pComparedgroup, 1);
							}
							else
							{
								mvClearGroupIfo(pgroup, 1);
							}
						}
					}
				}
				else
				{
					if (pComparedgroup->rtContour.width > srcRec.width)
					{
						mvClearGroupIfo(pgroup, 1);
					}
					else
					{
						mvClearGroupIfo(pComparedgroup, 1);
					}
				}

			}
			else  if (pgroup->nTruelyObj && !pComparedgroup->nTruelyObj)
			{
				mvClearGroupIfo(pComparedgroup, 1);
			}
			else if (!pgroup->nTruelyObj && pComparedgroup->nTruelyObj)
			{
				mvClearGroupIfo(pgroup, 1);
			}
		}
	}


	for (i = 0; i < MAX_LAYER_OBJ_GROUP_NUMS; i++) {
		pgroup = m_globlparam[scale_shink_2_id].m_pGroupSets + i;
		if (-1 == pgroup->nGroupId)
			continue;

		srcRec.x = pgroup->rtContour.x >> scale_shink_2_id;
 		srcRec.y = pgroup->rtContour.y >> scale_shink_2_id;
		srcRec.width = pgroup->rtContour.width >> scale_shink_2_id;
		srcRec.height = pgroup->rtContour.height >> scale_shink_2_id;
		srcRec.confidence = pgroup->rtContour.confidence;

		for (j = 0; j < MAX_LAYER_OBJ_GROUP_NUMS; j++) {
			pComparedgroup = m_globlparam[scale_shink_4_id].m_pGroupSets + j;
			if (-1 == pComparedgroup->nGroupId)
				continue;

			bFilte = (mvLapTrd(&srcRec, &pComparedgroup->rtContour, 0.7f)
				|| mvLapTrd(&pComparedgroup->rtContour, &srcRec, 0.7f));
			if (!bFilte)
				continue;

			if (pgroup->nTruelyObj && pComparedgroup->nTruelyObj)
			{
				if (pgroup->nPerLastDetFramseq == m_globlparam[scale_shink_2_id].scaleInput.nFramSeq)
				{
					if ( srcRec.confidence < pComparedgroup->rtContour.confidence )
					{
						mvClearGroupIfo(pgroup, 1);
					}
					else
					{
						// decide the two target if are the same
						s32 nDisTance = WS_ABS(pComparedgroup->rtContour.x + (pComparedgroup->rtContour.width >> 1) - \
							srcRec.x - (srcRec.width >> 1)) + \
							WS_ABS(pComparedgroup->rtContour.y + (pComparedgroup->rtContour.height >> 1) - \
							srcRec.y - (srcRec.height >> 1));

						if (WS_ABS(srcRec.width - pComparedgroup->rtContour.width) < 0.2 * srcRec.width || nDisTance < 0.4f * srcRec.width)
						{
							pComparedgroup->rtContour.confidence = pgroup->rtContour.confidence;
							mvClearGroupIfo(pgroup, 1);
						}
						else
						{
							imgage Carimg;
							float32_t fMatchScore;
							float32_t fAvgScore = 0.0f;
							AdasRect MatchRec;
							s32 noff = UPDATA_IMAGE_SIZE * UPDATA_IMAGE_SIZE;
							Carimg.nWid = UPDATA_IMAGE_SIZE;
							Carimg.nHig = UPDATA_IMAGE_SIZE;
							Carimg.ptr = (uint8_t*)m_globlparam[scale_shink_2_id].m_PublacSape;

							mvSetUpdataCarImg(pgroup->rtContour, &Carimg, scale_shink_2_id);
							for (j = 0; j < WS_MIN(pComparedgroup->nUpdataIndex, UPDATA_IMAGE_NUM); j++)
							{
								mvTemplatMatch(pComparedgroup->UpdatedImg[j], Carimg,
									adasrect(0, 0, UPDATA_IMAGE_SIZE,
									UPDATA_IMAGE_SIZE), &MatchRec, noff, 0,
									scale_shink_1_id, &fMatchScore);
								fAvgScore += fMatchScore;
							}
							fAvgScore /=
								WS_MIN(pComparedgroup->nUpdataIndex, UPDATA_IMAGE_NUM);

							if (fAvgScore > 0.5f)
							{
								pComparedgroup->rtContour.confidence = pgroup->rtContour.confidence;
								mvClearGroupIfo(pgroup, 1);
							}
							else
							{
								mvClearGroupIfo(pComparedgroup, 1);
							}
						}
					}

				}
				else if (pComparedgroup->nPerLastDetFramseq == m_globlparam[scale_shink_4_id].scaleInput.nFramSeq)
				{
					if ( srcRec.confidence > pComparedgroup->rtContour.confidence )
					{
						mvClearGroupIfo(pComparedgroup, 1);
					}
					else
					{
						// decide the two target if are the same
						s32 nDisTance = WS_ABS(pComparedgroup->rtContour.x + (pComparedgroup->rtContour.width >> 1) - \
							srcRec.x - (srcRec.width >> 1)) + \
							WS_ABS(pComparedgroup->rtContour.y + (pComparedgroup->rtContour.height >> 1) - \
							srcRec.y - (srcRec.height >> 1));

						if (WS_ABS(srcRec.width - pComparedgroup->rtContour.width) < 0.2 * srcRec.width || nDisTance < 0.4f * srcRec.width)
						{
							pgroup->rtContour.confidence = pComparedgroup->rtContour.confidence;
							mvClearGroupIfo(pComparedgroup, 1);
						}
						else
						{
							imgage Carimg;
							float32_t fMatchScore;
							float32_t fAvgScore = 0.0f;
							AdasRect MatchRec;
							s32 noff = UPDATA_IMAGE_SIZE * UPDATA_IMAGE_SIZE;
							Carimg.nWid = UPDATA_IMAGE_SIZE;
							Carimg.nHig = UPDATA_IMAGE_SIZE;
							Carimg.ptr = (uint8_t*)m_globlparam[scale_shink_4_id].m_PublacSape;

							mvSetUpdataCarImg(pComparedgroup->rtContour, &Carimg, scale_shink_4_id);
							for (j = 0; j < WS_MIN(pgroup->nUpdataIndex, UPDATA_IMAGE_NUM); j++)
							{
								mvTemplatMatch(pgroup->UpdatedImg[j], Carimg,
									adasrect(0, 0, UPDATA_IMAGE_SIZE,
									UPDATA_IMAGE_SIZE), &MatchRec, noff, 0,
									scale_shink_2_id, &fMatchScore);
								fAvgScore += fMatchScore;
							}
							fAvgScore /=
								WS_MIN(pgroup->nUpdataIndex, UPDATA_IMAGE_NUM);

							if (fAvgScore > 0.5f)
							{
								pgroup->rtContour.confidence = pComparedgroup->rtContour.confidence;
								mvClearGroupIfo(pComparedgroup, 1);
							}
							else
							{
								mvClearGroupIfo(pgroup, 1);
							}
						}
					}
				}
				else
				{
					if (pComparedgroup->rtContour.width > srcRec.width)
					{
						mvClearGroupIfo(pgroup, 1);
					}
					else
					{
						mvClearGroupIfo(pComparedgroup, 1);
					}
				}
			}
			else  if (pgroup->nTruelyObj && !pComparedgroup->nTruelyObj)
			{
				mvClearGroupIfo(pComparedgroup, 1);
			}
			else if (!pgroup->nTruelyObj && pComparedgroup->nTruelyObj)
			{
				mvClearGroupIfo(pgroup, 1);
			}

		}
	}



	for (i = 0; i < MAX_LAYER_OBJ_GROUP_NUMS; i++) {
		pgroup = m_globlparam[scale_shink_1_id].m_pGroupSets + i;
		if (-1 == pgroup->nGroupId)
			continue;

		srcRec.x = pgroup->rtContour.x >> scale_shink_4_id;
		srcRec.y = pgroup->rtContour.y >> scale_shink_4_id;
		srcRec.width = pgroup->rtContour.width >> scale_shink_4_id;
		srcRec.height = pgroup->rtContour.height >> scale_shink_4_id;
		srcRec.confidence = pgroup->rtContour.confidence;

		for (j = 0; j < MAX_LAYER_OBJ_GROUP_NUMS; j++) {
			pComparedgroup = m_globlparam[scale_shink_4_id].m_pGroupSets + j;
			if (-1 == pComparedgroup->nGroupId)
				continue;

			bFilte = (mvLapTrd(&srcRec, &pComparedgroup->rtContour, 0.7f)
				|| mvLapTrd(&pComparedgroup->rtContour, &srcRec, 0.7f));
			if (!bFilte)
				continue;

			if (pgroup->nTruelyObj && pComparedgroup->nTruelyObj)
			{
				if (pgroup->nPerLastDetFramseq == m_globlparam[scale_shink_1_id].scaleInput.nFramSeq)
				{
					if (srcRec.confidence < pComparedgroup->rtContour.confidence )
					{
						mvClearGroupIfo(pgroup, 1);
					}
					else
					{
						// decide the two target if are the same
						s32 nDisTance = WS_ABS(pComparedgroup->rtContour.x + (pComparedgroup->rtContour.width >> 1) - \
							srcRec.x - (srcRec.width >> 1)) + \
							WS_ABS(pComparedgroup->rtContour.y + (pComparedgroup->rtContour.height >> 1) - \
							srcRec.y - (srcRec.height >> 1));

						if (WS_ABS(srcRec.width - pComparedgroup->rtContour.width) < 0.2 * srcRec.width || nDisTance < 0.4f * srcRec.width)
						{
							pComparedgroup->rtContour.confidence = pgroup->rtContour.confidence;
							mvClearGroupIfo(pgroup, 1);
						}
						else
						{
							imgage Carimg;
							float32_t fMatchScore;
							float32_t fAvgScore = 0.0f;
							AdasRect MatchRec;
							s32 noff = UPDATA_IMAGE_SIZE * UPDATA_IMAGE_SIZE;
							Carimg.nWid = UPDATA_IMAGE_SIZE;
							Carimg.nHig = UPDATA_IMAGE_SIZE;
							Carimg.ptr = (uint8_t*)m_globlparam[scale_shink_1_id].m_PublacSape;

							mvSetUpdataCarImg(pgroup->rtContour, &Carimg, scale_shink_1_id);
							for (j = 0; j < WS_MIN(pComparedgroup->nUpdataIndex, UPDATA_IMAGE_NUM); j++)
							{
								mvTemplatMatch(pComparedgroup->UpdatedImg[j], Carimg,
									adasrect(0, 0, UPDATA_IMAGE_SIZE,
									UPDATA_IMAGE_SIZE), &MatchRec, noff, 0,
									scale_shink_1_id, &fMatchScore);
								fAvgScore += fMatchScore;
							}
							fAvgScore /=
								WS_MIN(pComparedgroup->nUpdataIndex, UPDATA_IMAGE_NUM);

							if (fAvgScore > 0.5f)
							{
								pComparedgroup->rtContour.confidence = pgroup->rtContour.confidence;
								mvClearGroupIfo(pgroup, 1);
							}
							else
							{
								mvClearGroupIfo(pComparedgroup, 1);
							}
						}
					}

				}
				else if (pComparedgroup->nPerLastDetFramseq == m_globlparam[scale_shink_4_id].scaleInput.nFramSeq)
				{
					if (srcRec.confidence > pComparedgroup->rtContour.confidence )
					{
						mvClearGroupIfo(pComparedgroup, 1);
					}
					else
					{
						// decide the two target if are the same
						s32 nDisTance = WS_ABS(pComparedgroup->rtContour.x + (pComparedgroup->rtContour.width >> 1) - \
							srcRec.x - (srcRec.width >> 1)) + \
							WS_ABS(pComparedgroup->rtContour.y + (pComparedgroup->rtContour.height >> 1) - \
							srcRec.y - (srcRec.height >> 1));

						if (WS_ABS(srcRec.width - pComparedgroup->rtContour.width) < 0.2 * srcRec.width || nDisTance < 0.4f * srcRec.width)
						{
							pgroup->rtContour.confidence = pComparedgroup->rtContour.confidence;
							mvClearGroupIfo(pComparedgroup, 1);
						}
						else
						{
							imgage Carimg;
							float32_t fMatchScore;
							float32_t fAvgScore = 0.0f;
							AdasRect MatchRec;
							s32 noff = UPDATA_IMAGE_SIZE * UPDATA_IMAGE_SIZE;
							Carimg.nWid = UPDATA_IMAGE_SIZE;
							Carimg.nHig = UPDATA_IMAGE_SIZE;
							Carimg.ptr = (uint8_t*)m_globlparam[scale_shink_4_id].m_PublacSape;

							mvSetUpdataCarImg(pComparedgroup->rtContour, &Carimg, scale_shink_4_id);
							for (j = 0; j < WS_MIN(pgroup->nUpdataIndex, UPDATA_IMAGE_NUM); j++)
							{
								mvTemplatMatch(pgroup->UpdatedImg[j], Carimg,
									adasrect(0, 0, UPDATA_IMAGE_SIZE,
									UPDATA_IMAGE_SIZE), &MatchRec, noff, 0,
									scale_shink_1_id, &fMatchScore);
								fAvgScore += fMatchScore;
							}
							fAvgScore /=
								WS_MIN(pgroup->nUpdataIndex, UPDATA_IMAGE_NUM);

							if (fAvgScore > 0.5f)
							{
								pgroup->rtContour.confidence = pComparedgroup->rtContour.confidence;
								mvClearGroupIfo(pComparedgroup, 1);
							}
							else
							{
								mvClearGroupIfo(pgroup, 1);
							}
						}
					}
				}
				else
				{
					if (pComparedgroup->rtContour.width > srcRec.width)
					{
						mvClearGroupIfo(pgroup, 1);
					}
					else
					{
						mvClearGroupIfo(pComparedgroup, 1);
					}
				}
			}
			else  if (pgroup->nTruelyObj && !pComparedgroup->nTruelyObj)
			{
				mvClearGroupIfo(pComparedgroup, 1);
			}
			else if (!pgroup->nTruelyObj && pComparedgroup->nTruelyObj)
			{
				mvClearGroupIfo(pgroup, 1);
			}

		}
	}
}

/*
 Function process:
 + set the value of pgroup->pMotion
 Fan-in :
 + mvMotionState()
 Fan-out:
 + N/A
 ATTENTION: __________
 */
static void mvSetGroupMotion(obj_group *pgroup, s32 nId, float32_t fZDis,
							 float32_t fXDis, float32_t fDelVanish) {
								 Motion *pGroupMotion;

								 pGroupMotion = pgroup->pMotion + (pgroup->nMotionLeng & MAX_MOTION_BIT);

								 pGroupMotion->z_fdis = fZDis;
								 pGroupMotion->x_fdis = fXDis;
								 pGroupMotion->ts = m_globlparam[nId].scaleInput.objTime;
								 pGroupMotion->delVanish = fDelVanish;
								 pGroupMotion->z_fSpeed = m_globlparam[nId].pOriInPutParam->pOriGrayfram.CANData.fSpeed;

								 pGroupMotion->groupRec.x = (pgroup->rtContour.x << nId); 
								 pGroupMotion->groupRec.y = (pgroup->rtContour.y << nId);
								 pGroupMotion->groupRec.width = (pgroup->rtContour.width << nId);
								 pGroupMotion->groupRec.height = (pgroup->rtContour.height << nId);

								 pgroup->nMotionLeng++;

}

/*
 Function process:
 + set the value of pgroup->pMotion of each sacle in m_globlparam[nId].m_pGroupSets
 Fan-in :
 + FCW_TRACK_MultieTrack()
 Fan-out:
 + mvSetGroupMotion()
 ATTENTION: __________
 */
static void mvMotionState() {
	s32 i, nId;
	obj_group *pgroup = 0;
	s32 nBottom_Z_Car, nBottom_X_Car;
	double fx, fz, delVanish;
	g_MuliTracker[g_OutIndex & 1].nTrackeNum = 0;

	for (nId = scale_shink_1_id; nId <= scale_shink_4_id; nId++) {

		for (i = 0; i < MAX_LAYER_OBJ_GROUP_NUMS; i++) {
			pgroup = m_globlparam[nId].m_pGroupSets + i;

			if (-1 != pgroup->nGroupId) {

				nBottom_Z_Car = 
						((pgroup->rtContour.y + pgroup->rtContour.height
								) << nId );

				nBottom_X_Car = 
						((pgroup->rtContour.x + pgroup->rtContour.width / 2)
								<<  nId );

#ifdef DETCOR_STAR
				LDWS_Get_Dist_xz(nBottom_X_Car, nBottom_Z_Car, &fx, &fz,
						&delVanish);
#else
				fz =0;
				fx =0;
				delVanish = 0;
#endif
				mvSetGroupMotion(pgroup, nId, (float32_t)fz, (float32_t)fx, (float32_t)delVanish);
			}
		}
	}

}

/*
 Function process:
 + caculate the time difference
 Fan-in :
 + mvGroupTimeToCollison()
 Fan-out:
 + N/A
 ATTENTION: __________
 */
static void mvSubTime(const systime *pA, const systime *pB, systime *pC) {
	pC->wHour = pA->wHour - pB->wHour;
	pC->wMin = pA->wMin - pB->wMin;
	pC->wSec = pA->wSec - pB->wSec;
	pC->wMilSec = pA->wMilSec - pB->wMilSec;

}

/*
 Function process:
 + Caculate the TTC and collison path for pgroup
 Fan-in :
 + mvTTC()
 Fan-out:
 + mvSubTime()
 ATTENTION: __________
 */
static void mvGroupTimeToCollison(obj_group *pgroup, s32 nId) {

	s32 nLengTrd = 20;
	Motion *pNearestMotion = 0;
	Motion *pfarestMotion = 0;
	float32_t dLmda, wLmda, lmda;
	float32_t fTimeSub;
	double W_HostCar_Lef, W_HostCar_Rig;
	double W_Group_Lef, W_Group_Rig;
	systime SystimeSub;
	double tempRight = 0.0;
	double tempLeft = 0.0;

	int Eu, Ev, Cx, Cy;
	LDWS_Get_inter_Pamer_N(&Eu, &Ev, &Cx, &Cy);

	/* get the Nearest group */
	pNearestMotion = pgroup->pMotion
			+ ((pgroup->nMotionLeng - 1) & MAX_MOTION_BIT);
	pNearestMotion->bInCollishionPath = 0;
	pNearestMotion->fTTC = TTC_MAX;
	pNearestMotion->dTTC = TTC_MAX;
	if (!pgroup->nTruelyObj) 
	{
		return;
	}

	//calculate the Collishion Path
	LDWS_GetVanishPointSet(&g_pLDWSVPoint);

	//printf("maxVPoint(x,y):%d,%d\n",(g_pLDWSVPoint).x,(g_pLDWSVPoint).y);
	W_HostCar_Lef = LDWS_GetXofWorld((g_pLDWSVPoint).x,
		(pgroup->rtContour.y + pgroup->rtContour.height)
		<< nId) - HALF_CAR_WIDTH;

	W_HostCar_Rig = W_HostCar_Lef + (HALF_CAR_WIDTH * 2);

	W_Group_Lef = LDWS_GetXofWorld(pgroup->rtContour.x << nId,
		(pgroup->rtContour.y + pgroup->rtContour.height) << nId);

	W_Group_Rig = LDWS_GetXofWorld(
		(pgroup->rtContour.x + pgroup->rtContour.width) << nId,
		(pgroup->rtContour.y + pgroup->rtContour.height) << nId);



	if (WS_ABS(W_HostCar_Lef)<50 && WS_ABS(W_HostCar_Rig)<50 && WS_ABS(W_Group_Lef) < 50 && WS_ABS(W_Group_Rig)< 50)
	{
		tempRight = WS_MAX(W_Group_Rig, W_HostCar_Rig);
		tempLeft = WS_MIN(W_HostCar_Lef, W_Group_Lef);
		//my_printf("tempRight %f tempLeft %f\n", tempRight, tempLeft);
		if (tempRight - tempLeft < (HALF_CAR_WIDTH * 2 + W_Group_Rig - W_Group_Lef)) {
				pNearestMotion->bInCollishionPath = 1;
				//my_printf("bin 1\n");
				//return;
		} else if (W_Group_Rig < W_HostCar_Lef) {
			pNearestMotion->bInCollishionPath = 2;
			//my_printf("bin 2\n");
		} else if (W_Group_Lef > W_HostCar_Rig) {
			pNearestMotion->bInCollishionPath = 3;
			//my_printf("bin 3\n");
		}
		else
		{
			//my_printf("bin unknow 1!\n");
		}

	}
	else
	{
		//my_printf("bin unknow 2!\n");
	}
	//my_printf("bin %d, :%d, %f ,%f, %f, %f, %f\n", pgroup->nGroupId, pNearestMotion->bInCollishionPath, W_HostCar_Lef, W_HostCar_Rig, W_Group_Lef, W_Group_Rig, pNearestMotion->z_fdis);

	//delete left & right side target
	/*if ( (pNearestMotion->bInCollishionPath != 1) && (pNearestMotion->groupRec.width << nId) > 200 )
	{
	mvClearGroupIfo(pgroup, 1);
	}*/

#if 0
	//calculate the TTC
	if ( pgroup->nMotionLeng  > nLengTrd )
	{
		uint8_t tempLeng = (pgroup->nMotionLeng - 1) % nLengTrd;

		/*system Kalman filter*/
		if ( tempLeng == 0)
		{
			pfarestMotion = pgroup->pMotion
				+ ((pgroup->nMotionLeng - 1 - nLengTrd) & MAX_MOTION_BIT);

#if 1
		    mvSubTime(&pNearestMotion->ts, &pfarestMotion->ts, &SystimeSub);
		    //fTimeSub = SystimeSub.wHour * 3600 + SystimeSub.wMin * 60 + SystimeSub.wSec
		    //	+ SystimeSub.wMilSec / 1000.0f;
		    fTimeSub = SystimeSub.wMilSec / 1000.0f;
		    //my_printf("fTimeSub : %f\n", fTimeSub);
#else
		    fTimeSub = WS_MIN(nLengTrd, pgroup->nMotionLeng) * 1.0f / FRAM_RATIO;
#endif

			AdasRect tempRec;
			tempRec.x = pNearestMotion->groupRec.x;
			tempRec.y = pNearestMotion->groupRec.y + pNearestMotion->groupRec.height;
			tempRec.width = pNearestMotion->groupRec.width;
			tempRec.height = pNearestMotion->groupRec.height;

			int Cy = LDWS_GetVanishY();
			//sysKalmanFilter(&pgroup->sysKalmState, 0.05 * nLengTrd, tempRec, (double)(Cy));
			sysKalmanFilter(&pgroup->sysKalmState, fTimeSub, tempRec, (double)(Cy));

			double zSpeed = (pgroup->sysKalmState.x[3] == 0) ? 1e-5 : pgroup->sysKalmState.x[3];
			pNearestMotion->fTTC = -pgroup->sysKalmState.x[1] / zSpeed;
		}
		else
		{
			pfarestMotion = pgroup->pMotion
				+ ((pgroup->nMotionLeng - 1 - tempLeng) & MAX_MOTION_BIT);

#if 1
			mvSubTime(&pNearestMotion->ts, &pfarestMotion->ts, &SystimeSub);
			//fTimeSub = SystimeSub.wHour * 3600 + SystimeSub.wMin * 60 + SystimeSub.wSec
			//	+ SystimeSub.wMilSec / 1000.0f;
			fTimeSub = SystimeSub.wMilSec / 1000.0f;
			//my_printf("fTimeSub : %f\n", fTimeSub);
#else
			fTimeSub = WS_MIN(nLengTrd, pgroup->nMotionLeng) * 1.0f / FRAM_RATIO;
#endif

			double zSpeed = (pgroup->sysKalmState.x[3] == 0) ? 1e-5 : pgroup->sysKalmState.x[3];
			//pNearestMotion->fTTC = -(pgroup->sysKalmState.x[1] / zSpeed + 0.05 * tempLeng);
			pNearestMotion->fTTC = -(pgroup->sysKalmState.x[1] / zSpeed + fTimeSub);
		}
	}


	float32_t vspMean = 0, vspVar = 0;
	int i;
	for (i = 0; i < nLengTrd; i++)
	{
		s32 temp;
		pfarestMotion = pgroup->pMotion
			+ ((pgroup->nMotionLeng - 1 - i) & MAX_MOTION_BIT);
		vspMean += pfarestMotion->delVanish;
	}
	vspMean /= nLengTrd;

	for (i = 0; i < nLengTrd; i++)
	{
		pfarestMotion = pgroup->pMotion
			+ ((pgroup->nMotionLeng - 1 - i) & MAX_MOTION_BIT);
		vspVar += (float)WS_ABS(pfarestMotion->delVanish - vspMean);
	}
	vspVar /= nLengTrd;
	if (vspVar > 1.5f)
	{
		pNearestMotion->fTTC = TTC_MAX;
		//printf("vspVar is %f\n", vspVar);
	}

	if (pNearestMotion->z_fSpeed > (URBAN_SPEED / 3.6) && pNearestMotion->z_fdis > 0)
	{
		pNearestMotion->dTTC = pNearestMotion->z_fdis / pNearestMotion->z_fSpeed;
	}

#else

	//calculate the TTC
	if (pgroup->nMotionLeng >= nLengTrd)//&& (pNearestMotion->groupRec.width << nId) > 48)
	{
		int16_t vanishPty;
		pfarestMotion = pgroup->pMotion
			+ ((pgroup->nMotionLeng - nLengTrd) & MAX_MOTION_BIT);

		vanishPty = (int16_t)((pNearestMotion->delVanish + pfarestMotion->delVanish) / 2);

		dLmda = -1;
		if (pfarestMotion->groupRec.y + pfarestMotion->groupRec.height > vanishPty &&
			pNearestMotion->groupRec.y + pNearestMotion->groupRec.height > vanishPty)
		{
			dLmda = (float32_t)(pNearestMotion->groupRec.y + pNearestMotion->groupRec.height - vanishPty)
				/ (pfarestMotion->groupRec.y + pfarestMotion->groupRec.height - vanishPty);
		}

		wLmda = -1;
		if (pfarestMotion->groupRec.width != 0)
		{
			wLmda = (float32_t)( pNearestMotion->groupRec.width )
				/ (pfarestMotion->groupRec.width );
		}

		lmda = 1;
		if (wLmda > 1.05f && dLmda > 1)
		{
		    lmda = WS_MIN(dLmda, wLmda);
		}

#if 1
		mvSubTime(&pNearestMotion->ts, &pfarestMotion->ts, &SystimeSub);
		//fTimeSub = SystimeSub.wHour * 3600 + SystimeSub.wMin * 60 + SystimeSub.wSec
		//	+ SystimeSub.wMilSec / 1000.0f;
		fTimeSub = SystimeSub.wMilSec / 1000.0f;
		//my_printf("fTimeSub : %f\n", fTimeSub);
#else
		fTimeSub = WS_MIN(nLengTrd, pgroup->nMotionLeng)*1.0f / FRAM_RATIO;
#endif

		if (lmda != 1)
		{
			pNearestMotion->fTTC = -fTimeSub / (1 - lmda);
		}

		if (pNearestMotion->z_fSpeed > (URBAN_SPEED / 3.6) && pNearestMotion->z_fdis > 0)
		{
			pNearestMotion->dTTC = pNearestMotion->z_fdis / pNearestMotion->z_fSpeed;
		}
	}
#endif

}

/*
Function process:
+ caculate TTC
Fan-in :
+ FCW_TRACK_MultieTrack()
Fan-out:
+ mvGroupTimeToCollison()
ATTENTION: __________
*/
static void mvTTC() {
	s32 i, j;
	s32 nObjGroupNum = 0;
	obj_group * pGroup = 0;
	obj_group * pOtherGroup = 0;
	obj_group * pObjGroup[MAX_OBJ_GROUP_NUMS] = { 0 };
	s32 pObjGroupScaleId[MAX_OBJ_GROUP_NUMS] = { 0 };
	Motion *pNearestMotion = 0;
	Motion *pOtherNearestMotion = 0;

	for (i = scale_shink_1_id; i <= scale_shink_4_id; i++) {
		for (j = 0; j < MAX_LAYER_OBJ_GROUP_NUMS; j++) {
			pGroup = m_globlparam[i].m_pGroupSets + j;

			if (-1 != pGroup->nGroupId) {
				mvGroupTimeToCollison(pGroup, i);
				//mvGroupTimeToCollison_point(pGroup, i);

				pObjGroup[nObjGroupNum] = pGroup;
				pObjGroupScaleId[nObjGroupNum] = i;
				nObjGroupNum++;

			}
		}
	}


	for (i = 0; i < nObjGroupNum - 1; i++) {
		pGroup = pObjGroup[i];

		pNearestMotion = pGroup->pMotion
			+ ((pGroup->nMotionLeng - 1) & MAX_MOTION_BIT);

		if (!pNearestMotion->bInCollishionPath) {
			continue;
		}

		for (j = i + 1; j < nObjGroupNum; j++) {
			pOtherGroup = pObjGroup[j];
			pOtherNearestMotion = pOtherGroup->pMotion
				+ ((pOtherGroup->nMotionLeng - 1) & MAX_MOTION_BIT);

			if (pOtherNearestMotion->bInCollishionPath
				== pNearestMotion->bInCollishionPath) {
				if (((pOtherGroup->rtContour.y + pOtherGroup->rtContour.height)
					<< pObjGroupScaleId[j])
		>(pGroup->rtContour.y + pGroup->rtContour.height)
		<< pObjGroupScaleId[i]) {
					pNearestMotion->bInCollishionPath = 0;

				}
				else {
					pOtherNearestMotion->bInCollishionPath = 0;
				}
			}

		}
	}

}

/*
 *************************************************************
 Function process:
 + Set the tracking result to g_MuliTracker
 Fan-in :
 + FCW_TRACK_MultieTrack()
 Fan-out:
 + N/A
 ATTENTION: __________
 */
static void mvSetTrackerReult() {

	s32 i, j, nTrackNum;
	obj_group *pgroup = 0;
	Motion *pNearestMotion = 0;
	//trakobj *pobjTrack;
	float32_t fTTC_WARNING_TIME;
	s32 nId;

	g_MuliTracker[g_OutIndex & 1].nTrackeNum = 0;

	for (nId = scale_shink_1_id; nId <= scale_shink_4_id; nId++) {
		nTrackNum = g_MuliTracker[g_OutIndex & 1].nTrackeNum;

		if (nTrackNum >= MAX_OBJ_GROUP_NUMS) {
			return;
		}

		for (i = 0; i < MAX_LAYER_OBJ_GROUP_NUMS; i++) {
			pgroup = m_globlparam[nId].m_pGroupSets + i;

			if (-1 != pgroup->nGroupId) {
				g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].nFramSeq =
						m_globlparam[nId].scaleInput.nFramSeq;

				if (scale_shink_1_id == nId) {
					g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].objRec.x =
							pgroup->rtContour.x >> scale_shink_2_id;
					g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].objRec.y =
							pgroup->rtContour.y >> scale_shink_2_id;
					g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].objRec.width =
							pgroup->rtContour.width >> scale_shink_2_id;
					g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].objRec.height =
							pgroup->rtContour.height >> scale_shink_2_id;
					g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].objRec.confidence =
							pgroup->rtContour.confidence;

					for (j = 0;
							j
									< WS_MIN(pgroup->Centr.nTrackLen,MAX_CORNER_OF_TRACK);
							j++) {
						g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].pCenTrajecy[j].point.x =
								(s32) (pgroup->Centr.pTrackPoint[j].point.x)
										>> scale_shink_2_id;
						g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].pCenTrajecy[j].point.y =
								(s32) (pgroup->Centr.pTrackPoint[j].point.y)
										>> scale_shink_2_id;
					}
				} else if (scale_shink_2_id == nId) {
					g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].objRec =
							pgroup->rtContour;
					memcpy(
							g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].pCenTrajecy,
							pgroup->Centr.pTrackPoint,
							sizeof(TrackPoint)
									* WS_MIN( pgroup->Centr.nTrackLen,MAX_CORNER_OF_TRACK));

				} else if (scale_shink_4_id == nId) {
					g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].objRec.x =
							pgroup->rtContour.x << scale_shink_2_id;
					g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].objRec.y =
							pgroup->rtContour.y << scale_shink_2_id;
					g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].objRec.width =
							pgroup->rtContour.width << scale_shink_2_id;
					g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].objRec.height =
							pgroup->rtContour.height << scale_shink_2_id;
					g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].objRec.confidence =
							pgroup->rtContour.confidence;

					for (j = 0;
							j
									< WS_MIN(pgroup->Centr.nTrackLen,MAX_CORNER_OF_TRACK);
							j++) {
						g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].pCenTrajecy[j].point.x =
								(s32) (pgroup->Centr.pTrackPoint[j].point.x)
										<< scale_shink_2_id;
						g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].pCenTrajecy[j].point.y =
								(s32) (pgroup->Centr.pTrackPoint[j].point.y)
										<< scale_shink_2_id;
					}
				}

				if (m_globlparam[nId].scaleInput.nFramSeq
						== pgroup->CarBottom.nDetBottomFam) {
					pgroup->CarBottom.nDistoBottom = g_MuliTracker[g_OutIndex
							& 1].pTrackerset[nTrackNum].DisToBottom =
							(pgroup->rtContour.y + pgroup->rtContour.height
									- pgroup->CarBottom.nDetYPos);
				} else if (!pgroup->nMotionLeng) {
					pgroup->CarBottom.nDistoBottom = 0;
				}

				g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].nTrackLen =
						pgroup->Centr.nTrackLen;
				g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].nId =
						pgroup->nGroupId;
				g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].bTrue =
						pgroup->nTruelyObj;
				g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].DisToBottom =
						(pgroup->CarBottom.nDistoBottom << nId) >> 1;
				g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].pMotion =
						pgroup->pMotion;
				g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].nMotionLeng =
						pgroup->nMotionLeng;
				pNearestMotion = pgroup->pMotion
					+ ((pgroup->nMotionLeng - 1) & MAX_MOTION_BIT);
#if 1
				fTTC_WARNING_TIME = 50.0f;
				g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].fTTC = pgroup->pMotion[(pgroup->nMotionLeng-1) & MAX_MOTION_BIT].fTTC;
				if (WS_ABS(
					pgroup->pMotion[(pgroup->nMotionLeng - 1)
					& MAX_MOTION_BIT].fTTC) < fTTC_WARNING_TIME) {
					if (pgroup->nMotionLeng > 3
						&& (((pgroup->pMotion[(pgroup->nMotionLeng - 1)
						& MAX_MOTION_BIT].fTTC > 0)
						&& (pgroup->pMotion[(pgroup->nMotionLeng - 1)
						& MAX_MOTION_BIT].fTTC
						< fTTC_WARNING_TIME))
						|| ((pgroup->pMotion[(pgroup->nMotionLeng
						- 2) & MAX_MOTION_BIT].fTTC > 0)
						&& (pgroup->pMotion[(pgroup->nMotionLeng
						- 2) & MAX_MOTION_BIT].fTTC
						< fTTC_WARNING_TIME))
						|| (pgroup->pMotion[(pgroup->nMotionLeng - 3)
						& MAX_MOTION_BIT].fTTC > 0
						&& pgroup->pMotion[(pgroup->nMotionLeng
						- 3) & MAX_MOTION_BIT].fTTC
						< fTTC_WARNING_TIME))) {

						//TTC median filter
						if (pgroup->pMotion[(pgroup->nMotionLeng - 1) & MAX_MOTION_BIT].fTTC > WS_MAX(pgroup->pMotion[(pgroup->nMotionLeng - 2) & MAX_MOTION_BIT].fTTC, pgroup->pMotion[(pgroup->nMotionLeng - 3) & MAX_MOTION_BIT].fTTC))
						{
							g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].fTTC = WS_MAX(pgroup->pMotion[(pgroup->nMotionLeng - 2) & MAX_MOTION_BIT].fTTC, pgroup->pMotion[(pgroup->nMotionLeng - 3) & MAX_MOTION_BIT].fTTC);
						}

						if (pgroup->pMotion[(pgroup->nMotionLeng - 1) & MAX_MOTION_BIT].fTTC < WS_MIN(pgroup->pMotion[(pgroup->nMotionLeng - 2) & MAX_MOTION_BIT].fTTC, pgroup->pMotion[(pgroup->nMotionLeng - 3) & MAX_MOTION_BIT].fTTC))
						{
							g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].fTTC = WS_MIN(pgroup->pMotion[(pgroup->nMotionLeng - 2) & MAX_MOTION_BIT].fTTC, pgroup->pMotion[(pgroup->nMotionLeng - 3) & MAX_MOTION_BIT].fTTC);
						}
					}
				}
#else

				// FCW
				if ( pgroup->pMotion[(pgroup->nMotionLeng - 1) & MAX_MOTION_BIT].fTTC > 0 && pgroup->pMotion[(pgroup->nMotionLeng - 1) & MAX_MOTION_BIT].fTTC < pgroup->pMotion[(pgroup->nMotionLeng - 1) & MAX_MOTION_BIT].dTTC )
					g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].fTTC = TTC_MAX;
				else
					g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].fTTC = pgroup->pMotion[(pgroup->nMotionLeng - 1) & MAX_MOTION_BIT].fTTC;
#endif

				if (pNearestMotion->z_fSpeed > 0 && pNearestMotion->bInCollishionPath == 1)
				{
					my_printf("ID: %d, speed: %f, limit speed: %f, distance: %f dTTC: %f fTTC: %f pNearestMotion->bInCollishionPath:%d\n", pgroup->nGroupId, pgroup->sysKalmState.x[3], URBAN_SPEED / 3.6, pNearestMotion->z_fdis, pNearestMotion->z_fdis / pNearestMotion->z_fSpeed, pNearestMotion->fTTC, pNearestMotion->bInCollishionPath);
				}

				// HMW
				g_MuliTracker[g_OutIndex & 1].pTrackerset[nTrackNum].dTTC = pgroup->pMotion[(pgroup->nMotionLeng - 1) & MAX_MOTION_BIT].dTTC;
			
				nTrackNum++;
				if (m_globlparam[nId].pOriInPutParam->pOriGrayfram.dayOrNight == 0)
				{
					if (pgroup->Centr.nTrackLen % 10 == 0)
						pgroup->rtContour.confidence--;
				}
				else if (m_globlparam[nId].pOriInPutParam->pOriGrayfram.dayOrNight == 1)
				{
					if (pgroup->Centr.nTrackLen % 5 == 0)
						pgroup->rtContour.confidence *= 0.9;
				}
				
				if (pgroup->rtContour.confidence < 0)
					pgroup->rtContour.confidence = 0;
			}

		}

		g_MuliTracker[g_OutIndex & 1].nTrackeNum = nTrackNum;
	}
}


/****************************************************************************************************************/
/*
 Function process:
 + malloc the memory for Tracking
 Fan-in :
 + main()
 Fan-out:
 + N/A
 ATTENTION: ImgSize is the size of resized img (1/2 of orig image)
 */
void FCW_TRACK_Init_adas_global_param(AdaSize ImgSize) {

	s32 i, j;
	uint8_t *pTr = 0;
	obj_group *pObjGroup = 0;
	trajecy *ptrajecy = 0;
	s32 nalloclSize = 0;
	s32 nCornerThresh[3] = { 7, 15, 15 };
	s32 nId = 0;
	static s32 initMalloc = 0;
	AdaSize oriSiz = ImgSize;
	float fscale[3] = { 2.0f, 1, 0.5f };

#ifdef USE_TAIL_LIGHT
	s32 rt = 0;
#endif // USE_TAIL_LIGHT


	for (nId = 0; nId < 3; nId++) {
		ImgSize.width = (int16_t)(oriSiz.width * fscale[nId]);
		ImgSize.height = (int16_t)(oriSiz.height * fscale[nId]);

		if (ALLOC_SUCCESS_STATE == m_globlparam[nId].m_nInitSuccess) {
			return;
		}

		nalloclSize = MAX_OBJ_GROUP_NUMS * sizeof(trakobj)
				+ MAX_OBJ_GROUP_NUMS * sizeof(TrackPoint) * MAX_CORNER_OF_TRACK
				+\
 +MAX_OBJ_GROUP_NUMS * sizeof(Motion) * MAX_MOTION_NUM\

				+ sizeof(xy) * MAX_XY_CORNER + sizeof(xy) * MAX_XY_CORNER
				+ sizeof(s32) * MAX_IMG_HEIGHT + sizeof(s32) * MAX_XY_CORNER
				+ sizeof(AdasCorner) * MAX_CORNERS_OF_NONMAX
				+ sizeof(AdasCorner) * MAX_CORNERS_PER_FRAME
				+ sizeof(uint8_t) * MAX_CORNERS_PER_FRAME * SURF_DESC_DIMENTION
				+ sizeof(AdasCorner) * MAX_CORNERS_OF_NONMAX
				+ sizeof(uint8_t) * MAX_CORNERS_PER_FRAME * SURF_DESC_DIMENTION
				+ sizeof(AdasRect) * MAX_NEW_GROUP_PER_FRAM
				+ sizeof(uint8_t) * ImgSize.width * ImgSize.height
				+ sizeof(uint8_t) * ImgSize.width * ImgSize.height
				+ sizeof(uint8_t) * ImgSize.width * ImgSize.height
				+ sizeof(obj_group) * MAX_LAYER_OBJ_GROUP_NUMS
				+ MAX_LAYER_OBJ_GROUP_NUMS * sizeof(s32) * 2
				+ MAX_LAYER_OBJ_GROUP_NUMS * sizeof(TrackPoint)
						* MAX_CORNER_OF_TRACK\

				+ MAX_LAYER_OBJ_GROUP_NUMS * sizeof(Motion) * MAX_MOTION_NUM
				+ MAX_LAYER_OBJ_GROUP_NUMS * sizeof(trajecy)
						* MAX_TRACKS_NUM_OF_GROUP
				+ MAX_LAYER_OBJ_GROUP_NUMS * MAX_TRACKS_NUM_OF_GROUP
						* (sizeof(TrackPoint) * MAX_CORNER_OF_TRACK
								+ sizeof(uint8_t) * SURF_DESC_DIMENTION
								+ sizeof(uint8_t) * SURF_DESC_DIMENTION)
				+ MAX_LAYER_OBJ_GROUP_NUMS
						* (MAX_TEMPLAT_NCC_TRACK_SIZE
								* MAX_TEMPLAT_NCC_TRACK_SIZE)
				+ MAX_TRACKS_NUM_OF_GROUP * sizeof(s32)
				+ MAX_TRACKS_NUM_OF_GROUP * sizeof(s32)
				+ MAX_TRACKS_NUM_OF_GROUP * MAX_TRACKS_NUM_OF_GROUP
						* sizeof(float32_t)
				+ MAX_PUBLIC_SPACE_SIZE * sizeof(s32)
				+ MAX_LAYER_OBJ_GROUP_NUMS * MAX_GROUP_IMG_BYTE
				+ MAX_LAYER_OBJ_GROUP_NUMS
				+ MAX_LAYER_OBJ_GROUP_NUMS * UPDATA_IMAGE_NUM
						* (UPDATA_IMAGE_SIZE * UPDATA_IMAGE_SIZE)
				+ sizeof(uint8_t) * ImgSize.width * ImgSize.height
				+ sizeof(AdasRect) * MAX_LAYER_OBJ_GROUP_NUMS
				+ MAX_EXTERN_SPACE_SIZE * sizeof(s32);

		nalloclSize += (1024 << 8);

		m_globlparam[nId].m_pAlloc = (uint8_t* )my_malloc(nalloclSize);
		m_globlparam[nId].m_nInitSuccess = ALLOC_SUCCESS_STATE;

		m_globlparam[nId].m_ImgWidth = ImgSize.width;
		m_globlparam[nId].m_ImgHeight = ImgSize.height;
		m_globlparam[nId].m_ImgSize = adasize(ImgSize.width, ImgSize.height);

		pTr = m_globlparam[nId].m_pAlloc;

		if (initMalloc == 0) {
			for (j = 0; j < 2; j++) {
				g_MuliTracker[j].pTrackerset = (trakobj*) pTr;
				pTr =
						ADAS_ALIGN_16BYTE(g_MuliTracker[j].pTrackerset + MAX_OBJ_GROUP_NUMS );
				g_MuliTracker[j].nTrackeNum = 0;

				for (i = 0; i < MAX_OBJ_GROUP_NUMS; i++) {
					g_MuliTracker[j].pTrackerset[i].pCenTrajecy =
							(TrackPoint*) pTr;
					pTr =
							ADAS_ALIGN_16BYTE(g_MuliTracker[j].pTrackerset[i].pCenTrajecy + MAX_CORNER_OF_TRACK);

					g_MuliTracker[j].pTrackerset[i].pMotion = (Motion*) pTr;
					pTr =
							ADAS_ALIGN_16BYTE(g_MuliTracker[j].pTrackerset[i].pMotion + MAX_MOTION_NUM);
				}
			}
			initMalloc = 1;
		}

		m_globlparam[nId].m_pXYCorners = (xy*) pTr;
		pTr = ADAS_ALIGN_16BYTE(m_globlparam[nId].m_pXYCorners + MAX_XY_CORNER);

		m_globlparam[nId].m_pXYNoMax = (xy*) pTr;
		pTr = ADAS_ALIGN_16BYTE(m_globlparam[nId].m_pXYNoMax + MAX_XY_CORNER);

		m_globlparam[nId].m_pRowStart = (s32 *) pTr;
		pTr = ADAS_ALIGN_16BYTE(m_globlparam[nId].m_pRowStart + MAX_IMG_HEIGHT);

		m_globlparam[nId].m_pScore = (s32 *) pTr;
		pTr = ADAS_ALIGN_16BYTE(m_globlparam[nId].m_pScore + MAX_XY_CORNER);

		m_globlparam[nId].m_pCornerPass = (AdasCorner*) pTr;
		pTr =
				ADAS_ALIGN_16BYTE(m_globlparam[nId].m_pCornerPass + MAX_CORNERS_OF_NONMAX);
		memset(m_globlparam[nId].m_pCornerPass, 0,
				sizeof(AdasCorner) * MAX_CORNERS_OF_NONMAX);

		m_globlparam[nId].m_pFastCorner = (AdasCorner*) pTr;
		memset(m_globlparam[nId].m_pFastCorner, 0,
				sizeof(AdasCorner) * MAX_CORNERS_OF_NONMAX);
		pTr =
				ADAS_ALIGN_16BYTE(m_globlparam[nId].m_pFastCorner + MAX_CORNERS_OF_NONMAX);

		m_globlparam[nId].m_pfeature = (uint8_t *) pTr;
		pTr =
				ADAS_ALIGN_16BYTE(m_globlparam[nId].m_pfeature + MAX_CORNERS_PER_FRAME * SURF_DESC_DIMENTION);
		memset(m_globlparam[nId].m_pfeature, 0,
				sizeof(uint8_t) * MAX_CORNERS_PER_FRAME * SURF_DESC_DIMENTION);

		m_globlparam[nId].m_Init_pFastCorner = (AdasCorner*) pTr;
		memset(m_globlparam[nId].m_Init_pFastCorner, 0,
				sizeof(AdasCorner) * MAX_CORNERS_OF_NONMAX);
		pTr =
				ADAS_ALIGN_16BYTE(m_globlparam[nId].m_Init_pFastCorner + MAX_CORNERS_OF_NONMAX);

		m_globlparam[nId].m__Init_pfeature = (uint8_t *) pTr;
		pTr =
				ADAS_ALIGN_16BYTE(m_globlparam[nId].m__Init_pfeature + MAX_CORNERS_PER_FRAME * SURF_DESC_DIMENTION);
		memset(m_globlparam[nId].m__Init_pfeature, 0,
				sizeof(uint8_t) * MAX_CORNERS_PER_FRAME * SURF_DESC_DIMENTION);

		m_globlparam[nId].m_NewRecNum = 0;
		m_globlparam[nId].m_PNewRec = (AdasRect*) pTr;
		pTr =
				ADAS_ALIGN_16BYTE(m_globlparam[nId].m_PNewRec + MAX_NEW_GROUP_PER_FRAM);

		m_globlparam[nId].m_pGrayData = (uint8_t *) pTr;
		pTr =
				ADAS_ALIGN_16BYTE(m_globlparam[nId].m_pGrayData+ ImgSize.width *ImgSize.height);

		m_globlparam[nId].m_preGrayData = (uint8_t *) pTr;
		pTr =
				ADAS_ALIGN_16BYTE(m_globlparam[nId].m_preGrayData + ImgSize.width *ImgSize.height);

		m_globlparam[nId].m_pMask = (uint8_t *) pTr;
		memset(m_globlparam[nId].m_pMask, 255,
				sizeof(uint8_t) * ImgSize.width * ImgSize.height);
		pTr =
				ADAS_ALIGN_16BYTE(m_globlparam[nId].m_pMask + ImgSize.width * ImgSize.height);

		m_globlparam[nId].m_pGroupSets = (obj_group *) pTr;
		pTr =
				ADAS_ALIGN_16BYTE(m_globlparam[nId].m_pGroupSets + MAX_LAYER_OBJ_GROUP_NUMS);

		for (i = 0; i < MAX_LAYER_OBJ_GROUP_NUMS; i++) {
			pObjGroup = m_globlparam[nId].m_pGroupSets + i;
			pObjGroup->histoyRec.pGroupSiz = (s32 *) pTr;
			pTr =
					ADAS_ALIGN_16BYTE(pObjGroup->histoyRec.pGroupSiz + MAX_HISRORY_GROUP_REC_NUM * 2 );

		}

		for (i = 0; i < MAX_LAYER_OBJ_GROUP_NUMS; i++) {
			pObjGroup = m_globlparam[nId].m_pGroupSets + i;
			pObjGroup->Templat.ptr = (uint8_t *) pTr;
			pObjGroup->Templat.nWid = 0;
			pObjGroup->Templat.nHig = 0;
			pObjGroup->updataFrambyCar = -1;
			pObjGroup->SerpreditNum = 0;
			pTr =
					ADAS_ALIGN_16BYTE(pObjGroup->Templat.ptr + (MAX_TEMPLAT_NCC_TRACK_SIZE * MAX_TEMPLAT_NCC_TRACK_SIZE) );
		}

		for (i = 0; i < MAX_LAYER_OBJ_GROUP_NUMS; i++) {
			pObjGroup = m_globlparam[nId].m_pGroupSets + i;
			pObjGroup->Centr.pTrackPoint = (TrackPoint *) pTr;
			pObjGroup->Centr.pfeature = 0;
			pTr =
					ADAS_ALIGN_16BYTE(pObjGroup->Centr.pTrackPoint + MAX_CORNER_OF_TRACK);

			pObjGroup->pMotion = (Motion *) pTr;
			pTr = ADAS_ALIGN_16BYTE(pObjGroup->pMotion + MAX_MOTION_NUM);

			mvClearTrajIfo(&pObjGroup->Centr);
			mvClearGroupIfo(pObjGroup, 0);
		}

		for (i = 0; i < MAX_LAYER_OBJ_GROUP_NUMS; i++) {
			pObjGroup = m_globlparam[nId].m_pGroupSets + i;
			pObjGroup->pObjtr = (trajecy*) pTr;
			pTr =
					ADAS_ALIGN_16BYTE(pObjGroup->pObjtr + MAX_TRACKS_NUM_OF_GROUP);
			pObjGroup->pOrInitGray = (uint8_t *) pTr;
			pTr =
					ADAS_ALIGN_16BYTE(pObjGroup->pOrInitGray + MAX_GROUP_IMG_BYTE);

			for (j = 0; j < UPDATA_IMAGE_NUM; j++) {
				pObjGroup->UpdatedImg[j].ptr = (uint8_t *) pTr;
				pTr =
						ADAS_ALIGN_16BYTE(pObjGroup->UpdatedImg[j].ptr + UPDATA_IMAGE_SIZE * UPDATA_IMAGE_SIZE);
				pObjGroup->UpdatedImg[j].nWid = UPDATA_IMAGE_SIZE;
				pObjGroup->UpdatedImg[j].nHig = UPDATA_IMAGE_SIZE;
				pObjGroup->UpdatedImg[j].nChannle = 1;
			}

		}

		for (i = 0; i < MAX_LAYER_OBJ_GROUP_NUMS; i++) {
			pObjGroup = m_globlparam[nId].m_pGroupSets + i;

			for (j = 0; j < MAX_TRACKS_NUM_OF_GROUP; j++) {
				ptrajecy = pObjGroup->pObjtr + j;

				ptrajecy->pTrackPoint = (TrackPoint*) pTr;
				pTr =
						ADAS_ALIGN_16BYTE(ptrajecy->pTrackPoint + MAX_CORNER_OF_TRACK);

				ptrajecy->pfeature = (uint8_t*) pTr;
				pTr =
						ADAS_ALIGN_16BYTE(ptrajecy->pfeature + SURF_DESC_DIMENTION);

				ptrajecy->processfeature = (uint8_t*) pTr;
				pTr =
						ADAS_ALIGN_16BYTE(ptrajecy->processfeature + SURF_DESC_DIMENTION);

				mvClearTrajIfo(ptrajecy);
			}

		}

		m_globlparam[nId].m_ndx = (s32 *) pTr;
		pTr =
				ADAS_ALIGN_16BYTE(m_globlparam[nId].m_ndx + MAX_TRACKS_NUM_OF_GROUP);

		m_globlparam[nId].m_ndy = (s32 *) pTr;
		pTr =
				ADAS_ALIGN_16BYTE(m_globlparam[nId].m_ndy + MAX_TRACKS_NUM_OF_GROUP);

		m_globlparam[nId].m_fsclare = (float32_t *) pTr;
		pTr =
				ADAS_ALIGN_16BYTE(m_globlparam[nId].m_fsclare + MAX_TRACKS_NUM_OF_GROUP * MAX_TRACKS_NUM_OF_GROUP);

		m_globlparam[nId].m_PublacSape = (s32*) pTr;
		pTr =
				ADAS_ALIGN_16BYTE(m_globlparam[nId].m_PublacSape + MAX_PUBLIC_SPACE_SIZE);

		m_globlparam[nId].m_pGroupIndex = (uint8_t *) pTr;
		pTr =
				ADAS_ALIGN_16BYTE(m_globlparam[nId].m_pGroupIndex + MAX_LAYER_OBJ_GROUP_NUMS);

		m_globlparam[nId].scaleInput.pGrayfram = (uint8_t *) pTr;
		pTr =
				ADAS_ALIGN_16BYTE(m_globlparam[nId].scaleInput.pGrayfram + ImgSize.width * ImgSize.height);

		m_globlparam[nId].scaleInput.objRec = (AdasRect *) pTr;
		pTr =
				ADAS_ALIGN_16BYTE(m_globlparam[nId].scaleInput.objRec + MAX_LAYER_OBJ_GROUP_NUMS);

		m_globlparam[nId].m_extern_Space = (s32 *) pTr;
		pTr =
				ADAS_ALIGN_16BYTE(m_globlparam[nId].m_extern_Space + MAX_EXTERN_SPACE_SIZE);

		m_globlparam[nId].m_nCornerThresh = nCornerThresh[nId];
		m_globlparam[nId].m_nCornerPassNum = 0;
		m_globlparam[nId].m_nFastCornerNum = 0;
		m_globlparam[nId].m_Init_nFastCornerNum = 0;
		m_globlparam[nId].m_NewRecNum = 0;
		m_globlparam[nId].m_GroupId = -1;
		m_globlparam[nId].scaleInput.nFramSeq = 0;
		m_globlparam[nId].m_GroupIndexNum = 0;
	}

#ifdef USE_TAIL_LIGHT
	rt = initTailLight(1280, 720, &gl_tailnightpara);
#endif // USE_TAIL_LIGHT
		
}

/*
 Function process:
 + The main realize of multi-tracking
 Fan-in :
 + main()
 Fan-out:
 + N/A
 ATTENTION: __________
 */
void FCW_TRACK_MultieTrack(PortInput *pInPutParam) {

	g_OutIndex++;

	mvfindGroundValue(pInPutParam);

#ifdef TIME_TEST
	double ts = cvGetTickCount();
#endif

	mvReInput(pInPutParam);

#ifdef TIME_TEST 
	TIME_TAKE(cvGetTickCount(),ts,"mvReInput");
	ts = cvGetTickCount();
#endif

	mvMultileScaleTrack(pInPutParam);

#ifdef TIME_TEST 
	TIME_TAKE(cvGetTickCount(),ts,"mvMultileScaleTrack");
	ts = cvGetTickCount();
#endif	

	mvfushionObj();

#ifdef TIME_TEST 
	TIME_TAKE(cvGetTickCount(),ts,"mvfushionObj");
	ts = cvGetTickCount();
#endif	

	mvMotionState();

#ifdef TIME_TEST 
	TIME_TAKE(cvGetTickCount(),ts,"mvMotionState");
	ts = cvGetTickCount();
#endif	

	mvTTC();

#ifdef TIME_TEST 
	TIME_TAKE(cvGetTickCount(),ts,"mvTTC");
	ts = cvGetTickCount();
#endif	

	mvSetTrackerReult();

#ifdef TIME_TEST 
	TIME_TAKE(cvGetTickCount(),ts,"mvSetTrackerReult");
	ts = cvGetTickCount();
#endif	

}

/*
 Function process:
 + Get the multi-tracking result
 Fan-in :
 + main()
 Fan-out:
 + N/A
 ATTENTION: tarcking rect is based on resized image (1/2 of orig image)
 */
void FCW_TRACK_GetResult(MuliTracker **pTrackOutput) {
	*pTrackOutput = g_MuliTracker + (g_OutIndex & 1);
}

/*
 Function process:
 + Clear m_globlparam[nId].m_pGroupSets
 Fan-in :
 + main()
 Fan-out:
 + N/A
 ATTENTION:
 */
void FCW_TRACK_mvClearWholeGroup() {
	s32 i;
	s32 nId;
	obj_group * pgroup = 0;

	for (nId = 0; nId < 16; nId++) {

		if (ALLOC_SUCCESS_STATE == m_globlparam[nId].m_nInitSuccess) {

			for (i = 0; i < MAX_LAYER_OBJ_GROUP_NUMS; i++) {
				pgroup = m_globlparam[nId].m_pGroupSets + i;

				if (pgroup->nGroupId != -1) {
					mvClearGroupIfo(pgroup, 1);
				}

			}
		}

	}

}

/*
 Function process:
 + Free memory
 Fan-in :
 + main()
 Fan-out:
 + N/A
 ATTENTION:
 */
void FCW_TRACK_Unit_adas_global_param() {
	int nId;

	for (nId = 0; nId < inner_global_param_num; nId++) {
		if (ALLOC_SUCCESS_STATE == m_globlparam[nId].m_nInitSuccess) {
			my_free(m_globlparam[nId].m_pAlloc);
			m_globlparam[nId].m_pAlloc = NULL;
			m_globlparam[nId].m_nInitSuccess = ALLOC_SUCCESS_STATE - 1;
		}
	}
#ifdef USE_TAIL_LIGHT
	freeTailLight(&gl_tailnightpara);
#endif // USE_TAIL_LIGHT		
}

/*
 **************************************************************/
static uint8_t mvOrinitConsensVote(obj_group *pgroup, s32 nId) {

	uint8_t bfsalcSucces;
	float32_t fscale;
	s32 j;
	trajecy *pTrajecy = 0;
	Vote CenVote;
	s32 nCenVotNum = 0;
	s32 nSimlarVote = 0;
	//AdasPoint Votec,AvgVotec;
	AdasPoint Votec;
	s32 nDisPowTrd;

	bfsalcSucces = mvScaleVal(pgroup, m_globlparam[nId].m_fsclare, &fscale,
			OrinitVote, nId);

	nDisPowTrd = pgroup->rtContour.width * pgroup->rtContour.width / 25;

	nDisPowTrd = WS_MIN(255,nDisPowTrd);

	if (bfsalcSucces) {
		memset(m_globlparam[nId].m_PublacSape, 0,
				sizeof(s32) * pgroup->ntrajecyNum);
		for (j = 0; j < pgroup->ntrajecyNum; j++) {
			pTrajecy = pgroup->pObjtr + j;

			if (pTrajecy->bOrInitVoteTracked) {
				CenVote = mvTrajecyVote(pTrajecy, fscale, OrinitVote);

				if (pgroup->Centr.nTrackLen > 2
						&& mvDisPow(
								&pgroup->Centr.pTrackPoint[(pgroup->Centr.nTrackLen
										- 1) & MAX_CORNER_OF_TRACK_BIT].point,
								&CenVote) < nDisPowTrd)

						{
					m_globlparam[nId].m_ndx[nCenVotNum] = CenVote.x;
					m_globlparam[nId].m_ndy[nCenVotNum] = CenVote.y;
					m_globlparam[nId].m_PublacSape[nCenVotNum] = j;
					nCenVotNum++;
				} else {
					pTrajecy->bOrInitVoteTracked = 0;
				}

			}
		}

		if (!nCenVotNum) {
			return 0;
		}

		memcpy(m_globlparam[nId].m_PublacSape + nCenVotNum,
				m_globlparam[nId].m_ndx, sizeof(s32) * nCenVotNum);
		memcpy(m_globlparam[nId].m_PublacSape + (nCenVotNum << 1),
				m_globlparam[nId].m_ndy, sizeof(s32) * nCenVotNum);

		binSort_INT(m_globlparam[nId].m_PublacSape + nCenVotNum, nCenVotNum);
		binSort_INT(m_globlparam[nId].m_PublacSape + (nCenVotNum << 1),
				nCenVotNum);

		Votec.x =
				m_globlparam[nId].m_PublacSape[nCenVotNum + (nCenVotNum >> 1)];
		Votec.y = m_globlparam[nId].m_PublacSape[(nCenVotNum << 1)
				+ (nCenVotNum >> 1)];

		nSimlarVote = mvDelFarCen(pgroup, Votec, m_globlparam[nId].m_PublacSape,
				nCenVotNum, nDisPowTrd, nId);


		if (nSimlarVote < VOTE_CONSENSE_TRD) {
			return 0;
		}

		pgroup->rtContour.width = (s32) (pgroup->OriInitContour.width * fscale);
		pgroup->rtContour.height =
				(s32) (pgroup->OriInitContour.height * fscale);
		pgroup->rtContour.x = Votec.x - (pgroup->OriInitContour.width >> 1);
		pgroup->rtContour.y = Votec.y - (pgroup->OriInitContour.height >> 1);

		pgroup->rtContour.height = pgroup->rtContour.width;

		pgroup->nFramseq = m_globlparam[nId].scaleInput.nFramSeq;
		pgroup->nTime = m_globlparam[nId].scaleInput.objTime;
		pgroup->histoyRec.pGroupSiz[pgroup->histoyRec.nSizNum
				% MAX_HISRORY_GROUP_REC_NUM] = pgroup->rtContour.width;
		pgroup->histoyRec.pGroupSiz[pgroup->histoyRec.nSizNum
				% MAX_HISRORY_GROUP_REC_NUM + MAX_HISRORY_GROUP_REC_NUM] =
				pgroup->rtContour.height;

		pgroup->histoyRec.nSizNum++;

		mvMidGroupRec(pgroup);

		mvScopeImg(&pgroup->rtContour,
				adasrect(0, 0, m_globlparam[nId].m_ImgWidth,
						m_globlparam[nId].m_ImgHeight));

	}

	return nSimlarVote;

}

/*
 **************************************************************/
static uint8_t mvCorrectProcessVotebyTempMatch(obj_group *pgroup, AdasRect *pMatchRec,
		s32 nId, float32_t *fMatchScore) {
	int i;
	int nAddress0ff = 0;
	imgage ResizeOrimg;
	imgage Tempimg, Curnimg;
	AdasRect SearcRe, MatchRec, TempRec;
	float32_t fShunkRio;
	uint8_t bMatchSucces;
	VoteErorrVal VoteCorreVal;
	trajecy *pTrajecy = 0;
	uint8_t * ptr = (uint8_t *) m_globlparam[nId].m_PublacSape;
	ResizeOrimg.ptr = ptr;

	ResizeOrimg.nWid = pgroup->ProcessContour.width;
	ResizeOrimg.nHig = pgroup->ProcessContour.height;

	if (WS_MAX(ResizeOrimg.nWid ,ResizeOrimg.nHig) > MAX_TEMPLAT_TRACK_SIZE
			|| (!pgroup->InitContour.width)
			|| pgroup->ProcessContour.width * pgroup->ProcessContour.height
					>= MAX_GROUP_IMG_BYTE) {
		return 0;
	}

	//
	mvOriObjResizeToDstImg(pgroup, ResizeOrimg.ptr, nId);

	nAddress0ff += ADAS_ALIGN_16BYTE_SIZE( ResizeOrimg.nWid * ResizeOrimg.nHig);
	ptr = (uint8_t *) m_globlparam[nId].m_PublacSape + nAddress0ff;

	Tempimg.ptr = ptr;
	mvSelcTemplatByCen(ResizeOrimg, &Tempimg, &TempRec);
	nAddress0ff += ADAS_ALIGN_16BYTE_SIZE( Tempimg.nWid * Tempimg.nHig);
	ptr = (uint8_t *) m_globlparam[nId].m_PublacSape + nAddress0ff;

	Curnimg.ptr = ptr;
	Curnimg.nWid = pgroup->ProcessContour.width;
	Curnimg.nHig = pgroup->ProcessContour.height;
	nAddress0ff += ADAS_ALIGN_16BYTE_SIZE( Curnimg.nWid * Curnimg.nHig);
	ptr = (uint8_t *) m_globlparam[nId].m_PublacSape + nAddress0ff;

	mvSelcImg(&Curnimg, pgroup->ProcessContour, 0, nId);

	fShunkRio = 1 / 10.0f;
	SearcRe.x = (s32) (fShunkRio * Curnimg.nWid);
	SearcRe.y = (s32) (fShunkRio * Curnimg.nWid);
	SearcRe.width = (s32) ((1 - fShunkRio * 2) * Curnimg.nWid);
	SearcRe.height = (s32) ((1 - fShunkRio * 2) * Curnimg.nHig);

	bMatchSucces = mvTemplatMatch(Curnimg, Tempimg, SearcRe, &MatchRec,
			nAddress0ff, 1, nId, fMatchScore);

	if (bMatchSucces) {
		VoteCorreVal.x = (Curnimg.nWid >> 1)
				- (MatchRec.x + (MatchRec.width >> 1));
		VoteCorreVal.y = (Curnimg.nHig >> 1)
				- (MatchRec.y + (MatchRec.height >> 1));

		for (i = 0; i < pgroup->ntrajecyNum; i++) {
			pTrajecy = pgroup->pObjtr + i;

			if (pTrajecy->bProcessTracked) {
				mvCorrectProcessVote(pTrajecy, VoteCorreVal);

			}
		}

		pgroup->ProcessContour.x -= VoteCorreVal.x;
		pgroup->ProcessContour.y -= VoteCorreVal.y;

		pMatchRec->x = MatchRec.x - TempRec.x + pgroup->ProcessContour.x; // ProcessContour
		pMatchRec->y = MatchRec.y - TempRec.y + pgroup->ProcessContour.y; //ProcessContour
		pMatchRec->width = ResizeOrimg.nWid;
		pMatchRec->height = ResizeOrimg.nHig;

		if ((MatchRec.x - TempRec.x) || (MatchRec.y - TempRec.y))
			mvSetTrajLenthOne(pgroup);
	}

	return bMatchSucces;

}

void mvDerivemgRio(imgage * pSrcImg, imgage * RioImg, AdasRect RioRec) {

	s32 j;
	uint8_t *pDstr = 0;
	uint8_t *pSrctr = 0;

	RioImg->nWid = RioRec.width;
	RioImg->nHig = RioRec.height;

	for (j = RioRec.y; j < RioRec.y + RioRec.height; j++) {
		pDstr = RioImg->ptr + RioImg->nWid * (j - RioRec.y);

		pSrctr = pSrcImg->ptr + pSrcImg->nWid * j;

		memcpy(pDstr, pSrctr + RioRec.x, RioRec.width);

	}

}

/*
**************************************************************/
uint8_t mvMatchByTemple(const imgage *pimg, const imgage *pTempimg,
		AdasRect SearcRec, AdasRect *pMatchRec, s32 nPubliBuffOff, s32 nId) {

#ifdef SHOW_TEMPLE
	IplImage *serchImg = Trans_Imgage_TO_cvIplImage(*pimg);
	IplImage *TemplateImg = Trans_Imgage_TO_cvIplImage(*pTempimg);
	cvShowImage("serchImg",serchImg);
	cvShowImage("TemplateImg",TemplateImg);
	cvWaitKey(10);
	cvReleaseImage(&serchImg);
	cvReleaseImage(&TemplateImg);
#endif

	imgage IntelImg;
	s32 m, n, i, j;
	s32 addresoff = 0;
	uint8_t* pTemplateptr = 0;
	s32* pdata = 0;
	float32_t* pSimlarTr = 0;
	imgage tempSubAvg;
	imgage MathchSimilar;
	s32 nTemplateAvg = 0;
	uint8_t* ptr = (uint8_t*) m_globlparam[nId].m_extern_Space + nPubliBuffOff;
	s32 powT = 0;
	s32 ntemplateSum = pTempimg->nHig * pTempimg->nWid;
	AdasPoint lt, br;
	s32 RioSubAvgSum = 0;
	float32_t fMatchVal = 0.0f;
	s32 nAvg;
	uint8_t *pMatchRio = 0;
	float32_t fMaxMatchSim = 0.0f;
	//float32_t fSecdMaxMatchSim = 0.0f;
	//float32_t fMinMatchSim = 1.0f;
	s32 nMaxExternByte = MAX_EXTERN_SPACE_SIZE * sizeof(s32);

	IntelImg.ptr = ptr;

	if (SearcRec.width < pTempimg->nWid || SearcRec.height < pTempimg->nHig) {
		return 0;
	}

	IntelImg.nWid = SearcRec.width + 1;
	IntelImg.nHig = SearcRec.height + 1;
	memset(IntelImg.ptr, 0, IntelImg.nWid * IntelImg.nHig * sizeof(s32));

	addresoff +=
			ADAS_ALIGN_16BYTE_SIZE(IntelImg.nWid * IntelImg.nHig * sizeof(s32));
	ptr += addresoff;

	if (nPubliBuffOff + addresoff > nMaxExternByte) {
		return 0;
	}

#ifdef SHOW_TEMPLE
	IplImage *serchImg_after = Trans_Imgage_TO_cvIplImage(*pimg);
	IplImage *TemplateImg_after = Trans_Imgage_TO_cvIplImage(*pTempimg);
	cvShowImage("serchImg_after",serchImg_after);
	cvShowImage("TemplateImg_after",TemplateImg_after);
	cvWaitKey(0);
	cvReleaseImage(&serchImg_after);
	cvReleaseImage(&TemplateImg_after);
#endif

	mvInterlimg(pimg, &IntelImg, adaspoint(SearcRec.x, SearcRec.y));

	tempSubAvg.ptr = ptr;
	tempSubAvg.nWid = pTempimg->nWid;
	tempSubAvg.nHig = pTempimg->nHig;
	addresoff +=
			ADAS_ALIGN_16BYTE_SIZE(pTempimg->nWid * pTempimg->nHig * sizeof(s32));
	ptr += addresoff;

	//
	MathchSimilar.ptr = ptr;
	MathchSimilar.nWid = SearcRec.width - pTempimg->nWid;
	MathchSimilar.nHig = SearcRec.height - pTempimg->nHig;
	addresoff +=
			ADAS_ALIGN_16BYTE_SIZE(MathchSimilar.nWid * MathchSimilar.nHig * sizeof(float32_t));

	if (nPubliBuffOff + addresoff > nMaxExternByte) {
		return 0;
	}

	for (m = 0; m < pTempimg->nHig; m++) {
		pTemplateptr = pTempimg->ptr + pTempimg->nWid * m;

		for (n = 0; n < pTempimg->nWid; n++) {
			nTemplateAvg += pTemplateptr[n];
		}
	}

	nTemplateAvg /= ntemplateSum;

	for (m = 0; m < tempSubAvg.nHig; m++) {
		pdata = (s32*) tempSubAvg.ptr + tempSubAvg.nWid * m;

		pTemplateptr = pTempimg->ptr + pTempimg->nWid * m;

		for (n = 0; n < tempSubAvg.nWid; n++) {
			pdata[n] = pTemplateptr[n] - nTemplateAvg;
			powT += pdata[n] * pdata[n];
		}
	}

	//match
	pMatchRec->width = pTempimg->nWid;
	pMatchRec->height = pTempimg->nHig;

	for (m = SearcRec.y; m < SearcRec.y + SearcRec.height - pTempimg->nHig;
			m++) {
		for (n = SearcRec.x; n < SearcRec.x + SearcRec.width - pTempimg->nWid;
				n++) {
			lt = adaspoint(n - SearcRec.x, m - SearcRec.y);

			br = adaspoint(n - SearcRec.x + pTempimg->nWid /*-1 + 1*/,
					m - SearcRec.y + pTempimg->nHig);

			nAvg = mvSumImgRio(&IntelImg, lt, br);

			nAvg = nAvg / ntemplateSum;

			RioSubAvgSum = 0;
			fMatchVal = 0.0f;

			for (j = 0; j < tempSubAvg.nHig; j++) {
				pdata = (s32*) tempSubAvg.ptr + tempSubAvg.nWid * j;

				pMatchRio = pimg->ptr + pimg->nWid * (m + j);

				for (i = 0; i < tempSubAvg.nWid; i++) {
					RioSubAvgSum += (pMatchRio[i + n] - nAvg)
							* (pMatchRio[i + n] - nAvg);
					fMatchVal += pdata[i] * (pMatchRio[i + n] - nAvg);
				}
			}

			fMatchVal = fMatchVal / sqrtf((float32_t) powT * RioSubAvgSum);

			pSimlarTr = (float32_t*) MathchSimilar.ptr
					+ MathchSimilar.nWid * (s32) (lt.y);
			pSimlarTr[(s32) (lt.x)] = fMatchVal;

			if (fMatchVal > fMaxMatchSim) {
				pMatchRec->x = n;
				pMatchRec->y = m;
				fMaxMatchSim = fMatchVal;
			}

		}
	}

#ifdef SHOW_TEMPLE
#endif

	return 1;

}

/*
 *************************************************************
 Function process:
 + Do the sample tracking based on temple tracking
 Fan-in :
 + main()
 Fan-out:
 + N/A
 ATTENTION: __________
 */
s32 FCW_TRACK_SimpleTrack(const AdasRect SrcRec, imgage *pSrcImg,
		const imgage *pDstImg, AdasPoint MotionVec, AdasRect *pDstRec) {

	imgage TempleImg;
	AdasRect TemplRec;
	float32_t fShink = 0.15f;
	float32_t fLager = 0.2f;
	uint8_t bTemplat;
	AdasRect SearcRec, MatchRec;
	s32 nPubliBuffOff;
	s32 nId = 0;

	if (m_globlparam[nId].m_nInitSuccess != ALLOC_SUCCESS_STATE) {
		pDstRec->x = SrcRec.x + MotionVec.x;
		pDstRec->y = SrcRec.y + MotionVec.y;
		pDstRec->width = SrcRec.width;
		pDstRec->height = SrcRec.height;
		mvScopeImg(pDstRec, adasrect(0, 0, pSrcImg->nWid, pSrcImg->nHig));
		return 1;
	}

	TemplRec.x = SrcRec.x + (int16_t)(SrcRec.width * fShink);
	TemplRec.y = SrcRec.y + (int16_t)(SrcRec.height * fShink);
	TemplRec.width = SrcRec.width - (int16_t)(SrcRec.width * fShink * 2);
	TemplRec.height = SrcRec.height - (int16_t)(SrcRec.height * fShink * 2);

	SearcRec.x = SrcRec.x - (int16_t)(SrcRec.width * fLager);
	SearcRec.y = SrcRec.y - (int16_t)(SrcRec.height * fLager);
	SearcRec.width = SrcRec.width + (int16_t)(SrcRec.width * fLager * 2);
	SearcRec.height = SrcRec.height + (int16_t)(SrcRec.height * fLager * 2);
	mvScopeImg(&SearcRec, adasrect(0, 0, pSrcImg->nWid, pSrcImg->nHig));

	if (WS_MIN(TemplRec.width,TemplRec.height ) < 5) {
		TemplRec = SrcRec;
	}

	TempleImg.ptr = (uint8_t *) m_globlparam[nId].m_extern_Space;
	nPubliBuffOff = TemplRec.width * TemplRec.height;

	mvDerivemgRio(pSrcImg, &TempleImg, TemplRec);

	bTemplat = mvMatchByTemple(pDstImg, &TempleImg, SearcRec, &MatchRec,
			nPubliBuffOff, nId);

	if (!bTemplat) {
		pDstRec->x = SrcRec.x + MotionVec.x;
		pDstRec->y = SrcRec.y + MotionVec.y;
		pDstRec->width = SrcRec.width;
		pDstRec->height = SrcRec.height;
	} else {
		pDstRec->x = MatchRec.x - (int16_t)(SrcRec.width * fShink);
		pDstRec->y = MatchRec.y - (int16_t)(SrcRec.height * fShink);
		pDstRec->width = SrcRec.width;
		pDstRec->height = SrcRec.height;
		mvScopeImg(pDstRec, adasrect(0, 0, pSrcImg->nWid, pSrcImg->nHig));
	}

	return 1;
}

