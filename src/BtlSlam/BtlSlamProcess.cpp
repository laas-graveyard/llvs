/*! ----------------------------------------------------
 *  Copyright 2010, CNRS-AIST JRL
 * 
 *  \brief  Btl Slam for LLVS
 *  \author Clement Petit
 *  \creation 24/06/2010
 * ---------------------------------------------------- */

#include "BtlSlam/BtlSlamProcess.h"

#if (LLVS_HAVE_BTL_SLAM>0)

#include <boost/serialization/extended_type_info.hpp>
/*! Includes slam specific */
#include <vslam_app.h>

/*! Includes llvs tools */
#include <llvs/tools/Debug.h>

/* ---------------------------------------------------
 * Initialization / Destruction
 * --------------------------------------------------- */

HRP2BtlSlamProcess::HRP2BtlSlamProcess()
	:m_pImageContainer(0)
{
	m_ProcessName = "BtlSlamProcess";
}

HRP2BtlSlamProcess::~HRP2BtlSlamProcess()
{
}


/* ---------------------------------------------------
 * Generic interface
 * --------------------------------------------------- */

int
HRP2BtlSlamProcess::pInitializeTheProcess()
{
	ODEBUG3("[BtlSlam] Initialize");
	return BTL_SLAM_RESULT_OK;
}

int
HRP2BtlSlamProcess::pRealizeTheProcess()
{
	ODEBUG3("[BtlSlam] Realize");
	return BTL_SLAM_RESULT_OK;
}

int
HRP2BtlSlamProcess::pCleanUpTheProcess()
{
	ODEBUG3("[BtlSlam] CleanUp");
	return BTL_SLAM_RESULT_OK;
}

int
HRP2BtlSlamProcess::pStartProcess()
{
	ODEBUG3("[BtlSlam] Start");
	return BTL_SLAM_RESULT_OK;
}

int
HRP2BtlSlamProcess::pStopProcess()
{
	ODEBUG3("[BtlSlam] Stop");
	return BTL_SLAM_RESULT_OK;
}

/* ---------------------------------------------------
 * Specific interface
 * --------------------------------------------------- */

void
HRP2BtlSlamProcess::SetInputImages(unsigned char** pImageContainer)
{
	ODEBUG3("[BtlSlam] Set image buffer address: " << (void*)pImageContainer);
	m_pImageContainer = pImageContainer;
}

#endif // LLVS_HAVE_BTL_SLAM
