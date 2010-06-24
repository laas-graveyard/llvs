/*! ----------------------------------------------------
 *  Copyright 2010, CNRS-AIST JRL
 * 
 *  \brief  Btl Slam for LLVS
 *  \author Clement Petit
 *  \creation 24/06/2010
 * ---------------------------------------------------- */

#include "BtlSlam/BtlSlam.h"

#if (LLVS_HAVE_BTL_SLAM>0)

#include <sstream>

/* ---------------------------------------------------
 * Initialization / Destruction
 * --------------------------------------------------- */

HRP2BtlSlamProcess::HRP2BtlSlamProcess()
	:m_pImageContainer(0)
{
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
	return BTL_SLAM_RESULT_OK;
}

int
HRP2BtlSlamProcess::pRealizeTheProcess()
{
	return BTL_SLAM_RESULT_OK;
}

int
HRP2BtlSlamProcess::pCleanUpTheProcess()
{
	return BTL_SLAM_RESULT_OK;
}

int
HRP2BtlSlamProcess::pStartProcess()
{
	return BTL_SLAM_RESULT_OK;
}

int
HRP2BtlSlamProcess::pStopProcess()
{
	return BTL_SLAM_RESULT_OK;
}

/* ---------------------------------------------------
 * Specific interface
 * --------------------------------------------------- */

void
HRP2BtlSlamProcess::setInputImages(unsigned char** pImageContainer)
{
	m_pImageContainer = pImageContainer;
}

#endif // LLVS_HAVE_BTL_SLAM
