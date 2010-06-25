/*! ----------------------------------------------------
 *  Copyright 2010, CNRS-AIST JRL
 * 
 *  \brief  Btl Slam for LLVS
 *  \author Clement Petit
 *  \creation 24/06/2010
 * ---------------------------------------------------- */

#ifndef _HRP2_BTL_SLAM_PROCESS_H_
#define _HRP2_BTL_SLAM_PROCESS_H_

#include "llvsConfig.h"

#if (LLVS_HAVE_BTL_SLAM>0)

#include "VisionBasicProcess.h"
 
class HRP2BtlSlamProcess : public HRP2VisionBasicProcess
{
	public:

		/*! Error codes */
		static const int BTL_SLAM_RESULT_OK = 0;

		/*! Constructor/Destructor */
		HRP2BtlSlamProcess();
		virtual ~HRP2BtlSlamProcess();

		/*! Generic process interface */
		int pInitializeTheProcess();
		int pRealizeTheProcess();
		int pCleanUpTheProcess();
		int pStartProcess();
		int pStopProcess();

		/*! Specific interface */
		void SetInputImages(unsigned char** pImageContainer);

	protected:
		unsigned char** m_pImageContainer;
};

#endif // LLVS_HAVE_BTL_SLAM

#endif // _HRP2_BTL_SLAM_PROCESS_H_
