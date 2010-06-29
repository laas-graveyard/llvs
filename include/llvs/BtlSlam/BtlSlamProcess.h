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

/*! Abstract class */
#include "VisionBasicProcess.h"

/*! Includes slam specific */
#include <SharedLLVSBuffer.h>

/*! Includes system */
#include <string>
 
/*! Declaration of CVD classes */
#include <cvd/byte.h>
#include <cvd/rgb.h>

class HRP2BtlSlamProcess : public HRP2VisionBasicProcess
{
	public:

		/*! Error codes */
		static const int BTL_SLAM_RESULT_OK                     = 0;
		static const int BTL_SLAM_ERROR_UNKNOWN_PARAMETER       = -1;
		static const int BTL_SLAM_ERROR_CONFIG_MISSING          = -2;
		static const int BTL_SLAM_ERROR_INITIALIZATION_FAILED   = -3;
		static const int BTL_SLAM_ERROR_PROCESS_ALREADY_STARTED = -4;
		static const int BTL_SLAM_ERROR_BAD_SOURCE_SYNTAX       = -5;

		/*! Constructor/Destructor */
		HRP2BtlSlamProcess();
		virtual ~HRP2BtlSlamProcess();

		/*! Generic process interface */
		int pInitializeTheProcess();
		int pRealizeTheProcess();
		int pCleanUpTheProcess();
		int pStartProcess();
		int pStopProcess();
		int pSetParameter(std::string aParameter, std::string aValue);

		/*! Specific interface */
		void SetInputImages(unsigned char** pImageContainer);

	protected:
	
		/*! Write current RGB image into a <filename> file */
		bool writeImageIntoFile(const char* rgbFrame, 
                            const std::string& filename) const;

		/*! Defined for user convenience only.
		 *  This a short way to use shared data type between process
		 *  and slam engine */
		typedef CVD::SharedLLVSBuffer<CVD::Rgb<CVD::byte> >::LLVSBuffer ImageType;

		/*! Formatted configuration after a setSlamConfig call */
		struct SlamConfig
		{
			/*! Right format for vslam engine */
			char** options;
			int size;

			/*! Location of the shared image */
			std::string server;
			std::string camera;
		} m_slamConfig;

		/*! Pointer to the area containing the latest grabbed image */
		unsigned char** m_pImageContainer;

		/*! Pointer to the shared memory area with slam engine */
		ImageType* m_pSharedBuffer;
		boost::interprocess::managed_shared_memory* m_pSharedSegment;

		/*! Process status */
		bool m_isAlreadyStarted;

	private:
		/*! Set <m_slamConfig> parsing the given <config> string.
		 *  If parsing fails, <m_slamConfig> is reset even if 
		 *  a previous valid configuration was set */
		int setSlamConfig(const std::string& config);

		/*! Clean up current configuration in m_slamConfig var.
		 *  If m_slamConfig has never been set, then nothing
		 *  is performed. */
		void cleanUpConfig();

		/*! Clean up all allocated segment in memory. Then 
		 * remove the whole segment and delete <m_pSharedSegment> */
		void cleanUpSharedMemory();
};

#endif // LLVS_HAVE_BTL_SLAM

#endif // _HRP2_BTL_SLAM_PROCESS_H_
