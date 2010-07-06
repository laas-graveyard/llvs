/*! ----------------------------------------------------
 *  Copyright 2010, CNRS-AIST JRL
 *
 *  \brief  Btl Slam for LLVS
 *  \author Clement Petit
 * ---------------------------------------------------- */

#ifndef _BTL_SLAM_INTERFACE_IMPL_H_
#define _BTL_SLAM_INTERFACE_IMPL_H_

#include "llvsConfig.h"

# if (LLVS_HAVE_HRP_BTL_SLAM>0) && (LLVS_HAVE_OMNIORB4>0)

#include <omniORB4/CORBA.h>
#include "BtlSlamInterface.hh"

/*! Class forwarding */
class VSLAM_App;

namespace llvs
{
  class BtlSlamInterface_impl
		: public virtual POA_BtlSlamInterface
  {
		public:
			/*! Constructor/Destructor */
			BtlSlamInterface_impl(const VSLAM_App& SlamInstance);
			~BtlSlamInterface_impl();

			/*! Services */
			BtlSlamInterface::Pose3D getCameraPose();

		protected:
			/*! Direct access to the engine */
			/* FIXME: Some methods of this instance are not reentrant
			 * but can be accessed from this servant or directly from
			 * LLVS project. It is possible to get undetermined behaviour
			 * if a bad use of the instance is made (unappropriate calls
			 * like initializing the slam engine in a service, etc) */
			const VSLAM_App& m_SlamInstance;
  };
};

# endif //!LLVS_HAVE_HRP_BTL_SLAM>0 && LLVS_HAVE_OMNIORB4>0
#endif //!_BTL_SLAM_INTERFACE_IMPL_H_
