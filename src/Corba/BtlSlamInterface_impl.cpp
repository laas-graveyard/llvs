/*! ----------------------------------------------------
 *  Copyright 2010, CNRS-AIST JRL
 *
 *  \brief  Btl Slam for LLVS
 *  \author Clement Petit
 * ---------------------------------------------------- */

#include <Corba/BtlSlamInterface_impl.h>

#if (LLVS_HAVE_HRP_BTL_SLAM>0) && (LLVS_HAVE_OMNIORB4>0)

#include <omniORB4/CORBA.h>
#include <vslam_app.h>
#include <llvs/tools/Debug.h>

/* ---------------------------------------------------
 * Initialization / Destruction
 * --------------------------------------------------- */

llvs::BtlSlamInterface_impl::BtlSlamInterface_impl(const VSLAM_App& SlamInstance)
	:m_SlamInstance(SlamInstance)
{
	ODEBUG3("[BtlSlamInterface] Service instanciated");
}

llvs::BtlSlamInterface_impl::~BtlSlamInterface_impl()
{
}

/* ---------------------------------------------------
 * Services
 * --------------------------------------------------- */

BtlSlamInterface::Pose3D
llvs::BtlSlamInterface_impl::getCameraPose()
{
	ODEBUG3("[BtlSlamInterface] Service called");
	BtlSlamInterface::Pose3D buffer;
	VSLAM_App::Pose3D pose(m_SlamInstance.getCameraPose());
	buffer.position[0] = pose.position.x;
	buffer.position[1] = pose.position.y;
	buffer.position[2] = pose.position.z;
	buffer.orientation[0] = pose.orientation.x;
	buffer.orientation[1] = pose.orientation.y;
	buffer.orientation[2] = pose.orientation.z;
	return buffer;
}

#endif //!LLVS_HAVE_HRP_BTL_SLAM>0 && LLVS_HAVE_OMNIORB4>0
