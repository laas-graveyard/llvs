/**
 * StereoVisionクラス定義ファイル
 * modified 020212 fukase
 * modified 020214 fukase
 * modified 020418 fukase for rbt2scmCalib
 */

#ifndef StereoVISION_IMPL
#define StereoVISION_IMPL

//#define USE_IMR  //(02/02/12 fukase)//uncomment 02/02/19 fukase

#include <stdlib.h>
#include <string>

#include "oocmap.h"

#ifdef __ORBIX__
#include "StereoVision_skel.h"
#endif

#ifdef OMNIORB4
#include <omniORB4/CORBA.h>
#include "StereoVision.hh"
#endif

// **********************************************************************
//
// StereoVisionクラス
//
// K Saito (Kernel Co.,Ltd.)
// version 0.2  (2000/03/28)
// modified 02/02/14
// modified 02/04/18 for rbt2scm Calibration
// **********************************************************************

namespace llvs
{
  class LowLevelVisionServer;

  class StereoVision_impl : virtual public POA_StereoVision,
    virtual public PortableServer::RefCountServantBase
    {

    private:

      /**
       * ORB
       */
      CORBA_ORB_var orb_;

    protected:
#ifdef USE_IMR
      PortableServer::POA_var poa_;
#endif

      // Ref. to LLVS to stop the grabbing.
      LowLevelVisionServer * m_LLVS;
      int m_LLVSGrabbingStatus;

      // The directory for the commands.
      std::string m_CommandsDir;

    public:


      /**
       * コンストラクタ
       * @param   orb     ORBへの参照
       */
#ifdef USE_IMR
      StereoVision_impl(CORBA::ORB_ptr orb,PortableServer::POA_ptr,
			LowLevelVisionServer * LLVS);
#else
      StereoVision_impl(CORBA_ORB_ptr orb,
			LowLevelVisionServer * LLVS);
#endif

      /**
       * デストラクタ
       */
      ~StereoVision_impl();

#ifdef USE_IMR
      virtual PortableServer::POA_ptr _default_POA();
#endif

      //
      // IDL:StereoVision/rbt2scmCalibStart:1.0
      // add 020418 fukase
      virtual CORBA::Boolean rbt2scmCalibStart()
	throw(CORBA::SystemException);

      //
      // IDL:StereoVision/rbt2scmCalibEnd:1.0
      // add 020418 fukase
      virtual CORBA::Boolean rbt2scmCalibEnd()
	throw(CORBA::SystemException);


      virtual CORBA::Boolean detectCrossMark(const TransformQuaternion& robotHeadPos,
					     const TransformQuaternion& robotHandPos)
	throw(CORBA::SystemException);

      virtual CORBA::Boolean getObjectPosition(
					       const char* name,
					       const TransformQuaternion& robotHeadPos,
					       TransformQuaternion_out ObjectPosition
					       ) throw(CORBA::SystemException);


      virtual CORBA::Boolean StartProcess(const char *aProcessName
					  )throw(CORBA::SystemException);

      virtual CORBA::Boolean StopProcess(const char *aProcessName
					 )throw(CORBA::SystemException);

      virtual CORBA::Long getImage(CORBA::Long CameraID, ImageData_out anImage, char *& Format
				   )throw(CORBA::SystemException);

      virtual CORBA::Long getRangeMap(RangeMap_out RangeMap, char *&Format
				      )throw(CORBA::SystemException);

      // Restore the previous grabbing status.
      void RestoreLLVSGrabbing();

      // Stop the grabbing and keep the previous status of the IEEE1394 visual
      // process status.
      void StopLLVSGrabbing();

    };

};
#include "LowLevelVisionServer.h"

#endif /* StereoVISION_IMPL */
