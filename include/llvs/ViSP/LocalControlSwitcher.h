/** @doc This object implements a simple controller switcher based on odometry.


   Copyright (c) 2011, 
   @author Olivier Stasse
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Redistribution and use in source and binary forms, with or without modification, 
   are permitted provided that the following conditions are met:
   
   * Redistributions of source code must retain the above copyright notice, 
   this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright notice, 
   this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
   * Neither the name of the CNRS and AIST nor the names of its contributors 
   may be used to endorse or promote products derived from this software without specific prior written permission.
   
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
   OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
   AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER 
   OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, 
   OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
   OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
   STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
   IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef _HRP2_SWITCH_CONTROLLER_H_
#define _HRP2_SWITCH_CONTROLLER_H_

/*! This object defines the abstract class defining a basic
 * vision process
 * 
 * Copyright (c) 2004 Olivier Stasse, JRL, CNRS/AIST
 *
 *
 */

#include <string>
#include <vector>

#ifdef OMNIORB4
#include <omniORB4/CORBA.h>
#endif

#ifdef __ORBIX__
#include <OBE/CORBA.h>
#include <OBE/CosNaming.h>
#endif 


#include <VisionBasicProcess.h>

#include <ConnectionToSot.h>

class LowLevelVisionSystem;
class HRP2ComputeControlLawProcess ;

/*! This object is used to derive all the vision process
 * at least for the LowLevelVisionServer.
 */
class HRP2ControlSwitcher : public HRP2VisionBasicProcess
{

 public:

  /*! Constructor */
  HRP2ControlSwitcher(llvs::LowLevelVisionServer * lLLVS=0);

  /*! Destructor */
  virtual ~HRP2ControlSwitcher();


  /*! Initialize the process. */
  virtual int pInitializeTheProcess();

  /*! Realize the process */
  virtual int pRealizeTheProcess();
  
  /*! Cleanup the process */
  virtual int pCleanUpTheProcess();

  /*! Try connection to visual servoing. */
  int TryConnectionToVisualServoing();

  /*! Set the pointer to connect to SoT. */
  void SetConnectionToSot(llvs::ConnectionToSot * aCTS);
/* Specify pointer on Compute Control Law Process. */
  void setComputeControlLaw(HRP2ComputeControlLawProcess *
			    aCCLP);

 protected:

  /*! Pointer on the control law of the tracker . */
  HRP2ComputeControlLawProcess *m_CCLP;
  
  /*! Number of cameras.
   * This number is initialized according to the SetInputImages
   * method called 1: (one argument), 2: (two arguments).
   */
  int m_NbOfCameras;

  /*! Link to ConnectionToSot.*/
  llvs::ConnectionToSot * m_CTS;
  
  /*! Link to low level vision server. */
  llvs::LowLevelVisionServer * m_LLVS;
  /*! State */

  enum STATE
  {
    IDLE,
    VISP_TRACKING,
    HEAD_MOVING,
    COLOR_TRACKING,
    END_BEHAVIOR
  };

  unsigned int m_CurrentState;

  double m_minResult[3];
  double m_normResult;

  double m_timeForHeadMoving;
};

#include <LowLevelVisionServer.h>
#endif /* _HRP2_SWITCH_CONTROLLER_H_ */
