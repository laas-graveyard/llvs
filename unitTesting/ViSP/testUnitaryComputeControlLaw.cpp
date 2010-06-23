/** @doc This file implements the access to IEEE 1394 cameras
    through the dc library.

   CVS Information:
   $Id$
   $Author$
   $Date$
   $Revision$
   $Source$
   $Log$

   Copyright (c) 2003-2006, 
   @author Claire Dune,
   2010
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

//standard lib and system lib
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>

using namespace std;

//LLVS lib
#include "ModelTracker/nmbtTrackingProcess.h"


#if LLVS_HAVE_VISP && LLVS_HAVE_NMBT

//tracker lib
#include <nmbt/nmbtTracking.h>

//visp
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplayX.h>
#include <visp/vpServo.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpFeatureTranslation.h>
#include <visp/vpFeatureThetaU.h>
#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>

// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>
#include <visp/vpServoDisplay.h>

#include <visp/vpImageIo.h>
#include <visp/vpDisplayX.h>
#include <visp/vp1394TwoGrabber.h> 

#include <ViSP/ComputeControlLawProcess.h>

#include <Debug.h>



// test the functions implemented in the class 
int main(void)
{
  cout <<" -------------------------- " << endl;
  cout << endl; 
  cout <<" Test functions of the class \n ComputeControlLaw" << endl;
  cout << endl; 
  cout <<" ---------------------------" << endl;
  

  HRP2ComputeControlLawProcess * CCLProcess;
  CCLProcess = new HRP2ComputeControlLawProcess;

  vpTwistMatrix hVc;
  CCLProcess->GethVc(hVc);
  

  vpColVector vcam(6),vwaist(6);
  vcam[2]=10;
  double poseHeadInFoot[6],poseWaistInFoot[6];
  for(int i =0 ;i<6;++i)
    {
      poseHeadInFoot[i]=0;
      poseWaistInFoot[i]=0;
    }
  poseHeadInFoot[2]=1.3;
  poseWaistInFoot[2]=0.6;

  vpHomogeneousMatrix tmp;

  int res;
  res = CCLProcess->changeVelocityFrame(vcam,
					vwaist,
					poseHeadInFoot,
					poseWaistInFoot,
					tmp);
   
   cout << "vcam" << endl;
  cout << vcam.t() << endl; 
  cout << "vres" << endl;
  cout << vwaist.t() << endl; 

  return 0;
}

#else

int main(void)
{
  cout <<" Cannot Test NMBT tracking class: NMBT not found!" << endl;
  return 0;
}

#endif
