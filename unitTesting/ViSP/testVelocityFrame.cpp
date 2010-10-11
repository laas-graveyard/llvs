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


//visp
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplayX.h>
#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpTwistMatrix.h>

// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>



#include <llvs/tools/Debug.h>



// test the functions implemented in the class
int main(void)
{
  vpColVector cV(6);

  cV[0]=0;
  cV[1]=0;
  cV[2]=1;
  cV[3]=0;
  cV[4]=0;
  cV[5]=0;


  vpHomogeneousMatrix cameraMhead;

  ifstream file;

  string name="../../data/ViSP/hrp2CamParam/cMh.3";
  file.open (name.c_str());

  for(int i=0; i<4; ++i)
    {
      for(int j=0; j<4; ++j)
	{
	  file>>cameraMhead[i][j];
	}
    }

  file.close();


  vpHomogeneousMatrix headMcamera;
  headMcamera = cameraMhead.inverse();

  vpTwistMatrix m_hVc;

  m_hVc.buildFrom(headMcamera);

  cout<<"hV :\n"<<m_hVc*cV<<endl;


  vpHomogeneousMatrix pMh;
  pMh[0][0]=0.999181;  pMh[0][1]= 0.00477719;  pMh[0][2]= -0.0401735;pMh[0][3]= 0.240749;
  pMh[1][0]=-0.00477289; pMh[1][1]= 0.999989; pMh[1][2]= 0.000203027; pMh[1][3]= 0.0564035;
  pMh[2][0]= 0.040174; pMh[2][1]=-1.11169e-05;pMh[2][2]= 0.999193; pMh[2][3]=1.29624;
  pMh[3][0]=0; pMh[3][1]= 0;pMh[3][2]= 0;pMh[3][3]= 1;

 cout<< "pMh :\n"<< pMh<<endl;

 vpHomogeneousMatrix pMw;
 pMw[0][0]=0.9999955239; pMw[0][1]= -0.002992005605; pMw[0][2]= 1.54321294e-16; pMw[0][3]=  0.2267555394 ;
 pMw[1][0]= 0.002992005605; pMw[1][1]= 0.9999955239; pMw[1][2]= -1.426504294e-17; pMw[1][3]=  0.0563132705  ;
 pMw[2][0]=-1.542779221e-16 ; pMw[2][1]=1.472670926e-17;  pMw[2][2]= 1 ; pMw[2][3]= 0.6487018512  ;
 pMw[3][0]=0;pMw[3][1]=  0; pMw[3][2]=0; pMw[3][3]=1;

 cout<< "pMw :\n"<< pMw<<endl;

 vpHomogeneousMatrix wMh;

 wMh = pMw.inverse()*pMh;

 cout<< "wMh :\n"<< wMh<<endl;

 vpTwistMatrix wVh(wMh);

 cout<< "wVh :\n"<< wVh<<endl;


 cout<<"wV :\n"<< wVh*m_hVc*cV<<endl;


  return 0;
}

#else

int main(void)
{
  cout <<" Need Visp to execute Test Velocity Frame!" << endl;
  return 0;
}

#endif
