/** @doc This object implements a color detection algorithm
    based on histograms.

   Copyright (c) 2003-2011, 
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

#include "LocalControlSwitcher.h"

using namespace llvs;

HRP2ControlSwitcher::HRP2ControlSwitcher(LowLevelVisionServer *lLLVS)
{
  m_ProcessName = "LocalControlSwitcher";
  m_LLVS=lLLVS;
  m_CurrentState = IDLE;
}

HRP2ControlSwitcher::~HRP2ControlSwitcher()
{
}

void HRP2ControlSwitcher::SetConnectionToSot(llvs::ConnectionToSot * aCTS)
{
  m_CTS = aCTS;
}

int HRP2ControlSwitcher::pInitializeTheProcess()
{
  m_CurrentState = VISP_TRACKING;
  m_minResult[0] = 10000000.0;
  m_minResult[1] = 10000000.0;
  m_minResult[2] = 10000000.0;
  m_normResult = 100000000000.0;
}

int HRP2ControlSwitcher::pRealizeTheProcess()
{
  if (m_CTS!=0)
    {

      if (m_CurrentState==VISP_TRACKING)
	{
	  double error = 0.07;

	  double waistrpy[6];
	  double result[3];

	  double waistpostoswitch[6] = { 3.73826,
					 -2.113995926,
					 0.0, 
					 0.0,
					 0.0,
					 1.742435326};

	  m_CTS->ReadWaistRPYSignals(waistrpy);
	  result[0] = fabs(waistrpy[0]-waistpostoswitch[0]);
	  result[1] = fabs(waistrpy[1]-waistpostoswitch[1]);
	  result[2] = fabs(waistrpy[5]-waistpostoswitch[5]);
	  
	  double normResult = sqrt(result[0]*result[0] +
				   result[1]*result[1] + 
				   result[2]*result[2]);
	  
	  
	  if (normResult<error)
	    {
	      
	      cout << "Perform control law switching: HEAD_MOVING"<<endl;
	      m_CCLP->StopProcess();
	      string aSoTCommand = "sot.rm taskHeadVS";
	      m_CTS->SendCommand(aSoTCommand);
	      aSoTCommand = "set necklimit.joint [2](-0.17,-0.17)";
	      m_CTS->SendCommand(aSoTCommand);
	      aSoTCommand = "sot.push taskHead";
	      m_CTS->SendCommand(aSoTCommand);
	      aSoTCommand = "sot.up taskHead";
	      m_CTS->SendCommand(aSoTCommand);
	      m_CurrentState = HEAD_MOVING;
	      m_LLVS->StartProcess("Color Detection Based On Histogram");
	      struct timeval atv;
	      gettimeofday(&atv,0);
	      m_timeForHeadMoving = atv.tv_sec + (double)atv.tv_usec / 1000000.0;
	    }
	  else 
	    {
	      if(normResult < m_normResult)
		{
		  
		  m_normResult = normResult;
		  m_minResult[0] = result[0];
		  
		  m_minResult[1] = result[1];
		  
		  m_minResult[2] = result[2];

		  /* 
		  cout << "r = " 
		       << result[0]<< " "
		       << result[1]<< " "
		       << result[2]<< " | "
		       << waistrpy[0]<< " "
		       << waistrpy[1]<< " "
		       << waistrpy[5]<< " | "
		       << m_normResult << " " 
		       << endl;

		  */
		}		  
	    }
	}
      else if (m_CurrentState==HEAD_MOVING)
	{
	  struct timeval atv;
	  gettimeofday(&atv,0);
	  double lCurrentTime = atv.tv_sec + (double)atv.tv_usec / 1000000.0;
	      
	  if (lCurrentTime-m_timeForHeadMoving>7.0)
	    {
	      cout << "Perform control law switching: COLOR_TRACKING"<<endl;
	      string aSoTCommand = "sot.rm taskHead";
	      m_CTS->SendCommand(aSoTCommand);
	      aSoTCommand = "sot.push taskHeadVS";
	      m_CTS->SendCommand(aSoTCommand);
	      aSoTCommand = "sot.up taskHeadVS";
	      m_CTS->SendCommand(aSoTCommand);
	      m_CurrentState = COLOR_TRACKING;
	    }
	}
      else if (m_CurrentState==COLOR_TRACKING)
	{
	  double waistrpy[6];
	  double result[3];
	  double error = 0.07;

	  double waistpostoswitch[6] = { 6.57244,
					 -0.481419,
					 0.0, 
					 0.0,
					 0.0,
					 -1.27349};

	  m_CTS->ReadWaistRPYSignals(waistrpy);
	  result[0] = fabs(waistrpy[0]-waistpostoswitch[0]);
	  result[1] = fabs(waistrpy[1]-waistpostoswitch[1]);
	  result[2] = fabs(waistrpy[5]-waistpostoswitch[5]);
		  
	  double normResult = sqrt(result[0]*result[0] +
				   result[1]*result[1] + 
				   result[2]*result[2]);

	  cout << "r = " 
	       << result[0]<< " "
	       << result[1]<< " "
	       << result[2]<< " | "
	       << waistrpy[0]<< " "
	       << waistrpy[1]<< " "
	       << waistrpy[5]<< " | "
	       << m_normResult << " " 
	       << endl;

	  if (normResult<error)
	    {
	      m_CurrentState = END_BEHAVIOR;
	      cout << "Perform control switch to END_BEHAVIOR" << endl;
	      string aSoTCommand = "sot.rm taskHeadVS";
	      m_CTS->SendCommand(aSoTCommand);
	    }

	}
      
      
    }
}

int HRP2ControlSwitcher::pCleanUpTheProcess()
{

}

void HRP2ControlSwitcher::setComputeControlLaw(HRP2ComputeControlLawProcess * aCCLP)
{
  m_CCLP = aCCLP;
}
