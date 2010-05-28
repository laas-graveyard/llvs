/** @doc This object implements a visual process to get a disparity map.
    
    CVS Information:
   $Id$
   $Author$
   $Date$
   $Revision$
   $Source$
   $Log$

   Copyright (c) 2003-2010, 
   @author Torea Foissotte
   
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
#include "ModelTracker/nmbtTrackingProcess.h"
#include <iostream>




#define ODEBUG2(x)
#define ODEBUG3(x) cerr << __FILE__ << ": l" << __LINE__ << ": " << x << endl
#define ODEBUG3_CONT(x) cerr << x 

#if 0
#define ODEBUG(x) cerr << __FILE__ << ": l" << __LINE__ << ": " << x << endl
#define ODEBUG_CONT(x) cerr << __FILE__ << ": l" << __LINE__ << ": " << x << endl
#else
#define ODEBUG(x) 
#define ODEBUG_CONT(x) 
#endif



HRP2nmbtTrackingProcess::HRP2nmbtTrackingProcess()
{
  m_ProcessName = "nmbtTrackingProcess";
  m_cameraParamLoaded = false;
  m_inputImagesLoaded = false;
  m_modelLoaded = false;
  m_trackerTrackSuccess =false;

}

/*HRP2nmbtTrackingProcess::HRP2nmbtTrackingProcess(const vpImage<unsigned char>& _I,
						 const vpCameraParameters & _cam, 
						 const vpHomogeneousMatrix & _cMo,
						 const std::string & _pathToModel, 
						 const std::string & _modelName)
{
  
  m_ProcessName = "nmbtTrackingProcess";
  m_cameraParamLoaded = true;
   
  m_inputVispImage = _I; 
  m_inputImagesLoaded = true;
  
  loadModel( _pathToModel, _modelName);
  tracker.setCameraParameters(_cam);
  m_modelLoaded = true;

  m_inputcMo = _cMo;
  tracker.setcMo(_cMo);
  
  }*/




HRP2nmbtTrackingProcess:: ~HRP2nmbtTrackingProcess()
{
  
}



/*! Sets the tracker parameters

The parameter names can be :

parameters for the moving edge tracking
VPME_MASK_SIZE        correlation mask size
VPME_RANGE            length of the research area
VPME_THRESHOLD        threshold for point selection, the higher, the less points
VPME_SAMPLE_STEP      set the number of smaple on a line
VPME_MU1              mu1
VPME_MU2              mu2

MODEL_PATH            path name to the place where the model are stored
MODEL_NAME            name of the current model 

*/
int HRP2nmbtTrackingProcess::SetParameter(std::string aParameter, std::string aValue)
{
  // use of the generic function to add the parameter in the parameter list
  // A parameter can be or cannot be associated with a value, 
  // thus an empty string for Value is correct.
  // If the parameter already exist is value is overwritten. 
  // If this is valid the index parameter >=0 is returned,
  // -1 otherwise.
  int outputVBPSetParameters = HRP2VisionBasicProcess::SetParameter(aParameter,aValue);

  // get the 4 first parameter to find the parameter type
  std::string paramType = aParameter.substr(0,4);
  // get the  first parameter to find the parameter type
  std::string paramId = aParameter.substr(6,8);
  

  return(outputVBPSetParameters);
}

/*! Initialize the process. */
int HRP2nmbtTrackingProcess:: InitializeTheProcess()
{
  m_outputcMo.setIdentity();
  m_trackerTrackSuccess = false;
  m_tracker.setcMo(m_inputcMo);
}

/*! Realize the process 
the tracker has previously been initialised with: 
a cMo that is the init transform between the camera and the object
a pointer on an Image
some parameter for the tracking
the object model
   
*/
int HRP2nmbtTrackingProcess::RealizeTheProcess()
{
   
   
   try
   {  	
     m_tracker.track(m_inputVispImage) ;
   }
   catch(std::string a) // tracking got lost
   {
     std::cerr << std::endl;
     std::cerr << "-----    -----   Failed with exception \"" << a << "\"     -----    -----" << std::endl;
     std::cerr << std::endl;
     
     // set the tracking flag
     m_trackerTrackSuccess= false;
     
     // set the cMo matrix to identity   
     m_outputcMo.setIdentity();
 
     // return a negative value
     return -1;
      
   }

   // tracking succeed
   
   // set the tracking flag
   m_trackerTrackSuccess= false;
   
   // set the resulting transform between the object and the image
   m_tracker.getPose(m_outputcMo);  

   return 0;

}
  
/*! Cleanup the process */
int HRP2nmbtTrackingProcess::CleanUpTheProcess()
{


}

/*! Load the Model*/
int HRP2nmbtTrackingProcess::loadModel( const std::string & pathToModel, const std::string & modelName)
{
  // load the initial position matrix file
  std::ostringstream tmp_stream; 
  tmp_stream << pathToModel<< "/" << modelName<<"/"<< modelName<<".wrl" ;
  
  m_tracker.loadModel(tmp_stream.str().c_str());
}

