/** @doc This object implements a visual process
    to compute a disparity map from 2 camera images.
    
    CVS Information:
   $Id$
   $Author$
   $Date$
   $Revision$
   $Source$
   $Log$

   Copyright (c) 2003-2010,
   @author Claire Dune
   
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
#ifndef _HRP2_NMBT_TRACKING_PROCESS_H_
#define _HRP2_NMBT_TRACKING_PROCESS_H_

#include <iostream>

#include "VisionBasicProcess.h"
/*! HRP2VisionBasicProcess is used to derive all the vision process
 * at least for the LowLevelVisionServer.
 */



#include<cv.h>


// include visp lib files
#include<visp/vpImage.h>
#include<visp/vpMe.h>
//include lagadic tracking lib files
#include"nmbtTracking.h"


/*!
 This class implements tracks an object model on a VISP image
 remark:all function that uses X11 are developped in the client
 */
class HRP2nmbtTrackingProcess : public HRP2VisionBasicProcess
{

 public:
 
  /*! Constructor */
  HRP2nmbtTrackingProcess();

  /*! Destructor */
  virtual ~HRP2nmbtTrackingProcess();


  /*! Initialize the process. */
  int InitializeTheProcess();

  /*! Realize the process */
  int RealizeTheProcess();
  
  /*! Cleanup the process */
  int CleanUpTheProcess();

  /*! Load the Model*/
  int loadModel( const std::string & pathToModel, const std::string & modelName);

 /*! Get the name of the process */
  string GetName();

  /*! Set a parameter */
  int SetParameter(string aParameter, string aValue);


  
  
  /*! Set tracker parameters : moving edge parameters*/
  inline void setMovingEdges(vpMe &_me){tracker.setMovingEdges(_me)};
  
  /*! Set tracker parameters : gain of the virtual visual servoing*/
  inline void setLambda(const double _lambda){tracker.setLambda(_lambda)};

  /*! Set tracker parameters : camera parameters */
  inline void setCameraParameters(const vpCameraParameters & _cam){tracker.setCameraParameters(_cam)};  

  /*! Set tracker parameters : camera/object pose cMo*/
  inline void setcMo(const vpHomogeneousMatrix & _cMo){tracker.setcMo(_cMo)};  

  /*! Set the image */
  inline void setInputVispImage(const vpImage<unsigned char> & _I){m_InputVispImage=_I;}  
 
  /*! Set the inputcMo */
  inline void setInputcMo(const vpHomogeneousMatrix & _inputcMo){this->m_inputcMo=_inputcMo;} 
  
  /*! Set the inputcMo */
  inline void setOutputcMo(const vpHomogeneousMatrix & _outputcMo){this->m_outputcMo=_inputcMo;}


  /*! Get tracker parameters : gain of the virtual visual servoing*/
  inline void getLambda(double &_lambda){tracker.setLambda()};

  /*! Get tracker parameters : camera parameters */
  inline void getCameraParameters(vpCameraParameters & _cam){tracker.getCameraParameters(_cam)};  

  /*! Get tracker parameters : cMo camera /object pose */
  inline void getcMo(vpHomogeneousMatrix &cMo){tracker.getcMo();};

  /*! Get the image */
  inline void getInputVispImage(vpImage<unsigned char> & _I){_I=this->m_InputVispImage;}  
  
  /*! Get the inputcMo */
  inline void getInputcMo(vpHomogeneousMatrix & _inputcMo){_inputcMo=this->m_inputcMo;} 
  
  /*! Get the inputcMo */
  inline void getOutputcMo(vpHomogeneousMatrix & _outputcMo){_outputcMo=this->m_outputcMo;} 
 



protected:
  // lagadic tracker
  nmbTracking m_tracker;
   
  // visp images
  vpImage<unsigned char> m_inputVispImage;
   
  /*! transformation between the object and the camera*/
  vpHomogeneousMatrix m_inputcMo;
   
  /*! computed transformation between the object and the camera*/
  vpHomogeneousMatrix m_outputcMo;  
 

 public:
  bool m_inputImagesLoaded=false;
  bool m_cameraParamLoaded=false;
  bool m_modelLoaded=false;
  bool m_trackerTrackSuccess =false;
	
};



#endif 
