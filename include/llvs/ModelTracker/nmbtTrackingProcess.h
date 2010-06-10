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


// include opencv files
// #include <cv.h>		// already included in "VisionBasicProcess.h"

#if LLVS_HAVE_VISP && LLVS_HAVE_NMBT

// include visp lib files
#include <visp/vpImage.h>
#include <visp/vpMe.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpHomogeneousMatrix.h>

//include lagadic tracking lib files
#include <nmbt/nmbtTracking.h>


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

  /*! Default Param*/
  int SetDefaultParam(); 

  /*! Start the process. */
  int pStartProcess();

  /*! Initialize the process. */
  int pInitializeTheProcess();

  /*! Realize the process */
  int pRealizeTheProcess();
  
  /*! Cleanup the process */
  int pCleanUpTheProcess();

  /*! Set a parameter */
  int SetParameter(string aParameter, string aValue);
    
  /*! Set tracker parameters : moving edge parameters*/
  void SetMovingEdge(vpMe &_me);
  
  /*! Set tracker parameters : gain of the virtual visual servoing*/
  void SetLambda(const double _lambda);

  /*! Set tracker parameters : camera parameters */
  void SetCameraParameters(const vpCameraParameters & _cam);
 
  /*! Set tracker parameters : camera/object pose cMo*/
  void SetcMo(const vpHomogeneousMatrix & _cMo);  

  /*! Set the image */
  void SetInputVispImages(vpImage<unsigned char> * _I);  
 
  /* Set input image : on passe un pointeur vers une image 
     acquise dans "LLVS"
     dans la fonction on la converti en image visp
     et on appelle SetInputVispImages 
  */
  int SetInputImages();

  /*! Set Image Height*/
  void SetHeight(const int&_height);   

  /*! Set Image Width*/
  void SetWidth(const int&_width);   
  
  /*! Get tracker parameters : camera parameters */
  void GetCameraParameters(vpCameraParameters & _cam); 
  
  /*! Get tracker parameters : cMo camera /object pose */
  void GetcMo(vpHomogeneousMatrix &cMo);
  
  /*! Get the image */
  void GetInputVispImages(vpImage<unsigned char> & _I); 
   
  /*! Get the inputcMo */
  void GetInputcMo(vpHomogeneousMatrix & _inputcMo); 
  
  /*! Get the inputcMo */
  void GetOutputcMo(vpHomogeneousMatrix & _outputcMo);
  
  /*! Get Image Height*/
  void GetHeight(int&_height)   ;
  
  /*! Get Image Width*/
  void GetWidth(int&_width)  ;

 
private:
  
  // TODO : uniformiser les 3.

  /*! Parse camera parameters*/
  int ParseCamParam();
  
  /*! Load the Model*/
  int LoadModel(const std::string & pathToModel);

  /*! Parse pose init*/
  int LoadPose();

  /*! Convert LLVS image To VISP Image*/
  int ConvertLLVSImageToViSPImage();  
  

protected:

  // lagadic tracker
  nmbtTracking m_tracker;
   
  /*! visp images*/
  vpImage<unsigned char> *m_inputVispImage;
  
  /*! image dimensions*/
  int m_imageHeight;
  int m_imageWidth;
    
  /*! transformation between the object and the camera*/
  vpHomogeneousMatrix m_inputcMo;
   
  /*! computed transformation between the object and the camera*/
  vpHomogeneousMatrix m_outputcMo;  
 
  /*! camera parameters*/
  vpCameraParameters m_cam; 

  /*! path cam param*/
  string m_pathCam;
  
  /*! name cam*/
  string m_nameCam;

  /*! path pose cMo*/
  string m_pathPose;

  /*! path vrml*/
  string m_pathVrml;

  /*! perspective type*/
  vpCameraParameters::vpCameraParametersProjType m_projType;

 public:
  bool m_inputImagesLoaded;
  bool m_cameraParamLoaded;
  bool m_modelLoaded;
  bool m_trackerTrackSuccess;
  bool m_initPoseLoaded;
 	
};

#endif // LLVS_HAVE_VISP && LLVS_HAVE_NMBT


#endif 
