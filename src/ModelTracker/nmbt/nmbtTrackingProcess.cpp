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
#include <sstream>

#include <visp/vpConfig.h>
#include <visp/vpXmlParserCamera.h>

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


/*!-------------------------------------
Default constructor
 -------------------------------------*/

HRP2nmbtTrackingProcess::HRP2nmbtTrackingProcess()
{
  m_ProcessName = "nmbtTrackingProcess";
  m_cameraParamLoaded = false;
  m_initPoseLoaded = false;
  m_inputImagesLoaded = false;
  m_modelLoaded = false;
  m_trackerTrackSuccess =false;

  SetDefaultParam();
}



/*!-------------------------------------
Destructor
------------------------------------- */
HRP2nmbtTrackingProcess:: ~HRP2nmbtTrackingProcess()
{
  m_cameraParamLoaded = false;
  m_inputImagesLoaded = false;
  m_modelLoaded = false;
  m_trackerTrackSuccess =false;
  m_initPoseLoaded = false;
}



/*!-------------------------------------
   Set Default Parameters
    - cMo  : init pose of the object in the image
    - path : path to file .wrl, the vrml model
-------------------------------------*/
int  HRP2nmbtTrackingProcess::SetDefaultParam()
{
  //----- create the path to the box
  char* homePath;
  homePath = getenv ("HOME");
  string defaultPath ( "data/model/WoodenBox/WoodenBox");
  ostringstream tmp_stream;
  tmp_stream<< homePath << "/"<<defaultPath<<".wrl";
  cout << "Path :" << tmp_stream.str()  <<endl;
  LoadModel(tmp_stream.str());
  m_modelLoaded = true;
 
  //----- Load the last poses from files
  fstream finitpos ;
  vpPoseVector initpos ;
  tmp_stream.str("");
  tmp_stream<< homePath << "/"<<defaultPath<<".0.pos";
  finitpos.open(tmp_stream.str().c_str()) ;
  if(finitpos.fail() ){
    cout << "cannot read " <<tmp_stream.str()<<endl << "Init Failed"  << endl;
    return -1;
  }
  finitpos >> initpos[0];
  finitpos >> initpos[1];
  finitpos >> initpos[2];
  finitpos >> initpos[3];
  finitpos >> initpos[4];
  finitpos >> initpos[5];

  finitpos.close(); 
  m_inputcMo.buildFrom(initpos) ;
  m_initPoseLoaded = true;

  //----- Load the default camera parameters
  tmp_stream.str("");
  string camParamPath ("data/hrp2CamParam/hrp2.xml");
  tmp_stream<<homePath<< "/"<< camParamPath;

#if defined(VISP_HAVE_XML2) 
  //create a parser
  vpXmlParserCamera parser;
  vpCameraParameters cam;
  int image_width(320);
  int image_height(240);
  string camName = "cam1394_4_rectif";
  parser.parse(cam,
  	       tmp_stream.str().c_str(),
  	       camName.c_str(),
  	       vpCameraParameters::perspectiveProjWithoutDistortion,
  	       image_width, 
	       image_height);
  m_tracker.setCameraParameters(cam) ;
#endif
  return 0;


}


/*!-------------------------------------
 Sets the tracker parameters

The parameter names can be :

parameters for the moving edge tracking
VPME_MASK_SIZE        correlation mask size
VPME_RANGE            length of the research area
VPME_THRESHOLD        threshold for point selection,
                      the higher, the less points
VPME_SAMPLE_STEP      set the number of smaple on a line
VPME_MU1              mu1
VPME_MU2              mu2

PATH_MODEL            path name to the place where 
                      the model are stored

TRAC_LAMBDA           lambda parameter gain of the 
                      virtual visual servoing

-------------------------------------*/
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
  string paramType = aParameter.substr(0,4);
  // get the  first parameter to find the parameter type
  string paramId = aParameter.substr(5,3);
  
  //cout << "paramType: " << paramType   << endl;
  //cout << "paramId: " << paramId   << endl;
  bool isAPathParam(false);
  bool isAVpMeParam(false);
  bool isATrackerParam(false);

  if (paramType=="VPME")
    {
      isAVpMeParam = true;
    }
  else if(paramType=="PATH")
    {
      isAPathParam = true;
    }
  else if(paramType=="TRAC")
    {
      isATrackerParam =true;  
    }
  else
    {
      cout << "Warning : unknown parameter :"<< aParameter << endl; 
      return -1;
    }
 
//--------VPME------------//
  if(isAVpMeParam)
    {
      // create a moving edge parameter
      vpMe me;

      // convert the string aValue into a double
      std::istringstream i(aValue);
      double value;
      i >> value;

      //fill the appropriate vpMe field
      if (paramId=="MAS")//"VPME_MASK_SIZE"
	{ 
	  me.setMaskSize(value);
	}
      else if (paramId=="RAN")//"VPME_RANGE"
	{
	  me.setRange(value);
	}
      else if (paramId=="THR")//"VPME_THRESHOLD"
	{
	  me.setThreshold(value);
	}
      else if (paramId=="SAM")//"VPME_SAMPLE_STEP"
	{
	  me.setSampleStep(value);
	}
      else if (paramId=="MU1")//"VPME_MU1 "
	{
	  me.setMu1(value);
	}
      else if (paramId=="MU2")//"VPME_MU2 "
	{
	  me.setMu2(value);
	}
      else 
	{
	  cout << "Warning : unknown vpme parameter :"<< paramId << endl; 
	  return -1;
	}
      m_tracker.setMovingEdge(me);
    }
  
//-------- PATH ------------//
  else if(isAPathParam)
    {
     if (paramId=="MOD")//"PATH_MODEL"
       { 
 	 LoadModel(aValue);
       }
     else 
       {
	 cout << "Warning : unknown path parameter :"<< paramId << endl; 
	 return -1;
       }
    }
//-------- TRACKER ------------//
  else if(isATrackerParam)
    {
     if (paramId=="LAM")//"TRAC_LAMBDA"
       { 
          std::istringstream i(aValue);
	  double value;
	  i >> value;
	  m_tracker.setLambda(value);
       }
     else 
       {
	 cout << "Warning : unknown path parameter :"<< paramId << endl; 
	 return -1;
       }
    }
  


  return(outputVBPSetParameters);
}

/*!-------------------------------------
Set cMo
------------------------------------- */
void HRP2nmbtTrackingProcess:: SetcMo(const vpHomogeneousMatrix & _cMo)
{
  m_inputcMo=_cMo;   
  m_tracker.setcMo(m_inputcMo);
  m_initPoseLoaded = true;
}  


/*!------------------------------------- 
Initialize the process. 
-------------------------------------*/
int HRP2nmbtTrackingProcess:: InitializeTheProcess()
{
  m_outputcMo.setIdentity();
  m_trackerTrackSuccess = false;
  m_tracker.setcMo(m_inputcMo);

  return 0;
}

/*!------------------------------------- 
Realize the process 
the tracker has previously been initialised with: 
a cMo that is the init transform between the camera and the object
a pointer on an Image
some parameter for the tracking
the object model
   
-------------------------------------*/
int HRP2nmbtTrackingProcess::RealizeTheProcess()
{
   
 
  if(m_inputImagesLoaded)
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
   m_trackerTrackSuccess= true;
   
   // set the resulting transform between the object and the image
   m_tracker.getPose(m_outputcMo);  

   return 0;

}
  
/*!-------------------------------------
 Cleanup the process 
-------------------------------------*/
int HRP2nmbtTrackingProcess::CleanUpTheProcess()
{

  return 0;
}

/*!-------------------------------------
 Load the Model
----------------------------------------*/
int HRP2nmbtTrackingProcess::LoadModel( const std::string & pathToModel)
{
         //TODO 
         // add a test to check that the 3 last letter are wrl 
         // add a test to check if the file exists
         // return an exception when one of these test fail
         //
  m_tracker.loadModel(pathToModel.c_str());
  
  return 0;
}

