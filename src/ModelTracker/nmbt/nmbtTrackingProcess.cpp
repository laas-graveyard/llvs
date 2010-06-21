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

#if LLVS_HAVE_VISP && LLVS_HAVE_NMBT

#include <sstream>

#include <visp/vpConfig.h>
#include <visp/vpXmlParserCamera.h>
#include <visp/vpImageIo.h>

#include <visp/vpDisplayX.h>

// Debug macro
#include <llvs/tools/Debug.h>

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
  m_inputVispImage=0x0;
  m_me_modified = false;
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
  m_inputVispImage=0x0;

  
}

/*!------------------------------------- 
Set tracker parameters : moving edge parameters
-------------------------------------*/
void HRP2nmbtTrackingProcess::SetMovingEdge(vpMe &_me)
{
  m_tracker.setMovingEdge(_me);
}
  
/*!------------------------------------- 
Set tracker parameters : gain of the virtual visual servoing
-------------------------------------*/
void HRP2nmbtTrackingProcess::SetLambda(const double _lambda)
{
  m_tracker.setLambda(_lambda);
}

/*!------------------------------------- 
Set tracker parameters : camera parameters 
-------------------------------------*/
void HRP2nmbtTrackingProcess::SetCameraParameters(const vpCameraParameters & _cam)
{
  m_cam=_cam;m_tracker.setCameraParameters(_cam);
}


/*!
 Set Image Height
*/
void  HRP2nmbtTrackingProcess::SetHeight(const int&_height)
{
  m_imageHeight= _height;
}   

/*! 
  Set Image Width
*/
void  HRP2nmbtTrackingProcess::SetWidth(const int&_width)
{
  m_imageWidth=_width;
} 

/*! Get tracker parameters : camera parameters */
void  HRP2nmbtTrackingProcess::GetCameraParameters(vpCameraParameters & cam)
{
  m_tracker.getCameraParameters(cam);
}  

/*! Get tracker parameters : cMo camera /object pose */
void  HRP2nmbtTrackingProcess::GetcMo(vpHomogeneousMatrix &cMo)
{
  m_tracker.getPose(cMo);
}

/*! Get the image */
void  HRP2nmbtTrackingProcess::GetInputVispImages(vpImage<unsigned char> & I)
{
  I=*(m_inputVispImage);
}  
  
/*! Get the inputcMo */
void  HRP2nmbtTrackingProcess::GetInputcMo(vpHomogeneousMatrix & inputcMo)
{
  inputcMo=this->m_inputcMo;
} 
  
/*! Get the inputcMo */
void  HRP2nmbtTrackingProcess::GetOutputcMo(vpHomogeneousMatrix & outputcMo)
{
  outputcMo=this->m_outputcMo;
} 
 
/*! Get Image Height*/
void  HRP2nmbtTrackingProcess::GetHeight(int&height)
{
  height= m_imageHeight;
}   

/*! Get Image Width*/
void  HRP2nmbtTrackingProcess::GetWidth(int&_width)
{
  _width = m_imageWidth;
}  

/*!-------------------------------------
   Set Default Parameters
    - cMo  : init pose of the object in the image
    - path : path to file .wrl, the vrml model
-------------------------------------*/
int HRP2nmbtTrackingProcess::SetDefaultParam()
{

  //-------------------------------
  // create the path to the box
  //-------------------------------

  // get the home env var
  char* homePath;
  homePath = getenv ("HOME");
  
  // set the model default path
  //  string defaultPath ( "data/model/WoodenBox/WoodenBox");
  //ostringstream tmp_stream;
  //tmp_stream<< homePath << "/"<<defaultPath;
  //m_pathPose = tmp_stream.str(); 

  string defaultPath ( "./data/model/WoodenBox/WoodenBox");
  m_pathPose =defaultPath;
  m_pathVrml = defaultPath +".wrl";

  //tmp_stream<<".wrl";
  // m_pathVrml = tmp_stream.str(); 

  // load the model and set the flag model loaded to true
  LoadModel( m_pathVrml.c_str());
  m_modelLoaded = true;

 
  //-------------------------------
  // Load the init box pose
  //-------------------------------

  // read the pose
  LoadPose();
  m_tracker.setcMo(m_inputcMo) ;
  m_initPoseLoaded = true;

  //---------------------------------
  // Load the default camera parameters
  //--------------------------------- 

  // init path to xml file
  //string camParamPath ("./data/hrp2CamParam/hrp2.xml");
  //tmp_stream.str("");
  //tmp_stream<<homePath<< "/"<< camParamPath;
  //m_pathCam = tmp_stream.str();
  string camParamPath ("./data/ViSP/hrp2CamParam/hrp2.xml");
  m_pathCam =camParamPath;

  // init cam name
  m_nameCam = "cam1394_3_rectif";

  // init proje Type
  m_projType= vpCameraParameters::perspectiveProjWithoutDistortion;
  
  // init image dim
  m_imageWidth= 320;
  m_imageHeight= 240;
  
  // parse the cam
  ParseCamParam();

  // set the tracker cam parameters
  m_tracker.setCameraParameters(m_cam) ;
  m_cameraParamLoaded = true;

  //---------------------------------
  // everything is ok
  //---------------------------------
  return 0;


}


/*!-------------------------------------

Parse camera parameters

----------------------------------------*/
int HRP2nmbtTrackingProcess::ParseCamParam()
{

#if defined(VISP_HAVE_XML2) 
  //create a parser
  vpXmlParserCamera parser;
  parser.parse(m_cam,
		m_pathCam.c_str(),
		m_nameCam.c_str(),
		m_projType,
		m_imageWidth, 
		m_imageHeight);

  ODEBUG("camera parameter:\n"<< m_cam);
  ODEBUG("camera Path:\n"<< m_pathCam.c_str());
   m_cameraParamLoaded=true;
   return 0;
#else
  
   cerr << "Error: No parser available cannot parse the camera file"<<end;
   return -1;
   
#endif

   
}

/*------------------------------------

Read cMo From File
Load the last poses from files
--------------------------------------*/
int HRP2nmbtTrackingProcess::LoadPose()
{
   
  // create the file name
  ostringstream tmp_stream;
  tmp_stream << m_pathPose<<".0.pos";
  
  // open the file
  fstream finitpos ;
  finitpos.open(tmp_stream.str().c_str()) ;
  if(finitpos.fail() ){
    cout << "cannot read " <<tmp_stream.str()<<endl << "Init Failed"  << endl;
    return -1;
  }
  
  // if the file can be read
  vpPoseVector initpos ;
  finitpos >> initpos[0]; // tx
  finitpos >> initpos[1]; // ty
  finitpos >> initpos[2]; // tz
  finitpos >> initpos[3]; // thetaU x
  finitpos >> initpos[4]; // thetaU y
  finitpos >> initpos[5]; // thetaU z

  // build cMo
  m_inputcMo.buildFrom(initpos) ;
  
  //close file
  finitpos.close();
  
  // evrything went well  
  return 0;
}

/*-------------------------------------

Set input image

 --------------------------------------*/

void HRP2nmbtTrackingProcess::SetInputVispImages(vpImage<unsigned char> * _I)
{
  m_inputVispImage=_I;
  m_inputImagesLoaded=true;
 
  // if the dimension are different from the stored
  // one update the dimension and the calib param
  if (m_imageWidth!=(int) m_inputVispImage->getWidth())
    {
      m_imageWidth  = m_inputVispImage->getWidth();
      m_imageHeight = m_inputVispImage->getWidth();
      ParseCamParam();
      m_tracker.setCameraParameters(m_cam) ;
    }
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
PATH_CAM              path name to the cam xml file

TRAC_LAMBDA           lambda parameter gain of the 
                      virtual visual servoing

CAME_PROJ_TYP         camera projection type : "withDistorsion" or "withoutDistorsion"
CAME_NAME             camera name

-------------------------------------*/
int HRP2nmbtTrackingProcess::pSetParameter(std::string aParameter, std::string aValue)
{
  // use of the generic function to add the parameter in the parameter list
  // A parameter can be or cannot be associated with a value, 
  // thus an empty string for Value is correct.
  // If the parameter already exist is value is overwritten. 
  // If this is valid the index parameter >=0 is returned,
  // -1 otherwise.
  
  
  //int outputVBPSetParameters = HRP2VisionBasicProcess::SetParameter(aParameter,aValue);

  // get the 4 first parameter to find the parameter type
  // get 4 letters starting from the letter number 0
  string paramType = aParameter.substr(0,4);
  // get the  first parameter to find the parameter type
  // get 3 letters starting from the letter number 5
  string paramId = aParameter.substr(5,3);
  
  //cout << "paramType: " << paramType   << endl;
  //cout << "paramId: " << paramId   << endl;
  bool isAPathParam(false);
  bool isAVpMeParam(false);
  bool isATrackerParam(false);
  bool isACameraParam(false);


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
  else if(paramType=="CAME")
    {
      isACameraParam =true; 
    }
  else
    {
      cout << "Warning : unknown parameter :"<< aParameter << endl; 
      return -1;
    }
 
//--------VPME------------//
  if(isAVpMeParam)
    {
      static unsigned int paramnb=0;
      ODEBUG(" ENTER VPME CASE " << paramnb++);
  

      // convert the string aValue into a double
      std::istringstream i(aValue);
      double value;
      i >> value;

      //fill the appropriate vpMe field
      if (paramId=="MAS")//"VPME_MASK_SIZE"
	{ 
	  m_me.setMaskSize(value);
	  ODEBUG(" ENTER setMaskSize CASE value : "<<value );

	}
      else if (paramId=="RAN")//"VPME_RANGE"
	{
	  m_me.setRange(value);
	  ODEBUG(" ENTER setRange CASE value : "<<value );
	}
      else if (paramId=="THR")//"VPME_THRESHOLD"
	{
	  m_me.setThreshold(value);
	  ODEBUG(" ENTER THRESHOLD value : "<<value );
	}
      else if (paramId=="SAM")//"VPME_SAMPLE_STEP"
	{
	  m_me.setSampleStep(value);
	  ODEBUG(" ENTER SAM value : "<<value );
	}
      else if (paramId=="MU1")//"VPME_MU1 "
	{
	  m_me.setMu1(value);
	  ODEBUG(" ENTER MU1 value : "<<value );
	}
      else if (paramId=="MU2")//"VPME_MU2 "
	{
	  m_me.setMu2(value);
	  ODEBUG(" ENTER MU2 value : "<<value );
	}
      else if (paramId=="MU3")
	{
	  ODEBUG(" ENTER MU1 value : "<<value );
	  m_me.setMu1(value);
	  i >> value ;
	  m_me.setMu2(value);
	  ODEBUG(" ENTER MU2 value : "<<value );
	}
      else 
	{
	  cout << "Warning : unknown vpme parameter :"<< paramId << endl; 
	  return -1;
	}
      // If the tracker is not working
      if (!m_Computing)
	{
	  // the modification is realized 
	  // NOW !
	  m_tracker.setMovingEdge(m_me);
	}
      else 
	// Otherwise it is left to the time
	// where the tracker can makes 
	// the parameter changes.
	m_me_modified = true;

      //m_me.print();
      //m_tracker.getMovingEdge()->print();
    }
  
//-------- PATH ------------//
  else if(isAPathParam)
    {
     if (paramId=="MOD")//"PATH_MODEL"
       { 
 	 LoadModel(aValue);
         m_pathVrml = aValue;
       }
     else if (paramId =="CAM")//"PATH_CAM"
       {
         m_pathCam = aValue;
         ParseCamParam();
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
  
  //------- CAMERA--------------//
  else if(isACameraParam )
    {
      if (paramId=="PRO")//"CAME_PROJ_TYP"
       { 
         if(aValue=="withDistorsion")
	   {
	     m_projType=vpCameraParameters::perspectiveProjWithDistortion;
	     ParseCamParam();
	   }
	 else if(aValue=="withoutDistorsion")
	   {
	     m_projType=vpCameraParameters::perspectiveProjWithoutDistortion;
	     ParseCamParam();
	   }
	 else
           {
	     cout << "Warning : unknown distorsion type :"<< aValue << endl; 
             return -2;
           }

       }
      else if(paramId=="NAM")//"CAME_NAME"
       {
	 m_nameCam = aValue;
	 ParseCamParam();
       }
      else 
       {
	 cout << "Warning : unknown path parameter :"<< paramId << endl; 
	 return -1;
       }

    }
 
  //return(outputVBPSetParameters);
  return 0;
}

/*!-------------------------------------
Set cMo
------------------------------------- */
void HRP2nmbtTrackingProcess:: SetcMo(const vpHomogeneousMatrix & cMo)
{
  ODEBUG("GOING HERE INSIDE SetcMo " << cMo);
  m_inputcMo=cMo;   

  m_tracker.setcMo(m_inputcMo);
  m_initPoseLoaded = true;
}  


/*!------------------------------------- 
Initialize the process : initialize the Tracker


Using the current pointed image 
and the pose m_inputcMo,
 
The lines of the model are projected on the
image plane and some points on the lines are then used to define 
patches.

This Patches will be used to track the line in the image.


-------------------------------------*/
int HRP2nmbtTrackingProcess:: pInitializeTheProcess()
{
  ODEBUG("Initialize the process.");
  m_outputcMo.setIdentity();
  m_trackerTrackSuccess = false;
  m_tracker.init(*m_inputVispImage,m_inputcMo );
  ODEBUG("End of initialize the process.");
  return 0;
}


/*!----------------------------------------
Start the process
The tracker is initialize in this process
------------------------------------------*/
int HRP2nmbtTrackingProcess::pStartProcess()
{
  cout << "Go through pStartProcess" << endl;
  int r= pInitializeTheProcess();
  cout << "Went through pStartProcess" << endl;
  return r;

}


/*!------------------------------------- 
Realize the process 
the tracker has previously been initialised with: 
a cMo that is the init transform between the camera and the object
a pointer on an Image
some parameter for the tracking
the object model
   
-------------------------------------*/
int HRP2nmbtTrackingProcess::pRealizeTheProcess()
{
  m_trackerTrackSuccess = false;
  
  unsigned int r=0;

  if(m_inputImagesLoaded)
    {
 
      try
	{  	
	  m_tracker.track(*m_inputVispImage) ;
	}
      catch(std::string a) // tracking got lost
	{
	  
#if 1
	  std::cerr << std::endl;
	  std::cerr << "-----    -----   Failed with exception \"" << a << "\"     -----    -----" << std::endl;
	  std::cerr << std::endl;
#endif
	  
	  // set the tracking flag
	  m_trackerTrackSuccess= false;
	  
	  // set the cMo matrix to identity   
	  m_outputcMo.setIdentity();
	  
	  // return a negative value
	  r=-1;
	  

	}
      if (r==0)
	{
	  // tracking succeed
	  m_trackerTrackSuccess= true;
	  
	  // set the resulting transform between the object and the image
	  m_tracker.getPose(m_outputcMo);  
	}
#if 0
      static vpDisplayX display(*m_inputVispImage,0,0,"Tracking Server");
      vpDisplay::display(*m_inputVispImage);
      
      m_tracker.display(*m_inputVispImage,m_outputcMo,m_cam, vpColor::green,2);
#endif

      if (m_me_modified)
	{
	  m_tracker.setMovingEdge(m_me);
	  m_me_modified = false;
	}
      return r;
    }
  else 
    {
      cerr << "Error :HRP2nmbtTrackingProcess::RealizeTheProcess>>  No image " <<endl; 
      return -1;   
    }
 
}


  
/*!-------------------------------------
 Cleanup the process 
-------------------------------------*/
int HRP2nmbtTrackingProcess::pCleanUpTheProcess()
{
  m_tracker.resetTracker();
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


/*!-------------------------------------- 
  Convert LLVS image To VISP Image
-----------------------------------------*/
int HRP2nmbtTrackingProcess::ConvertLLVSImageToViSPImage()
{
  
  cout << "ConvertLLVSImageToViSPImage >> To be implemented" <<endl;
 
  return 0;
}  



/*!-------------------------------------- 
  Convert LLVS image To VISP Image
-----------------------------------------*/
int HRP2nmbtTrackingProcess::SetInputImages()
{

  //ConvertLLVSImageToViSPImage();
  cout << "SetInputImages >> To be implemented" <<endl;
  //SetInputVispImages(vpImage<unsigned char> * _I);  
  return 0;
}  

#endif
