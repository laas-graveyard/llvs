/** @doc This object implements a visual process to get a disparity map.

    Copyright (c) 2010, 
    @author Stephane Embarki
   
    JRL-Japan, CNRS/AIST
    
    See license file for information on license.
*/
#include <Debug.h>

#include "ViSP/vispUndistordedProcess.h"
#include <iostream>
#include <sstream>

#include <visp/vpConfig.h>
#include <visp/vpImageTools.h>
/*!-------------------------------------
  Default constructor
  -------------------------------------*/

HRP2vispUndistordedProcess::HRP2vispUndistordedProcess(typeConversion type)
{
  m_CameraParamLoaded     =  false;
  m_conversion = type;

  m_ProcessName = "vispUndistordedProcess";
  m_ImagesInitialized	= false;
  m_imageUndistortSucces	= false;
  m_flip		= false;

  m_VispGreyImages=0x0;
  m_VispRGBaImages=0x0 ;
  m_RawImages=0x0 ;

}



/*!-------------------------------------
  Destructor
  ------------------------------------- */
HRP2vispUndistordedProcess:: ~HRP2vispUndistordedProcess()
{

  m_CameraParamLoaded     =  false;
  m_ImagesInitialized	= false;
  m_imageUndistortSucces	= false;
  m_flip		= false;

  m_VispGreyImages=0x0;
  m_VispRGBaImages=0x0 ;
  m_RawImages=0x0; 
 
}


/*! Set the images */
void  HRP2vispUndistordedProcess::SetImages(unsigned char ** Iraw 
					    ,vpImage<unsigned char>* Ivisp )
{
 //TODO some verification and resize?
  m_ImagesInitialized	= false;
  m_VispGreyImages      = Ivisp;
  m_RawImages           = Iraw;

  m_ImgParam.width=Ivisp->getWidth();
  m_ImgParam.height=Ivisp->getHeight();

  m_tmpVispGreyImages.resize( m_ImgParam.height, m_ImgParam.width);

  m_ImagesInitialized	= true;
}  

void HRP2vispUndistordedProcess::SetImages(unsigned char ** Iraw 
					    ,vpImage<vpRGBa>* Ivisp )
{
  //TODO some verification and resize?
  m_ImagesInitialized	= false;
  m_VispRGBaImages      = Ivisp;
  m_RawImages           = Iraw;

  m_ImagesInitialized	= true;
}  
/*! Set the camera paramaters */
void HRP2vispUndistordedProcess::SetCameraParameters(const vpCameraParameters &_cam)
{ 
  m_CameraParamLoaded     =  false;
  m_CamParam=_cam;
 m_CameraParamLoaded     =  true;
}

/*!-------------------------------------
  Sets the parameters

  The parameter names can be :

  FLIP = ON or OFF
  TYPE_CONV = RGB_VISPU8 or RGB_VISPU8_NONE
  -------------------------------------*/
int HRP2vispUndistordedProcess::SetParameter(std::string aParameter, 
					      std::string aValue)
{
  // use of the generic function to add the parameter in the parameter list
  // A parameter can be or cannot be associated with a value, 
  // thus an empty string for Value is correct.
  // If the parameter already exist is value is overwritten. 
  // If this is valid the index parameter >=0 is returned,
  // -1 otherwise.
  int outputVBPSetParameters = HRP2VisionBasicProcess::SetParameter(aParameter,aValue);


  if (aParameter=="FLIP")
    {
      if ( aValue == "ON")
	{
	  m_flip = true;
	}
      else if(aValue == "OFF")
	{
	  m_flip = false;
	}
      else 
	{
	  cout << "Warning : unknown \"FLIP\" value :"<< aValue << endl; 
	  return -1;
	}
    }
  else  if (aParameter=="TYPE_CONV")
    {
      if (aValue=="RGB_VISPU8")
	{
	  m_conversion = RGB_VISPU8;
	}
      else if (aValue=="RGB_VISPU8_NONE")
	{
	  m_conversion = RGB_VISPU8_NONE;
	}
    }


  return(outputVBPSetParameters);
}

/*!------------------------------------- 
  Initialize the process. 
  -------------------------------------*/
int HRP2vispUndistordedProcess:: pInitializeTheProcess()
{
  m_imageUndistortSucces  = false;

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
int HRP2vispUndistordedProcess::pRealizeTheProcess()
{
  m_imageUndistortSucces = false;

  if(m_ImagesInitialized)
    {

      if(m_conversion == RGB_VISPU8 ) 
	{
	  
	  vpImageConvert::RGBToGrey( *m_RawImages,
				     m_tmpVispGreyImages.bitmap,
				     m_ImgParam.width,
				     m_ImgParam.height, m_flip);

	  vpImageTools::undistort(m_tmpVispGreyImages,
				  m_CamParam,
				  *(m_VispGreyImages));

	}

      if(m_conversion == RGB_VISPU8_NONE) 
	{
	  
	  vpImageConvert::RGBToGrey( *m_RawImages,
				     m_VispGreyImages->bitmap,
				     m_ImgParam.width,
				     m_ImgParam.height, m_flip);
	}

    }

  m_imageUndistortSucces = true;

  return 0;
 
}

int  HRP2vispUndistordedProcess::pCleanUpTheProcess()
{

  return 0;
} 
