/** @doc This object implements a visual process to get a disparity map.

    Copyright (c) 2010, 
    @author Stephane Embarki
   
    JRL-Japan, CNRS/AIST
    
    See license file for information on license.
*/
#include <Debug.h>

#include "ViSP/vispConvertImageProcess.h"
#include <iostream>
#include <sstream>

#include <visp/vpConfig.h>

/*!-------------------------------------
  Default constructor
  -------------------------------------*/

HRP2vispConvertImageProcess::HRP2vispConvertImageProcess(typeConversion type)
{

  m_conversion = type;

  m_ProcessName = "vispConvertImageProcess";
  m_ImagesInitialized	= false;
  m_imageConvertSucces	= false;
  m_flip		= false;

  m_VispGreyImage=0x0;
  m_VispRGBaImage=0x0;
  m_MatImage=0x0;

}



/*!-------------------------------------
  Destructor
  ------------------------------------- */
HRP2vispConvertImageProcess:: ~HRP2vispConvertImageProcess()
{
  m_ImagesInitialized	= false;
  m_imageConvertSucces	= false;
  m_flip		= false;

  m_VispGreyImage=0x0;
  m_VispRGBaImage=0x0;
  m_MatImage=0x0;
}


/*! Get the images */
void  HRP2vispConvertImageProcess::SetImages(vpImage<unsigned char>* &Ivisp,
					     cv::Mat* &Icv)
{
  m_ImagesInitialized	= false;
  m_VispGreyImage = Ivisp;
  m_MatImage = Icv;

  if (m_conversion == MAT_VISPU8)
    {
      m_ImgParam.nChannel = m_MatImage->channels();
      m_ImgParam.depth = m_MatImage->depth();
      m_ImgParam.height = m_MatImage->rows;
      m_ImgParam.width = m_MatImage->cols;
      m_ImgParam.widthStep = m_MatImage->step;

      m_VispGreyImage -> resize(m_ImgParam.height,
				m_ImgParam.width) ;

    }


  if (m_conversion == VISPU8_MAT)
    {
      m_ImgParam.height = m_VispGreyImage->getHeight();
      m_ImgParam. width  = m_VispGreyImage->getWidth();

      m_ImgParam. depth = 0;
      m_ImgParam.nChannel = 1;

      if (m_MatImage != NULL)
	{
	  if(m_MatImage->channels() != (int) m_ImgParam.nChannel || 
	     m_MatImage->depth() != (int) m_ImgParam. depth || 
	     m_MatImage->rows !=  (int) m_ImgParam.height || 
	     m_MatImage->cols !=  (int) m_ImgParam. width)
	    {
	      if(m_MatImage->channels() != 0) 
		m_MatImage->release();

	      m_MatImage->create(m_ImgParam.height, m_ImgParam.width, CV_8UC1);
	    } 
	}
      else 
	m_MatImage->create(m_ImgParam.height, m_ImgParam.width, CV_8UC1);
      
      m_ImgParam.widthStep = m_MatImage->step;

    }


  m_ImagesInitialized	= true;
}  

void HRP2vispConvertImageProcess::SetImages(vpImage<vpRGBa>* & Ivisp,
						cv::Mat* & Icv)
{
  m_ImagesInitialized	= false;
  m_VispRGBaImage = Ivisp;
  m_MatImage = Icv;

  if (m_conversion == MAT_VISPRGB)
    {
      m_ImgParam.nChannel = m_MatImage->channels();
      m_ImgParam.depth = m_MatImage->depth();
      m_ImgParam.height = m_MatImage->rows;
      m_ImgParam.width = m_MatImage->cols;
      m_ImgParam.widthStep = m_MatImage->step;

      m_VispRGBaImage -> resize(m_ImgParam.height,
				m_ImgParam.width) ;

    }

  if (m_conversion == VISPRGB_MAT)
    {
      m_ImgParam.height = m_VispRGBaImage->getHeight();
      m_ImgParam. width  = m_VispRGBaImage->getWidth();

      m_ImgParam. depth = 0;
      m_ImgParam.nChannel = 3;

      if (m_MatImage != NULL)
	{
	  if(m_MatImage->channels() != (int) m_ImgParam.nChannel || 
	     m_MatImage->depth() != (int) m_ImgParam. depth || 
	     m_MatImage->rows !=  (int) m_ImgParam.height || 
	     m_MatImage->cols !=  (int) m_ImgParam. width)
	    {
	      if(m_MatImage->channels() != 0) 
		m_MatImage->release();

	      m_MatImage->create(m_ImgParam.height, m_ImgParam.width, CV_8UC3);
	    } 
	}
      else 
	m_MatImage->create(m_ImgParam.height, m_ImgParam.width, CV_8UC3);

      m_ImgParam.widthStep = m_MatImage->step;
    } 

    m_ImagesInitialized	= true;
}  



/*!-------------------------------------
  Sets the parameters

  The parameter names can be :

  -------------------------------------*/
int HRP2vispConvertImageProcess::SetParameter(std::string aParameter, 
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
 
  return(outputVBPSetParameters);
}

/*!------------------------------------- 
  Initialize the process. 
  -------------------------------------*/
int HRP2vispConvertImageProcess:: InitializeTheProcess()
{
  m_imageConvertSucces  = false;
 
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
int HRP2vispConvertImageProcess::RealizeTheProcess()
{
  m_imageConvertSucces = false;

  if(m_ImagesInitialized)
    {
      if ( m_conversion == MAT_VISPRGB)
	{
	  ConvertMatToViSPRGBaImage(m_flip);
	}
      else if	(m_conversion == MAT_VISPU8)	
	{
	  ConvertMatToViSPU8Image(m_flip);
	}
      else if	(m_conversion == VISPRGB_MAT)	
	{
	  ConvertViSPRGBaToMatImage();
	}
      else if	(m_conversion == VISPU8_MAT)	
	{
	  ConvertViSPU8ToMatImage();
	}
    }

  m_imageConvertSucces = true;

  return 0;
 
}
  
/*!-------------------------------------
  Cleanup the process 
  -------------------------------------*/
int HRP2vispConvertImageProcess::CleanUpTheProcess()
{
  return 0;
}

/*!--------------------------------------------
  Convert OpenCV MAT image To VISP Grey Image
  -----------------------------------------------*/

void HRP2vispConvertImageProcess::ConvertMatToViSPU8Image(bool flip)
{

  int lineStep = (flip) ? 1 : 0;
 
  if (flip == false)
    {
      if(m_ImgParam.widthStep == m_ImgParam.width){
	if(m_ImgParam.nChannel == 1 && m_ImgParam.depth == 0){

	  memcpy(m_VispGreyImage->bitmap, 
		 m_MatImage->data,
		 m_ImgParam.height* m_ImgParam.width);
	}
	if(m_ImgParam.nChannel == 3 && m_ImgParam.depth == 0)
	  {
  	    vpImageConvert::BGRToGrey((unsigned char*)m_MatImage->data,
				      m_VispGreyImage->bitmap,
				      m_ImgParam.width,
				      m_ImgParam.height,false);
  	  }
      }
      else{
	if(m_ImgParam.nChannel == 1 && m_ImgParam.depth == 0){

	  for (unsigned int i =0  ; i < m_ImgParam.height ; i++){

	    memcpy(m_VispGreyImage->bitmap+i*m_ImgParam.width, 
		   m_MatImage->data + i*m_ImgParam.widthStep,
		   m_ImgParam.width);
	  }
	}
	if(m_ImgParam.nChannel == 3 && m_ImgParam.depth == 0){
	  m_VispGreyImage->resize(m_ImgParam.height,m_ImgParam.width) ;
	  for (unsigned int i = 0  ; i < m_ImgParam.height ; i++){
	    vpImageConvert::BGRToGrey((unsigned char*)m_MatImage->data + i*m_ImgParam.widthStep,
				      m_VispGreyImage->bitmap + i*m_ImgParam.width,
				      m_ImgParam.width,1,false);
	  }
	}
      }
    }
  else 
    {
      if(m_ImgParam.nChannel == 1 && m_ImgParam.depth == 0){
	unsigned char* beginOutput = (unsigned char*)m_VispGreyImage->bitmap;
	m_VispGreyImage->resize(m_ImgParam.height,
				m_ImgParam.width) ;
	for (unsigned int i =0  ; i < m_ImgParam.height ; i++){
	  memcpy(beginOutput + lineStep * ( 4 * m_ImgParam.width * ( m_ImgParam.height - 1 - i ) ) , 
		 m_MatImage->data + i*m_ImgParam.widthStep,
		 m_ImgParam.width);
	}
      }
      if(m_ImgParam.nChannel == 3 && m_ImgParam.depth == 0){
	m_VispGreyImage->resize(m_ImgParam.height,m_ImgParam.width) ;
	//for (int i = 0  ; i < height ; i++){
	vpImageConvert::BGRToGrey((unsigned char*)m_MatImage->data /*+ i*widthStep*/,
				  m_VispGreyImage->bitmap /*+ i*width*/,
				  m_ImgParam.width,m_ImgParam.height/*1*/,true);
	//}
      }
    }
}  
  
/*!----------------------------------------------
  Convert OpenCV MAT image To VISP RGBa Image
  -------------------------------------------------*/

void HRP2vispConvertImageProcess::ConvertMatToViSPRGBaImage(bool flip)
{
  int lineStep = (flip) ? 1 : 0;

  if(m_ImgParam.nChannel == 3 &&m_ImgParam. depth == 0)
    {
       //starting source address
      unsigned char* input = 
	(unsigned char*)m_MatImage->data;
      unsigned char* line;
      unsigned char* beginOutput = 
	(unsigned char*)m_VispRGBaImage->bitmap;
      unsigned char* output = NULL;

      for(unsigned int i=0 ; i <m_ImgParam. height ; i++)
	{
	  line = input;
	  output = beginOutput + lineStep 
	    * ( 4 * m_ImgParam.width * (m_ImgParam. height - 1 - i ) ) 
	    + (1-lineStep) * 4 *m_ImgParam.width * i;
	  for(unsigned int j=0 ; j < m_ImgParam.width ; j++)
	    {
	      *(output++) = *(line+2);
	      *(output++) = *(line+1);
	      *(output++) = *(line);
	      *(output++) = 0;

	      line+=3;
	    }
	  //go to the next line
	  input+=m_ImgParam.widthStep;
	}
    }
  else if(m_ImgParam.nChannel == 1 && m_ImgParam.depth == 0 )
    {
      //starting source address
      unsigned char * input = 
	(unsigned char*)m_MatImage->data;
      unsigned char * line;
      unsigned char* beginOutput = 
	(unsigned char*)m_VispRGBaImage->bitmap;
      unsigned char* output = NULL;

      for(unsigned int i=0 ; i <m_ImgParam.height ; i++)
	{
	  line = input;
	  output = beginOutput 
	    + lineStep * ( 4 * m_ImgParam.width * ( m_ImgParam.height - 1 - i ) ) 
	    + (1-lineStep) * 4 * m_ImgParam.width * i;
	  for(unsigned int j=0 ; j < m_ImgParam.width ; j++)
	    {
	      *output++ = *(line);
	      *output++ = *(line);
	      *output++ = *(line);
	      *output++ = *(line);

	      line++;
	    }
	  //go to the next line
	  input+=m_ImgParam.widthStep;
	}
    }
}



/*!--------------------------------------------------------
  Convert VISP RGBa Image To OpenCV Mat image
  ----------------------------------------------------------*/
void HRP2vispConvertImageProcess::ConvertViSPRGBaToMatImage()
{

  //starting source address
  unsigned char * input = 
    (unsigned char*)m_VispRGBaImage->bitmap;//rgba image
  unsigned char * line;
  unsigned char * output = 
    (unsigned char*)m_MatImage->data;//bgr image

  unsigned int j=0;
  unsigned int i=0;

  for(i=0 ; i < m_ImgParam.height ; i++)
    {
      output = (unsigned char*)m_MatImage->data + i*m_ImgParam.widthStep;
      line = input;
      for( j=0 ; j <  m_ImgParam.width ; j++)
	{
	  *output++ = *(line+2);  //B
	  *output++ = *(line+1);  //G
	  *output++ = *(line);  //R

	  line+=4;
	}
      //go to the next line
      input+=4* m_ImgParam.width;
    }
}

/*!--------------------------------------------------------
  Convert VISP U8 Image To OpenCV Mat image
  ----------------------------------------------------------*/
void HRP2vispConvertImageProcess::ConvertViSPU8ToMatImage()
{

  if ( m_ImgParam.width == m_ImgParam.widthStep){
    memcpy(m_MatImage->data,m_VispGreyImage->bitmap,m_ImgParam.width*m_ImgParam.height);
  }
  else{
    //copying each line taking account of the widthStep
    for (unsigned int i =0  ; i < m_ImgParam.height ; i++){
          memcpy(m_MatImage->data + i*m_ImgParam.widthStep,m_VispGreyImage->bitmap + i*m_ImgParam.width,
                m_ImgParam.width);
    }
  }
}






