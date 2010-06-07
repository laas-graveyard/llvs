/** @doc Object to convert back and forth a ViSP image into a OpenCV image 
    

   Copyright (c) 2003-2010,
   @author Stephane Embarki

   see License file for more information
   on the license applied to this code.
   
   
*/
#ifndef _HRP2_VISP_IMAGE_CONVERT_PROCESS_H_
#define _HRP2_VISP_IMAGE_CONVERT_PROCESS_H_

#include <iostream>

#include "VisionBasicProcess.h"
/*! HRP2VisionBasicProcess is used to derive all the vision process
 * at least for the LowLevelVisionServer.
 */


// include opencv files
#include<cv.h>


// include visp lib files
#include<visp/vpImage.h>
#include <visp/vpImageIo.h>


/*!  \brief This class implements tracks an object model on a VISP image
  remark:all function that uses X11 are developped in the client
*/
class HRP2vispConvertImageProcess : public HRP2VisionBasicProcess
{

 public:

  typedef enum
    {  
	MAT_VISPRGB,
	MAT_VISPU8,
	VISPRGB_MAT,
	VISPU8_MAT

    } typeConversion ;	
 

  typedef struct 
  {
    unsigned int nChannel;
    unsigned int depth;
    unsigned int width;
    unsigned int widthStep;
    unsigned int height;
  } sImageParameters;
  
  /*! Constructor */
  HRP2vispConvertImageProcess(HRP2vispConvertImageProcess::typeConversion type);

  /*! Destructor */
  virtual ~HRP2vispConvertImageProcess();

  /*! Initialize the process. */
  int pInitializeTheProcess();

  /*! Realize the process */
  int pRealizeTheProcess();
  
   /*! Cleanup the process */
  int pCleanUpTheProcess();
 
  /*! Set a parameter */
  int SetParameter(string aParameter, string aValue);
   
  /*! Get the image */
  void SetImages(vpImage<unsigned char>* & ,cv::Mat* & ); 
  void SetImages(vpImage<vpRGBa>* &,cv::Mat* & );  


 
private:

  /*! Convert OpenCV Mat image To VISP U8 Image*/
  void ConvertMatToViSPU8Image( bool flip=false);  
  
  /*! Convert OpenCV Mat image To VISP RGBa Image*/
  void ConvertMatToViSPRGBaImage( bool flip=false); 

  /*! Convert VISP RGBa Image To OpenCV Mat image*/
  void ConvertViSPRGBaToMatImage();

  /*! Convert VISP U8 Image To OpenCV Mat image*/
  void ConvertViSPU8ToMatImage();


protected:

  /*! visp images*/
  vpImage<unsigned char> *m_VispGreyImage;
  vpImage<vpRGBa>        *m_VispRGBaImage;

  /*! OpenCV images*/
  cv::Mat		 *m_MatImage;

public:
  bool m_ImagesInitialized;
  bool m_imageConvertSucces;
  
  bool m_flip;
  
  typeConversion m_conversion;
  sImageParameters m_ImgParam;
 	
};



#endif 
