/** @doc Object to convert back and forth a ViSP image into a OpenCV image 
    

   Copyright (c) 2003-2010,
   @author Stephane Embarki

   see License file for more information
   on the license applied to this code.
   
   
*/
#ifndef _HRP2_VISP_IMAGE_UNDISTORT_PROCESS_H_
#define _HRP2_VISP_IMAGE_UNDISTORT_PROCESS_H_

#include <iostream>

#include "VisionBasicProcess.h"
/*! HRP2VisionBasicProcess is used to derive all the vision process
 * at least for the LowLevelVisionServer.
 */


// include visp lib files
#include<visp/vpImage.h>
#include<visp/vpImageIo.h>
#include<visp/vpCameraParameters.h>

/*!  \brief This class implements tracks an object model on a VISP image
  remark:all function that uses X11 are developped in the client
*/
class HRP2vispUndistordedProcess : public HRP2VisionBasicProcess
{

 public:

  typedef enum
    {  
	RGB_VISPRGB,
	RGB_VISPU8

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
  HRP2vispUndistordedProcess(HRP2vispUndistordedProcess::typeConversion type);

  /*! Destructor */
  virtual ~HRP2vispUndistordedProcess();

  /*! Initialize the process. */
  int InitializeTheProcess();

  /*! Realize the process */
  int RealizeTheProcess();
  
  /*! Cleanup the process */
   int CleanUpTheProcess();

  /*! Set a parameter */
  int SetParameter(string aParameter, string aValue);
   
  /*! Set the image */
  void SetImages(unsigned char * Iraw , vpImage<unsigned char>* &Ivisp ); 
  void SetImages(unsigned char *  Iraw , vpImage<vpRGBa>* Ivisp );  

  /*! Set the camera parameter*/
  void SetCameraParameters(const vpCameraParameters & _cam);


protected:

  /*! visp images*/

  vpImage<unsigned char> m_tmpVispGreyImages;
  vpImage<vpRGBa>         m_tmpVispRGBaImages;
  vpImage<unsigned char>* m_VispGreyImages;
  vpImage<vpRGBa> *       m_VispRGBaImages;

  /*! unsigned char images vector*/
  unsigned char *         m_RawImages; 



public:
  bool m_ImagesInitialized;
  bool m_imageUndistortSucces;
  bool m_CameraParamLoaded;

  bool m_flip;
  
  typeConversion m_conversion;
  sImageParameters m_ImgParam;
  vpCameraParameters   	m_CamParam;	
};



#endif 
