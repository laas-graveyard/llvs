#ifndef _HRP2_File_INPUT_METHOD_H_
#define _HRP2_File_INPUT_METHOD_H_

/* This object implements a method to read images
   and put them as an entry point ot the low level vision 
   system.
   
   Copyright CNRS,JRL/AIST, 2004,
   Olivier Stasse

   21/05/2004: Creation
   29/06/2004: Add OpenCV possibilities.
*/

#include <string>
#include <iostream>
#include <fstream>
#include <ImagesInputMethod.h>
#include <vector> 
#include <assert.h>

#if (LLVS_HAVE_OPENCV>0)
#include <cv.h>
#endif 

using namespace std;

namespace llvs {

  /*! This object defines the class for the input method using files
    and related to the low level HRP2 vision system.
    This will be used to input either one or several files.
  */
  class HRP2FileImagesInputMethod : public HRP2ImagesInputMethod
    {
      typedef struct s_SimpleImage
      {
	unsigned int width, height;
	unsigned char *Data;
	unsigned int depth;
      } SimpleImage;

    public:
  
      /* Constants */
      static const int ONEIMAGE = 0;
      static const int DIRECTORY =1;

      /*! Constructor */
      HRP2FileImagesInputMethod(int MethodForInputImages);
  
      /*! Destructor */
      virtual ~HRP2FileImagesInputMethod();

      /*! Takes a new image.
       * Input: 
       * unsigned char * ImageLeft : A pointer where to store the bottom left image.
       * unsigned char * ImageRight : A pointer where to store the bottom right image.
       * unsigned char * ImageUp : A pointer where to store the upper image.
       */
      virtual int GetImage(unsigned char **ImageLeft, unsigned char **ImageRight, unsigned char **ImageUp);

      /*! Takes a new image.*/
      virtual unsigned int GetSingleImage(unsigned char **Image, const unsigned int& camera, double &timestamp);

      /*! Get the current format of the image */
      virtual string GetFormat(const unsigned int& CameraNumber) const;


      /*! Set the base name for reading the file containing the images. */
      int SetBaseName(string afilename);

      /*! Get the base name for reading the images. */
      string GetBaseName(void) const;

      /*! Read EPBM image */
      int ReadEPBMFileImage(string & afilename, unsigned char ** ImageLeft, unsigned char **ImageRight, unsigned char ** ImageUp,
			    int mode);

      /*! Read the body of one sub epbm image */
      int ReadEPBMBodyImage(ifstream & aifstream, unsigned char ** buffer, int lw, int lh, int depth);

      /*! Read the header of one sub epbm image */
      int ReadEPBMImageHeader(ifstream & aifstream, int CameraNumber, int &depth);
  
      /*! Returns the number of cameras */
      virtual unsigned int GetNumberOfCameras() const;

      /*! \brief Returns true if one camera is present */
      virtual bool CameraPresent() const;


      /*! \brief Initialize the grabbing system. 
	@return: True if initialization was successful.
	False otherwise.
       */
      virtual bool Initialize();

      /*! \brief Cleanup the grabbing system. 
       */
      virtual void Cleanup();

      /*! \brief Return the link between the detected camera 
       and its semantic. */
      virtual int GetSemanticOfCamera(const unsigned int& CameraNumberOnWS)
      { /* TODO */ 
	std::cerr<< __FILE__ << __LINE__ << " To implement" << std::endl;
	assert(false); 
	return -1;};

    protected:

      /* Name of the file to read from */
      string m_BaseName;

      /* How to read images */
      int m_MethodHowReading;

      /* Number of images */
      int m_NbOfImages;

      /* Image pointer (needed since GetSingleImage ) */
      vector< SimpleImage *> m_ReadImageData;
  
      /* Initialization during Get Number of Cameras . */
			//FIXME: See comment at GetNumberOfCameras definition
      //bool m_IsReallocationNeeded;
    };
} 

#endif /* _HRP2_INPUT_METHOD_H_ */
