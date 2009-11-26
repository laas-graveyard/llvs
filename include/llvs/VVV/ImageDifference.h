#ifndef _HRP2_IMAGE_DIFFERENCE_PROCESS_H_
#define _HRP2_IMAGE_DIFFERENCE_PROCESS_H_

#include "VisionBasicProcess.h"

class HRP2ImageDifferenceProcess : public HRP2VisionBasicProcess
{
 public:
  /* Constructor */
  HRP2ImageDifferenceProcess();

  /* Destructor */
  ~HRP2ImageDifferenceProcess();
  
  /*! Initialize the process */
  virtual int InitializeTheProcess();

  /*! Compute the edges */
  virtual int RealizeTheProcess();

  /*! Cleanup the process */
  virtual int CleanUpTheProcess();

  /*! Set the input images 
   * This method is needed to set up the reference to the input images.
   * They should be specified only once. The first image is the left image.
   * The second image is the second image. It is assume that those images
   * are corrected.
   */
  int SetInputImages(EPBM lInputImages[3]);

  
  /* Set Image to process.
     Index for which the value is different from 0 
     calls for process. */
  void SetImageOnWhichToProcess(int ImagesOnWhichToProcess[3]);

  /*! Reimplement the set parameter functions. */
  int SetParameter(string aParameter, string aValue);

  /*! Difference of Images */
  EPBM m_DifferenceOfImages[3];
  
  /*! Save the images */
  void SaveImages();

  /*! Specify the pointer for the time stamps. */
  int SetTimeStamp(struct timeval * ltimestamps);

 protected:
  
  /*! Input Image */
  EPBM m_InputImage[3];

  /*! Previous input Images */
  EPBM m_PrevInputImage[3];

  /*! Images on which perform the edge detection  3 at max.*/
  int m_ImagesOnWhichToProcess[3];

  /*! Buffer of images. */
  unsigned char * m_BufferOfImages;

  /*! Index of image. */
  int m_IndexOfImage;

  /*! Boolean to save images. */
  bool m_SaveImage;

  /*! Boolean to buffer images. */
  bool m_BufferImage;

  /*! Number of seconds to grab. */
  int m_NbOfSecondsToGrab;

  /*! Buffer of timestamp. */
  double *m_BufferOfTimeStamp;

  /*! Store the buffer related to the timestamps. */
  struct timeval *m_timestamps;
};
#endif /* _HRP2_IMAGE_DIFFERENCE_PROCESS_H_ */
