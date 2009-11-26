/** @doc This object implements a visual process
    to get IEEE camera images.
    
    CVS Information:
   $Id$
   $Author$
   $Date$
   $Revision$
   $Source$
   $Log$

   Copyright (c) 2003-2006, 
   @author Olivier Stasse
   
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
#ifndef _HRP2_IEEE1394_INPUT_METHOD_H_
#define _HRP2_IEEE1394_INPUT_METHOD_H_



#ifdef HAVE_CONFIG_H
#  include "vvvconf.h"
#endif

extern "C"
{
#include "vvvstd.h"
#include "vvvsize.h"
#include "vvverror.h"

#include "vfgb.h"
#include "vfgbtype.h"
#include "vfgbutil.h"
}


#define HAVE_TUToolsPP

#include <TU/Image++.h>
#include <TU/Ieee1394++.h>
using namespace TU;
#include <ImagesInputMethod.h>
#include <VisionBasicProcess.h>
#include <vfgb.h>
#include <string>

#include <vector>
using namespace std;

/* The class IEEE1394Board is coming from Ueshiba San library unfortunatly
   the camera in the private  field prevent any nice heritance and modification */
/************************************************************************
*  class IEEE1394Board							*
************************************************************************/
class IEEE1394Boardv2
{
  public:
  class Camera : public Ieee1394Camera
    {
      public:
	Camera(Ieee1394Port& port, int channel, u_int64 uniqId)		;

	void	snap()							;
	template <class T> void snap(T _lbuf)                               ;
	template <class T> void snap(T _lbuf, T _lcbuf);
	template <class T> void snapc(T _lbuf);
	void        doFlushListenBuffer()                                   ;
	u_char	get_pixel(int col, int row)				;
	TU::RGB	get_rgbPixel(int col, int row)				;
	template <class T> const Ieee1394Camera&
		operator >>(Image<T>& image)			const	;
	
      private:
	Image<u_char>	_grayimage;
	bool		_grayimageIsReady;
	Image<TU::RGB>	_rgbimage;
	bool		_rgbimageIsReady;
    };
    
  public:
    IEEE1394Boardv2(int board_id)						;
    ~IEEE1394Boardv2()							;

    u_int	nchildren()		const	{return _cameras.size();}
    u_int	width()			const	;
    u_int	height()		const	;

    void	snap()							;
    template <class T>
    void        snap(T* _lbuf);
    template <class T>
      void      snapc(T* _lbuf);
    template <class T> 
      void snapfromI(T *_lbuf, unsigned int camera);
    template <class T> 
      void snapcfromI(T *_lbuf, unsigned int camera);

    void	continuousShot()					;
    void	stopContinuousShot()					;
    template <class T>
    void	get_image(u_int camera_no, T* img)		const	;
    template <class T>
    void	get_images(T** imgs)				const	;
    u_char	get_pixel(u_int camera_no, int col, int row)	const	;
    TU::RGB		get_rgbPixel(u_int camera_no, int col, int row)	const	;
    
    void SetFeatureForCamera(u_int camera, Ieee1394Camera::Feature aFeature, u_int avalue);
    void GetFeatureForCamera(u_int camera, Ieee1394Camera::Feature aFeature, u_int &avalue);

    Ieee1394Port	_port;
    vector<Camera*>	_cameras;
};

inline u_int
IEEE1394Boardv2::width() const
{
    return (nchildren() > 0 ? _cameras[0]->width() : 0);
}

inline u_int
IEEE1394Boardv2::height() const
{
    return (nchildren() > 0 ? _cameras[0]->height() : 0);
}

template <class T> inline void
IEEE1394Boardv2::get_image(u_int camera_no, T* img) const
{
    if (camera_no >= nchildren())
	throw std::invalid_argument("Invalid camera number.");
    Image<T>	image(img, width(), height());
    *_cameras[camera_no] >> image;
}

template <class T> inline void
IEEE1394Boardv2::get_images(T** imgs) const
{
    for (u_int i = 0; i < nchildren(); ++i)
	get_image(i, imgs[i]);
}

inline u_char
IEEE1394Boardv2::get_pixel(u_int camera_no, int col, int row) const
{
    if (camera_no >= nchildren())
	throw std::invalid_argument("Invalid camera number.");
    return _cameras[camera_no]->get_pixel(col, row);
}

inline TU::RGB
IEEE1394Boardv2::get_rgbPixel(u_int camera_no, int col, int row) const
{
    if (camera_no >= nchildren())
	throw std::invalid_argument("Invalid camera number.");
    return _cameras[camera_no]->get_rgbPixel(col, row);
}

inline void
IEEE1394Boardv2::Camera::snap()
{
    _grayimageIsReady = false;
    _rgbimageIsReady = false;
    Ieee1394Camera::snap();
    
}

inline void
IEEE1394Boardv2::Camera::doFlushListenBuffer()
{
    Ieee1394Camera::flushListenBuffer();
    
}

template <class T>inline void
IEEE1394Boardv2::Camera::snap(T _lbuf)
{
    _grayimageIsReady = false;
    _rgbimageIsReady = false;
    if (bayerTileMapping() != Ieee1394Camera::YYYY &&
	(Ieee1394Camera::pixelFormat() == MONO_8 || Ieee1394Camera::pixelFormat() == MONO_16))
      Ieee1394Camera::snap().captureBayerRaw(_lbuf);
    else
      Ieee1394Camera::snap().captureRaw(_lbuf);

}

template <class T>inline void
IEEE1394Boardv2::Camera::snap(T _lbuf, T _lcbuf)
{
    _grayimageIsReady = false;
    _rgbimageIsReady = false;
    
    Ieee1394Camera::snap().captureBayerRaw(_lbuf);
    Ieee1394Camera::snap().captureRaw(_lcbuf);

}

template <class T>inline void
IEEE1394Boardv2::Camera::snapc(T _lbuf)
{
    _grayimageIsReady = false;
    _rgbimageIsReady = false;
    
    Ieee1394Camera::snap().captureRaw(_lbuf);

}

template <class T> inline const Ieee1394Camera&
IEEE1394Boardv2::Camera::operator >>(Image<T>& image) const
{
    return Ieee1394Camera::operator >>(image);
}
    
template <> inline const Ieee1394Camera&
IEEE1394Boardv2::Camera::operator >>(Image<TU::RGB>& image) const
{
    return captureRGBImage(image);
}
    



class HRP2IEEE1394ImagesInputMethod : public HRP2ImagesInputMethod, public HRP2VisionBasicProcess
{
 public:
  
  /*! Constantes */
  static const int CAMERA_LEFT = 0;
  static const int CAMERA_RIGHT = 1;
  static const int CAMERA_UP = 2;

  /*! Constructor */
  HRP2IEEE1394ImagesInputMethod(void);
  
  /*! Destructor */
  virtual ~HRP2IEEE1394ImagesInputMethod();

  /*! Takes a new image.
   * Input: 
   * \param unsigned char * ImageLeft : A pointer where to store the bottom left image.
   * \param unsigned char * ImageRight : A pointer where to store the bottom right image.
   * \param unsigned char * ImageUp : A pointer where to store the upper image.
   */
  virtual int GetImage(unsigned char **ImageLeft, unsigned char **ImageRight, 
		       unsigned char **ImageUp, unsigned char **ImageWide);

  /*! Takes a new image.
   * Input :
   * \param unsigned char * Image:  A pointer where to store the image.
   * \param int camera: The camera index.
   */
  virtual int GetSingleImage(unsigned char **Image, int camera,struct timeval &timestamp);

  /* Real implementation for PGM */
  int GetImagePGM(unsigned char **ImageLeft, unsigned char **ImageRight, 
		  unsigned char **ImageUp, unsigned char **ImageWide);

  int GetImageSinglePGM(unsigned char **Image, int camera, struct timeval &timestamp);
  int GetImageSingleRaw(unsigned char **Image, int camera, struct timeval &timestamp);
  int GetImageSingleRGB(unsigned char **Image, int camera, struct timeval &timestamp);

  /* Real implementation for single PGM */
  int GetImagePGM(unsigned char *Image, int camera);
  
  /* Real implementation for RGB */
  int GetImageRGB(unsigned char **ImageLeft, unsigned char **ImageRight, 
		     unsigned char **ImageUp, unsigned char **ImageWide);

  /* Real implementation for Raw */
  int GetImageRaw(unsigned char **ImageLeft, unsigned char **ImageRight, 
		     unsigned char **ImageUp, unsigned char **ImageWide);
  
  /*! Get the current format of the image */
  virtual string GetFormat();

  /*! Set the format of the current image: default PGM */
  int SetFormat(string aFormat);

  /*! Get the current image size for the appropriate camera */
  virtual int GetImageSize(int &lw, int &lh, int CameraNumber);

  /*! Set the size of the image willing to be grabbed. */
  virtual int SetImageSize(int lw, int lh, int CameraNumber);


  /*! Initialize the board */
  void InitializeBoard();
  
  /*! Stop the the board */
  void StopBoard();
  
  /*! Set parameter value */
  virtual int SetParameter(string aParameter, string aValue);
  
  /*! Override Start Process */
  virtual int StartProcess();
  
  /*! Override Stop Process */
  virtual int StopProcess();

  void GetCameraFeatureValue(string aCamera, string aFeature, string &aValue);
  void SetCameraFeatureValue(string aCamera, string aFeature, string aValue);

  /*! Returns the number of cameras */
  virtual int GetNumberOfCameras();

  void StartContinuousShot();
  void StopContinuousShot();
  
  /*! \brief Returns the next time when the camera CameraNumber
    will  grab. */
  virtual double NextTimeForGrabbing(int CameraNumber);
  
  /*! \brief From FrameRate to Time */
  void FromFrameRateToTime(int CameraNumber);
  
  /*! \brief Returns true if at least one camera is present. */
  virtual bool CameraPresent();

 protected:

  /*! File descriptor to the frame grabber */
  IEEE1394Boardv2 * m_Board;
  
  /*! Pointer to the copy memory. */
  unsigned char * m_TmpImage[4];

  /*! Format */
  string m_Format;

  /*! Prefixes for cameras */
  string m_Prefixes[4];
  
  /*! Prefixes for features */
  string m_Features[6];

  /*! Keep time for each camera. */
  vector <double> m_LastGrabbingTime;
  
  /*! Keep the period for each grabbing. */
  vector<double> m_GrabbingPeriod;

  /*! \brief This field tells us if one camera has been detected or not. */
  bool m_AtLeastOneCameraPresent;
  
    
};
  
#endif /* _HRP2_INPUT_METHOD_H_ */
