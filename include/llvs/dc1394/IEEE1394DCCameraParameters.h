/** @doc This object implements a visual process
    to get IEEE camera images using the DC libraries.
    

   Copyright (c) 2003-2006, 
   @author Olivier Stasse, 
   
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
#ifndef _HRP2_IEEE1394_DC_CAMERA_PARAMETERS_H_
#define _HRP2_IEEE1394_DC_CAMERA_PARAMETERS_H_

#include <string>

namespace llvs
{
  /*! \brief This class handle a user based description of the camera. 
    It gives for instance default parameters values.
  */
  class IEEE1394DCCameraParameters
    {
    public:
      /*! \brief Constructor. */
      IEEE1394DCCameraParameters();

      /*! \brief Destructor. */
      ~IEEE1394DCCameraParameters();
      /*! \name Setter and getter 
       @{ */

      const unsigned int & GetBoardNumber() const;
      void SetBoardNumber(const unsigned int & );

      const unsigned int & GetCameraNumberInUserSemantic() const ;
      void SetCameraNumberInUserSemantic(const unsigned int &);
      
      const std::string & GetGUID() const;
      void SetGUID(const std::string &);
      
      const std::string & GetFormat() const;
      void SetFormat(const std::string &);

      const std::string & GetFPS() const;
      void SetFPS(const std::string & );

      const unsigned int & GetBrightness() const;
      void SetBrightness(const unsigned int & );
      
      const unsigned int & GetExposure() const;
      void SetExposure(const unsigned int & );

      void GetWhiteBalance(unsigned int WhiteBalance[2]) const;
      void SetWhiteBalance(unsigned int WhiteBalance[2]);

      const unsigned int & GetGamma() const;
      void SetGamma(const unsigned int & );

      const unsigned int & GetShutter() const;
      void SetShutter(const unsigned int & );

      const unsigned int & GetGain() const;
      void SetGain(const unsigned int & );
      
      /*! @} */
      
    private:

      /*! \brief Board on which the camera is connected. */
      unsigned int m_BoardNumber;
      
      /*! \brief Camerea number in the user semantic. 
	default : the order of the detection is the semantic order. */
      unsigned int m_CameraNumberInUserSemantic;
      
      /*! \brief IEEE 1394 global unique identifier. */
      std::string m_GUID; 
      
      /*! \brief Format ( Color + Image size )*/
      std::string m_Format;

      /*! \brief Frame rate per second*/
      std::string m_FPS;
      
      /*! \brief Brightness. */
      unsigned int m_Brightness;
      
      /*! \brief Auto exposure. */
      unsigned int m_Exposure;
      
      /*! \brief White balance values. */
      unsigned int m_WhiteBalance[2];
      
      /*! \brief Gamma */
      unsigned int m_Gamma;
      
      /*! \brief Shutter */
      unsigned int m_Shutter;
      
      /*! \brief Gain */
      unsigned int m_Gain;
      
    };
};

#endif 
