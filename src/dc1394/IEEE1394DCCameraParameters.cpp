#include <iostream>
#include <fstream>
#include <dc1394/IEEE1394DCCameraParameters.h>

using namespace std;
using namespace llvs;

IEEE1394DCCameraParameters::IEEE1394DCCameraParameters()
{
}


IEEE1394DCCameraParameters::~IEEE1394DCCameraParameters()
{
}

const unsigned int & IEEE1394DCCameraParameters::GetBoardNumber() const
{
  return m_BoardNumber;
}

void IEEE1394DCCameraParameters::SetBoardNumber(const unsigned int &aBoardNb)
{
  m_BoardNumber = aBoardNb;
}


const unsigned int & IEEE1394DCCameraParameters::GetCameraNumberInUserSemantic() const
{
  return m_CameraNumberInUserSemantic;
}

void IEEE1394DCCameraParameters::SetCameraNumberInUserSemantic(const unsigned int & aCamNbInUserSemantic) 
{
  m_CameraNumberInUserSemantic = aCamNbInUserSemantic;
}

const string & IEEE1394DCCameraParameters::GetGUID() const
{
  return m_GUID;
}

void IEEE1394DCCameraParameters::SetGUID(const string &aGUID) 
{
  m_GUID = aGUID;
}

const string & IEEE1394DCCameraParameters::GetFormat() const
{
  return m_Format;
}

void IEEE1394DCCameraParameters::SetFormat(const string &aFormat) 
{
  m_Format = aFormat;
}


const unsigned int & IEEE1394DCCameraParameters::GetBrightness() const
{
  return m_Brightness;
}

void IEEE1394DCCameraParameters::SetBrightness(const unsigned int &aBrightness)
{
  m_Brightness = aBrightness;
}

void IEEE1394DCCameraParameters::GetWhiteBalance(unsigned int WhiteBalance[2]) const
{
  WhiteBalance[0] = m_WhiteBalance[0];
  WhiteBalance[1] = m_WhiteBalance[1];
}

void IEEE1394DCCameraParameters::SetWhiteBalance(unsigned int WhiteBalance[2])
{
  m_WhiteBalance[0] = WhiteBalance[0];
  m_WhiteBalance[1] = WhiteBalance[1];
}

const unsigned int & IEEE1394DCCameraParameters::GetExposure() const
{
  return m_Exposure;
}

void IEEE1394DCCameraParameters::SetExposure(const unsigned int &aExposure)
{
  m_Exposure = aExposure;
}

const unsigned int & IEEE1394DCCameraParameters::GetGamma() const
{
  return m_Gamma;
}

void IEEE1394DCCameraParameters::SetGamma(const unsigned int &aGamma)
{
  m_Gamma = aGamma;
}

const unsigned int & IEEE1394DCCameraParameters::GetShutter() const
{
  return m_Shutter;
}

void IEEE1394DCCameraParameters::SetShutter(const unsigned int &aShutter)
{
  m_Shutter = aShutter;
}

const unsigned int & IEEE1394DCCameraParameters::GetGain() const
{
  return m_Gain;
}

void IEEE1394DCCameraParameters::SetGain(const unsigned int &aGain)
{
  m_Gain = aGain;
}

const string & IEEE1394DCCameraParameters::GetFPS() const
{
  return m_FPS;
}

void IEEE1394DCCameraParameters::SetFPS(const string &aFPS)
{
  m_FPS = aFPS;
}






