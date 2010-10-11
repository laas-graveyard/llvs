/** @doc This object implements a visual process
    to find features inside the environment and get their
    3D location.

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
#ifndef _HRP2_FIND_FEATURES_IN_IMAGE_H_
#define _HRP2_FIND_FEATURES_IN_IMAGE_H_

#include <string>
#include <vector>

extern "C"
{
#include "epbm.h"
}

using namespace::std;

#include <VisionBasicProcess.h>
#include <FindFeaturesFromWideImage.h>
using namespace Geometry;
/*! This object find features inside the wide lens camera
 * and proposes a set of different angles to perform
 * head-motion. Once the motion are realized it is possible
 * to have the 3D position of each feature.
 */
class HRP2FindFeaturesInImage : public HRP2VisionBasicProcess
{

 public:

  /*! Constructor */
  HRP2FindFeaturesInImage();

  /*! Destructor */
  virtual ~HRP2FindFeaturesInImage();

  /* Stop the process */
  virtual int StopProcess();

  /* Start the process */
  virtual int StartProcess();

  /* Returns the current status of the process:
   * 0 : not running
   * 1 : running
   */
  int GetStatus();

  /*! Initialize the process. */
  virtual int InitializeTheProcess(string PathForFirstCalibrationSet,
				   string PathForSndCalibrationSet="",
				   string ConvexPolytopePath="");

  /*! Realize the process */
  virtual int RealizeTheProcess();

  /*! Cleanup the process */
  virtual int CleanUpTheProcess();

  /*! Get the name of the process */
  virtual string GetName();

  /*! Get the level of verbosity */
  unsigned char GetLevelOfVerbosity();

  /*! Set the level of verbosity
   * Higher is the value, higher the process will be verbose.
   * By default its value is 0
   */
  void SetLevelOfVerbosity(unsigned char LevelOfVerbosity);

  /*! Set a parameter,
   * A parameter can be or cannot be associated with a value,
   * thus an empty string for Value is correct.
   * If the parameter already exist is value is overwritten.
   * If this is valid the index parameter >=0 is returned,
   * -1 otherwise.
   */
  virtual int SetParameter(string aParameter, string aValue);

  /*! Get a parameter and its possible value according to its index */
  virtual int GetParameter(string & aParameter, string & aValue, int anIndex);

  /*! Get the value of a parameter if such parameter exist */
  virtual int GetValueOfParameter(string aParameter, string &aValue);

  /*! Get the parameters and their values */
  virtual int GetParametersAndValues(vector<string> & ListOfParameters, vector<string> & ListOfValues);

  /*! Set the input images. */
  int SetInputImages(EPBM *aInputImage);

  /*! Returns the pointer towards the FindFeaturesInWideLens
    object */
  FindFeaturesFromWideImage *GetFindFeaturesFromWideImage();

 protected:

  /*! Name of the process */
  string m_ProcessName;

  /*! Verbosity level */
  unsigned char m_Verbosity;

  /*! List of parameters store as a list of strings */
  vector<string> m_VectorOfParameters;

  /*! List of values related to the parameters */
  vector<string> m_VectorOfValuesForParameters;

  /*! Size of parameters */
  int m_ParametersSize;

  /*! Boolean on computing or not */
  unsigned char m_Computing ;

  /*! Input images */
  EPBM m_InputImage[4];

  /* ! Image to be stored */
  VW::ImageMono<unsigned char> *m_grabbed_image[3];

  /*! State of the visual process. */
  int m_State;

  /*! Object to deal with most of the work.
   * The visual process object is just doing the glue
   * with the other parts of the system.
   */
  FindFeaturesFromWideImage *m_FFFWI;

  /*! Feature to identify and to get in 3D */
  int m_FeatureID;
};

#endif /* _HRP2_FIND_FEATURES_IN_IMAGE_H_ */
