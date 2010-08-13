/** @doc This object implements the abstract basic visual process
    for the vision system.

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
#ifndef _HRP2_VISION_BASIC_PROCESS_H_
#define _HRP2_VISION_BASIC_PROCESS_H_

/*! This object defines the abstract class defining a basic
 * vision process
 * 
 * Copyright (c) 2004 Olivier Stasse, JRL, CNRS/AIST
 *
 *
 */

#include <string>
#include <vector>

#include "llvsConfig.h"

#if (LLVS_HAVE_OPENCV>0)
#include <cv.h>
#endif /* LLVS_HAVE_OPENCV */

#if (LLVS_HAVE_VVV>0)
extern "C" 
{
#include "epbm.h"
}
#endif /* LLVS_HAVE_VVV */

using namespace::std;

/*! This object is used to derive all the vision process
 * at least for the LowLevelVisionServer.
 */
class HRP2VisionBasicProcess
{

 public:

  /*! Constructor */
  HRP2VisionBasicProcess(int Instance=0);

  /*! Destructor */
  virtual ~HRP2VisionBasicProcess();

  /* Stop the process */
  int StopProcess();
  
  /* Start the process */
  virtual int StartProcess();

  /* Returns the current status of the process:
   * 0 : not running 
   * 1 : running 
   */
  int GetStatus();

  /*! Initialize the process. */
  int InitializeTheProcess();

  /*! Realize the process */
  int RealizeTheProcess();
  
  /*! Cleanup the process */
  int CleanUpTheProcess();

  /*! Get the name of the process */
  virtual string GetName();

  /*! Get the number of the instance */
  int GetInstance();

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
  int SetParameter(string aParameter, string aValue);

  /*! Get a parameter and its possible value according to its index */
  int GetParameter(string & aParameter, string & aValue, int anIndex);

  /*! Get the value of a parameter if such parameter exist */
  int GetValueOfParameter(string aParameter, string &aValue);

  /*! Get the parameters and their values */
  int GetParametersAndValues(vector<string> & ListOfParameters, vector<string> & ListOfValues);

#if ((LLVS_HAVE_OPENCV>0) && (LLVS_HAVE_VVV>0))
  #warning "Here .."
  static const int HEADER_IPL=0;
  static const int EXISTING_IPL=1;
  static const int NEW_IPL=2;
  
  /*! Create a set of 3 IPL images 
   *
   */
  IplImage ** ToIPL(unsigned char *Images[3], int Sizes[3][2]);

  /* Record an EPBM file */
  void ToEPBMFile(IplImage *ThreeImages[3], string BaseName);

  /* From a EPBM structure to a IPL structure */
  void FromEPBMToIPL(EPBM anEpbm, IplImage *anImage);

  /* Transfert from EPBM to IPL using 3 modes:
     NEW_IPL: Create one IPL totally different.
     HEADER_IPL: Create one IPL with the header different, and pointing to 
     the same data.
     EXISTING_IPL: Using an existing IPL and changing the header.
  */
  IplImage * FromEPBMToIPL(EPBM &anEPBM, int aMode, IplImage *ExistingIPL);
#endif

 protected:
  /*! Hooks on methods without the p 
    To be redefined and called only by inherited classes */

  /* Processes status */
  virtual int pInitializeTheProcess()=0;

  virtual int pRealizeTheProcess()=0;
  
  virtual int pCleanUpTheProcess()=0;

  /* Methods related to parameters */
  virtual int pGetParameter(string & aParameter, string & aValue, int anIndex)
  { return 0;};

  virtual int pSetParameter(string aParameter, string aValue)
  { return 0;};
  
  virtual int pGetValueOfParameter(string aParameter, string &aValue)
  { return 0;};

  virtual int pGetParametersAndValues(vector<string> & ListOfParameters, vector<string> & ListOfValues)
  { return 0;};

  /* Start and stop the process */
  virtual int pStartProcess()
  {return 0;};

  virtual int pStopProcess()
  {return 0;};

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

  /*! Numero de l'instance. */
  int m_Instance;

  /*! Internal state. */
  unsigned int m_State;

  /*! Four state. */
  enum InternalState
  {
    STATE_LIMBO,
    STATE_INITIALIZE,
    STATE_INITIALIZE_FINISHED,
    STATE_START_PROCESS,
    STATE_START_PROCESS_FINISHED,
    STATE_EXECUTE_PROCESS,
    STATE_EXECUTE_PROCESS_FINISHED,
    STATE_STOP_PROCESS,
    STATE_STOP_PROCESS_FINISHED,
    STATE_CLEANUP,
    STATE_CLEANUP_FINISHED
  };

};

#endif /* _HRP2_VISION_BASIC_PROCESS_H_ */
