/** @doc This object implements a visual process
    performing optical flow.

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
#ifndef _HRP2_OPTICAL_FLOW_PROCESS_H_
#define _HRP2_OPTICAL_FLOW_PROCESS_H_

#include <set>
#include <VisionBasicProcess.h>
#include <MMXHarris.h>
#include <libmmx.h>
#include <MotionEvaluationProcess.h>
#include <epbm.h>
#include <cv.h>
using namespace std;

/*! This vision process implements the Optical Flow method
 * of Lucas and Kanade modified by Simoncelli.
 *
 * Copyright 2004 (c), Olivier Stasse, JRL, CNRS/AIST
 *
 */


class HRP2OpticalFlowProcess : public HRP2VisionBasicProcess
{
 public:

  static const int HARRIS_OPENCV=0;
  static const int HARRIS_MMX=1;
  static const int HARRIS_MMX_AND_OPENCV=2;

  static const int FLOW_STRUCTURE_OPF=1;
  static const int FLOW_STRUCTURE_HARRIS=0;
  /*! Constructor
   * \param Width: width of the image used as input.
   * \param Height: height of the image used as input.
   */
  HRP2OpticalFlowProcess(int Width, int Height);

  /*! Destructor */
  virtual ~HRP2OpticalFlowProcess();

  /*! Initialize the process. */
  virtual int InitializeTheProcess();

  /*! Realize the process. */
  virtual int RealizeTheProcess();

  /*! Cleanup the process */
  virtual int CleanUpTheProcess();

 /*! Computes Optical Flow and Harris detector */
 int ComputesOpticalFlowAndHarris();

 /*! Perform the Optical Flow structure's memory allocation */
 int AllocateOpticalFlowStructures();

 /*! Perform the Optifcal Flow structure's memory release */
 int FreeOpticalFlowStructures();

 /*! Set to zero the references to the optical flow structures. */
 int SetToZeroOpticalFlowStructures();

 /*! Computes the Harris Detector Point Matching */
 int ComputesHarrisMatchingPoints();

 /*! Set the input images */
 int SetInputImages(EPBM InputImages[3]);

 /*! Say to the process to save the last used images */
 int SaveLastImages(void);

 /*! Filter Harris Points in order to limit their numbers and their
   location */
 int FilteringHarrisPoint(MMXFlow *aFlow, ImageOfTiles_t *anImageOfTiles);

 /*! Update of a Motion Evaluation Process  */
 int UpdateAMotionEvaluationProcess(HRP2MotionEvaluationProcess *aMEP);

 /*! Create the matching between two consequent views using Harris
   detectors */
 int CreateMatchingBetween2ConsequentViews(ImageOfTiles_t *anIOTAtT,
					   ImageOfTiles_t *anIOTAtTpdT,
					   MMXFlow *aFlow);

 /*! Perform Harris detector computation using the OpenCV library */
 int HarrisWithOpenCV();

 /*! Set the dumping level
  */
 void SetDumpingLevel(int aDumpingLevel);

 /*! Get the dumping level
  */
 int GetDumpingLevel();

 /*! Set the method to compute Harris detector
  */
 int SetHarrisDetectorMethod(int aMethod);

 /*!Get the method to compute the Harris detector
  */
 int GetHarrisDetectorMethod();

 /*! Get the optical flow
  *  returns a pointer in the structure aFlow.
  * This will give the :
  * * The temporal and spatial derivative.
  * * A filtered image.
  * * The harris detector.
  * Only the left and the right images can be obtained.
  */
 int GetFlowStructure(MMXFlow **aFlow, int aCameraID, int HarrisOrOPF);


 /*! This method allows to set the dumping level.
  * @param aMode : the dumping level.
  */
 int SetModeDump(int aMode);

 /*! This method allows to get the dumping level.
  */
 int GetModeDump();

 /*! This method allows to set a parameter.
  */
 int SetParameter(string aParameter, string aValue);

 protected:

 /*! MMX Matrix for computing Optical Flow and filtered image. */

 /*! The right image */
 /*! The original image */
 MMXMatrix *m_RIGHT_OPF_m;

 /* Flow for the right image */
 MMXFlow *m_RIGHT_Flow;

 /* Flow structure to compute Harris detectors */
 MMXFlow *m_RIGHT_Harris_Flow;

 /*! The left image */
 /*! The original image */
 MMXMatrix *m_LEFT_OPF_m;

 /* Flow for the left image */
 MMXFlow *m_LEFT_Flow;

 /* Flow structure to compute Harris detectors */
 MMXFlow *m_LEFT_Harris_Flow;

 /*! Tau parameter for the optical flow */
 float m_tau;

 /*! K parameter for the Harris corner detector. */
 float m_Harris_k;

 /*! Ratio between the size of the harris image and the original image.
  * This parameter is used to compute the harris image size. */
 int m_SizeOfTheImage;

 /*! Number of possible points per tile. */
 int m_NbOfPossiblePtPerTile;

 /*! Separation space: number of pixels between two harris points */
 int m_SeparationSpace;

 /*! Total number of points. */
 int m_TotalNbOfPoints;


 /*! Specifies the camera on which the OPF computation should be done. */
 unsigned char m_ProcessOPFLeftCamera, m_ProcessOPFRightCamera;

 /*! Specifies the camera on which the Harris dectection should be done. */
 unsigned char m_ProcessHarrisLeftCamera, m_ProcessHarrisRightCamera;

 /*! Array of Matching Points for the Harris detector */
 MatchingPoint_t * m_HarrisMatchingPoint;

 /*! Maximum number of Matching Points for the Harris detector. */
 int m_MaxNbOfHarrisMatchingPoint;

 /*! Threshold for filtering Harris detector */
 float m_HarrisThreshold;

 /*! Size of the image */
 int m_Width, m_Height;

 /*! Set the input images */
 EPBM m_InputImages[3];

 /*! Remember the previous filtered images. */
 MMXMatrix *m_LEFT_PreviousImage;
 MMXMatrix *m_LEFT_Harris_PreviousImage;

 /*! Tiled image of Harris Points */
 ImageOfTiles_t * m_ImageOfTiles[2];

 /*! Previous Tiled image of Harris Points */
 ImageOfTiles_t * m_PreviousImageOfTiles;

 /*! Matching realization. */
 multiset <MatchingPoint_t *, MatchingPoint_lt> m_SetOfMatchingPoint;

 /*! Buffer for computing harris points using OpenCV */

 /*! The eigenvalues for images */
 IplImage * m_eigImage;

 /*! Temporary image for filtering */
 IplImage * m_tempImage;

 /*! Input Image in the Ipl format */
 IplImage * m_IplInputImage;

 /*! Level of dumping for intermediates images */
 int m_DumpingLevel;

 /*! Method to use for computing Harris detector
  */
 int m_HarrisMethod;
};

#endif /* _HRP2_OPTICAL_FLOW_PROCESS_H_ */
