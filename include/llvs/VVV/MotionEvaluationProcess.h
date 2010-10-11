/** @doc This object implements a visual process
    performing visual motion evaluation.

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
#ifndef _HRP2_MOTION_EVALUATION_PROCESS_H_
#define _HRP2_MOTION_EVALUATION_PROCESS_H_

/*! This object computes a motion evaluation
 * based on two kinds of information:
 *  - A set of Harris operators matched on two different views.
 *    This set of Harris operators is given through their position
 *    in the different considered images.
 *  - Two disparity maps given as images for which the
 *    corresponding 3D points is known.
 * Obviously the image used as a reference is both information should be
 * the same.
 *
 * Copyright (c) 2004 Olivier Stasse, JRL, CNRS/AIST
 *
 *
 */

#include <string>
#include <range.h>
#include <VisionBasicProcess.h>

#include <MMXHarris.h>
#include <set>
#include <vector>

using namespace::std;

/*!
 * Data structure to store 3d points for matching.
 */
typedef struct
{
  float Coord[3];
  float FeatureValue;
}
HRP23DPoint_t;

/*! This object is used to derive all the vision process
 * at least for the LowLevelVisionServer.
 */
class HRP2MotionEvaluationProcess: public HRP2VisionBasicProcess
{

 public:

  /*! Constructor */
  HRP2MotionEvaluationProcess();

  /*! Destructor */
  virtual ~HRP2MotionEvaluationProcess();

  /*! Initialize the process. */
  virtual int InitializeTheProcess();

  /*! Realize the process. */
  virtual int RealizeTheProcess();

  /*! Cleanup the process. */
  virtual int CleanUpTheProcess();

  /*! Update range data. */
  int UpdateRangeData(RANGE &arng);

  /*! Update Harris matching points.*/
  int UpdateMatchingPointsWithHarris(multiset <MatchingPoint_t *, MatchingPoint_lt> aSetOfMatchingPoints);

  /*! Duplicate range without reallocation. */
  int DuplicateRangeWithoutReallocation(RANGE &orig, RANGE &copy);

  /*! Create two clouds of 3 points using the information given by the
    Harris detector. */
  int WithMatched3DHarrisPoints();

 protected:

  /*! Range data */
  RANGE m_ranges[2];

  /*! Which is the last ? */
  int m_LastRangeIndex;

  /*! Set of matching points between two consecutive images */
  multiset <MatchingPoint_t *, MatchingPoint_lt> m_SetOfHarrisMatchingPoints;

  /*! Vectors for the two clouds of 3D points. */
  vector <HRP23DPoint_t> m_Clouds[2];

  /*! Mean values for the two clouds.
   * This gives an approximation for the translation. */
  float m_Translation[3];
};

#endif /* _HRP2_MotionEvaluation_PROCESS_H_ */
