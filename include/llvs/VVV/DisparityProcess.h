/** @doc This object implements a visual process
    performing disparity computation.

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
#ifndef _HRP2_DISPARITY_PROCESS_H_
#define _HRP2_DISPARITY_PROCESS_H_

#include "VisionBasicProcess.h"
#include "main_correlation.h"
#include "MotionEvaluationProcess.h"
#include <cv.h>
#include <interval.h>
#include <isoluminance.h>
#include <TU/Vector++.h>

/*!
 * Data structure to store a 3D point.
 */
typedef struct {

  /*! Point's 3D position */
  double xyz[3];

  /*! Color point */
  unsigned char aColor;

} DataPoint_t;

/*!
 * Data structure to store 3D point
 * plus its bounding box.
 * Used to store error model parameters.
 */
typedef struct {

  float Center[3];
  float Radius[3];

} DataIntervalPoint_t;

/*!
 * This class computes a disparity map
 * and the associate cloud of 3D points
 * using the camera parameters embedded in the EPBM image files.
 * Optionnally it also embeds an error model computation.
 *
 * 2004 Olivier Stasse, JRL CNRS/AIST
 */
class HRP2DisparityProcess : public HRP2VisionBasicProcess
{
 public:

  /*! Constantes for dumping 3D Points :
   * DUMP_MODE_3D_POINTS_OFF          : Turn it off
   * DUMP_MODE_3D_POINTS_MATCHING     : Dump the points which are matching
   *                                    between two frames
   * DUMP_MODE_3D_POINTS_PLY          : Record the 3D points in the ply format.
   * DUMP_MODE_3D_RANGE_DATA          : Record the range data.
   * DUMP_MODE_3D_RANGE_DATA_AND_PLY  : Perform the two previous choices at the same time
   */
  static const int DUMP_MODE_3D_POINTS_OFF =0;
  static const int DUMP_MODE_3D_POINTS_MATCHING =1;
  static const int DUMP_MODE_3D_POINTS_PLY=2;
  static const int DUMP_MODE_3D_RANGE_DATA=3;
  static const int DUMP_MODE_3D_RANGE_DATA_AND_PLY=4;
  static const int DUMP_MODE_3D_POINTS_INTERVALS=5;
  static const int DUMP_MODE_3D_POINTS_INTERVALS_MATCHING=6;
  /*! Constantes for error models:
   */
  static const int ERROR_MODEL_ONLINE_INTERVALS = 0; /* Use on-line interval analysis for each point */
  static const int ERROR_MODEL_LUT_INTERVALS = 1;    /* Use pre-computed interval analysis look up table */
  static const int ERROR_MODEL_HIRSCHMULLER = 2;     /* Use Hirschmuller error model */
  static const int ERROR_MODEL_HZ = 3;               /* Use Hartley Zimmermann error model */

  /*! Constructor
   * If BooleanForErrorModel equals 1 then an error model
   * is used for each 3D points. This error model used the interval
   * analysis to find a bouding box where the 3D box is guarranted to be.
   */
  HRP2DisparityProcess(unsigned char BooleanForErrorModel=0);

  /*! Destructor */
  virtual ~HRP2DisparityProcess();

  /*! Initialize the process */
  virtual int InitializeTheProcess(int CalibrationWidth, int CalibrationHeight);

  /*! Compute disparity based on the given image */
  virtual int RealizeTheProcess();

  /*! Cleanup the process */
  virtual int CleanUpTheProcess();

  /*! Get bounding boxes */
  float * GetBoundingBoxes();

  /*! Initialize the error model structure */
  void InitializeErrorModel();

  /*! Set the input images
   * This method is needed to set up the reference to the input images.
   * They should be specified only once. The first image is the left image.
   * The second image is the second image. It is assume that those images
   * are corrected.
   */
  int SetInputImages(EPBM lInputImages[2]);

  /*! Find matching in the images coordinates
   * After calling this method, m_NbOfPoitnsWithInformation gives the number of pairs found.
   * m_PointsInImageLeft[2*i] and m_PointsInImageLeft[2*i+1] contains
   * the coordinates x and y respectivly which match the point
   * m_PointsInImageRight[2*i] and m_PointsInImageRight[2*i+1]
   * in the Right image.
   */
  int FindMatchingInImagesCoordinates();

  /*! Use interval analysis to compute the bouding box
   * If MethodToCompute equals 1 then the GAUSS_SIEDEL contractor
   * is used to compute the bouding box.
   * If MethodToCompute equals 2, a lookup table will be used.
   */
  int ComputeIntervalError(int MethodToCompute);

  /*! Use gauge based analysis to compute the covariance matrix
   * of the 3D point.
   */
  int ComputeHZError(void);

  /*! Compute a covariance matrix based on the reconstruction error */
  TU::Matrix<double> BuildCovarianceMatrixHZ(TU::Vector<double> q1,
					     TU::Vector<double> q2,
					     TU::Matrix<double> P1,
					     TU::Matrix<double> P2,
					     TU::Matrix<double> CovX,
					     TU::Matrix<double> Theta);
  /*! Read the lookup table for the camera model
   * in the file LUT.dat .
   */
  int ReadLookUpTable(void);

  /*! Set the size of the image related to the error model coded in the lookup table */
  int SetImageSizeInLUT(int width, int height);

  /*! Get the size of the image related to the error model coded in the look up table */
  int GetImageSizeInLUT(int & width, int & height);

  /*! Set the calibration size */
  int SetCalibrationSize(int width, int height);

  /*! Get the calibration size */
  int GetCalibrationSize(int &width, int &height);

  /*! Update of a motion evaluation process */
  int UpdateAMotionEvaluationProcess(HRP2MotionEvaluationProcess *aMEP);

  /*! Set Mode to dump matching 3D points,
   * cf DUMP_MODE_* constantes for the possible choice.
   */
  int SetModeDumpMatching3DPoints(int aMode);

  /*! Returns the mode to dump matching 3D points,
   * see below for the semantic.
   */
  int GetModeDumpMatching3DPoints();

  /*! Modify the subsampling for the interval analysis */
  void SetSubsampleIA(int aValue);

  /*! Get the subsampling for the interval analysis */
  int GetSubsampleIA();

  /*! Get the subsampling for the HZ error */
  int GetSubsampleHZ();

  /*! Get range data */
  RANGE GetRangeData();


  /*! Write the range data in Ply format */
  void WriteToPlyFormat();

  /*! Modify the parameter of the disparity process */
  virtual int SetParameter(string aParameter, string aValue);

  /*! Set the error model
   * @param anErrorMode: One the constantes defined for the error model.
   * @return : 0, if the specified error model is correct, -1 otherwise.
   */
  int SetErrorModel(int anErrorModel);

  /*! Get the error model
   * @return: one the error model constant defined below.
   */
  int GetErrorModel(void);

 protected:

  /*! Input Image */
  EPBM m_InputImage[2];

  /*! Disparity image */
  EPBM m_Depbm;

  /*! Range data */
  RANGE m_rng;

  /*! Parameters for the computation */
  PARA m_para;

  /*! Camera parameters */
  SCM_PARAMETER m_sp;

  /*! Initialization parameters for the correlation */
  InitParaForCorr_t m_IPFC;

  /*! Store the maching in the initial image frame of a set of points */
  int * m_PointsInImageLeft, * m_PointsInImageRight;
  unsigned char * m_ColorInImageLeft;

  /*! Store the bounding box given by the interval analysis
     (center x, center y, center z, radius x, radius y, radius z) */
  float * m_BoundingBox;

  /*! Number of points containing some information. */
  int m_NbOfPointsWithInformation;

  /*! Projective models of the cameras store using the opencv data structure */
  OCVMatrix m_CamL, m_CamR;

  /*! Projective models of the cameras stored using TU::Matrix data structure */
  TU::Matrix<double> * m_PLeft, * m_PRight;

  /*! Subsampling of the 3D points for Hartley Zimmerman error model */
  int m_SubsampleHZ;

  /*! Model of the camera for a very specific size. */
  DataIntervalPoint_t ** m_DIPs;

  /*! Boolean specifying if an error model should be used or not. */
  unsigned char m_UseErrorModel;

  /*! Integer specifying which error model to use. */
  int m_ErrorModel;

  /*! Size of the image in the lookup table 0: Width 1: Height (80x60 by default). */
  int m_ImageSizeInLUT[2];

  /*! Boolean on dumping the 3D points */
  int m_ModeToDumpMatching3DPoints;

  /*! Boolean on using textureless masking */
  bool m_TexturelessMasking;

  /*! Textureless image */
  EPBM m_TexturelessImage[3];

  /*! Initialization parameters for the textureless masking */
  struct InitParForIsoLuminance_s m_InitParForIsoL;

  /*! Subsampling for the intervals */
  int m_SubsampleIA;

  /*! Rigid motion to reach the world reference frame */
  double m_w2c[2][3][4];

  /*! Gray tempory image in case of color input images */
  EPBM m_GrayInputImage[2];
};
#endif /* _HRP2_DISPARITY_PROCESS_H_ */
