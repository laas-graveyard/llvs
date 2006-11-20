/** @doc This object implements a visual process
    to build boundary representation.
    
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
#ifndef _HRP2_BREP_DETECTION_PROCESS_H_
#define _HRP2_BREP_DETECTION_PROCESS_H_

#include <OBE/CORBA.h>
#include "BRep.h"
#include "VisionBasicProcess.h"

extern "C"
{
#include "epbmtobrep.h"
}

class HRP2BRepDetectionProcess : public HRP2VisionBasicProcess
{
 public:
  /* Constructor */
  HRP2BRepDetectionProcess();

  /* Destructor */
  ~HRP2BRepDetectionProcess();
  
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

  /*! Edge Image */
  EPBM m_Edge[3];
  
  /*! BRep Image */
  BREP m_BRep[3];
  
  /* Returns an arry where the images to be processed are specified */
  int GetImagesOnWhichToProcess(int ImagesOnWhichToProcess[3]);

  /* Build the sequence of Corba B Representations starting from the VVV structure */
  int BuildBrepVar(CBREPSeq_var &aBrepVar);

  /*! Create a CORBA compatible version of the VVV brep */
  int FromBREPToCorbaBREP(CBREP &aCBREP_var, int anImage);

  /*! */
  int FromBREPPointToCorbaBREPPoint(BREP_Point *abrepp, CBREP_Point &aCBREP_Point_var);

  /*! */
  int FromBREPCnetToCorbaBREPCnet(BREP_Cnet * aBRep_Cnet, CBREP_Cnet &aCBREP_Cnet_var);
    
  /*! */
  int FromBREPRegionToCorbaBREPRegion(BREP_Region * aBRep_Region, CBREP_Region &aCBREP_Region_var);

  /*! */
  int FromBREPBoundaryToCorbaBREPBoundary(BREP_Bound * aBRep_Boundary, CBREP_Bound &aCBREP_Bound_var);

  /*! */
  int FromBREPSegmentToCorbaBREPSegment(BREP_Segment * aBRep_Segment, CBREP_Segment &aBREP_Segment_var);

  virtual int SetParameter(string aParameter, string aValue);
 protected:
  
  /*! Input Image */
  EPBM m_InputImage[3];

  /*! Images on which perform the edge detection  3 at max.*/
  int m_ImagesOnWhichToProcess[3];

  EpbmToBrepOptions m_opt;
};
#endif /* _HRP2_BREP_DETECTION_PROCESS_H_ */
