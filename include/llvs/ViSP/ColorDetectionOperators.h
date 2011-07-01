/** 
 * @file   MorphologicalOperators.h
 * @author Olivier Stasse
 * @date   27/05/2005
 * This object performs color detection using histogram.
 * 
 *
 *              (c) JRL, CNRS/AIST
 *
 *   This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2 of the License, or (at your option) any later version.
 *
 *   This library is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public
 *   License along with this library; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */


#ifndef _ColorDetectionOperators_h_
#define _ColorDetectionOperators_h_

/*! Visp headers */
#include <visp/vpConfig.h>
#include <visp/vpImageTools.h>

class ColorDetectionOperators
{
 public:
  // Create the class of morphological operators.
  ColorDetectionOperators();

  // Delete the class of morphological operators.
  ~ColorDetectionOperators();
      
  /*! Initialization of the intermediate structure 
    to call this operator in a flow like computing.*/
  int InitializeIntermediateStructure(vpImage<vpRGBa> * in);


  /*! This method read an image from the file FileName, and transform it into the RGB
    color space.  An histogram is then created for each of the 3 channels RGB.
    The histograms are stored in the array Histogram. */
  int ReadFromFileAndCreateThePixelList(char *FileName);
      
  /*! This method segment an image provided by */ 
  int FilterOnHistogram(unsigned char * in,
			vpImage<unsigned char> * out);


  /*! Compute CoG */
  int ComputeCoG(vpImage<unsigned char> *in,
		 double &x, double &y);

 private:

  /*! Histogram to search for.
   * It is supposed to handle 3 channels of 256 possibles values.
   */
  double m_Histogram[3][256];

  double m_HistogramMono[256];

  /*! Intermediate result for the probability */
  float *m_ImgRes;
  
  /*! Intermediate image for filtering. */
  vpImage<unsigned char> * m_ImageIntermediate;
  
};

#endif

