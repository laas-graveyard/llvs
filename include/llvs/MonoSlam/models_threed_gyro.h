/*  MonoSLAM: Real-Time Single Camera SLAM

    MonoSLAM/models_threed_gyro.h
    Copyright (C) 2005 University of Oxford

    Author
    Andrew Davison
    ajd@robots.ox.ac.uk
    Scene home page: http://www.robots.ox.ac.uk/~ajd/Scene/

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

// Model for internal measurement of angular velocity from a 3 axis gyro
// Assumes MonoSLAM state vector
//                 x
//  r              y
//                 z
//  -              -
//                 q0
//  q              qx
//                 qy
//                 qz
//  -      =       -
//                 vx
//  v              vy
//                 vz
//  -              -
//                 omegax
//  omega          omegay
//                 omegaz

#ifndef _monoslam_models_threed_gyro_h_
#define _monoslam_models_threed_gyro_h_

#include "Scene/models_base.h"


class ThreeD_Gyro_Internal_Measurement_Model
: public Internal_Measurement_Model
{
 public:
  ThreeD_Gyro_Internal_Measurement_Model(Motion_Model *motion_model);
  ~ThreeD_Gyro_Internal_Measurement_Model();

  const double SD_meas_filter;             // rads^-1
  const double SD_meas_simulation;         // for simulation

  // Redefined virtual functions
  void func_hv_and_dhv_by_dxv(const VNL::Vector<double> &xv);

  // Measurement noise
  void func_Rv(const VNL::Vector<double> &hv);

  // Innovation calculation
  void func_nuv(const VNL::Vector<double> &hv,
			const VNL::Vector<double> &zv);

  // Noisy measurement for use in simulation
  void func_hv_noisy(const VNL::Vector<double> &xv_true);

  // Test for feasibility of measurement
  bool feasibility_test(const VNL::Vector<double> &xv,
			const VNL::Vector<double> &hv);
};

#endif
