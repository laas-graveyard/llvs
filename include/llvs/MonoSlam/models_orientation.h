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

#ifndef _monoslam_models_orientation_h_
#define _monoslam_models_orientation_h_

#include "Scene/models_base.h"


class Orientation_Internal_Measurement_Model
: public Internal_Measurement_Model
{
 public:
  Orientation_Internal_Measurement_Model(Motion_Model *motion_model);
  ~Orientation_Internal_Measurement_Model();

  const double SD_meas_filter;             // ms^-1
  const double SD_meas_simulation;         // for simulation

  const double SD_ang_meas_filter;             // rads^-1
  const double SD_ang_meas_simulation;         // for simulation

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

  // Calculate commonly used Jacobian part dq(omega * delta_t) by domega
  void dqomegadt_by_domega(const VW::Vector3D &omega,
			   const double delta_t,
			   VNL::MatrixFixed<4,3,double> &dqomegadt_by_domega);

  // Ancillary functions: calculate parts of Jacobian dq_by_domega
  // which are repeatable due to symmetry.
  double dq0_by_domegaA(const double omegaA, const double omega,
			const double delta_t);
  double dqA_by_domegaA(const double omegaA, const double omega,
			const double delta_t);
  double dqA_by_domegaB(const double omegaA, const double omegaB,
			const double omega,
			const double delta_t);

  void extract_r_q_v_omega( const VNL::Vector<double> &xv,
			    VW::Vector3D &r,
			    VW::Quaternion &q,
			    VW::Vector3D &v,
			    VW::Vector3D &omega);


};


#endif
