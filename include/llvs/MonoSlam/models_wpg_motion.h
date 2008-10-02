/*  SceneLib: software for real-time SLAM

    models_wpg_motion.h
    Copyright (C) 2005 University of Oxford

    Author
    Andrew Davison
    ajd@doc.ic.ac.uk

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

// HRP-2 Walking Pattern motion model in 3D
// Assumes information on incremental motion comes in from pattern generator
// State vector: 13 elements
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

// Control vector has 7 elements
// TW = translation in world frame from old to new robot positions
// qRoldRnew = rotation between old and new robot frames

// rnew     =   r + TW
// qnew     =   q x qRoldRnew


#ifndef _monoslam_models_wpg_h_
#define _monoslam_models_wpg_h_

#include "Scene/models_base.h"


class WPG_ThreeD_Motion_Model : public ThreeD_Motion_Model
{
 public:
  WPG_ThreeD_Motion_Model();
  ~WPG_ThreeD_Motion_Model();

  // Constants
  // Standard deviation to be added in linear directions
  double SD_r_component_filter;
  // Standard deviation to be added in linear directions for simulation
  double SD_r_component;
  // Standard deviation to be added to quaternion components
  double SD_q_component_filter;
  // Standard deviation to be added to quaternion components for simulation 
  double SD_q_component;

  // Redefined virtual functions
  void func_fv_and_dfv_by_dxv(const VNL::Vector<double> &xv, 
			      const VNL::Vector<double> &u, 
			      const double delta_t);

  void func_Q(const VNL::Vector<double> &xv, const VNL::Vector<double> &u, 
	      const double delta_t);

  void func_xp(const VNL::Vector<double> &xv);

  void func_dxp_by_dxv(const VNL::Vector<double> &xv);

  void func_fv_noisy(const VNL::Vector<double> &xv_true, 
		     const VNL::Vector<double> &u_true, const double delta_t);

  void func_xvredef_and_dxvredef_by_dxv_and_dxvredef_by_dxpdef  
    (const VNL::Vector<double> &xv, const VNL::Vector<double> &xpdef);

  void func_xvnorm_and_dxvnorm_by_dxv(const VNL::Vector<double> &xv);


  // Easy access to state blocks: fill matrices r, q, v, omega with
  // values based on state xv
  void extract_r_q_v_omega(const VNL::Vector<double> &xv, 
			   VW::Vector3D &r, 
			   VW::Quaternion &q,
			   VW::Vector3D &v, 
			   VW::Vector3D &omega);
  // The opposite: put r, q, v, omega back into vector xv
  void compose_xv(const VW::Vector3D &r, 
		  const VW::Quaternion &q,
		  const VW::Vector3D &v, 
		  const VW::Vector3D &omega, 
		  VNL::Vector<double> &xvnew);

 protected:
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
};

#endif
