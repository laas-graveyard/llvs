/*  SceneLib: software for real-time SLAM

    models_wpg_motion.cpp
    Copyright (C) 2005 University of Oxford

    Author
    Andrew Davison
    ajd@doc.ic.ac.uk

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include <iostream>
#include <VNL/sample.h>
#include "models_wpg_motion.h"


// Debug macros
#include <llvs/tools/Debug.h>

using namespace std;

// Constructor
WPG_ThreeD_Motion_Model::WPG_ThreeD_Motion_Model()
  : ThreeD_Motion_Model(13, 7, "WPG_THREED"),
    SD_r_component_filter(0.09), // m
   SD_r_component(0.09), // for simulation
   SD_q_component_filter(0.0001),  // dimensionless
    SD_q_component(0.0001) // for simulation
{
  ODEBUG3("Went there");
}

WPG_ThreeD_Motion_Model::~WPG_ThreeD_Motion_Model()
{}

void WPG_ThreeD_Motion_Model::func_fv_and_dfv_by_dxv(
                                     const VNL::Vector<double> &xv,
				     const VNL::Vector<double> &u,
				     const double delta_t)
{
  VW::Vector3D rold, vold, omegaold,
    rnew, vnew, omeganew;
  VW::Quaternion qold, qnew;


  ODEBUG3("wpg1");

  // Separate things out to make it clearer
  extract_r_q_v_omega(xv, rold, qold, vold, omegaold);

  ODEBUG3("wpg2");

  // Components of control vector
  VW::Vector3D TW (u.Extract(3, 0));

  ODEBUG3("wpg3");

  VW::Quaternion qRoldRnew;
  ODEBUG3("wpg4");

  qRoldRnew.SetRXYZ(u.Extract(4, 3));

  ODEBUG3("wpg5");

  // Here we go with the super simple prediction
  rnew = rold + TW;

  ODEBUG3("wpg6");

  qnew = qold * qRoldRnew;

  ODEBUG3("wpg7");

  vnew = vold;

  ODEBUG3("wpg8");

  omeganew = omegaold;

  ODEBUG3("wpg9");

  // Put it all together
  compose_xv(rnew, qnew, vnew, omeganew, fvRES);

  // ODEBUG3("rold qold vold omegaold" << rold << qold
  //      << vold << omegaold);
  // ODEBUG3( "rnew qnew vnew omeganew" << rnew << qnew
  //      << vnew << omeganew);

  // Now on to the Jacobian...
  // Identity is a good place to start since overall structure is like this
  // I       0             0             0
  // 0       dqnew_by_dq   0             0
  // 0       0             I             0
  // 0       0             0             I
  ODEBUG3("wpg10");

  dfv_by_dxvRES.SetIdentity();

  // Fill in dqnew_by_dq
  // qnew = qold x qRoldRnew  ( = q3 = q2 x q1 in Scene/newbits.cc language)
  ODEBUG3("wpg11");

  VNL::MatrixFixed<4,4,double> Temp44A = dq3_by_dq2(qRoldRnew);
  ODEBUG3("wpg12");

  dfv_by_dxvRES.Update(Temp44A, 3, 3);

  // ODEBUG3( "dfv_by_dxvRES" << dfv_by_dxvRES );
}

void WPG_ThreeD_Motion_Model::func_Q(const VNL::Vector<double> &xv,
					 const VNL::Vector<double> &,
					 const double delta_t)
{
  ODEBUG3("func_Q begin ");
  const double Z_REDUCTION_FACTOR = 3;

  // Fill Q with very simple constant value for now
  double r_component_noise_variance =
    SD_r_component_filter * SD_r_component_filter;
  double q_component_noise_variance =
    SD_q_component_filter * SD_q_component_filter;

  QxRES.Fill(0.0);
  QxRES.Put(0, 0, r_component_noise_variance);
  QxRES.Put(1, 1, r_component_noise_variance);
  QxRES.Put(2, 2, r_component_noise_variance / Z_REDUCTION_FACTOR);
  QxRES.Put(3, 3, q_component_noise_variance);
  QxRES.Put(4, 4, q_component_noise_variance);
  QxRES.Put(5, 5, q_component_noise_variance);
  QxRES.Put(6, 6, q_component_noise_variance);

  ODEBUG3("func_Q end ");
  //  ODEBUG3( "QxRES" << QxRES );
}

//
// FUNC XP
/** Extract the position and orientation from the state vector. (This is the
first seven elements in this case.)
**/
void WPG_ThreeD_Motion_Model::func_xp(const VNL::Vector<double> &xv)
{
  ODEBUG3("func_xp begin ");
  xpRES = xv.Extract(7, 0);
  ODEBUG3("func_xp end ");
}

//
// FUNC DXP BY DXV
/** Calculate Jacobian for func_xp. (This is just the identity in this case.)
**/
void WPG_ThreeD_Motion_Model::
func_dxp_by_dxv(const VNL::Vector<double> &)
{
  ODEBUG3("func_dxp_by_dxv begin ");
  dxp_by_dxvRES.Fill(0.0);

  dxp_by_dxvRES.Put(0, 0, 1.0);
  dxp_by_dxvRES.Put(1, 1, 1.0);
  dxp_by_dxvRES.Put(2, 2, 1.0);
  dxp_by_dxvRES.Put(3, 3, 1.0);
  dxp_by_dxvRES.Put(4, 4, 1.0);
  dxp_by_dxvRES.Put(5, 5, 1.0);
  dxp_by_dxvRES.Put(6, 6, 1.0);
  ODEBUG3("func_dxp_by_dxv end ");
}

// Noisy process equation for simulation
// Simply perturb xv with Gaussian noise and send it through func_fv
void WPG_ThreeD_Motion_Model::
func_fv_noisy(const VNL::Vector<double> &xv_true,
	      const VNL::Vector<double> &u_true, const double delta_t)
{
  ODEBUG3("Hello from func_fv_noisy... what are you doing in here???");
}

void WPG_ThreeD_Motion_Model::
func_xvredef_and_dxvredef_by_dxv_and_dxvredef_by_dxpdef(
const VNL::Vector<double> &xv, const VNL::Vector<double> &xpdef)
{
  // When we redefine axes:
  // r and q change as normal
  // v and omega are vectors so they change in the same way as r


  // We can mainly use the stuff from the general redefinition of axes in
  // position coordinates, but need to change it a little bit
  // State is
  //               x, y, z, q0, qx, qy, qz, vx, vy, vz, omegax, omegay, omegaz
  // Position state is
  //               x, y, z, q0, qx, qy, qz

  func_xp(xv);
  VNL::Vector<double> local_xp = xpRES;

  func_xpredef_and_dxpredef_by_dxp_and_dxpredef_by_dxpdef(local_xp, xpdef);

  ODEBUG3("func_xvredef... not yet implemented for WPG_ThreeD_Motion_Model"
       << "... though surely it's not too hard?");
  assert(false);
}

void WPG_ThreeD_Motion_Model::
func_xvnorm_and_dxvnorm_by_dxv(const VNL::Vector<double> &xv)
{
  ODEBUG3("func_xvnorm and dxnorm _by_dxv begin ");
  // Normalise the state vector: since quaternion is redundant we sometimes
  // need to enforce that it stays with size 1

  // Most parts of the state vector don't change so copy as starting point
  xvnormRES.Update(xv);

  // Most parts of Jacobian are identity
  dxvnorm_by_dxvRES.SetIdentity();

  // Extract quaternion
  func_xp(xv);
  func_q(xpRES);

  VW::Quaternion Tempqa = qRES;

  VW::Quaternion Tempqb = (Tempqa);
  VNL::MatrixFixed<4,4,double> Temp44a = dqnorm_by_dq(Tempqa);

  xvnormRES.Update(Tempqb.GetRXYZ(), 3);
  dxvnorm_by_dxvRES.Update(Temp44a, 3, 3);

  ODEBUG3("func_xvnorm and dxnorm _by_dxv end ");
}

//
// EXTRACT R Q V OMEGA
/** Extract the component parts of the state \f$ x_v \f$. Fills matrices r, q, v, omega with
values.
**/
void WPG_ThreeD_Motion_Model::extract_r_q_v_omega(
                                         const VNL::Vector<double> &xv,
					 VW::Vector3D &r,
					 VW::Quaternion &q,
					 VW::Vector3D &v,
					 VW::Vector3D &omega)
{
  ODEBUG3("func_extract_r_q_v_omega begin ");
  r.SetVNL3(xv.Extract(3, 0));

  VNL::VectorFixed<4,double> qRXYZ = xv.Extract(4, 3);
  q.SetRXYZ(qRXYZ);

  v.SetVNL3(xv.Extract(3, 7));

  omega.SetVNL3(xv.Extract(3, 10));

  ODEBUG3("func_extract_r_q_v_omega end ");
}

//
// EXTRACT R Q V OMEGA
/** Create a state vector \f$ x_v \f$ from its component parts. Puts the matrices r, q, v, omega into their right places.
**/
void WPG_ThreeD_Motion_Model::compose_xv(const VW::Vector3D &r,
					     const VW::Quaternion &q,
					     const VW::Vector3D &v,
					     const VW::Vector3D &omega,
					     VNL::Vector<double> &xv)
{
  ODEBUG3("compose_Xv begin ");
  xv.Update(r.GetVNL3(), 0);

  xv.Update(q.GetRXYZ(), 3);

  xv.Update(v.GetVNL3(), 7);

  xv.Update(omega.GetVNL3(), 10);

  ODEBUG3("compose_Xv end ");
}

//
// DQOMEGADT BY DOMEGA
/// Calculate commonly used Jacobian part \f$ \partial q(\omega * \Delta t) / \partial \omega \f$.
void WPG_ThreeD_Motion_Model::
dqomegadt_by_domega(const VW::Vector3D &omega,
		    const double delta_t,
		    VNL::MatrixFixed<4,3,double> &dqomegadt_by_domega)
{
  ODEBUG3("dqomegadt_by_domega begin");
  // Modulus
  double omegamod = sqrt(omega.GetX() * omega.GetX() +
			 omega.GetY() * omega.GetY() +
			 omega.GetZ() * omega.GetZ());

  // Use generic ancillary functions to calculate components of Jacobian
  dqomegadt_by_domega.Put(0, 0,
			  dq0_by_domegaA(omega.GetX(), omegamod, delta_t));
  dqomegadt_by_domega.Put(0, 1,
			  dq0_by_domegaA(omega.GetY(), omegamod, delta_t));
  dqomegadt_by_domega.Put(0, 2,
			  dq0_by_domegaA(omega.GetZ(), omegamod, delta_t));
  dqomegadt_by_domega.Put(1, 0,
			  dqA_by_domegaA(omega.GetX(), omegamod, delta_t));
  dqomegadt_by_domega.Put(1, 1,
			  dqA_by_domegaB(omega.GetX(), omega.GetY(), omegamod,
					 delta_t));
  dqomegadt_by_domega.Put(1, 2,
			  dqA_by_domegaB(omega.GetX(), omega.GetZ(), omegamod,
					 delta_t));
  dqomegadt_by_domega.Put(2, 0,
			  dqA_by_domegaB(omega.GetY(), omega.GetX(), omegamod,
					 delta_t));
  dqomegadt_by_domega.Put(2, 1,
			  dqA_by_domegaA(omega.GetY(), omegamod, delta_t));
  dqomegadt_by_domega.Put(2, 2,
			  dqA_by_domegaB(omega.GetY(), omega.GetZ(), omegamod,
					 delta_t));
  dqomegadt_by_domega.Put(3, 0,
			  dqA_by_domegaB(omega.GetZ(), omega.GetX(), omegamod,
					 delta_t));
  dqomegadt_by_domega.Put(3, 1,
			  dqA_by_domegaB(omega.GetZ(), omega.GetY(), omegamod,
					 delta_t));
  dqomegadt_by_domega.Put(3, 2,
			  dqA_by_domegaA(omega.GetZ(), omegamod, delta_t));

  ODEBUG3("dqomegadt_by_domega end");
}


// Ancillary functions: calculate parts of Jacobian dq_by_domega
// which are repeatable due to symmetry.
// Here omegaA is one of omegax, omegay, omegaz
// omegaB, omegaC are the other two
// And similarly with qA, qB, qC

//
// DQ0 BY DOMEGAA
/**
Ancillary function to calculate part of Jacobian \f$ \partial q / \partial \omega \f$ which is repeatable due to symmetry. Here omegaA is one of omegax, omegay, omegaz.
**/
double WPG_ThreeD_Motion_Model::dq0_by_domegaA(const double omegaA,
						   const double omega,
						   const double delta_t)
{
  return (-delta_t / 2.0) * (omegaA / omega) * sin(omega * delta_t / 2.0);
}

//
// DQA BY DOMEGAA
/**
Ancillary function to calculate part of Jacobian \f$ \partial q / \partial \omega \f$ which is repeatable due to symmetry. Here omegaA is one of omegax, omegay, omegaz and similarly with qA.
**/
double WPG_ThreeD_Motion_Model::dqA_by_domegaA(const double omegaA,
						   const double omega,
						   const double delta_t)
{
  return (delta_t / 2.0) * omegaA * omegaA / (omega * omega)
    * cos(omega * delta_t / 2.0)
    + (1.0 / omega) * (1.0 - omegaA * omegaA / (omega * omega))
    * sin(omega * delta_t / 2.0);
}

//
// DQA BY DOMEGAB
/**
Ancillary function to calculate part of Jacobian \f$ \partial q / \partial \omega \f$ which is repeatable due to symmetry. Here omegaB is one of omegax, omegay, omegaz and similarly with qA.
**/
double WPG_ThreeD_Motion_Model::dqA_by_domegaB(const double omegaA,
						   const double omegaB,
						   const double omega,
						   double delta_t)
{
  return (omegaA * omegaB / (omega * omega)) *
    ( (delta_t / 2.0) * cos(omega * delta_t / 2.0)
      - (1.0 / omega) * sin(omega * delta_t / 2.0) );
}
