/*  SceneLib: software for real-time SLAM

    models_wpg_hybrid_motion.cpp
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
#include "models_wpg_hybrid_motion.h"

// Debug macros
#include "Debug.h"

using namespace std;



// Constructor
WPG_Hybrid_ThreeD_Motion_Model::WPG_Hybrid_ThreeD_Motion_Model()
  : ThreeD_Motion_Model(13, 7, "WPG_HYBRID_THREED"),
    SD_A_component_filter(0.00005), // m s^-2
    SD_A_component(0.00005), // for simulation
    SD_alpha_component_filter(0.0001),  // rad s^-2
    SD_alpha_component(0.0001) // for simulation
{}

WPG_Hybrid_ThreeD_Motion_Model::~WPG_Hybrid_ThreeD_Motion_Model()
{}

void WPG_Hybrid_ThreeD_Motion_Model::func_fv_and_dfv_by_dxv(
                                     const VNL::Vector<double> &xv, 
				     const VNL::Vector<double> &u, 
				     const double delta_t)
{
  // This is just like the impulse motion model except we set the x, q 
  // update from PG information!!
  VW::Vector3D rold, vold, omegaold, 
    rnew, vnew, omeganew;
  VW::Quaternion qold, qnew;

  // Separate things out to make it clearer
  extract_r_q_v_omega(xv, rold, qold, vold, omegaold);


  // And now the old stuff to work out the rest
  VW::Vector3D acceleration(0.0);

  // Keep qwt ( = q(omega * delta_t)) for later use
  VW::Quaternion qwt(omegaold * delta_t);

#if 1
  // Components of control vector
  VW::Vector3D TW (u.Extract(3, 0));

  VW::Quaternion qRoldRnew;

  qRoldRnew.SetRXYZ(u.Extract(4, 3));

  // Here we go with the super simple prediction from PG
  rnew = rold + TW;


  cout << "qRoldRnew" << qRoldRnew << endl;
  cout << "qwt" << qwt << endl;

  qnew = qold * qRoldRnew;
#endif



#if 0
  // rnew = r + v * delta_t
  rnew = rold + vold * delta_t;

  // qnew = q x q(omega * delta_t)

  qnew = qold * qwt;
#endif




  // vnew = v
  vnew = vold + acceleration * delta_t;
  
  // omeganew = omega
  omeganew = omegaold;

  // Put it all together
  compose_xv(rnew, qnew, vnew, omeganew, fvRES);

  // cout << "rold qold vold omegaold" << rold << qold 
  //      << vold << omegaold;
  // cout << "rnew qnew vnew omeganew" << rnew << qnew 
  //      << vnew << omeganew;

  // Now on to the Jacobian...
  // Identity is a good place to start since overall structure is like this
  // I       0             I * delta_t   0
  // 0       dqnew_by_dq   0             dqnew_by_domega
  // 0       0             I             0
  // 0       0             0             I
  dfv_by_dxvRES.SetIdentity();

  // Fill in dxnew_by_dv = I * delta_t
  VNL::MatrixFixed<3,3,double> Temp33A;
  Temp33A.SetIdentity();
  Temp33A *= delta_t;
  dfv_by_dxvRES.Update(Temp33A, 0, 7);

  // Fill in dqnew_by_dq
  // qnew = qold x qwt  ( = q3 = q2 x q1 in Scene/newbits.cc language)
  VNL::MatrixFixed<4,4,double> Temp44A = dq3_by_dq2(qwt); 
  dfv_by_dxvRES.Update(Temp44A, 3, 3);

  // Fill in dqnew_by_domega = d(q x qwt)_by_dqwt . dqwt_by_domega
  Temp44A = dq3_by_dq1(qold); // Temp44A is d(q x qwt) by dqwt
 
  // Use function below for dqwt_by_domega
  VNL::MatrixFixed<4,3,double> Temp43A;
  dqomegadt_by_domega(omegaold, delta_t, Temp43A);
  // Multiply them together
  VNL::MatrixFixed<4,3,double> Temp43B = Temp44A * Temp43A;
  // And plug it in
  dfv_by_dxvRES.Update(Temp43B, 3, 10);

  // cout << "dfv_by_dxvRES" << dfv_by_dxvRES;
}

void WPG_Hybrid_ThreeD_Motion_Model::func_Q(const VNL::Vector<double> &xv, 
					 const VNL::Vector<double> &, 
					 const double delta_t)
{
  // Fill noise covariance matrix Pnn: this is the covariance of 
  // the noise vector (V)
  //                  (Omega)
  // that gets added to the state. 
  // Form of this could change later, but for now assume that 
  // V and Omega are independent, and that each of their components is
  // independent... 
  double linear_velocity_noise_variance = 
     SD_A_component_filter * SD_A_component_filter * delta_t * delta_t;
  double angular_velocity_noise_variance =
     SD_alpha_component_filter * SD_alpha_component_filter * delta_t * delta_t;

  // Independence means that the matrix is diagonal
  VNL::MatrixFixed<6,6,double> Pnn;
  Pnn.Fill(0.0);
  Pnn.Put(0, 0, linear_velocity_noise_variance);
  Pnn.Put(1, 1, linear_velocity_noise_variance);
  Pnn.Put(2, 2, linear_velocity_noise_variance);
  Pnn.Put(3, 3, angular_velocity_noise_variance);
  Pnn.Put(4, 4, angular_velocity_noise_variance);
  Pnn.Put(5, 5, angular_velocity_noise_variance);

  // Form Jacobian dxnew_by_dn
  // Is like this:
  // I * delta_t     0
  // 0               dqnew_by_dOmega
  // I               0
  // 0               I

  // Start by zeroing
  VNL::MatrixFixed<13,6,double> dxnew_by_dn;
  dxnew_by_dn.Fill(0.0);

  // Fill in easy bits first
  VNL::MatrixFixed<3,3,double> Temp33A;
  Temp33A.SetIdentity();
  
  dxnew_by_dn.Update(Temp33A, 7, 0);
  dxnew_by_dn.Update(Temp33A, 10, 3);
  Temp33A *= delta_t;
  dxnew_by_dn.Update(Temp33A, 0, 0);

  // Tricky bit is dqnew_by_dOmega
  // Is actually the same calculation as in func_fv...
  // Since omega and Omega are additive...?
  VW::Vector3D rold, vold, omegaold;
  VW::Quaternion qold;
  extract_r_q_v_omega(xv, rold, qold, vold, omegaold); // overkill but easy
  // Fill in dqnew_by_domega = d(q x qwt)_by_dqwt . dqwt_by_domega
  // Temp44A is d(q x qwt) by dqwt
  VNL::MatrixFixed<4,4,double> Temp44A = dq3_by_dq1(qold); 
  // Use function below for dqwt_by_domega
  VNL::MatrixFixed<4,3,double> Temp43A;
  dqomegadt_by_domega(omegaold, delta_t, Temp43A);
  // Multiply them together
  VNL::MatrixFixed<4,3,double> Temp43B = Temp44A * Temp43A;
  // And then plug into Jacobian
  dxnew_by_dn.Update(Temp43B, 3, 3);

  // Finally do Q = dxnew_by_dn . Pnn . dxnew_by_dnT
  QxRES.Update( dxnew_by_dn * Pnn * dxnew_by_dn.Transpose() );

  //  cout << "QxRES" << QxRES;
}

// 
// FUNC XP
/** Extract the position and orientation from the state vector. (This is the 
first seven elements in this case.)
**/
void WPG_Hybrid_ThreeD_Motion_Model::func_xp(const VNL::Vector<double> &xv)
{
  xpRES = xv.Extract(7, 0);
}

//
// FUNC DXP BY DXV
/** Calculate Jacobian for func_xp. (This is just the identity in this case.)
**/
void WPG_Hybrid_ThreeD_Motion_Model::
func_dxp_by_dxv(const VNL::Vector<double> &)
{
  dxp_by_dxvRES.Fill(0.0);

  dxp_by_dxvRES.Put(0, 0, 1.0);
  dxp_by_dxvRES.Put(1, 1, 1.0);
  dxp_by_dxvRES.Put(2, 2, 1.0);
  dxp_by_dxvRES.Put(3, 3, 1.0);
  dxp_by_dxvRES.Put(4, 4, 1.0);
  dxp_by_dxvRES.Put(5, 5, 1.0);
  dxp_by_dxvRES.Put(6, 6, 1.0);
}

// Noisy process equation for simulation
// Simply perturb xv with Gaussian noise and send it through func_fv
void WPG_Hybrid_ThreeD_Motion_Model::
func_fv_noisy(const VNL::Vector<double> &xv_true, 
	      const VNL::Vector<double> &u_true, const double delta_t)
{
  VNL::Vector<double> xv_noisy = xv_true;

  // Linear velocity
  for (int row = 7; row < 10; row++)
    xv_noisy(row) 
      = VNL::SampleNormal(xv_true(row), SD_A_component * delta_t);
  // Angular velocity
  for (int row = 10; row < 13; row++)
    xv_noisy(row) 
      = VNL::SampleNormal(xv_true(row), SD_alpha_component * delta_t);

  // Now send through normal process equaion
  func_fv_and_dfv_by_dxv(xv_noisy, u_true, delta_t);

  // And copy result
  fv_noisyRES.Update(fvRES);
}

void WPG_Hybrid_ThreeD_Motion_Model::
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

  cerr << "func_xvredef... not yet implemented for WPG_Hybrid_ThreeD_Motion_Model"
       << "... though surely it's not too hard?"
       << endl;
  assert(0);
}

void WPG_Hybrid_ThreeD_Motion_Model::
func_xvnorm_and_dxvnorm_by_dxv(const VNL::Vector<double> &xv)
{
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
}

//
// EXTRACT R Q V OMEGA
/** Extract the component parts of the state \f$ x_v \f$. Fills matrices r, q, v, omega with
values.
**/
void WPG_Hybrid_ThreeD_Motion_Model::extract_r_q_v_omega(
                                         const VNL::Vector<double> &xv, 
					 VW::Vector3D &r, 
					 VW::Quaternion &q, 
					 VW::Vector3D &v, 
					 VW::Vector3D &omega)
{
  r.SetVNL3(xv.Extract(3, 0));

  VNL::VectorFixed<4,double> qRXYZ = xv.Extract(4, 3);
  q.SetRXYZ(qRXYZ);

  v.SetVNL3(xv.Extract(3, 7));

  omega.SetVNL3(xv.Extract(3, 10));
}

//
// EXTRACT R Q V OMEGA
/** Create a state vector \f$ x_v \f$ from its component parts. Puts the matrices r, q, v, omega into their right places.
**/
void WPG_Hybrid_ThreeD_Motion_Model::compose_xv(const VW::Vector3D &r, 
					     const VW::Quaternion &q,
					     const VW::Vector3D &v, 
					     const VW::Vector3D &omega, 
					     VNL::Vector<double> &xv)
{
  xv.Update(r.GetVNL3(), 0);

  xv.Update(q.GetRXYZ(), 3);

  xv.Update(v.GetVNL3(), 7);
  
  xv.Update(omega.GetVNL3(), 10);
}

//
// DQOMEGADT BY DOMEGA
/// Calculate commonly used Jacobian part \f$ \partial q(\omega * \Delta t) / \partial \omega \f$.
void WPG_Hybrid_ThreeD_Motion_Model::
dqomegadt_by_domega(const VW::Vector3D &omega, 
		    const double delta_t,
		    VNL::MatrixFixed<4,3,double> &dqomegadt_by_domega)
{
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
double WPG_Hybrid_ThreeD_Motion_Model::dq0_by_domegaA(const double omegaA, 
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
double WPG_Hybrid_ThreeD_Motion_Model::dqA_by_domegaA(const double omegaA, 
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
double WPG_Hybrid_ThreeD_Motion_Model::dqA_by_domegaB(const double omegaA, 
						   const double omegaB, 
						   const double omega, 
						   double delta_t)
{
  return (omegaA * omegaB / (omega * omega)) * 
    ( (delta_t / 2.0) * cos(omega * delta_t / 2.0)
      - (1.0 / omega) * sin(omega * delta_t / 2.0) );
}
