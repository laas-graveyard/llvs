/*  MonoSLAM: Real-Time Single Camera SLAM

    MonoSLAM/models_orientation.cpp
    Copyright (C) 2005 University of Oxford

    Author
    Olivier Stasse
    olivier.stasse@aist.go.jp

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

#include <Scene/newbits.h>
#include <VNL/sample.h>
#include "models_orientation.h"

Orientation_Internal_Measurement_Model::
Orientation_Internal_Measurement_Model(Motion_Model *motion_model)
  : Internal_Measurement_Model(6, motion_model, "ORIENTATION"),
#if 1
    SD_meas_filter(0.02),
    SD_meas_simulation(0.02),
    SD_ang_meas_filter(0.005),
    SD_ang_meas_simulation(0.005)
#else
    SD_meas_filter(0.02),
    SD_meas_simulation(0.02)
  
#endif 
{
}

Orientation_Internal_Measurement_Model::
~Orientation_Internal_Measurement_Model()
{

}


void Orientation_Internal_Measurement_Model::
func_hv_and_dhv_by_dxv(const VNL::Vector<double> &xv)
{
  //  cout << "xv as an entry "<< xv<< endl;
  // Just pull out v part of state
  VW::Vector3D vW;
  vW.SetVNL3(xv.Extract(3, 7));

  VW::Quaternion qWR(0.5,0.5,0.5,0.5);
  //  qWR.SetRXYZ(xv.Extract(4, 3));

  VW::Quaternion qRW = qWR.Inverse();

  VNL::MatrixFixed<3,3,double> RRW = qRW.RotationMatrix();

  // Now hv = vR = RRW vW
  VW::Vector3D vR = RRW * vW;
  hvRES.Update(vR.GetVNL3(),0);
  hvRES.Update(xv.Extract(3, 10),3);
  
  // dhv_by_dxv = (0 dhv_by_dqWR dhv_by_dvR 0)
  dhv_by_dxvRES.Fill(0.0);
    
  dhv_by_dxvRES.Update(RRW, 0, 7);

  // Just pull out omega part of state
  VNL::Matrix<double> I33(3, 3);
  I33.SetIdentity();
  dhv_by_dxvRES.Update(I33, 3, 10);

  // dhv_by_dxv = (dhv_by_dr dhv_by_dq dhv_by_dv dhv_by_domega)
  //dhv_by_dxvRES[5][12] = 1.0;
  
  //cout << "hvRES: " << hvRES<< endl;
  //  cout << "dhv_by_dxvRES: " << endl << dhv_by_dxvRES<< endl;
}

void Orientation_Internal_Measurement_Model::
func_Rv(const VNL::Vector<double> &hv)
{
  // Diagonal measurement covariance
  double measurement_noise_variance = SD_meas_filter * SD_meas_filter;
  double measurement_ang_noise_variance = SD_ang_meas_filter * SD_ang_meas_filter;

  RvRES.SetIdentity();

  for(int i=0;i<3;i++)
    RvRES[i][i] *= measurement_noise_variance;
  
  for(int i=0;i<3;i++)
    RvRES[i+3][i+3] *= measurement_ang_noise_variance;
  //  cout << "measurement_ang_noise_variance" << endl << RvRES << endl;
}

void Orientation_Internal_Measurement_Model::
func_nuv(const VNL::Vector<double> &hv, const VNL::Vector<double> &zv)
{
#if 0
  // Extraction
  VW::Vector3D rhv;

  rhv.SetVNL3(hv.Extract(3, 0));

  VNL::VectorFixed<4,double> qRXYZ_hv = hv.Extract(4, 3);

  VW::Vector3D rzv;

  rzv.SetVNL3(zv.Extract(3, 0));

  VNL::VectorFixed<4,double> qRXYZ_zv = zv.Extract(4, 3);
  
  double dotp = DotProduct(qRXYZ_hv,qRXYZ_zv);
  double d = acos(dotp);

  for(int li=0;li<4;li++)
    nuvRES[li+3] = d;
  

  for(int li=0;li<3;li++)
    nuvRES[li] = rzv[li]-rhv[li];
#else
  
  nuvRES.Update(zv-hv);
  //  cout << "zv : " << zv << endl;
  //  cout << "hv : " << hv << endl;
  //  cout << "nuvRES  " << nuvRES << endl;
#endif
}

void Orientation_Internal_Measurement_Model::
func_hv_noisy(const VNL::Vector<double> &xv_true)
{
  func_hv_and_dhv_by_dxv(xv_true);

  for(int li=0;li<3;li++)
    hv_noisyRES.Put(li, VNL::SampleNormal(hvRES(li), SD_meas_simulation));

  for(int li=3;li<6;li++)
    hv_noisyRES.Put(li, VNL::SampleNormal(hvRES(li), SD_ang_meas_simulation));
  //  cout << "hv_noisyRES " << hv_noisyRES<< endl;
}

bool Orientation_Internal_Measurement_Model::
feasibility_test(const VNL::Vector<double> &xv, const VNL::Vector<double> &hv)
{
  // Always feasible
  return true;
}

// DQOMEGADT BY DOMEGA
/// Calculate commonly used Jacobian part \f$ \partial q(\omega * \Delta t) / \partial \omega \f$.
void Orientation_Internal_Measurement_Model::
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
double Orientation_Internal_Measurement_Model::dq0_by_domegaA(const double omegaA, 
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
double Orientation_Internal_Measurement_Model::dqA_by_domegaA(const double omegaA, 
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
double Orientation_Internal_Measurement_Model::dqA_by_domegaB(const double omegaA, 
						   const double omegaB, 
						   const double omega, 
						   double delta_t)
{
  return (omegaA * omegaB / (omega * omega)) * 
    ( (delta_t / 2.0) * cos(omega * delta_t / 2.0)
      - (1.0 / omega) * sin(omega * delta_t / 2.0) );
}
//
// EXTRACT R Q V OMEGA
/** Extract the component parts of the state \f$ x_v \f$. Fills matrices r, q, v, omega with
values.
**/
void Orientation_Internal_Measurement_Model::extract_r_q_v_omega(
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
