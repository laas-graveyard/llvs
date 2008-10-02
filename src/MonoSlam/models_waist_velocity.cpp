/*  MonoSLAM: Real-Time Single Camera SLAM

    MonoSLAM/models_waist_velocity.cpp
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

#include <VNL/sample.h>
#include "models_waist_velocity.h"

Waist_Velocity_Internal_Measurement_Model::
Waist_Velocity_Internal_Measurement_Model(Motion_Model *motion_model)
  : Internal_Measurement_Model(3, motion_model, "THREED_WAIST_VELOCITY"),
    SD_meas_filter(0.03),
    SD_meas_simulation(0.03)
{

}

Waist_Velocity_Internal_Measurement_Model::
~Waist_Velocity_Internal_Measurement_Model()
{

}


void Waist_Velocity_Internal_Measurement_Model::
func_hv_and_dhv_by_dxv(const VNL::Vector<double> &xv)
{
  assert(xv.Size() == motion_model->STATE_SIZE);

  // Just pull out v part of state
  VW::Vector3D vW;
  vW.SetVNL3(xv.Extract(3, 7));

  VW::Quaternion qWR(0.5,0.5,0.5,0.5);
  //  qWR.SetRXYZ(xv.Extract(4, 3));

  VW::Quaternion qRW = qWR.Inverse();

  VNL::MatrixFixed<3,3,double> RRW = qRW.RotationMatrix();

  // Now hv = vR = RRW vW
  VW::Vector3D vR = RRW * vW;
  hvRES.Update(vR.GetVNL3());

  //  cout << "wv hvRES: " << hvRES << endl;
  // dhv_by_dxv = (0 dhv_by_dqWR dhv_by_dvR 0)
  dhv_by_dxvRES.Fill(0.0);

#if 0
  // dhv_by_dqWR = d_by_dqRW(RRW vW) * dqRW_by_dqWR
  VNL::MatrixFixed<3,4,double> dhv_by_dqWR =
    dRq_times_a_by_dq(qRW, vW) * dqbar_by_dq();

  dhv_by_dxvRES.Update(dhv_by_dqWR, 0, 3);
#endif

  // dhv_by_dvR = RRW
  dhv_by_dxvRES.Update(RRW, 0, 7);
  //  cout << "wv dhvRES: " << endl << dhv_by_dxvRES << endl;
}

void Waist_Velocity_Internal_Measurement_Model::
func_Rv(const VNL::Vector<double> &hv)
{
  // Diagonal measurement covariance
  double measurement_noise_variance = SD_meas_filter * SD_meas_filter;

  RvRES.SetIdentity();

  RvRES *= measurement_noise_variance;
}

void Waist_Velocity_Internal_Measurement_Model::
func_nuv(const VNL::Vector<double> &hv, const VNL::Vector<double> &zv)
{
  // Straightforward
  nuvRES.Update(zv - hv);
}

void Waist_Velocity_Internal_Measurement_Model::
func_hv_noisy(const VNL::Vector<double> &xv_true)
{
  func_hv_and_dhv_by_dxv(xv_true);

  hv_noisyRES.Put(0, VNL::SampleNormal(hvRES(0), SD_meas_simulation));
  hv_noisyRES.Put(1, VNL::SampleNormal(hvRES(1), SD_meas_simulation));
  hv_noisyRES.Put(2, VNL::SampleNormal(hvRES(2), SD_meas_simulation));
}

bool Waist_Velocity_Internal_Measurement_Model::
feasibility_test(const VNL::Vector<double> &xv, const VNL::Vector<double> &hv)
{
  // Always feasible
  return true;
}

