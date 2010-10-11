/** @doc This object implements a visual process
    to get single camera Self Localization and map building.

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
#include <LowLevelVisionServer.h>
#include <SingleCameraSLAMProcess.h>
#include <MonoSLAM/robot.h>
#include <MonoSLAM/model_creators.h>
#include "hrp_model_creators.h"
#include <sys/time.h>
#include <time.h>

#include <MonoSLAM/robot_patch.h>

// Debug macros
#include <llvs/tools/Debug.h>

using namespace VW;


#include <VNL/matrix.h>
#include <VNL/Algo/matrixinverse.h>

HRP2SingleCameraSLAMProcess::HRP2SingleCameraSLAMProcess(CORBA::ORB_var orb,
							 CosNaming::NamingContext_var cxt,
							 LowLevelVisionServer *aLLVS)
  : m_TrackingState(false),
    m_MappingState(false),
    m_VisionFlag(true),
    m_GyroFlag(true),
    m_WaistVelocityFlag(true),
    m_CameraHeightFlag(true),
    m_OrientationFlag(true),
    m_NUMBER_OF_FEATURES_TO_SELECT(10),
    m_NUMBER_OF_FEATURES_TO_KEEP_VISIBLE(12),
    m_MAX_FEATURES_TO_INIT_AT_ONCE(1),
    m_MIN_LAMBDA(0.5),
    m_MAX_LAMBDA(5.0),
    m_NUMBER_OF_PARTICLES(100),
    m_STANDARD_DEVIATION_DEPTH_RATIO(0.3),
    m_MIN_NUMBER_OF_PARTICLES(20),
    m_PRUNE_PROBABILITY_THRESHOLD(0.05),
    m_ERASE_PARTIALLY_INIT_FEATURE_AFTER_THIS_MANY_ATTEMPTS(10)
{
  m_LLVS = aLLVS;
  m_orb = orb;
  m_cxt = cxt;

  m_TrackingState = false;
  m_MappingState = false;
  m_ProcessName = "Single Camera SLAM";

  /* Default value for the Vision state and the mapping state */
  string aParameter = "TrackingState";
  string aValue = "false";
  SetParameter(aParameter,aValue);

  aParameter = "MappingState";
  aValue = "false";
  SetParameter(aParameter,aValue);

  aParameter = "VisionFlag";
  aValue = "true";
  SetParameter(aParameter,aValue);

  aParameter = "GyroFlag";
  aValue = "true";
  SetParameter(aParameter,aValue);

  aParameter = "WaistVelocityFlag";
  aValue = "true";
  SetParameter(aParameter,aValue);

  aParameter = "OrientationFlag";
  aValue = "true";
  SetParameter(aParameter,aValue);

  aParameter = "CameraHeightFlag";
  aValue = "true";
  SetParameter(aParameter,aValue);

  aParameter = "ConnectToGGAA";
  aValue = "0";
  SetParameter(aParameter,aValue);

  m_grabbed_image = new ImageMono<unsigned char>(320,240);
  m_grabbed_image->AllocImageData(320,240);

  HRP2MonoSLAM_Motion_Model_Creator mm_creator;
  MonoSLAM_Feature_Measurement_Model_Creator fmm_creator;
  // Locally defined creator for internal measurement models
  HRP_Internal_Measurement_Model_Creator imm_creator;

  m_WL2Head.Resize(4,4);

  m_MonoSLAMHRP =   new MonoSLAMHRP("monoslam_state.ini",
				    &mm_creator,
				    &fmm_creator,
				    &imm_creator,
				    m_NUMBER_OF_FEATURES_TO_SELECT,
				    m_NUMBER_OF_FEATURES_TO_KEEP_VISIBLE,
				    m_MAX_FEATURES_TO_INIT_AT_ONCE,
				    m_MIN_LAMBDA,
				    m_MAX_LAMBDA,
				    m_NUMBER_OF_PARTICLES,
				    m_STANDARD_DEVIATION_DEPTH_RATIO,
				    m_MIN_NUMBER_OF_PARTICLES,
				    m_PRUNE_PROBABILITY_THRESHOLD,
				    m_ERASE_PARTIALLY_INIT_FEATURE_AFTER_THIS_MANY_ATTEMPTS);

  ODEBUG3("Finished the instanciation of m_MonoSLAMHRP");
}

HRP2SingleCameraSLAMProcess::~HRP2SingleCameraSLAMProcess()
{
  if (m_grabbed_image!=0)
    delete m_grabbed_image;
}

int HRP2SingleCameraSLAMProcess::pInitializeTheProcess()
{
  return 0;
}


int HRP2SingleCameraSLAMProcess::pRealizeTheProcess()
{
  double lGyro[3]={0.0,0.0,0.0};
  double lAccelerometer[3] = { 0.0, 0.0, 0.0 };
  double lWaistVelocity[2]={0.0,0.0};
  double lPosOrientation[8]={0.0,0.0,0.0,0.0,0.0,0.0,0.0, 0.0};
  static double prev_time;
  double ldelta_t;
  static unsigned char lFirstTime = 1;
  struct timeval tod1;
  double time1;

  VNL::Vector<double> gyro(3);
  VNL::Vector<double> accelerometer(3);
  VNL::Vector<double> waist_velocity(3);
  VNL::Vector<double> camera_height(1);
  VNL::Vector<double> posorientation(7);

  ODEBUG("Went through HERE !");
  if (!m_Computing)
    return 0;

  ODEBUG("Before grabbing the image ");
  /* Copy the image into scene control internal structure */

  gettimeofday(&tod1, NULL);
  time1 = tod1.tv_sec + tod1.tv_usec / 1000000.0;
  if (lFirstTime)
    {
      ldelta_t = 0.033;
      ODEBUG("Here 1");
      // Compute now the matrix from the head to the wide lens camera.
      if (m_FFFWI!=0)
	{
	  // Take the wide lens extrinsic parameters inside the
	  // first vision reference frame.
	  VNL::Matrix<double> FromCamToW(4,4);
	  FromCamToW.SetIdentity();
	  FromCamToW[0][0]=0;FromCamToW[0][1]=-1.0;
	  FromCamToW[1][1]=0;FromCamToW[1][2]=-1.0;
	  FromCamToW[2][2]=0;FromCamToW[2][0]= 1.0;
	  ODEBUG("Here 2");
	  VNL::Matrix<double> WL2V1 = m_FFFWI->GetWideLensPositionInV1();
	  ODEBUG("Here 3");
	  cout << "WL2V1" << endl << WL2V1<< endl;
	  VNL::Matrix<double> V12Head(4,4),M(4,4);
	  for(int li=0;li<4;li++)
	    for(int lj=0;lj<4;lj++)
	      V12Head[li][lj]=m_headTorg[li*4+lj];
	  m_WL2Head =V12Head * WL2V1 *FromCamToW;
	  // Final rotation of ten degrees around Y
	  VW::Vector3D RotY(0.0,1.0,0.0);
	  VW::RotationMatrix aR(RotY,-10.0*M_PI/180.0);
	  FromCamToW.SetIdentity();
	  for(int li=0;li<3;li++)
	    for(int lj=0;lj<3;lj++)
	      FromCamToW[li][lj]= aR[li][lj];
	  // Now express it in the head reference frame.
	  for(int li=0;li<3;li++)
	    m_WL2Head[li][3]/=1000.0;
	  cout << "m_WL2Head :" << endl << m_WL2Head << endl;
	  m_WL2Head = FromCamToW * m_WL2Head;
	  cout << "m_WL2Head :" << endl << m_WL2Head << endl;
	}
      ODEBUG("Here End");
    }
  else
    {
      int NbOfFramePassed;
      ldelta_t = time1 - prev_time;
      //      NbOfFramePassed = (int) ceil(ldelta_t * 30.0);
      //      ldelta_t = 0.033 * NbOfFramePassed;
    }

  //  cerr << "Frame acquired: absolute time = " << time1 - time0
  //      << " seconds." << endl;

  ODEBUG("Before test on Gyro And Accelerometer");
  if (!CORBA::is_nil(m_GyroAndAccelerometer))
    {

      double timeref = time1 - 0.033;
      if (GetGyroAcceleroFromTimeStamp(lGyro,lAccelerometer,timeref,lWaistVelocity,lPosOrientation)<0)
	{
	  lGyro[0] = 0; lGyro[1]= 0; lGyro[2] = 0.0;
	  lAccelerometer[0] =
	    lAccelerometer[1] =
	    lAccelerometer[2] = 0.0;
	  lWaistVelocity[0] =
	    lWaistVelocity[1] = 0.0;
	  for(int li=0;li<7;li++)
	    lPosOrientation[li] = 0.0;
	}

      // Update using the gyro values....
      gyro(0) = lGyro[0];
      gyro(1) = lGyro[1];
      gyro(2) = lGyro[2];

      waist_velocity(0) = lWaistVelocity[0];
      waist_velocity(1) = lWaistVelocity[1];
      waist_velocity(2) = 0.0;

      //      camera_height(0) = 1.40;

      for(int li=0;li<7;li++)
	posorientation(li)= lPosOrientation[li];

      // From the current position build a matrix
      // from the head to the world.
      VW::Quaternion qco(posorientation(1),
			 posorientation(2),
			 posorientation(3),
			 posorientation(0));
      VW::RotationMatrix R(qco);
      VNL::Matrix<double> M(4,4);
      M.SetIdentity();
      for(int i=0;i<3;i++)
	{
	  for(int j=0;j<3;j++)
	    M[i][j]=R[i][j];
	  M[i][3]=posorientation(4+i);
	}
      // Create the matrix from the WL to the world.
      M= M*m_WL2Head;
      if (lFirstTime)
	{
	  cout << "M : " << endl << M<<endl;
	  cout << "pos: " << endl << posorientation << endl;
	}
      // Get back the new rotation matrix
      for(int i=0;i<3;i++)
	  for(int j=0;j<3;j++)
	    R[i][j]=M[i][j];

      // Put it back in quaternion + position.
      VW::Quaternion qf(R);
      // right now just the position.
      for(int i=0;i<3;i++)
	posorientation(4+i)=M[i][3];

      camera_height(0) = posorientation(6);

    }
  else
    {
      //      cout << "NO GYRO TAKING INTO ACCOUNT " << endl;
    }

  /// Get back to the image.
  unsigned char *dst = (unsigned char *)m_grabbed_image->GetImageBuffer() ;
  unsigned char *src = (unsigned char *)m_InputImage->Image;
  for(int li=0;li<m_InputImage->Width * m_InputImage->Height;li++)
    *dst++ = *src++;

#if 0
  ImageConversions::MonoBuffer_to_VWImage((char *)m_InputImage->Image,m_grabbed_image,
					  m_InputImage->Width,
					  m_InputImage->Height,
					  1, 1);
#endif



  // Accelerometer
  VNL::Vector<double> u(3);
  // u(0) = lAccelerometer[1];
  ///u(1) = lAccelerometer[2];
  // u(2) = lAccelerometer[0];

  // Ignore accelerometer
  u(0) = 0.0;
  u(1) = 0.0;
  u(2) = 0.0;

  //ODEBUG( " U for Andy: " << u);

  ODEBUG("Grabbing the image finished. Before HandleNewFrame() " << " " << m_grabbed_image );

  ODEBUG("ldelta_t " << ldelta_t);
  if (m_MonoSLAMHRP!=0)
    {
      ODEBUG("Perform MonoSLAMHRP "
	      << m_MappingState << " "
	      << m_TrackingState << " "
	      << m_VisionFlag << " "
	      << m_GyroFlag << " "
	      << m_WaistVelocityFlag << " "
	      << m_CameraHeightFlag );
      ODEBUG3("Orientation : " << posorientation << " " << lPosOrientation[7] <<
	      " 0 0 " << sin(0.5*lPosOrientation[7]) << " " << cos(0.5*lPosOrientation[7]) );
      ODEBUG( "Gyro: " << gyro[0]<< " " << gyro[1]<<  " " << gyro[2]);
      ODEBUG("m_MonoSLAMHRP :" << m_MonoSLAMHRP << endl <<
	      "m_MonoSLAMHRP->GetRobotNoConst() :" << m_MonoSLAMHRP->GetRobotNoConst() << endl);
      m_MonoSLAMHRP->GetRobotNoConst()->load_new_image(m_grabbed_image);
      ODEBUG("here ");
      m_MonoSLAMHRP->GoOneStepHRP(m_grabbed_image,
				  0.033,
				  m_MappingState,
				  m_VisionFlag,
				  m_GyroFlag,
				  m_WaistVelocityFlag,
				  m_CameraHeightFlag,
				  m_OrientationFlag,
				  gyro,
				  waist_velocity,
				  camera_height,
				  posorientation,
				  u);
    }

  ODEBUG("HandleNewFrame() finished");
  struct timeval tod2;
  double time2;
  gettimeofday(&tod2, NULL);
  time2 = tod2.tv_sec + tod2.tv_usec / 1000000.0;
  //cerr << "Computation time SLAM alone " << time2 - time1
  //<< " seconds." << endl;
  prev_time = time1;
  lFirstTime = 0;
  return 0;
}


int HRP2SingleCameraSLAMProcess::pCleanUpTheProcess()
{
  return 0;
}


int HRP2SingleCameraSLAMProcess::SetParameter(string aParameter, string aValue)
{
  int r=-1;
  unsigned char ok = 1;

  if (aParameter=="TrackingState")
    {
      if (aValue=="false")
	m_TrackingState = false;
      else if (aValue=="true")
	m_TrackingState = true;
      else
	ok = 0;
    }

  if (aParameter=="MappingState")
    {
      if (aValue=="false")
	m_MappingState = false;
      else if (aValue=="true")
	m_MappingState = true;
      else
	ok = 0;
    }

  if (aParameter=="VisionFlag")
    {
      if (aValue=="false")
	m_VisionFlag = false;
      else if (aValue=="true")
	m_VisionFlag = true;
      else
	ok = 0;
    }

  if (aParameter=="GyroFlag")
    {
      if (aValue=="false")
	m_GyroFlag = false;
      else if (aValue=="true")
	m_GyroFlag = true;
      else
	ok = 0;
    }

  if (aParameter=="WaistVelocityFlag")
    {
      if (aValue=="false")
	m_WaistVelocityFlag = false;
      else if (aValue=="true")
	m_WaistVelocityFlag = true;
      else
	ok = 0;
    }

  if (aParameter=="CameraHeightFlag")
    {
      if (aValue=="false")
	m_CameraHeightFlag = false;
      else if (aValue=="true")
	m_CameraHeightFlag = true;
      else
	ok = 0;
    }

  if (aParameter=="OrientationFlag")
    {
      if (aValue=="false")
	m_OrientationFlag = false;
      else if (aValue=="true")
	m_OrientationFlag = true;
      else
	ok = 0;
    }

  if (aParameter=="SetSLAMImage")
    {
      istringstream aIstrm(aValue);
      int indexImage;
      aIstrm >> indexImage;
      m_LLVS->SetTheSLAMImage(indexImage);
      ODEBUG("Index image for SLAM:" << indexImage);
    }

  if (aParameter=="ConnectToGGAA")
    {
      GetCorbaConnectionToGGAAplugin();
    }
  if (ok)
    r = HRP2VisionBasicProcess::SetParameter(aParameter, aValue);

  return r;

}


int HRP2SingleCameraSLAMProcess::SetInputImages(EPBM * aInputImage)
{
  m_InputImage = aInputImage;
  return 0;
}

int HRP2SingleCameraSLAMProcess::GetPositionAndCovariance(double Position[7], double Covariance[9])
{
  VNL::Vector<double> xv;
  VNL::Matrix<double> Pxx;

  Scene_Single * scene;
  scene = (Scene_Single *)m_MonoSLAMHRP->GetScene();

  // Takes the data from the scene object.
  xv = scene->get_xv();
  Pxx = scene->get_Pxx();

  for(unsigned int i=0;i<xv.size();i++)
    Position[i] = xv[i];

  for(unsigned int j=0;j<Pxx.Rows();j++)
    for(unsigned int i=0;i<Pxx.Cols();i++)
      Covariance[j*Pxx.Cols() + i] = Pxx[j][i];

  return 0;
}

int HRP2SingleCameraSLAMProcess::CreateCopyOfScene(SceneObject_var &aSO_var)
{
  VNL::Vector<double> xv;
  VNL::Matrix<double> Pxx;

  Scene_Single * scene;
  scene = (Scene_Single *)m_MonoSLAMHRP->GetScene();

  // Takes the data from the scene object.
  xv = scene->get_xv();
  Pxx = scene->get_Pxx();

  // Update the number of feature.
  aSO_var->NoFeatures = (CORBA::Long)scene->get_no_features();

  // Update the number of selected features.
  aSO_var->NoSelected = (CORBA::Long)scene->get_no_selected();

  // Create the CORBA objects.

  // State vector
  aSO_var->xv.data.length(xv.size());
  aSO_var->xv.nrows = xv.size();
  aSO_var->xv.ncols = 1;
  for(unsigned int i=0;i<xv.size();i++)
    aSO_var->xv.data[i] = xv[i];


  // Covariance matrix
  int lncols;
  aSO_var->Pxx.data.length(Pxx.size());
  aSO_var->Pxx.nrows = Pxx.Rows();
  aSO_var->Pxx.ncols = lncols = Pxx.Cols();
  for(unsigned int j=0;j<Pxx.Rows();j++)
    for(unsigned int i=0;i<Pxx.Cols();i++)
      aSO_var->Pxx.data[j*lncols + i] = Pxx[j][i];

  aSO_var->Features.length(scene->get_no_features());
  vector<Feature *> ListOfFeatures = scene->get_feature_list_noconst();
  int r = ListOfFeatures.size();
  ODEBUG("ListOfFeatures :"<< r);
  for(unsigned int l=0;l<ListOfFeatures.size();l++)
    {
      VNL::Vector<double> h;
      VNL::Vector<double> z;
      VNL::Vector<double> y;
      VNL::Matrix<double> S;
      VNL::Matrix<double> Pyy;

      ODEBUG("Feature "<< l);
      y = ListOfFeatures[l]->get_y();
      aSO_var->Features[l].y.data.length(y.size());
      aSO_var->Features[l].y.nrows = y.size();
      aSO_var->Features[l].y.ncols = 1;
      for(unsigned int i=0;i<y.size();i++)
	{
	  aSO_var->Features[l].y.data[i] = y[i];
	}



      Pyy = ListOfFeatures[l]->get_Pyy();
      aSO_var->Features[l].Pyy.data.length(Pyy.size());
      aSO_var->Features[l].Pyy.nrows = Pyy.Rows();
      aSO_var->Features[l].Pyy.ncols = lncols = Pyy.Cols();
      for(unsigned int j=0;j<Pyy.Rows();j++)
	for(unsigned int i=0;i<Pyy.Cols();i++)
	  aSO_var->Features[l].Pyy.data[j*lncols + i] = Pyy[j][i];

      aSO_var->Features[l].label = ListOfFeatures[l]->get_label();
      aSO_var->Features[l].SuccessfulMeasurementFlag = ListOfFeatures[l]->get_successful_measurement_flag();
      aSO_var->Features[l].SelectedFlag = ListOfFeatures[l]->get_selected_flag();

      h = ListOfFeatures[l]->get_h();
      aSO_var->Features[l].h.data.length(h.size());
      aSO_var->Features[l].h.nrows = h.size();
      aSO_var->Features[l].h.ncols = 1;
      for(unsigned int i=0;i<h.size();i++)
	aSO_var->Features[l].h.data[i] = h[i];


      z = ListOfFeatures[l]->get_z();
      aSO_var->Features[l].z.data.length(z.size());
      aSO_var->Features[l].z.nrows = z.size();
      aSO_var->Features[l].z.ncols = 1;
      for(unsigned int i=0;i<z.size();i++)
	{
	  aSO_var->Features[l].z.data[i] = z[i];
	}

      S = ListOfFeatures[l]->get_S();
      aSO_var->Features[l].S.data.length(S.size());
      aSO_var->Features[l].S.nrows = S.Rows();
      aSO_var->Features[l].S.ncols =  lncols = S.Cols();
      for(unsigned int j=0;j<S.Rows();j++)
	for(unsigned int i=0;i<S.Cols();i++)
	  aSO_var->Features[l].S.data[j*lncols + i] = S[j][i];



      RobotPatchFeature * aRPF =  (RobotPatchFeature *)ListOfFeatures[l]->get_identifier();

      //      ImageMonoExtraData * p = (ImageMonoExtraData *)ListOfFeatures[l]->get_identifier();
      VW::ImageMono<unsigned char> *p = aRPF->big_image;

#if 1
      {
	static int local_counter = 0;
	char Buffer[1024];
	bzero(Buffer,1024);
	sprintf(Buffer,"/tmp/patch_%06d.pgm",local_counter++);
	p->WriteImage(Buffer);
      }
#endif
      int patchw = p->GetWidth();
      int patchh = p->GetHeight();
      unsigned char *pbuf = (unsigned char *)p->GetRawBuffer();

      aSO_var->Features[l].Identifier.length(patchw*patchh+2*sizeof(int));

      unsigned char *ppw = (unsigned char *)&patchw;
      unsigned int loffset = 0;

      for(unsigned int i=0;i<sizeof(int);i++)
	aSO_var->Features[l].Identifier[i] = ppw[i];
      loffset+=sizeof(int);

      unsigned char *pph = (unsigned char *)&patchh;
      for(unsigned int i=0;i<sizeof(int);i++)
	aSO_var->Features[l].Identifier[i+loffset] = pph[i];
      loffset+=sizeof(int);


      for(int j=0;j<patchh;j++)
	for( int i=0;i<patchw;i++)
	  aSO_var->Features[l].Identifier[loffset+j*patchw+i] =pbuf[j*patchw+i];

    }
  return 0;
}


void HRP2SingleCameraSLAMProcess::GetCorbaConnectionToGGAAplugin()
{


  ODEBUG(" Try to connect to GGAA");

#if 0
  CORBA::Object_var ns;
  try {
    ns = m_orb -> resolve_initial_references("NameService");
  }
  catch (const CORBA::ORB::InvalidName&) {
    cerr << "LLVS: can't resolve `NameService'" << endl;
  }
  catch(...)
    {
      cerr << "LLVS: can't resolve `NameService'" << endl;
      return;
    }
  if(CORBA::is_nil(ns)) {
    cerr <<"LLVS : `NameService' is a nil object reference"
	<< endl;
  }

  CosNaming::NamingContext_var rootnc;
  try {
    rootnc = CosNaming::NamingContext::_narrow(ns);
    if(CORBA::is_nil(rootnc)) {
      cerr << "LLVS: `NameService' is not a NamingContext object reference"
	   << endl;
    }
  }
  catch(...)
    {
      cerr << "LLVS: 'Unable to connect to name service" << endl;
      return;
    }
#endif
  CosNaming::Name ncName;
  ncName.length(1);
  ncName[0].kind = CORBA::string_dup("");

#if 0
#ifdef ORBIXE
  cxt = rootnc;

#else
  ncName[0].id = CORBA::string_dup("openhrp");
  try {
    cxt = CosNaming::NamingContext::_narrow(rootnc -> resolve(ncName));
  } catch (const CosNaming::NamingContext::NotFound&) {
    cerr << "openhrp context not found" << endl;
    cxt = rootnc -> new_context();
    rootnc -> rebind_context(ncName, cxt);
  }
#endif
#endif

  if (CORBA::is_nil(m_cxt))
    return;

  ncName[0].id = CORBA::string_dup("ggaa");

  CORBA::Object_ptr anObject=0;

  try
    {
      anObject = m_cxt->resolve(ncName);
    }
  catch(...)
    {
      ODEBUG("GGAA not found");
    }

  try{
    m_GyroAndAccelerometer = GyroAndAccelerometerServer::_narrow(anObject);
  }catch(...){
    ODEBUG("GGAA not narrowed");
  }

  if (!CORBA::is_nil(m_GyroAndAccelerometer))
    {
    }
  else
    {
      m_GyroFlag = false;
      m_WaistVelocityFlag = false;
    }
  ODEBUG("Success in connecting to GGAA");
}

int HRP2SingleCameraSLAMProcess::GetGyroAcceleroFromTimeStamp(double lGyro[3],
							      double lAccelerometer[3],
							      double timeref,
							      double lWaistVelocity[2],
							      double lPosOrientation[8])
{
  static int lcounter=0;
  double time1= 0.0,time2=0.0;
  double timediff = 1.0,r=-1.0,r2=-1.0,timediff2=1.0;
  double timeref2= timeref; // The 30 ms futur.
  double lPosOrientationPast[7]={-1.0,-1.0,-1.0,0.0,0.0,0.0,-1.0};;
  int index1=-1, index2=-1;
  // The 60 ms delay
  timeref += 0.06;


  double timedelay;
  ODEBUG("Going through int HRP2SingleCameraSLAMProcess::GetGyroAcceleroFromTimeStamp");
  if (!CORBA::is_nil(m_GyroAndAccelerometer))
    {

      seqGyroAndAccelerometerOutput_var aseqGGAAO;

      ODEBUG("Going through int HRP2SingleCameraSLAMProcess::GetGyroAcceleroFromTimeStamp step 1");
      try
	{

	  // This part take into account the delay between the two computers.
	  struct timeval timebegin,timeend;
	  double dtimebegin, dtimeend;
	  gettimeofday(&timebegin,0);

	  // Make the request.
	  CORBA::Double PrevTimestamp;
	  m_GyroAndAccelerometer->getGyroAndAccelerometer(aseqGGAAO,PrevTimestamp);


	  gettimeofday(&timeend,0);
	  dtimebegin = timebegin.tv_sec + 0.000001 * timebegin.tv_usec;
	  dtimeend = timeend.tv_sec + 0.000001 * timeend.tv_usec;
	  timedelay = 0.5 * (dtimebegin-dtimeend) +
	    PrevTimestamp - dtimebegin;
	  timeref += timedelay;
	  timeref2 += timedelay;
	}
      catch (...)
	{
	  cout << "Error while accessing GyroAndAccelerometer plugin "<<endl;

	  return -1;
	}

      ODEBUG("Going through int HRP2SingleCameraSLAMProcess::GetGyroAcceleroFromTimeStamp step 2");
      for(unsigned int j=0;j<60;j++)
	{

	  if ((r=fabs(aseqGGAAO[j].timestamp-timeref)) < timediff)
	    {
	      timediff = r;
	      time1 = aseqGGAAO[j].timestamp;
	      for(unsigned int k=0;k<3;k++)
		{
		  lGyro[k] = aseqGGAAO[j].Gyro[k];
		  lAccelerometer[k]  =aseqGGAAO[j].Accelerometer[k];
		}
	      ///	      cout << "From GGAA " ;
	      for(unsigned int k=0;k<2;k++)
		{
		  //		  cout << aseqGGAAO[j].WaistVelocity[k] << " " ;
		  lWaistVelocity[k] = aseqGGAAO[j].WaistVelocity[k];
		}
	      //	      cout << endl;
	      // Take the orientation of the camera.
	      for(unsigned int k=0;k<4;k++)
		{
		  lPosOrientation[k] = aseqGGAAO[j].HeadOrientation[k];
		}
	      lPosOrientation[7] =aseqGGAAO[j].HeadOrientation[4];
	      // Take the position of the camera.
	      for(unsigned int k=0;k<3;k++)
		{
		  lPosOrientation[k+4] = aseqGGAAO[j].HeadPosition[k][3];
		}
	      //	      lPosOrientation[5]= aseqGGAAO[j].HeadPosition[0][1];
	      index1=j;
	    }

	}

      if (m_Verbosity>=3)
	{
	  cout << " " << timeref - time1 << " " << timeref - time2 << " " << index1 << " " << index2<< endl;
	  index1++;
	  if (index1>60)
	    index1=0;
	  cout << " " << timeref - aseqGGAAO[index1].timestamp << endl;


	  cout << r << " " << r2 << " " << timediff << " "<< timediff2 << endl;
	  cout << "lPosOrientation + 0.3 :" ;
	  for(unsigned int k=0;k<7;k++)
	    cout << lPosOrientationPast[k] << " ";
	  cout << endl;
	  cout << "lPosOrientation + 0.6 :" ;
	  for(unsigned int k=0;k<7;k++)
	    cout << lPosOrientation[k] << " ";
	  cout << endl;

	  cout << 180*lPosOrientation[5]/M_PI << " " << 180*lPosOrientationPast[5]/M_PI<< " " <<
	    (lPosOrientation[5]-lPosOrientationPast[5])/0.03 << endl;
	  static double IncTheta=0;


	  ODEBUG("Going through int HRP2SingleCameraSLAMProcess::GetGyroAcceleroFromTimeStamp step 3 : 60");

	  ODEBUG("lcounter: "<<lcounter);

	  {
	    //	  if (lcounter==30)
	    {
	      for(unsigned int j=0;j<60;j++)
		{
		  cout << "Measurement "<<j<< endl;
		  cout << " Gyro: "<<endl;
		  cout << "  " << aseqGGAAO[j].Gyro[0]
		       << "  " << aseqGGAAO[j].Gyro[1]
		       << "  " << aseqGGAAO[j].Gyro[2] << endl;
		  cout << " Accelerometer: "<<endl;
		  cout << "  " << aseqGGAAO[j].Accelerometer[0]
		       << "  " << aseqGGAAO[j].Accelerometer[1]
		       << "  " << aseqGGAAO[j].Accelerometer[2] << endl;
		  cout << " Waist velocity: " << endl;
		  cout << "  " << aseqGGAAO[j].WaistVelocity[0]
		       << "  " << aseqGGAAO[j].WaistVelocity[1] << endl;
		  cout << " TimeStamp:" <<endl;
		  cout << " Position- Orientation " << endl;
		  for(unsigned int k=0;k<7;k++)
		    cout << lPosOrientation[k]<< " ";
		  cout << endl;
		  printf("  %60.30f\n",aseqGGAAO[j].timestamp);
		}
	      lcounter =0;
	    }
	    lcounter++;
	  }
	}
    }
  else
    {
      ODEBUG("NOT CONNECTED TO GGAA");
    }
  return 0;
}

int HRP2SingleCameraSLAMProcess::SetFindFeaturesFromWideImage(FindFeaturesFromWideImage * aFFFWI,
							      double aheadTorg[16])
{
  m_FFFWI = aFFFWI;
  for(int i=0;i<16;i++)
    m_headTorg[i]=aheadTorg[i];
  return -1;
}

