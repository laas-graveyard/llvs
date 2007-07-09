#include "Scene/control_general.h"
#include "monoslamhrp.h"

#ifdef VERBOSE
#undef VERBOSE
#endif
#define VERBOSE false

#include <VNL/sample.h>


#define ODEBUG2(x)
#define ODEBUG3(x) cerr << "MonoSLAMHRP:" << x << endl

#if 0
#define ODEBUG(x) cerr << "MonoSLAMHRP:" <<  x << endl
#else
#define ODEBUG(x) 
#endif


MonoSLAMHRP::MonoSLAMHRP(const std::string& initialisation_file,
			 Motion_Model_Creator *mm_creator,
			 Feature_Measurement_Model_Creator *fmm_creator,
			 Internal_Measurement_Model_Creator *imm_creator,
			 unsigned int number_of_features_to_select,
			 unsigned int number_of_features_to_keep_visible,
			 unsigned int max_features_to_init_at_once,
			 double min_lambda,
			 double max_lambda,
			 unsigned int number_of_particles,
			 double standard_deviation_depth_ratio,
			 unsigned int min_number_of_particles,
			 double prune_probability_threshold,
			 unsigned int erase_partially_init_feature_after_this_many_attempts)

  : MonoSLAMInterface(initialisation_file,
		      mm_creator,
		      fmm_creator,
		      imm_creator,
		      number_of_features_to_select,
		      number_of_features_to_keep_visible,
		      max_features_to_init_at_once,
		      min_lambda,
		      max_lambda,
		      number_of_particles,
		      standard_deviation_depth_ratio,
		      min_number_of_particles,
		      prune_probability_threshold,
		      erase_partially_init_feature_after_this_many_attempts) 
{
  m_InitialPose.Resize(7);
  m_InitializationPhase = 1;
  m_prev_wpg_pose.Resize(7);
}

bool MonoSLAMHRP::GoOneStepHRP(const VW::ImageMono<unsigned char> *new_image,
			       double delta_t,
			       bool currently_mapping_flag,
			       bool use_vision_flag,
			       bool use_gyro_flag,
			       bool use_waist_velocity_flag,
			       bool use_camera_height_flag,
			       bool use_orientation_flag,
			       const VNL::Vector<double> &current_gyro,
			       const VNL::Vector<double> &current_waist_velocity,
			       const VNL::Vector<double> &current_camera_height,
			       const VNL::Vector<double> &current_orientation,
			       const VNL::Vector<double> &current_control)
{
  ODEBUG3("Beginning " << scene->get_xv() );
  ODEBUG(current_waist_velocity(0) << " " <<
	  current_waist_velocity(1) << " " <<
	  current_waist_velocity(2) );
  // Initialization phase of the state.
  // Ignore the monoslam.ini file, and use the build in
  // below code.
  if (m_InitializationPhase)
    {
      ODEBUG3("Enter initialization procedure");
      VW::Quaternion qco(current_orientation(2),
			 current_orientation(3),
			 current_orientation(1),
			 current_orientation(0));
      // cout << "QCo: 1 " <<qco << endl;
      VW::Quaternion qR(0.5,0.5,0.5,0.5);
      VW::Quaternion qWR;
      // cout << "QCo: 2 " << qco << endl;
      qWR = qR * qco;
      
      VNL::Vector<double> lxv = scene->get_xv();
      
      /*      double a[7] = {current_orientation(4),
		     current_orientation(5),
		     current_orientation(6),
		     qWR.R(),qWR.X(),qWR.Y(),qWR.Z()}; */
      double a[7] = {lxv(0),
		     lxv(1),
		     lxv(2),
		     qWR.R(),qWR.X(),qWR.Y(),qWR.Z()};
      
      // Get total state and covariance
      int scene_state_size = scene->get_total_state_size();
      VNL::Vector<double> x(scene_state_size);
      VNL::Matrix<double> P(scene_state_size, scene_state_size);
      ODEBUG3("Before constructing total state and covariance matrix.");
      scene->construct_total_state_and_covariance(x, P);
      ODEBUG3("right after constructing total state and covariance matrix.");
      VNL::Vector<double> V(13,0.0);
      for(unsigned int i=0;i<7;i++)
	{
	  V(i) = a[i];
	  m_InitialPose(i) = a[i];
	}

      // Nullify Z for the initial pose.
      m_InitialPose(2) = 0.0;

      V(9) = -0.0001;
      V(12) = 0.0001;
      VNL::Matrix<double> M(13,13,0.0);

      ODEBUG3("V:" << V <<endl <<
	      "M:" << M <<endl);
      x.Update(V, 0);

      P.Update(M, 0, 0);
      
      ODEBUG3("x:" << x <<endl <<
	      "P:" << P <<endl);
	    
      scene->fill_state_and_covariance(x, P);
            
      for(unsigned int i=0;i<7;i++)
	m_prev_wpg_pose[i] = a[i];
    }

  ODEBUG("time at the beginning " << timer1);  
  ODEBUG3( "GoOneStepHRP " <<  current_orientation );
  // Nullify image selection
  robot->nullify_image_selection();
  ODEBUG3( "After nullify image selection" );
  init_feature_search_region_defined_flag = false;

  ODEBUG3("Start Monoslam " << scene->get_xv());
  if (use_gyro_flag) 
    {
      ODEBUG("Use gyro ");
      // First do internal measurement for last step
      scene->predict_internal_measurement(0);
      
      VNL::Vector<double> omegaC_meas(3);
      double a[] = {current_gyro(1),
		    current_gyro(2),
		    current_gyro(0)};
      omegaC_meas.CopyIn(a);
      ODEBUG("Gyro: " << omegaC_meas );
      scene->successful_internal_measurement(omegaC_meas, 0);
      kalman->update_filter_internal_measurement_slow(scene, 
						      0);
      
    }
  ODEBUG3("After gyro " << scene->get_xv() );
  ODEBUG("GoOneStepHRP " );
  // cout << current_orientation << endl;

  //  cout << "After gyro" << scene->get_xv() << endl;
  if (use_waist_velocity_flag) {
    // First do internal measurement for last step
    scene->predict_internal_measurement(1);

    VNL::Vector<double> vC_meas(3);
    double a[] = {current_waist_velocity(1),
		  current_waist_velocity(2),
		  current_waist_velocity(0)};
    vC_meas.CopyIn(a);
    // cout << "Waist velocity " << vC_meas << endl;
    scene->successful_internal_measurement(vC_meas, 1);
    kalman->update_filter_internal_measurement_slow(scene, 
						    1);
  }
  // cout << "GoOneStepHRP " << endl;
  // cout << current_orientation << endl;

  ODEBUG3("After waist velocity " << scene->get_xv() );
  if (use_camera_height_flag) {
    // First do internal measurement for last step
    scene->predict_internal_measurement(2);

    scene->successful_internal_measurement(current_camera_height, 2);
    kalman->update_filter_internal_measurement_slow(scene, 
						    2);
  }
  ODEBUG3("after camera height " << scene->get_xv() );
  // cout << "GoOneStepHRP " << endl;
  // cout << current_orientation << endl;

#if 1
  if(use_orientation_flag)
    {
      ODEBUG("Inside orientation flag");
      scene->predict_internal_measurement(3);
      ODEBUG("Went up to here ");
      ODEBUG3( " Current Orientation :" << current_orientation);
      VW::Quaternion qco(current_orientation(2),
			 current_orientation(3),
			 current_orientation(1),
			 current_orientation(0));
      // cout << "QCo: 1 " <<qco << endl;
      VW::Quaternion qR(0.5,0.5,0.5,0.5);
      VW::Quaternion qWR;
      // cout << "QCo: 2 " << qco << endl;
      qWR = qR* qco;

      // cout << "qWR: " << qWR << endl;
      VNL::Vector<double> vC_meas(7);
      double a[7] = {current_orientation(4),
		     current_orientation(5),
		     current_orientation(6),
		     qWR.R(),qWR.X(),qWR.Y(),qWR.Z()};

      vC_meas.CopyIn(a);
      /* cout << current_orientation(3) << " " 
	 << current_orientation(4) << " "
	 << current_orientation(5) << endl;  */
      // cout << "Orientation from PG : " <<qco.R() << " " << qco.X()<< " " << qco.Y() << " " << qco.Z() << endl;
      // cout << "Orientation from WR : " <<qWR.R() << " " << qWR.X()<< " " << qWR.Y() << " " << qWR.Z() << endl;
      // cout << "Orientation from odometrbby : " <<vC_meas << endl;
      scene->successful_internal_measurement(vC_meas, 3);
      // cout << "Before update " << scene->get_xv() << endl;
      kalman->update_filter_internal_measurement_slow(scene,3);
      // cout << "After update " << scene->get_xv() << endl;

    
    }
#endif

  ODEBUG("time after orientation " << timer1);
  ODEBUG3("after orientation " << scene->get_xv() );


  int lcontrolsize = scene->get_motion_model()->CONTROL_SIZE;
  VNL::Vector<double> u(lcontrolsize,0.0);
  if (lcontrolsize==7)
    {
      // Control vector of accelerations
      
      VW::Quaternion qco(current_orientation(2),
			 current_orientation(3),
			 current_orientation(1),
			 current_orientation(0));
      // cout << "QCo: 1 " <<qco << endl;
      VW::Quaternion qR(0.5,0.5,0.5,0.5);
      VW::Quaternion qWR2,qR1R2;
      cout << "QCo: " << qco << endl;
      qWR2 = qR* qco;

      VW::Quaternion qWR1(m_prev_wpg_pose(4),
			  m_prev_wpg_pose(5),
			  m_prev_wpg_pose(6),
			  m_prev_wpg_pose(3));

      VW::Quaternion qInitial(m_InitialPose(4),
			      m_InitialPose(5),
			      m_InitialPose(6),
			      m_InitialPose(3));
			      
      ODEBUG("qWR1: " << qWR1 << endl << "qWR2: " <<  qWR2);
      qR1R2 = qWR1.Inverse()*qWR2;

      u(3) = qR1R2.GetR();
      u(4) = qR1R2.GetX();
      u(5) = qR1R2.GetY();
      u(6) = qR1R2.GetZ();

      ODEBUG("u: " << u);
      {

	double a[7] = {current_orientation(4),
		       current_orientation(5),
		       current_orientation(6),
		       qWR2.R(),qWR2.X(),qWR2.Y(),qWR2.Z()};

	if (m_InitializationPhase)
	  {
	    for(unsigned int i=0;i<3;i++)
	      u(i) = a[i]  - m_prev_wpg_pose(i) + m_InitialPose(i);
	  }
	else
	  {
	    for(unsigned int i=0;i<3;i++)
	      u(i) = a[i]  - m_prev_wpg_pose(i) ;
	  }
	  
	    
	for(int i=0;i<7;i++)
	  m_prev_wpg_pose(i) = a[i];

      }

        ODEBUG("End of u computation");
	
    }

  ODEBUG("After set control");
  kalman->predict_filter_fast(scene, u, delta_t);
  ODEBUG(scene->get_Pxx());
  
  ODEBUG3("Before auto select n features.");
  unsigned int number_of_visible_features = 
    scene->auto_select_n_features(NUMBER_OF_FEATURES_TO_SELECT);

  //  scene->print_selected_features();

  if (scene->get_no_selected() != 0)
    {
      scene->predict_measurements(); 
      ODEBUG3( "Time after predicting measurements " << timer1 );

      if (use_vision_flag) {
	// Calls function in control_general.cc
	make_measurements(scene, sim_or_rob);

	ODEBUG3( "Time after making measurements " << timer1);

	if (scene->get_successful_measurement_vector_size() != 0) {
	  kalman->total_update_filter_slow(scene);

	  ODEBUG3( "Time after total_update_filter " << timer1 );

	  scene->normalise_state();
      
	  ODEBUG3("Time after normalise_state " << timer1 );
	}
      }
    }  

  ODEBUG3("HEre .");
  scene->delete_bad_features();

  // Let's enforce symmetry of covariance matrix...
  // Add to transpose and divide by 2
  VNL::Matrix<double> Pxx(scene->get_total_state_size(),
			  scene->get_total_state_size());
  scene->construct_total_covariance(Pxx);
  VNL::Matrix<double> PxxT = Pxx.Transpose();

  Pxx.Update(Pxx * 0.5 + PxxT * 0.5);
  scene->fill_covariances(Pxx);

  // Look at camera speed estimate
  double speedx = current_waist_velocity(0);
  double speedy = current_waist_velocity(1);
  double speedz = current_waist_velocity(2);;
  double speed = sqrt(speedx * speedx + 
		      speedy * speedy + 
		      speedz * speedz);
  if (VERBOSE) cout << "Camera speed " << speed << " ms^-1 " << endl;
  /*
    cout << "Orientation from EKF :" << scene->get_xv()(3) << " "
    << scene->get_xv()(4) << " "
    << scene->get_xv()(5) << " "
    << scene->get_xv()(6) << endl;
  */

  ODEBUG( "Speed:" << speed << " " 
	  << current_waist_velocity(0) 
	  << " " << current_waist_velocity(1) 
	  << " " << current_waist_velocity(2)  );
  ODEBUG("vision flag : " << use_vision_flag 
	  <<  " mapping flag : " << currently_mapping_flag);
  if (use_vision_flag) 
    {
      //if (speed>0.1 && currently_mapping_flag) 
      if (currently_mapping_flag) 
	{
	  ODEBUG("The CONDITION HAS BEEN FULFILLED");
	  /*
	    cout <<  number_of_visible_features <<  " "
	    << scene->get_feature_init_info_vector().size() <<
	    endl;
	  */
	  
	  if (number_of_visible_features < NUMBER_OF_FEATURES_TO_KEEP_VISIBLE && 
	      scene->get_feature_init_info_vector().size() < 
	      (unsigned int) (MAX_FEATURES_TO_INIT_AT_ONCE)) 
	    {
	      AutoInitialiseFeature(u, delta_t);
	    }
	}

      if (VERBOSE) cout << "Time after matching point features: " 
			<< timer1 << endl;
      
      MatchPartiallyInitialisedFeatures();
      
      if (VERBOSE) cout << "Time after matching partially init. feature: " 
			<< timer1 << endl;
    }


  //  scene->output_state_to_file();
  //  cout << "End of EKF : " << scene->get_xv() << endl; 
  if (isnan(scene->get_xv()[0]))
    exit(0);

  m_InitializationPhase = 0;
  
  ODEBUG("End");
  return true;
}

Scene_Single * MonoSLAMHRP::GetScene()
{
  return scene;
}

void MonoSLAMHRP::SetInitializationPhase(unsigned char anInitialization)
{
  m_InitializationPhase = anInitialization;
}

unsigned char MonoSLAMHRP::GetInitializationPhase()
{
  return m_InitializationPhase;
}
