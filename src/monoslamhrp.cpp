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

  ODEBUG("time at the beginning " << timer1);  
  ODEBUG( "GoOneStepHRP " <<  current_orientation );
  // Nullify image selection
  robot->nullify_image_selection();

  init_feature_search_region_defined_flag = false;

  ODEBUG("Start Monoslam " << scene->get_xv());
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

  ODEBUG("After waist velocity " << scene->get_xv() );
  if (use_camera_height_flag) {
    // First do internal measurement for last step
    scene->predict_internal_measurement(2);

    scene->successful_internal_measurement(current_camera_height, 2);
    kalman->update_filter_internal_measurement_slow(scene, 
						    2);
  }
  ODEBUG("after camera height " << scene->get_xv() );
  // cout << "GoOneStepHRP " << endl;
  // cout << current_orientation << endl;

 if(use_orientation_flag)
    {
      ODEBUG("Inside orientation flag");
      scene->predict_internal_measurement(3);
      ODEBUG("Went up to here ");
      ODEBUG(current_orientation << 
	     current_orientation(2) << " " <<
	     current_orientation(3) << " "  <<
	     current_orientation(1) << " " <<
	     current_orientation(0) );
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
            // cout << "Orientation from odometry : " <<vC_meas << endl;
      scene->successful_internal_measurement(vC_meas, 3);
            // cout << "Before update " << scene->get_xv() << endl;
      kalman->update_filter_internal_measurement_slow(scene,3);
            // cout << "After update " << scene->get_xv() << endl;

    
    }
 ODEBUG("time after orientation " << timer1);
 ODEBUG("after orientation " << scene->get_xv() );
  // Control vector of accelerations
  VNL::Vector<double> u(3);
  u.Fill(0.0);

  sim_or_rob->set_control(u, delta_t);

  kalman->predict_filter_fast(scene, u, delta_t);

  unsigned int number_of_visible_features = 
    scene->auto_select_n_features(NUMBER_OF_FEATURES_TO_SELECT);

  //  scene->print_selected_features();

  if (scene->get_no_selected() != 0)
  {
    scene->predict_measurements(); 
    if (VERBOSE) cout << "Time after predicting measurements " 
		      << timer1 << endl;

    if (use_vision_flag) {
      // Calls function in control_general.cc
      make_measurements(scene, sim_or_rob);

      if (VERBOSE) cout << "Time after making measurements " << timer1 << endl;

      if (scene->get_successful_measurement_vector_size() != 0) {
	kalman->total_update_filter_slow(scene);

	if (VERBOSE) cout << "Time after total_update_filter " << timer1 << endl;

	scene->normalise_state();
      
	if (VERBOSE) cout << "Time after normalise_state " << timer1 << endl;
      }
    }
  }  

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
  double speedx = scene->get_xv()(7);
  double speedy = scene->get_xv()(8);
  double speedz = scene->get_xv()(9);
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
  /* 
  cout << speedz << " " << speedx <<  " " << current_waist_velocity(0) 
       << " " << current_waist_velocity(1)  << endl;
  */
  if (use_vision_flag) {
    if (currently_mapping_flag) { 
      /*
      cout <<  number_of_visible_features <<  " "
      << scene->get_feature_init_info_vector().size() <<
      endl;
      */
      
      if (number_of_visible_features < NUMBER_OF_FEATURES_TO_KEEP_VISIBLE && 
	  scene->get_feature_init_info_vector().size() < 
	  (unsigned int) (MAX_FEATURES_TO_INIT_AT_ONCE)) {
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

  return true;
}

Scene_Single * MonoSLAMHRP::GetScene()
{
  return scene;
}
