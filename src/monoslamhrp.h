#ifndef _monoslamhrpglow_monoslamhrp_h_
#define _monoslamhrpglow_monoslamhrp_h_

#include <MonoSLAM/monoslaminterface.h>

// MonoSLAM class which just provides its own GoOneStep function
class MonoSLAMHRP : public MonoSLAMInterface
{
 public:
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
		      erase_partially_init_feature_after_this_many_attempts) {}

  bool GoOneStepHRP(const VW::ImageMono<unsigned char> *new_image,
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
		    const VNL::Vector<double> &current_control);
  
  Scene_Single * GetScene();


};


#endif
