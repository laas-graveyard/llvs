#ifndef _hrp_model_creators_h_
#define _hrp_model_creators_h_

#include <Scene/models_base.h>

// This is a class which can serve internal motion models (so that the 
// set being used can be changed at run-time rather than compile-time)

class HRP_Internal_Measurement_Model_Creator 
  : public Internal_Measurement_Model_Creator
{
public:
  /// Constructor.
  HRP_Internal_Measurement_Model_Creator() {} 
  
  ~HRP_Internal_Measurement_Model_Creator() {} 

  Internal_Measurement_Model* create_model(const std::string& type,
					   Motion_Model *motion_model);
};

/** 
Class to create an instance of a motion model class given its type string. 
Default creator for HRPMonoSLAM; make your own if you need other models.
@ingroup gMonoSLAM
**/
class HRP2MonoSLAM_Motion_Model_Creator : public Motion_Model_Creator
{
 public:
  HRP2MonoSLAM_Motion_Model_Creator() {} ///< Constructor. Does nothing.
  ~HRP2MonoSLAM_Motion_Model_Creator() {} ///< Destructor. Does nothing.

  Motion_Model* create_model(const std::string& type);
};

#endif
