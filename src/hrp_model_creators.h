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

#endif
