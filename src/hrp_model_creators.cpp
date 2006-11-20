#include <iostream>

#include "hrp_model_creators.h"

#include "models_camera_height.h"
#include "models_threed_gyro.h"
#include "models_waist_velocity.h"
#include "models_orientation.h"

/** Returns an instance of the requested internal measurement model
    (using <code>new</code>). Returns NULL if there was no match.
**/
Internal_Measurement_Model* HRP_Internal_Measurement_Model_Creator::
create_model(const std::string& type, Motion_Model *motion_model)
{
  // Try creating each model that we can and see if the name is the same
  std::cout << "Creating a " << type << " internal measurement model" 
            << std::endl;
  Internal_Measurement_Model* pModel;

  pModel = new
    Camera_Height_Internal_Measurement_Model(motion_model);
  if(pModel->internal_type == type)
    return pModel;
  else
    delete pModel;
  
  pModel = new
    ThreeD_Gyro_Internal_Measurement_Model(motion_model);
  if(pModel->internal_type == type)
    return pModel;
  else
    delete pModel;
  
  pModel = new
    Waist_Velocity_Internal_Measurement_Model(motion_model);
  if(pModel->internal_type == type)
    return pModel;
  else
    delete pModel;
  
  pModel = new Orientation_Internal_Measurement_Model(motion_model);
  if(pModel->internal_type == type)
    return pModel;
  else
    delete pModel;

  return NULL;
}
