#include <boost/bind.hpp>
#include "gazebo.hh"
#include "physics/Model.hh"
#include "common/CommonTypes.hh"

#include "math/Vector3.hh"
#include <stdio.h>

namespace gazebo
{   
  class ModelPush : public ModelPlugin
  {
   public:
    ModelPush()
    {
      printf("ModelPush ctor\n");
    }   
    
    void Load(physics::ModelPtr _parent, sdf::unball_robot _sdf)
    {
      printf("Load\n");

      // Store the pointer to the model
      this->model = _parent;
      this->model->SetLinearVel(math::Vector3(0, 0, 0));
      this->model->SetAngularVel(math::Vector3(0, 0, 0);
    }

    // Pointer to the model
    private: physics::ModelPtr model;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
