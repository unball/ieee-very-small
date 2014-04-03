/**
 * @file   simulator_plugin.cpp
 * @author Icaro da Costa Mota
 * @author Matheus Vieira Portela
 * @date   31/03/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  Simulator plugin to control robots and get the simulation state
 * 
 * This plugin is connected to each robot, which can be controlled through it.
 */

#include <gazebo.hh>
#include <physics/Model.hh>

#include <math/Vector3.hh>

#include <ros/ros.h>

using namespace gazebo;

class RobotPlugin : public ModelPlugin
{
  public:
    RobotPlugin()
    {
        ROS_INFO("Robot plugin constructor");
    }   

    void Load(physics::ModelPtr parent, sdf::ElementPtr sdf_element)
    {
        ROS_INFO("Robot plugin load");

        // Store the pointer to the model
        this->model_ = parent;
        this->model_->SetLinearVel(math::Vector3(0, 0, 0));
        this->model_->SetAngularVel(math::Vector3(0, 0, 0));
    }
    
  private:
    physics::ModelPtr model_; // Pointer to the model
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(RobotPlugin)
