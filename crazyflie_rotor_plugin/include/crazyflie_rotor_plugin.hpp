#ifndef CRAZYFLIE_ROTOR_PLUGIN_HPP
#define CRAZYFLIE_ROTOR_PLUGIN_HPP

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace gazebo
{
  class CrazyflieRotorPlugin : public ModelPlugin
  {
    public:
      CrazyflieRotorPlugin();
      virtual ~CrazyflieRotorPlugin();
      void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

    private:
      void OnRosMsg(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
      void OnUpdate();

      physics::ModelPtr model_;
      event::ConnectionPtr updateConnection_;
      std::vector<physics::LinkPtr> rotors_;
      std::vector<float> rotor_velocities_;
      physics::LinkPtr body_link_;
      rclcpp::Node::SharedPtr ros_node_;
      rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
  };
}
#endif 