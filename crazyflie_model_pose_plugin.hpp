#ifndef CRAZYFLIE_MODEL_POSE_PLUGIN_HPP
#define CRAZYFLIE_MODEL_POSE_PLUGIN_HPP

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace gazebo
{
  class CrazyflieModelPosePlugin : public ModelPlugin
  {
    public:
      CrazyflieModelPosePlugin();
      virtual ~CrazyflieModelPosePlugin();
      void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

    private:
      void OnUpdate();

      physics::ModelPtr model_;
      event::ConnectionPtr updateConnection_;
      rclcpp::Node::SharedPtr ros_node_;
      rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
      rclcpp::Time last_pub_time_;
      double pub_period_sec_;
  };
}
#endif