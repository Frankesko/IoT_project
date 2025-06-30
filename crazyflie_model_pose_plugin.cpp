#include "crazyflie_model_pose_plugin.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace gazebo
{
  CrazyflieModelPosePlugin::CrazyflieModelPosePlugin() {}

  CrazyflieModelPosePlugin::~CrazyflieModelPosePlugin() {}

  void CrazyflieModelPosePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    model_ = _model;

    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);
    ros_node_ = rclcpp::Node::make_shared("crazyflie_model_pose_plugin_node");

    pose_pub_ = ros_node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/model_pose", 10);

    // Leggi il periodo di pubblicazione dal SDF, default 0.1s (10Hz)
    pub_period_sec_ = 0.1;
    if (_sdf->HasElement("publish_period")) {
      pub_period_sec_ = _sdf->Get<double>("publish_period");
    }
    last_pub_time_ = ros_node_->now();

    updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&CrazyflieModelPosePlugin::OnUpdate, this));

    std::thread([this]() { rclcpp::spin(ros_node_); }).detach();
    gzdbg << "CrazyflieModelPosePlugin loaded and publishing on /model_pose\n";
  }

  void CrazyflieModelPosePlugin::OnUpdate()
  {
    rclcpp::Time now = ros_node_->now();
    if ((now - last_pub_time_).seconds() < pub_period_sec_) {
      return;
    }
    last_pub_time_ = now;

    ignition::math::Pose3d pose = model_->WorldPose();

    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = ros_node_->get_clock()->now();
    msg.header.frame_id = "world";
    msg.pose.position.x = pose.Pos().X();
    msg.pose.position.y = pose.Pos().Y();
    msg.pose.position.z = pose.Pos().Z();
    msg.pose.orientation.x = pose.Rot().X();
    msg.pose.orientation.y = pose.Rot().Y();
    msg.pose.orientation.z = pose.Rot().Z();
    msg.pose.orientation.w = pose.Rot().W();

    pose_pub_->publish(msg);
  }

  GZ_REGISTER_MODEL_PLUGIN(CrazyflieModelPosePlugin)
}