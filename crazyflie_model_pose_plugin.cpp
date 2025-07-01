#include "crazyflie_model_pose_plugin.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace gazebo
{
  CrazyflieModelPosePlugin::CrazyflieModelPosePlugin() {}

  CrazyflieModelPosePlugin::~CrazyflieModelPosePlugin() {}

  void CrazyflieModelPosePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    model_ = _model;

    // Ottieni il link principale "body" per leggere le velocità
    body_link_ = model_->GetLink("body");
    if (!body_link_)
      gzerr << "[CrazyflieModelPosePlugin] Link 'body' non trovato!\n";

    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);
    ros_node_ = rclcpp::Node::make_shared("crazyflie_model_pose_plugin_node");

    pose_pub_ = ros_node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/model_pose", 10);
    twist_pub_ = ros_node_->create_publisher<geometry_msgs::msg::Twist>(
      "/model_twist", 10);

    // Leggi il periodo di pubblicazione dal SDF, default 0.01s (100Hz) per controllo fluido
    pub_period_sec_ = 0.01;
    if (_sdf->HasElement("publish_period")) {
      pub_period_sec_ = _sdf->Get<double>("publish_period");
    }
    last_pub_time_ = ros_node_->now();

    updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&CrazyflieModelPosePlugin::OnUpdate, this));

    std::thread([this]() { rclcpp::spin(ros_node_); }).detach();
    gzdbg << "CrazyflieModelPosePlugin loaded and publishing on /model_pose and /model_twist\n";
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

    // Pubblica la velocità lineare e angolare del body
    if (body_link_ && twist_pub_)
    {
      ignition::math::Vector3d lin_vel = body_link_->WorldLinearVel();
      ignition::math::Vector3d ang_vel = body_link_->WorldAngularVel();
      geometry_msgs::msg::Twist twist_msg;
      twist_msg.linear.x = lin_vel.X();
      twist_msg.linear.y = lin_vel.Y();
      twist_msg.linear.z = lin_vel.Z();
      twist_msg.angular.x = ang_vel.X();
      twist_msg.angular.y = ang_vel.Y();
      twist_msg.angular.z = ang_vel.Z();
      twist_pub_->publish(twist_msg);
    }
  }

  GZ_REGISTER_MODEL_PLUGIN(CrazyflieModelPosePlugin)
}