#include "crazyflie_rotor_plugin.hpp"
#include <gazebo/common/Events.hh>
#include <gazebo/physics/Link.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace gazebo
{
  CrazyflieRotorPlugin::CrazyflieRotorPlugin() {}

  CrazyflieRotorPlugin::~CrazyflieRotorPlugin() {}

  void CrazyflieRotorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    model_ = _model;
    // Assume i link si chiamano rotor1, rotor2, rotor3, rotor4
    rotors_.resize(4);
    rotor_velocities_ = {0, 0, 0, 0};
    for (int i = 0; i < 4; ++i)
    {
      std::string rotor_name = "rotor" + std::to_string(i+1);
      rotors_[i] = model_->GetLink(rotor_name);
      if (!rotors_[i])
        gzerr << "Rotor link " << rotor_name << " not found!\n";
    }

    // Inizializza ROS2 node
    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);
    ros_node_ = rclcpp::Node::make_shared("crazyflie_rotor_plugin_node");

    // Subscriber per i comandi dei rotori
    sub_ = ros_node_->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/sim_crazyflie/rotor_speeds", 10,
      std::bind(&CrazyflieRotorPlugin::OnRosMsg, this, std::placeholders::_1)
    );

    // Aggiorna la fisica ad ogni step
    updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&CrazyflieRotorPlugin::OnUpdate, this)
    );

    std::thread([this]() { rclcpp::spin(ros_node_); }).detach();
    gzdbg << "CrazyflieRotorPlugin loaded and listening on /sim_crazyflie/rotor_speeds\n";
  }

  void CrazyflieRotorPlugin::OnRosMsg(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() == 4)
      for (int i = 0; i < 4; ++i)
        rotor_velocities_[i] = msg->data[i];
  }

  void CrazyflieRotorPlugin::OnUpdate()
  {
      // Non applicare nulla se tutti i rotori sono a zero
      bool all_zero = true;
      for (int i = 0; i < 4; ++i)
          if (rotor_velocities_[i] != 0) all_zero = false;
      if (all_zero) return;

      // Parametri fisici (aumentati per flip più efficace)
      double k_thrust = 3e-6; // coefficiente di spinta (aumentato da 1e-6)
      double k_torque = 6e-7; // coefficiente di coppia (aumentato da 2e-7)

      double total_thrust = 0.0;

      // Applica la forza e la coppia a ciascun rotore
      for (int i = 0; i < 4; ++i)
      {
          if (!rotors_[i]) continue;
          double omega = rotor_velocities_[i];
          double thrust = k_thrust * omega * omega;
          double torque = k_torque * omega * omega;

          ignition::math::Vector3d force(0, 0, thrust);
          ignition::math::Vector3d torque_vec(0, 0, (i % 2 == 0 ? 1 : -1) * torque);

          rotors_[i]->AddRelativeForce(force);
          rotors_[i]->AddRelativeTorque(torque_vec);

          total_thrust += thrust;
      }

      // Applica la spinta totale al link body (in world frame)
      if (body_link_)
      {
          ignition::math::Vector3d body_force(0, 0, total_thrust);
          body_link_->AddForce(body_force);
      }
  }

  GZ_REGISTER_MODEL_PLUGIN(CrazyflieRotorPlugin)
} 