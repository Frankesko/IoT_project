#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <iostream>

namespace gazebo
{
  class CrazyflieLedPlugin : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr model, sdf::ElementPtr /*_sdf*/) override
    {
      try {
        this->model = model;
        std::string modelName = model->GetName();
        std::string worldName = model->GetWorld()->Name();

        this->gzNode = transport::NodePtr(new transport::Node());
        this->gzNode->Init(worldName);

        std::string topic = "~/model/" + modelName + "/link/led/visual/led_visual/material";
        std::cout << "[CrazyflieLedPlugin] Publishing to topic: " << topic << std::endl;
        this->ledPub = this->gzNode->Advertise<gazebo::msgs::Material>(topic);

        if (!rclcpp::ok()) {
          rclcpp::init(0, nullptr);
        }
        rosNode = std::make_shared<rclcpp::Node>("crazyflie_led_plugin");
        sub = rosNode->create_subscription<std_msgs::msg::String>(
          "/drone_led_status", 10,
          std::bind(&CrazyflieLedPlugin::LedCallback, this, std::placeholders::_1)
        );
        rosThread = std::thread([this]() { rclcpp::spin(this->rosNode); });
        rosThread.detach();
        std::cout << "[CrazyflieLedPlugin] Loaded and listening for /drone_led_status" << std::endl;
      } catch (const std::exception& e) {
        std::cerr << "[CrazyflieLedPlugin] Exception in Load: " << e.what() << std::endl;
      } catch (...) {
        std::cerr << "[CrazyflieLedPlugin] Unknown exception in Load!" << std::endl;
      }
    }

    void LedCallback(const std_msgs::msg::String::SharedPtr msg)
    {
      std::string color = msg->data;
      std::cout << "[CrazyflieLedPlugin] Received color: " << color << std::endl;
      gazebo::msgs::Material matMsg;
      if (color == "green") {
        SetColor(matMsg, 0, 1, 0, 1);
      } else if (color == "yellow") {
        SetColor(matMsg, 1, 1, 0, 1);
      } else if (color == "red") {
        SetColor(matMsg, 1, 0, 0, 1);
      } else {
        SetColor(matMsg, 0, 1, 0, 1); // Default green
      }
      this->ledPub->Publish(matMsg);
      std::cout << "[CrazyflieLedPlugin] Published new material" << std::endl;
    }

    void SetColor(gazebo::msgs::Material &matMsg, float r, float g, float b, float a)
    {
      matMsg.mutable_ambient()->set_r(r); matMsg.mutable_ambient()->set_g(g); matMsg.mutable_ambient()->set_b(b); matMsg.mutable_ambient()->set_a(a);
      matMsg.mutable_diffuse()->set_r(r); matMsg.mutable_diffuse()->set_g(g); matMsg.mutable_diffuse()->set_b(b); matMsg.mutable_diffuse()->set_a(a);
      matMsg.mutable_emissive()->set_r(r); matMsg.mutable_emissive()->set_g(g); matMsg.mutable_emissive()->set_b(b); matMsg.mutable_emissive()->set_a(a);
    }

    ~CrazyflieLedPlugin() override
    {
      // Non chiamare rclcpp::shutdown()!
      // Il thread è stato staccato, non serve join()
    }

  private:
    physics::ModelPtr model;
    transport::NodePtr gzNode;
    transport::PublisherPtr ledPub;

    // ROS2
    rclcpp::Node::SharedPtr rosNode;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
    std::thread rosThread;
  };

  GZ_REGISTER_MODEL_PLUGIN(CrazyflieLedPlugin)
} 