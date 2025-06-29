#!/usr/bin/env python3
"""
Gazebo World Setup for Crazyflie Drone Simulation
Configures the Gazebo world with proper physics and drone model
"""

import os
import xml.etree.ElementTree as ET
from typing import Dict, List, Optional
import subprocess
import time

class GazeboWorldSetup:
    """Setup per il mondo Gazebo con drone Crazyflie"""
    
    def __init__(self, world_name: str = "crazyflie_world"):
        self.world_name = world_name
        self.world_path = f"worlds/{world_name}.world"
        self.model_path = "models/crazyflie"
        self.spawn_script_path = "scripts/spawn_crazyflie.py"
        
        # Parametri del mondo
        self.world_config = {
            "gravity": [0, 0, -9.81],
            "wind": [0, 0, 0],
            "ambient_light": [0.8, 0.8, 0.8],
            "background": [0.7, 0.7, 0.7],
            "physics_rate": 1000,
            "real_time_factor": 1.0,
            "max_step_size": 0.001
        }
        
        # Parametri del drone
        self.drone_config = {
            "mass": 0.027,
            "inertia": [1.4e-5, 1.4e-5, 2.17e-5],
            "rotor_count": 4,
            "rotor_positions": [
                [0.046, 0.046, 0],
                [-0.046, 0.046, 0],
                [-0.046, -0.046, 0],
                [0.046, -0.046, 0]
            ],
            "rotor_thrust_coefficient": 1.0e-6,
            "rotor_torque_coefficient": 1.0e-8
        }
    
    def create_world_file(self) -> str:
        """Crea il file world per Gazebo"""
        world_content = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="{self.world_name}">
    <!-- Physics -->
    <physics type="ode">
      <max_step_size>{self.world_config['max_step_size']}</max_step_size>
      <real_time_factor>{self.world_config['real_time_factor']}</real_time_factor>
      <real_time_update_rate>{self.world_config['physics_rate']}</real_time_update_rate>
      <gravity>{self.world_config['gravity'][0]} {self.world_config['gravity'][1]} {self.world_config['gravity'][2]}</gravity>
    </physics>
    
    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Ambient light -->
    <light type="directional" name="ambient_light">
      <diffuse>{self.world_config['ambient_light'][0]} {self.world_config['ambient_light'][1]} {self.world_config['ambient_light'][2]}</diffuse>
      <specular>0.1 0.1 0.1</specular>
      <attenuation>
        <range>20</range>
        <constant>0.5</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <cast_shadows>false</cast_shadows>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    
    <!-- Background -->
    <scene>
      <ambient>{self.world_config['background'][0]} {self.world_config['background'][1]} {self.world_config['background'][2]}</ambient>
      <background>{self.world_config['background'][0]} {self.world_config['background'][1]} {self.world_config['background'][2]}</background>
      <sky></sky>
    </scene>
    
    <!-- GUI -->
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>5 -5 2 0 0.785 0.785</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>
    
    <!-- Plugin for ROS2 integration -->
    <plugin name="ros2_control" filename="libgazebo_ros2_control.so">
      <parameters>$(find crazyflie_gazebo)/config/crazyflie_controllers.yaml</parameters>
    </plugin>
    
  </world>
</sdf>"""
        
        # Crea la directory se non esiste
        os.makedirs(os.path.dirname(self.world_path), exist_ok=True)
        
        # Scrivi il file
        with open(self.world_path, 'w') as f:
            f.write(world_content)
        
        return self.world_path
    
    def create_drone_model(self) -> str:
        """Crea il modello SDF del drone Crazyflie"""
        model_content = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="crazyflie">
    <pose>0 0 0.7 0 0 0</pose>
    
    <!-- Main body -->
    <link name="base_link">
      <inertial>
        <mass>{self.drone_config['mass']}</mass>
        <inertia>
          <ixx>{self.drone_config['inertia'][0]}</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>{self.drone_config['inertia'][1]}</iyy>
          <iyz>0</iyz>
          <izz>{self.drone_config['inertia'][2]}</izz>
        </inertia>
      </inertial>
      
      <collision name="collision">
        <geometry>
          <box>
            <size>0.092 0.092 0.029</size>
          </box>
        </geometry>
        <surface>
          <contact>
            <ode>
              <max_vel>0.1</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
          <friction>
            <ode>
              <mu>0.8</mu>
              <mu2>0.8</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      
      <visual name="visual">
        <geometry>
          <box>
            <size>0.092 0.092 0.029</size>
          </box>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.2 1</ambient>
          <diffuse>0.8 0.8 0.8 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
      
      <!-- Sensors -->
      <sensor name="imu" type="imu">
        <always_on>true</always_on>
        <update_rate>100</update_rate>
        <visualize>false</visualize>
        <topic>imu</topic>
        <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
          <ros>
            <namespace>/crazyflie</namespace>
            <remapping>~/out:=imu</remapping>
          </ros>
        </plugin>
      </sensor>
      
      <sensor name="tof_sensor" type="ray">
        <pose>0 0 -0.015 0 0 0</pose>
        <always_on>true</always_on>
        <update_rate>10</update_rate>
        <visualize>true</visualize>
        <topic>tof</topic>
        <ray>
          <scan>
            <horizontal>
              <samples>1</samples>
              <resolution>1</resolution>
              <min_angle>0</min_angle>
              <max_angle>0</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>0.02</min>
            <max>4.0</max>
            <resolution>0.01</resolution>
          </range>
        </ray>
        <plugin name="tof_plugin" filename="libgazebo_ros_ray_sensor.so">
          <ros>
            <namespace>/crazyflie</namespace>
            <remapping>~/out:=tof</remapping>
          </ros>
        </plugin>
      </sensor>
      
    </link>
    
    <!-- Rotors -->
    {self._generate_rotor_links()}
    
    <!-- Battery -->
    <link name="battery">
      <pose>0 0 0.01 0 0 0</pose>
      <inertial>
        <mass>0.005</mass>
        <inertia>
          <ixx>1e-6</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>1e-6</iyy>
          <iyz>0</iyz>
          <izz>1e-6</izz>
        </inertia>
      </inertial>
      <visual name="battery_visual">
        <geometry>
          <box>
            <size>0.03 0.02 0.005</size>
          </box>
        </geometry>
        <material>
          <ambient>0.1 0.8 0.1 1</ambient>
          <diffuse>0.2 1.0 0.2 1</diffuse>
        </material>
      </visual>
    </link>
    
    <!-- Joints -->
    <joint name="battery_joint" type="fixed">
      <parent>base_link</parent>
      <child>battery</child>
    </joint>
    
    <!-- Plugin for rotor control -->
    <plugin name="rotor_control" filename="libgazebo_ros_planar_move.so">
      <commandTopic>/crazyflie/rotor_commands</commandTopic>
      <odometryTopic>/crazyflie/odometry</odometryTopic>
      <odometryFrame>odom</odometryFrame>
      <odometryRate>20.0</odometryRate>
      <rosDebugLevel>na</rosDebugLevel>
      <robotBaseFrame>base_link</robotBaseFrame>
    </plugin>
    
    <!-- Plugin for battery simulation -->
    <plugin name="battery_plugin" filename="libgazebo_ros_battery.so">
      <ros>
        <namespace>/crazyflie</namespace>
        <remapping>~/out:=battery</remapping>
      </ros>
      <battery>
        <voltage>3.7</voltage>
        <capacity>240</capacity>
        <current_drain>0.1</current_drain>
      </battery>
    </plugin>
    
  </model>
</sdf>"""
        
        # Crea la directory se non esiste
        os.makedirs(self.model_path, exist_ok=True)
        
        # Scrivi il file
        model_file = os.path.join(self.model_path, "model.sdf")
        with open(model_file, 'w') as f:
            f.write(model_content)
        
        return model_file
    
    def _generate_rotor_links(self) -> str:
        """Genera i link per i rotori"""
        rotor_links = ""
        rotor_names = ["rotor_fr", "rotor_fl", "rotor_bl", "rotor_br"]
        
        for i, (name, pos) in enumerate(zip(rotor_names, self.drone_config['rotor_positions'])):
            rotor_links += f"""
    <link name="{name}">
      <pose>{pos[0]} {pos[1]} {pos[2]} 0 0 0</pose>
      <inertial>
        <mass>0.001</mass>
        <inertia>
          <ixx>1e-7</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>1e-7</iyy>
          <iyz>0</iyz>
          <izz>1e-7</izz>
        </inertia>
      </inertial>
      <visual name="{name}_visual">
        <geometry>
          <cylinder>
            <radius>0.02</radius>
            <length>0.005</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.8 0.2 0.2 1</ambient>
          <diffuse>1.0 0.3 0.3 1</diffuse>
        </material>
      </visual>
      <collision name="{name}_collision">
        <geometry>
          <cylinder>
            <radius>0.02</radius>
            <length>0.005</length>
          </cylinder>
        </geometry>
      </collision>
    </link>
    
    <joint name="{name}_joint" type="revolute">
      <parent>base_link</parent>
      <child>{name}</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-1e6</lower>
          <upper>1e6</upper>
          <effort>100</effort>
          <velocity>1000</velocity>
        </limit>
        <dynamics>
          <damping>0.01</damping>
          <friction>0.01</friction>
        </dynamics>
      </axis>
    </joint>"""
        
        return rotor_links
    
    def create_spawn_script(self) -> str:
        """Crea lo script per spawnare il drone"""
        spawn_content = f"""#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
import os

class CrazyflieSpawner(Node):
    def __init__(self):
        super().__init__('crazyflie_spawner')
        
        # Client per spawnare entità
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        
        # Aspetta che il servizio sia disponibile
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')
        
        # Spawna il drone
        self.spawn_drone()
    
    def spawn_drone(self):
        # Leggi il file del modello
        model_path = os.path.join(os.getcwd(), '{self.model_path}', 'model.sdf')
        with open(model_path, 'r') as f:
            model_xml = f.read()
        
        # Crea la richiesta
        request = SpawnEntity.Request()
        request.name = 'crazyflie'
        request.xml = model_xml
        request.initial_pose = Pose()
        request.initial_pose.position.x = 0.0
        request.initial_pose.position.y = 0.0
        request.initial_pose.position.z = 0.7
        
        # Invia la richiesta
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().success:
            self.get_logger().info('Crazyflie spawned successfully')
        else:
            self.get_logger().error('Failed to spawn Crazyflie')

def main(args=None):
    rclpy.init(args=args)
    spawner = CrazyflieSpawner()
    rclpy.spin(spawner)
    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()"""
        
        # Crea la directory se non esiste
        os.makedirs(os.path.dirname(self.spawn_script_path), exist_ok=True)
        
        # Scrivi il file
        with open(self.spawn_script_path, 'w') as f:
            f.write(spawn_content)
        
        # Rendi eseguibile
        os.chmod(self.spawn_script_path, 0o755)
        
        return self.spawn_script_path
    
    def create_launch_file(self) -> str:
        """Crea il file launch per ROS2"""
        launch_content = f"""from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Argomenti
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('crazyflie_gazebo'),
            'worlds',
            '{self.world_name}.world'
        ]),
        description='Path to world file'
    )
    
    # Avvia Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={{
            'world': LaunchConfiguration('world'),
            'verbose': 'true'
        }}.items()
    )
    
    # Spawn del drone
    spawn_drone = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'crazyflie',
            '-file', PathJoinSubstitution([
                FindPackageShare('crazyflie_gazebo'),
                'models',
                'crazyflie',
                'model.sdf'
            ]),
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.7'
        ],
        output='screen'
    )
    
    # Controller ROS2
    controller = Node(
        package='crazyflie_gazebo',
        executable='ros2_gazebo_integration.py',
        name='crazyflie_controller',
        output='screen'
    )
    
    # Bridge per MQTT
    mqtt_bridge = Node(
        package='crazyflie_gazebo',
        executable='mqtt_ros2_bridge.py',
        name='mqtt_bridge',
        output='screen'
    )
    
    return LaunchDescription([
        world_arg,
        gazebo,
        spawn_drone,
        controller,
        mqtt_bridge
    ])"""
        
        launch_file = f"launch/crazyflie_simulation.launch.py"
        os.makedirs(os.path.dirname(launch_file), exist_ok=True)
        
        with open(launch_file, 'w') as f:
            f.write(launch_content)
        
        return launch_file
    
    def setup_package(self):
        """Setup completo del package ROS2"""
        # Crea la struttura del package
        package_structure = {
            "package.xml": self._create_package_xml(),
            "setup.py": self._create_setup_py(),
            "CMakeLists.txt": self._create_cmakelists(),
            "config/crazyflie_controllers.yaml": self._create_controllers_config(),
            "resource/crazyflie_gazebo": ""
        }
        
        for file_path, content in package_structure.items():
            if content:
                os.makedirs(os.path.dirname(file_path), exist_ok=True)
                with open(file_path, 'w') as f:
                    f.write(content)
            else:
                os.makedirs(file_path, exist_ok=True)
        
        # Crea i file principali
        self.create_world_file()
        self.create_drone_model()
        self.create_spawn_script()
        self.create_launch_file()
        
        print("Package ROS2 setup completato!")
    
    def _create_package_xml(self) -> str:
        return """<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>crazyflie_gazebo</name>
  <version>0.0.1</version>
  <description>Gazebo simulation for Crazyflie drone with rotor-level control</description>
  <maintainer email="user@example.com">Your Name</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>gazebo_ros</depend>
  <depend>gazebo_ros2_control</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>std_msgs</depend>

  <export>
    <build_type>ament_python</build_type>
    <gazebo_ros gazebo_model_path="${prefix}/models"/>
    <gazebo_ros gazebo_media_path="${prefix}/worlds"/>
  </export>
</package>"""
    
    def _create_setup_py(self) -> str:
        return """from setuptools import setup
import os
from glob import glob

package_name = 'crazyflie_gazebo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'models'), glob('models/**/*', recursive=True)),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@example.com',
    description='Gazebo simulation for Crazyflie drone',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_gazebo_integration = crazyflie_gazebo.ros2_gazebo_integration:main',
            'mqtt_ros2_bridge = crazyflie_gazebo.mqtt_ros2_bridge:main',
        ],
    },
)"""
    
    def _create_cmakelists(self) -> str:
        return """cmake_minimum_required(VERSION 3.8)
project(crazyflie_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_ros REQUIRED)

ament_package()"""
    
    def _create_controllers_config(self) -> str:
        return """crazyflie_controller_manager:
  ros__parameters:
    update_rate: 50

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    rotor_controller:
      type: crazyflie_controllers/RotorController
      joints:
        - rotor_fr_joint
        - rotor_fl_joint
        - rotor_bl_joint
        - rotor_br_joint"""
    
    def launch_simulation(self):
        """Lancia la simulazione"""
        try:
            # Build del package
            subprocess.run(["colcon", "build", "--packages-select", "crazyflie_gazebo"], check=True)
            
            # Source dell'ambiente
            subprocess.run(["source", "install/setup.bash"], shell=True, check=True)
            
            # Lancia la simulazione
            subprocess.run([
                "ros2", "launch", "crazyflie_gazebo", "crazyflie_simulation.launch.py"
            ], check=True)
            
        except subprocess.CalledProcessError as e:
            print(f"Errore nel lancio della simulazione: {e}")
        except Exception as e:
            print(f"Errore generico: {e}")

def main():
    """Funzione principale per setup e lancio"""
    setup = GazeboWorldSetup()
    
    print("Setting up Gazebo world and ROS2 package...")
    setup.setup_package()
    
    print("Setup completato! Per lanciare la simulazione:")
    print("1. cd crazyflie_gazebo")
    print("2. colcon build")
    print("3. source install/setup.bash")
    print("4. ros2 launch crazyflie_gazebo crazyflie_simulation.launch.py")
    
    # Chiedi se lanciare subito
    response = input("Vuoi lanciare la simulazione ora? (y/n): ")
    if response.lower() == 'y':
        setup.launch_simulation()

if __name__ == "__main__":
    main() 