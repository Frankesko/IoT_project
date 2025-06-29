#!/usr/bin/env python3
"""
Setup Script for ROS2 and Gazebo Integration
Installs and configures ROS2, Gazebo, and the Crazyflie simulation environment
"""

import os
import sys
import subprocess
import platform
import shutil
from pathlib import Path
import json

class ROS2GazeboSetup:
    """Setup per ROS2 e Gazebo"""
    
    def __init__(self):
        self.system = platform.system().lower()
        self.ros2_distro = "humble"  # ROS2 Humble per Ubuntu 22.04
        self.workspace_path = Path.cwd()
        self.package_name = "crazyflie_gazebo"
        
        # Verifica sistema operativo
        if self.system not in ["linux", "windows"]:
            print(f"Sistema operativo non supportato: {self.system}")
            sys.exit(1)
    
    def check_prerequisites(self) -> bool:
        """Verifica i prerequisiti"""
        print("Verificando prerequisiti...")
        
        # Verifica Python
        if sys.version_info < (3, 8):
            print("Errore: Python 3.8+ richiesto")
            return False
        
        # Verifica pip
        try:
            subprocess.run([sys.executable, "-m", "pip", "--version"], 
                         check=True, capture_output=True)
        except subprocess.CalledProcessError:
            print("Errore: pip non disponibile")
            return False
        
        # Verifica git
        try:
            subprocess.run(["git", "--version"], check=True, capture_output=True)
        except subprocess.CalledProcessError:
            print("Errore: git non disponibile")
            return False
        
        print("Prerequisiti verificati con successo!")
        return True
    
    def install_ros2(self):
        """Installa ROS2"""
        print("Installando ROS2...")
        
        if self.system == "linux":
            self._install_ros2_linux()
        elif self.system == "windows":
            self._install_ros2_windows()
    
    def _install_ros2_linux(self):
        """Installa ROS2 su Linux"""
        try:
            # Aggiungi repository ROS2
            subprocess.run([
                "sudo", "apt", "update"
            ], check=True)
            
            # Installa curl se non presente
            subprocess.run([
                "sudo", "apt", "install", "-y", "curl", "software-properties-common"
            ], check=True)
            
            # Aggiungi chiave GPG
            subprocess.run([
                "sudo", "curl", "-sSL", 
                f"https://raw.githubusercontent.com/ros/rosdistro/master/ros.key", 
                "-o", "/usr/share/keyrings/ros-archive-keyring.gpg"
            ], check=True)
            
            # Aggiungi repository
            subprocess.run([
                "echo", f'"deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main"',
                "|", "sudo", "tee", "/etc/apt/sources.list.d/ros2.list", ">", "/dev/null"
            ], shell=True, check=True)
            
            # Aggiorna e installa ROS2
            subprocess.run([
                "sudo", "apt", "update"
            ], check=True)
            
            subprocess.run([
                "sudo", "apt", "install", "-y", f"ros-{self.ros2_distro}-desktop"
            ], check=True)
            
            # Installa dipendenze aggiuntive
            subprocess.run([
                "sudo", "apt", "install", "-y",
                f"ros-{self.ros2_distro}-gazebo-ros2-control",
                f"ros-{self.ros2_distro}-gazebo-plugins",
                f"ros-{self.ros2_distro}-gazebo-dev",
                "python3-colcon-common-extensions"
            ], check=True)
            
            print("ROS2 installato con successo!")
            
        except subprocess.CalledProcessError as e:
            print(f"Errore nell'installazione di ROS2: {e}")
            sys.exit(1)
    
    def _install_ros2_windows(self):
        """Installa ROS2 su Windows"""
        print("Installazione ROS2 su Windows richiede setup manuale.")
        print("Segui le istruzioni su: https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html")
        
        # Installa Chocolatey se non presente
        try:
            subprocess.run(["choco", "--version"], check=True, capture_output=True)
        except subprocess.CalledProcessError:
            print("Installando Chocolatey...")
            subprocess.run([
                "powershell", "-Command", 
                "Set-ExecutionPolicy Bypass -Scope Process -Force; [System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072; iex ((New-Object System.Net.WebClient).DownloadString('https://community.chocolatey.org/install.ps1'))"
            ], check=True)
        
        # Installa ROS2 tramite Chocolatey
        try:
            subprocess.run([
                "choco", "install", "ros2", "-y"
            ], check=True)
            print("ROS2 installato con successo!")
        except subprocess.CalledProcessError as e:
            print(f"Errore nell'installazione di ROS2: {e}")
            sys.exit(1)
    
    def install_gazebo(self):
        """Installa Gazebo"""
        print("Installando Gazebo...")
        
        if self.system == "linux":
            try:
                subprocess.run([
                    "sudo", "apt", "install", "-y", "gazebo"
                ], check=True)
                print("Gazebo installato con successo!")
            except subprocess.CalledProcessError as e:
                print(f"Errore nell'installazione di Gazebo: {e}")
                sys.exit(1)
        elif self.system == "windows":
            print("Gazebo è incluso con ROS2 su Windows")
    
    def setup_workspace(self):
        """Setup del workspace ROS2"""
        print("Configurando workspace ROS2...")
        
        # Crea workspace se non esiste
        workspace_path = self.workspace_path / "crazyflie_ws"
        workspace_path.mkdir(exist_ok=True)
        
        # Crea src directory
        src_path = workspace_path / "src"
        src_path.mkdir(exist_ok=True)
        
        # Crea package directory
        package_path = src_path / self.package_name
        package_path.mkdir(exist_ok=True)
        
        # Copia i file del package
        self._create_package_structure(package_path)
        
        print(f"Workspace creato in: {workspace_path}")
        return workspace_path
    
    def _create_package_structure(self, package_path: Path):
        """Crea la struttura del package ROS2"""
        
        # Crea directory
        directories = [
            "launch",
            "worlds", 
            "models/crazyflie",
            "config",
            "resource",
            "scripts"
        ]
        
        for dir_name in directories:
            (package_path / dir_name).mkdir(parents=True, exist_ok=True)
        
        # Crea package.xml
        package_xml_content = f"""<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{self.package_name}</name>
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
  <depend>trajectory_msgs</depend>

  <export>
    <build_type>ament_python</build_type>
    <gazebo_ros gazebo_model_path="${{prefix}}/models"/>
    <gazebo_ros gazebo_media_path="${{prefix}}/worlds"/>
  </export>
</package>"""
        
        with open(package_path / "package.xml", "w") as f:
            f.write(package_xml_content)
        
        # Crea setup.py
        setup_py_content = f"""from setuptools import setup
import os
from glob import glob

package_name = '{self.package_name}'

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
    entry_points={{
        'console_scripts': [
            'ros2_gazebo_integration = {self.package_name}.ros2_gazebo_integration:main',
            'mqtt_ros2_bridge = {self.package_name}.mqtt_ros2_bridge:main',
            'advanced_flight_controller = {self.package_name}.advanced_flight_controller:main',
        ],
    }},
)"""
        
        with open(package_path / "setup.py", "w") as f:
            f.write(setup_py_content)
        
        # Crea __init__.py
        with open(package_path / f"{self.package_name}" / "__init__.py", "w") as f:
            f.write("# Crazyflie Gazebo Package\n")
        
        # Copia i file Python
        python_files = [
            "ros2_gazebo_integration.py",
            "mqtt_ros2_bridge.py", 
            "advanced_flight_controller.py"
        ]
        
        for file_name in python_files:
            if (self.workspace_path / file_name).exists():
                shutil.copy2(
                    self.workspace_path / file_name,
                    package_path / f"{self.package_name}" / file_name
                )
        
        # Crea resource file
        with open(package_path / "resource" / self.package_name, "w") as f:
            f.write("")
    
    def create_world_file(self, package_path: Path):
        """Crea il file world per Gazebo"""
        world_content = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="crazyflie_world">
    <!-- Physics -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.81</gravity>
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
      <diffuse>0.8 0.8 0.8</diffuse>
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
      <ambient>0.7 0.7 0.7</ambient>
      <background>0.7 0.7 0.7</background>
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
    
  </world>
</sdf>"""
        
        with open(package_path / "worlds" / "crazyflie_world.world", "w") as f:
            f.write(world_content)
    
    def create_launch_file(self, package_path: Path):
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
            FindPackageShare('{self.package_name}'),
            'worlds',
            'crazyflie_world.world'
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
                FindPackageShare('{self.package_name}'),
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
        package='{self.package_name}',
        executable='ros2_gazebo_integration',
        name='crazyflie_controller',
        output='screen'
    )
    
    # Advanced Flight Controller
    advanced_controller = Node(
        package='{self.package_name}',
        executable='advanced_flight_controller',
        name='advanced_flight_controller',
        output='screen'
    )
    
    # Bridge per MQTT
    mqtt_bridge = Node(
        package='{self.package_name}',
        executable='mqtt_ros2_bridge',
        name='mqtt_bridge',
        output='screen'
    )
    
    return LaunchDescription([
        world_arg,
        gazebo,
        spawn_drone,
        controller,
        advanced_controller,
        mqtt_bridge
    ])"""
        
        with open(package_path / "launch" / "crazyflie_simulation.launch.py", "w") as f:
            f.write(launch_content)
    
    def build_workspace(self, workspace_path: Path):
        """Build del workspace ROS2"""
        print("Building workspace ROS2...")
        
        try:
            # Cambia directory nel workspace
            os.chdir(workspace_path)
            
            # Build del package
            subprocess.run([
                "colcon", "build", "--packages-select", self.package_name
            ], check=True)
            
            print("Workspace buildato con successo!")
            
        except subprocess.CalledProcessError as e:
            print(f"Errore nel build del workspace: {e}")
            sys.exit(1)
    
    def create_demo_script(self, workspace_path: Path):
        """Crea script di demo"""
        demo_script_content = f"""#!/bin/bash
# Demo script per Crazyflie Gazebo Simulation

echo "Avviando simulazione Crazyflie con ROS2 e Gazebo..."

# Source dell'ambiente
source {workspace_path}/install/setup.bash

# Lancia la simulazione
ros2 launch {self.package_name} crazyflie_simulation.launch.py

echo "Simulazione terminata."
"""
        
        demo_script_path = workspace_path / "run_demo.sh"
        with open(demo_script_path, "w") as f:
            f.write(demo_script_content)
        
        # Rendi eseguibile
        os.chmod(demo_script_path, 0o755)
        
        print(f"Script demo creato: {demo_script_path}")
    
    def create_readme(self, workspace_path: Path):
        """Crea README per il progetto"""
        readme_content = f"""# Crazyflie Gazebo Simulation

Simulazione avanzata del drone Crazyflie con ROS2 e Gazebo, con controllo a livello di rotori e manovre acrobatiche.

## Caratteristiche

- Simulazione fisica realistica con Gazebo
- Controllo a livello di rotori per manovre precise
- Manovre acrobatiche (flip, spirali, figure-8)
- Integrazione con sistema IoT esistente via MQTT
- Controller di volo avanzato con traiettorie personalizzate

## Prerequisiti

- ROS2 {self.ros2_distro}
- Gazebo
- Python 3.8+
- MQTT Broker (Mosquitto)

## Installazione

1. Esegui lo script di setup:
   ```bash
   python3 setup_ros2_gazebo.py
   ```

2. Source dell'ambiente:
   ```bash
   source {workspace_path}/install/setup.bash
   ```

## Utilizzo

### Avvio della simulazione
```bash
cd {workspace_path}
./run_demo.sh
```

### Comandi di manovra
```bash
# Flip in avanti
ros2 topic pub /crazyflie/maneuver_command std_msgs/msg/String '{{"maneuver_type": "flip_forward"}}'

# Spirale
ros2 topic pub /crazyflie/maneuver_command std_msgs/msg/String '{{"maneuver_type": "spiral", "parameters": {{"radius": 1.0, "height_gain": 2.0}}}}'

# Figura 8
ros2 topic pub /crazyflie/maneuver_command std_msgs/msg/String '{{"maneuver_type": "figure_8"}}'
```

### Controllo via MQTT
Il sistema è integrato con il broker MQTT esistente. Pubblica comandi su:
- `crazyflie/commands` - Comandi generali
- `crazyflie/flip_command` - Comandi di flip
- `crazyflie/position_command` - Comandi di posizione

## Struttura del Progetto

```
{workspace_path}/
├── src/
│   └── {self.package_name}/
│       ├── launch/          # File launch ROS2
│       ├── worlds/          # Mondi Gazebo
│       ├── models/          # Modelli 3D
│       ├── config/          # Configurazioni
│       └── {self.package_name}/  # Codice Python
├── build/                   # File di build
├── install/                 # File installati
└── run_demo.sh             # Script di demo
```

## Troubleshooting

1. **Gazebo non si avvia**: Verifica che Gazebo sia installato correttamente
2. **Errori di build**: Verifica che ROS2 sia installato e configurato
3. **Problemi MQTT**: Verifica che il broker MQTT sia in esecuzione

## Contributi

Questo progetto estende il sistema IoT esistente con simulazione Gazebo e controllo avanzato.
"""
        
        with open(workspace_path / "README.md", "w") as f:
            f.write(readme_content)
    
    def run_setup(self):
        """Esegue il setup completo"""
        print("=== Setup ROS2 e Gazebo per Crazyflie Simulation ===")
        
        # Verifica prerequisiti
        if not self.check_prerequisites():
            return False
        
        # Installa ROS2
        self.install_ros2()
        
        # Installa Gazebo
        self.install_gazebo()
        
        # Setup workspace
        workspace_path = self.setup_workspace()
        
        # Crea file di configurazione
        package_path = workspace_path / "src" / self.package_name
        self.create_world_file(package_path)
        self.create_launch_file(package_path)
        
        # Build workspace
        self.build_workspace(workspace_path)
        
        # Crea script e documentazione
        self.create_demo_script(workspace_path)
        self.create_readme(workspace_path)
        
        print("\n=== Setup completato con successo! ===")
        print(f"Workspace: {workspace_path}")
        print("Per avviare la simulazione:")
        print(f"1. cd {workspace_path}")
        print("2. source install/setup.bash")
        print("3. ./run_demo.sh")
        
        return True

def main():
    """Funzione principale"""
    setup = ROS2GazeboSetup()
    
    try:
        success = setup.run_setup()
        if success:
            print("\nSetup completato! Puoi ora avviare la simulazione.")
        else:
            print("\nSetup fallito. Controlla i messaggi di errore.")
            sys.exit(1)
    except KeyboardInterrupt:
        print("\nSetup interrotto dall'utente.")
        sys.exit(1)
    except Exception as e:
        print(f"\nErrore durante il setup: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main() 