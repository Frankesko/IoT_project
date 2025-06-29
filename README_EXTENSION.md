# Estensione ROS2/Gazebo per Progetto IoT Crazyflie

## Panoramica

Questa estensione integra il sistema IoT esistente con una simulazione Gazebo e controllo ROS2, permettendo di controllare un drone Crazyflie a livello di rotori per eseguire manovre avanzate come flip, spirali e traiettorie personalizzate.

## Caratteristiche Principali

### 🚁 Controllo a Livello di Rotori
- Controllo diretto di ogni rotore del drone
- Allocazione di spinta e coppia ottimizzata
- Limitazioni di sicurezza integrate
- Boost di spinta per manovre acrobatiche

### 🎯 Manovre Avanzate
- **Flip**: Forward, Backward, Left, Right
- **Rotazioni 360°**: Roll, Pitch, Yaw
- **Traiettorie Complesse**: Spirali, Figure-8
- **Traiettorie Personalizzate**: Definibili via ROS2

### 🔄 Integrazione IoT-ROS2
- Bridge MQTT-ROS2 bidirezionale
- Sincronizzazione tra simulazione IoT e Gazebo
- Comandi remoti via MQTT
- Telemetria unificata

### 🎮 Controllo Real-time
- Loop di controllo a 50Hz
- Controller PID avanzati
- Gestione degli errori robusta
- Monitoraggio dello stato in tempo reale

## Architettura del Sistema

```
┌─────────────────┐    MQTT    ┌─────────────────┐    ROS2    ┌─────────────────┐
│   IoT System    │◄──────────►│  MQTT-ROS2      │◄──────────►│  Gazebo         │
│   (Existing)    │            │  Bridge         │            │  Simulation     │
└─────────────────┘            └─────────────────┘            └─────────────────┘
                                       │                              │
                                       ▼                              ▼
                              ┌─────────────────┐            ┌─────────────────┐
                              │ Advanced Flight │            │ Crazyflie       │
                              │ Controller      │            │ Model           │
                              └─────────────────┘            └─────────────────┘
```

## File Principali

### 1. `ros2_gazebo_integration.py`
Controller principale per il controllo a livello di rotori:
- Gestione comandi dei rotori
- Controllo PID per posizione e orientazione
- Esecuzione di flip e manovre base
- Integrazione con sensori Gazebo

### 2. `advanced_flight_controller.py`
Controller avanzato per manovre complesse:
- Generazione di traiettorie
- Manovre acrobatiche predefinite
- Controller PID 3D
- Gestione di traiettorie personalizzate

### 3. `mqtt_ros2_bridge.py`
Bridge tra sistema IoT e ROS2:
- Traduzione messaggi MQTT-ROS2
- Sincronizzazione stati
- Comandi remoti
- Telemetria unificata

### 4. `gazebo_world_setup.py`
Setup del mondo Gazebo:
- Configurazione fisica
- Modello del drone
- Plugin ROS2
- File di launch

### 5. `setup_ros2_gazebo.py`
Script di setup automatico:
- Installazione ROS2/Gazebo
- Configurazione workspace
- Build del package
- Script di demo

## Installazione

### Prerequisiti
- Ubuntu 20.04+ (raccomandato) o Windows 10+
- Python 3.8+
- Git
- MQTT Broker (Mosquitto)

### Setup Automatico
```bash
# Clona il repository
git clone <repository-url>
cd IoT_project

# Esegui setup automatico
python3 setup_ros2_gazebo.py
```

### Setup Manuale
```bash
# 1. Installa ROS2 Humble
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop

# 2. Installa Gazebo
sudo apt install gazebo

# 3. Installa dipendenze ROS2
sudo apt install ros-humble-gazebo-ros2-control ros-humble-gazebo-plugins python3-colcon-common-extensions

# 4. Setup workspace
mkdir -p crazyflie_ws/src
cd crazyflie_ws/src
# Copia i file del package qui
cd ..
colcon build
source install/setup.bash
```

## Utilizzo

### Avvio della Simulazione
```bash
# Source dell'ambiente
source crazyflie_ws/install/setup.bash

# Lancia la simulazione
ros2 launch crazyflie_gazebo crazyflie_simulation.launch.py
```

### Comandi di Manovra

#### Via ROS2 Topics
```bash
# Flip in avanti
ros2 topic pub /crazyflie/maneuver_command std_msgs/msg/String '{"maneuver_type": "flip_forward"}'

# Spirale
ros2 topic pub /crazyflie/maneuver_command std_msgs/msg/String '{"maneuver_type": "spiral", "parameters": {"radius": 1.0, "height_gain": 2.0}}'

# Figura 8
ros2 topic pub /crazyflie/maneuver_command std_msgs/msg/String '{"maneuver_type": "figure_8"}'

# Posizione specifica
ros2 topic pub /crazyflie/position_command geometry_msgs/msg/PoseStamped '{"pose": {"position": {"x": 1.0, "y": 0.0, "z": 1.0}}}'
```

#### Via MQTT
```bash
# Flip via MQTT
mosquitto_pub -h localhost -t "crazyflie/commands" -m '{"command_type": "execute_flip", "data": {"direction": "forward"}}'

# Cambio altitudine
mosquitto_pub -h localhost -t "crazyflie/commands" -m '{"command_type": "set_target_altitude", "data": {"target_altitude": 1.5}}'

# Posizione specifica
mosquitto_pub -h localhost -t "crazyflie/commands" -m '{"command_type": "set_position", "data": {"x": 1.0, "y": 0.0, "z": 1.0}}'
```

### Monitoraggio

#### Telemetria ROS2
```bash
# Stato del drone
ros2 topic echo /crazyflie/status

# Odometria
ros2 topic echo /crazyflie/odometry

# Stato delle manovre
ros2 topic echo /crazyflie/maneuver_status

# Comandi dei rotori
ros2 topic echo /crazyflie/advanced_rotor_commands
```

#### Telemetria MQTT
```bash
# Telemetria IoT
mosquitto_sub -h localhost -t "iot/telemetry"

# Telemetria Gazebo
mosquitto_sub -h localhost -t "gazebo/telemetry"

# Alert
mosquitto_sub -h localhost -t "crazyflie/alerts"
```

## Configurazione Avanzata

### Parametri del Drone
Modifica `ros2_gazebo_integration.py` per cambiare:
- Massa del drone
- Posizioni dei rotori
- Coefficienti di spinta/coppia
- Limiti di sicurezza

### Controller PID
Modifica i parametri PID in `advanced_flight_controller.py`:
```python
self.position_controller = PIDController3D(kp=2.0, ki=0.1, kd=0.5)
self.orientation_controller = PIDController3D(kp=3.0, ki=0.1, kd=0.8)
```

### Traiettorie Personalizzate
Crea nuove traiettorie in `TrajectoryGenerator`:
```python
def generate_custom_trajectory(self, parameters):
    # Implementa la tua traiettoria
    pass
```

## Troubleshooting

### Problemi Comuni

1. **Gazebo non si avvia**
   ```bash
   # Verifica installazione
   gazebo --verbose
   
   # Reinstalla se necessario
   sudo apt remove gazebo
   sudo apt install gazebo
   ```

2. **Errori di build ROS2**
   ```bash
   # Pulisci build
   cd crazyflie_ws
   rm -rf build/ install/
   colcon build --packages-select crazyflie_gazebo
   ```

3. **Problemi MQTT**
   ```bash
   # Verifica broker
   systemctl status mosquitto
   
   # Riavvia se necessario
   sudo systemctl restart mosquitto
   ```

4. **Controller non risponde**
   ```bash
   # Verifica topic
   ros2 topic list
   ros2 topic info /crazyflie/odometry
   
   # Verifica nodi
   ros2 node list
   ros2 node info /crazyflie_controller
   ```

### Debug Avanzato

#### Log ROS2
```bash
# Abilita log dettagliati
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}]: {message}"
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=1
```

#### Profiling Performance
```bash
# Monitora CPU/GPU
htop
nvidia-smi  # se disponibile

# Profiling ROS2
ros2 run performance_test_fixture performance_test
```

## Estensioni Future

### Possibili Miglioramenti
1. **Machine Learning**: Controllo basato su reti neurali
2. **Multi-Drone**: Simulazione di swarm
3. **Reality Gap**: Bridge con drone reale
4. **Web Interface**: Dashboard web per controllo
5. **Recording/Playback**: Registrazione e riproduzione manovre

### Contributi
Per contribuire al progetto:
1. Fork del repository
2. Crea branch per feature
3. Implementa e testa
4. Submit pull request

## Licenza

MIT License - vedi file LICENSE per dettagli.

## Contatti

Per domande o supporto:
- Email: user@example.com
- GitHub Issues: [Repository Issues]
- Documentation: [Link alla documentazione]

---

**Nota**: Questa estensione è progettata per uso educativo e di ricerca. Non utilizzare su droni reali senza adeguata supervisione e test. 