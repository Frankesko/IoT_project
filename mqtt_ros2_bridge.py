#!/usr/bin/env python3
"""
MQTT-ROS2 Bridge for Crazyflie IoT System
Bridges the existing MQTT-based IoT system with ROS2/Gazebo simulation
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import paho.mqtt.client as mqtt
import json
import threading
import time
from typing import Dict, List, Optional, Any
from dataclasses import dataclass, asdict
import numpy as np

# ROS2 Messages
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, BatteryState
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32MultiArray, Bool

@dataclass
class IoTTelemetry:
    """Struttura per i dati di telemetria IoT"""
    timestamp: float
    altitude_tof: float
    altitude_barometer: float
    battery_voltage: float
    temperature: float
    vibration_x: float
    vibration_y: float
    vibration_z: float
    signal_strength: int
    packet_loss_rate: float
    fsm_state: str
    scenario: str

@dataclass
class IoTAlert:
    """Struttura per gli alert IoT"""
    alert_type: str
    message: str
    severity: str
    details: Dict[str, Any]
    timestamp: float

class MQTTROS2Bridge(Node):
    """
    Bridge tra il sistema MQTT IoT esistente e ROS2/Gazebo
    Permette l'integrazione seamless tra i due sistemi
    """
    
    def __init__(self):
        super().__init__('mqtt_ros2_bridge')
        
        # Configurazione MQTT
        self.mqtt_broker = "localhost"
        self.mqtt_port = 1883
        self.mqtt_client_id = f"ros2_bridge_{int(time.time())}"
        
        # Client MQTT
        self.mqtt_client = mqtt.Client(client_id=self.mqtt_client_id)
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_message = self._on_mqtt_message
        self.mqtt_client.on_disconnect = self._on_mqtt_disconnect
        
        # Stato del bridge
        self.connected = False
        self.iot_telemetry = None
        self.iot_alerts = []
        self.gazebo_state = {}
        
        # Setup QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers ROS2
        self.iot_telemetry_pub = self.create_publisher(
            String,
            '/iot/telemetry',
            qos_profile
        )
        
        self.iot_alert_pub = self.create_publisher(
            String,
            '/iot/alerts',
            qos_profile
        )
        
        self.gazebo_status_pub = self.create_publisher(
            String,
            '/gazebo/status',
            qos_profile
        )
        
        # Subscribers ROS2
        self.gazebo_odometry_sub = self.create_subscription(
            Odometry,
            '/crazyflie/odometry',
            self.gazebo_odometry_callback,
            qos_profile
        )
        
        self.gazebo_imu_sub = self.create_subscription(
            Imu,
            '/crazyflie/imu',
            self.gazebo_imu_callback,
            qos_profile
        )
        
        self.gazebo_battery_sub = self.create_subscription(
            BatteryState,
            '/crazyflie/battery',
            self.gazebo_battery_callback,
            qos_profile
        )
        
        self.drone_status_sub = self.create_subscription(
            String,
            '/crazyflie/status',
            self.drone_status_callback,
            qos_profile
        )
        
        # Command subscribers
        self.flip_command_sub = self.create_subscription(
            String,
            '/iot/flip_command',
            self.flip_command_callback,
            qos_profile
        )
        
        self.position_command_sub = self.create_subscription(
            PoseStamped,
            '/iot/position_command',
            self.position_command_callback,
            qos_profile
        )
        
        # Timer per sincronizzazione
        self.sync_timer = self.create_timer(0.1, self.sync_loop)  # 10Hz
        
        # Thread per MQTT
        self.mqtt_thread = threading.Thread(target=self._mqtt_loop, daemon=True)
        
        # Handlers per comandi MQTT
        self.mqtt_command_handlers = {
            'inject_anomaly': self._handle_inject_anomaly,
            'set_target_altitude': self._handle_set_target_altitude,
            'reset_simulation': self._handle_reset_simulation,
            'execute_flip': self._handle_execute_flip,
            'set_position': self._handle_set_position
        }
        
        # Connessione MQTT
        self._connect_mqtt()
        
        self.get_logger().info("MQTT-ROS2 Bridge initialized")
    
    def _connect_mqtt(self):
        """Connette al broker MQTT"""
        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
            self.mqtt_thread.start()
            self.get_logger().info("Connected to MQTT broker")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MQTT broker: {e}")
    
    def _on_mqtt_connect(self, client, userdata, flags, rc):
        """Callback per connessione MQTT"""
        if rc == 0:
            self.connected = True
            self.get_logger().info("MQTT connection established")
            
            # Sottoscrizione ai topic
            topics = [
                "crazyflie/telemetry",
                "crazyflie/status",
                "crazyflie/alerts",
                "crazyflie/commands"
            ]
            
            for topic in topics:
                client.subscribe(topic)
                self.get_logger().info(f"Subscribed to {topic}")
        else:
            self.get_logger().error(f"MQTT connection failed with code {rc}")
    
    def _on_mqtt_disconnect(self, client, userdata, rc):
        """Callback per disconnessione MQTT"""
        self.connected = False
        self.get_logger().warning("MQTT connection lost")
    
    def _on_mqtt_message(self, client, userdata, msg):
        """Callback per messaggi MQTT"""
        try:
            topic = msg.topic
            payload = json.loads(msg.payload.decode('utf-8'))
            
            if topic == "crazyflie/telemetry":
                self._handle_iot_telemetry(payload)
            elif topic == "crazyflie/status":
                self._handle_iot_status(payload)
            elif topic == "crazyflie/alerts":
                self._handle_iot_alert(payload)
            elif topic == "crazyflie/commands":
                self._handle_iot_command(payload)
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid JSON in MQTT message: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing MQTT message: {e}")
    
    def _handle_iot_telemetry(self, payload: Dict):
        """Gestisce i dati di telemetria IoT"""
        try:
            telemetry = IoTTelemetry(
                timestamp=payload.get('timestamp', time.time()),
                altitude_tof=payload.get('altitude_tof', 0.0),
                altitude_barometer=payload.get('altitude_barometer', 0.0),
                battery_voltage=payload.get('battery_voltage', 3.7),
                temperature=payload.get('temperature', 25.0),
                vibration_x=payload.get('vibration_x', 0.0),
                vibration_y=payload.get('vibration_y', 0.0),
                vibration_z=payload.get('vibration_z', 0.0),
                signal_strength=payload.get('signal_strength', 90),
                packet_loss_rate=payload.get('packet_loss_rate', 0.01),
                fsm_state=payload.get('fsm_state', 'UNKNOWN'),
                scenario=payload.get('scenario', 'unknown')
            )
            
            self.iot_telemetry = telemetry
            
            # Pubblica su ROS2
            msg = String()
            msg.data = json.dumps(asdict(telemetry))
            self.iot_telemetry_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error handling IoT telemetry: {e}")
    
    def _handle_iot_status(self, payload: Dict):
        """Gestisce lo stato IoT"""
        # Aggiorna lo stato interno
        self.iot_status = payload
        
        # Pubblica su ROS2 se necessario
        msg = String()
        msg.data = json.dumps(payload)
        self.gazebo_status_pub.publish(msg)
    
    def _handle_iot_alert(self, payload: Dict):
        """Gestisce gli alert IoT"""
        try:
            alert = IoTAlert(
                alert_type=payload.get('alert_type', 'UNKNOWN'),
                message=payload.get('message', ''),
                severity=payload.get('severity', 'WARNING'),
                details=payload.get('details', {}),
                timestamp=payload.get('timestamp', time.time())
            )
            
            self.iot_alerts.append(alert)
            
            # Mantieni solo gli ultimi 20 alert
            if len(self.iot_alerts) > 20:
                self.iot_alerts = self.iot_alerts[-20:]
            
            # Pubblica su ROS2
            msg = String()
            msg.data = json.dumps(asdict(alert))
            self.iot_alert_pub.publish(msg)
            
            self.get_logger().warning(f"IoT Alert: {alert.alert_type} - {alert.message}")
            
        except Exception as e:
            self.get_logger().error(f"Error handling IoT alert: {e}")
    
    def _handle_iot_command(self, payload: Dict):
        """Gestisce i comandi IoT"""
        command_type = payload.get('command_type', '')
        command_data = payload.get('data', {})
        
        if command_type in self.mqtt_command_handlers:
            self.mqtt_command_handlers[command_type](command_data)
        else:
            self.get_logger().warning(f"Unknown command type: {command_type}")
    
    def _handle_inject_anomaly(self, data: Dict):
        """Gestisce l'iniezione di anomalie"""
        anomaly_type = data.get('anomaly_type', '')
        active = data.get('active', True)
        duration = data.get('duration', None)
        
        # Pubblica su ROS2 per il controller
        msg = String()
        msg.data = json.dumps({
            'command': 'inject_anomaly',
            'anomaly_type': anomaly_type,
            'active': active,
            'duration': duration
        })
        
        # Pubblica su topic specifico per anomalie
        self.get_logger().info(f"Injecting anomaly: {anomaly_type}")
    
    def _handle_set_target_altitude(self, data: Dict):
        """Gestisce il cambio di altitudine target"""
        target_alt = data.get('target_altitude', 0.7)
        
        # Crea comando di posizione
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.z = target_alt
        
        # Pubblica il comando
        self.position_command_sub.get_publisher().publish(pose_msg)
        
        self.get_logger().info(f"Setting target altitude to {target_alt}m")
    
    def _handle_reset_simulation(self, data: Dict):
        """Gestisce il reset della simulazione"""
        # Pubblica comando di reset
        msg = String()
        msg.data = json.dumps({'command': 'reset_simulation'})
        
        self.get_logger().info("Resetting simulation")
    
    def _handle_execute_flip(self, data: Dict):
        """Gestisce l'esecuzione di flip"""
        flip_direction = data.get('direction', 'forward')
        
        # Pubblica comando di flip
        msg = String()
        msg.data = flip_direction
        
        self.flip_command_sub.get_publisher().publish(msg)
        
        self.get_logger().info(f"Executing {flip_direction} flip")
    
    def _handle_set_position(self, data: Dict):
        """Gestisce il posizionamento del drone"""
        x = data.get('x', 0.0)
        y = data.get('y', 0.0)
        z = data.get('z', 0.7)
        
        # Crea comando di posizione
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        
        # Pubblica il comando
        self.position_command_sub.get_publisher().publish(pose_msg)
        
        self.get_logger().info(f"Setting position to ({x}, {y}, {z})")
    
    def gazebo_odometry_callback(self, msg: Odometry):
        """Callback per odometria Gazebo"""
        self.gazebo_state['odometry'] = {
            'position': {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'z': msg.pose.pose.position.z
            },
            'orientation': {
                'x': msg.pose.pose.orientation.x,
                'y': msg.pose.pose.orientation.y,
                'z': msg.pose.pose.orientation.z,
                'w': msg.pose.pose.orientation.w
            },
            'velocity': {
                'linear': {
                    'x': msg.twist.twist.linear.x,
                    'y': msg.twist.twist.linear.y,
                    'z': msg.twist.twist.linear.z
                },
                'angular': {
                    'x': msg.twist.twist.angular.x,
                    'y': msg.twist.twist.angular.y,
                    'z': msg.twist.twist.angular.z
                }
            },
            'timestamp': self.get_clock().now().nanoseconds / 1e9
        }
    
    def gazebo_imu_callback(self, msg: Imu):
        """Callback per IMU Gazebo"""
        self.gazebo_state['imu'] = {
            'linear_acceleration': {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z
            },
            'angular_velocity': {
                'x': msg.angular_velocity.x,
                'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z
            },
            'orientation': {
                'x': msg.orientation.x,
                'y': msg.orientation.y,
                'z': msg.orientation.z,
                'w': msg.orientation.w
            },
            'timestamp': self.get_clock().now().nanoseconds / 1e9
        }
    
    def gazebo_battery_callback(self, msg: BatteryState):
        """Callback per batteria Gazebo"""
        self.gazebo_state['battery'] = {
            'percentage': msg.percentage,
            'voltage': msg.voltage,
            'current': msg.current,
            'power_supply_status': msg.power_supply_status,
            'timestamp': self.get_clock().now().nanoseconds / 1e9
        }
    
    def drone_status_callback(self, msg: String):
        """Callback per stato del drone"""
        self.gazebo_state['drone_status'] = msg.data
    
    def flip_command_callback(self, msg: String):
        """Callback per comandi di flip da IoT"""
        # Inoltra al controller ROS2
        flip_msg = String()
        flip_msg.data = msg.data
        self.flip_command_sub.get_publisher().publish(flip_msg)
    
    def position_command_callback(self, msg: PoseStamped):
        """Callback per comandi di posizione da IoT"""
        # Inoltra al controller ROS2
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose = msg.pose
        self.position_command_sub.get_publisher().publish(pose_msg)
    
    def sync_loop(self):
        """Loop di sincronizzazione tra IoT e Gazebo"""
        if not self.connected:
            return
        
        # Pubblica stato Gazebo su MQTT
        if self.gazebo_state:
            gazebo_telemetry = {
                'timestamp': time.time(),
                'source': 'gazebo',
                'data': self.gazebo_state
            }
            
            self.mqtt_client.publish(
                'gazebo/telemetry',
                json.dumps(gazebo_telemetry),
                qos=0
            )
        
        # Pubblica stato IoT su MQTT se disponibile
        if self.iot_telemetry:
            iot_telemetry = {
                'timestamp': time.time(),
                'source': 'iot_simulation',
                'data': asdict(self.iot_telemetry)
            }
            
            self.mqtt_client.publish(
                'iot/telemetry',
                json.dumps(iot_telemetry),
                qos=0
            )
    
    def _mqtt_loop(self):
        """Loop MQTT in thread separato"""
        while True:
            try:
                if not self.connected:
                    time.sleep(1)
                    continue
                
                # Il loop MQTT è gestito da paho-mqtt
                time.sleep(0.1)
                
            except Exception as e:
                self.get_logger().error(f"Error in MQTT loop: {e}")
                time.sleep(1)
    
    def destroy_node(self):
        """Cleanup alla chiusura"""
        if self.connected:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    bridge = MQTTROS2Bridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 