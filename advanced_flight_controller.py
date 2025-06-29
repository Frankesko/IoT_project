#!/usr/bin/env python3
"""
Advanced Flight Controller for Crazyflie Drone
Implements complex maneuvers like flips, acrobatics, and precision control
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import math
import time
from typing import Dict, List, Optional, Tuple, Callable
from dataclasses import dataclass
from enum import Enum
import threading

# ROS2 Messages
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32MultiArray, Bool
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

class ManeuverType(Enum):
    """Tipi di manovre disponibili"""
    HOVER = "hover"
    FLIP_FORWARD = "flip_forward"
    FLIP_BACKWARD = "flip_backward"
    FLIP_LEFT = "flip_left"
    FLIP_RIGHT = "flip_right"
    ROLL_360 = "roll_360"
    PITCH_360 = "pitch_360"
    YAW_360 = "yaw_360"
    FIGURE_8 = "figure_8"
    SPIRAL = "spiral"
    CUSTOM_TRAJECTORY = "custom_trajectory"

@dataclass
class FlightState:
    """Stato di volo del drone"""
    position: np.ndarray  # [x, y, z]
    velocity: np.ndarray  # [vx, vy, vz]
    orientation: np.ndarray  # [roll, pitch, yaw]
    angular_velocity: np.ndarray  # [wx, wy, wz]
    timestamp: float

@dataclass
class ManeuverConfig:
    """Configurazione per una manovra"""
    maneuver_type: ManeuverType
    duration: float
    parameters: Dict[str, float]
    safety_limits: Dict[str, float]

class TrajectoryGenerator:
    """Generatore di traiettorie per manovre complesse"""
    
    def __init__(self):
        self.trajectories = {}
    
    def generate_flip_trajectory(self, direction: str, duration: float = 1.0) -> List[Dict]:
        """Genera traiettoria per un flip"""
        trajectory = []
        steps = int(duration * 100)  # 100Hz
        
        for i in range(steps):
            t = i / steps
            progress = t
            
            # Profilo di velocità angolare (accensione rapida, spegnimento graduale)
            if t < 0.3:
                angular_velocity = 2 * math.pi / 0.3 * t
            elif t < 0.7:
                angular_velocity = 2 * math.pi / 0.3
            else:
                angular_velocity = 2 * math.pi / 0.3 * (1 - (t - 0.7) / 0.3)
            
            # Calcola orientazione target
            if direction == "forward":
                target_roll = 2 * math.pi * progress
                target_pitch = 0.0
            elif direction == "backward":
                target_roll = -2 * math.pi * progress
                target_pitch = 0.0
            elif direction == "left":
                target_roll = 0.0
                target_pitch = -2 * math.pi * progress
            elif direction == "right":
                target_roll = 0.0
                target_pitch = 2 * math.pi * progress
            
            # Normalizza angoli
            target_roll = self._normalize_angle(target_roll)
            target_pitch = self._normalize_angle(target_pitch)
            
            trajectory.append({
                'timestamp': t * duration,
                'target_orientation': np.array([target_roll, target_pitch, 0.0]),
                'angular_velocity': angular_velocity,
                'thrust_boost': 1.2 if t < 0.5 else 1.0  # Boost durante il flip
            })
        
        return trajectory
    
    def generate_spiral_trajectory(self, radius: float = 1.0, height_gain: float = 2.0, 
                                 duration: float = 10.0) -> List[Dict]:
        """Genera traiettoria a spirale"""
        trajectory = []
        steps = int(duration * 50)  # 50Hz
        
        for i in range(steps):
            t = i / steps
            angle = 2 * math.pi * 3 * t  # 3 giri completi
            
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            z = height_gain * t
            
            trajectory.append({
                'timestamp': t * duration,
                'target_position': np.array([x, y, z]),
                'target_orientation': np.array([0.0, 0.0, angle]),
                'velocity': np.array([
                    -radius * 2 * math.pi * 3 * math.sin(angle) / duration,
                    radius * 2 * math.pi * 3 * math.cos(angle) / duration,
                    height_gain / duration
                ])
            })
        
        return trajectory
    
    def generate_figure8_trajectory(self, width: float = 2.0, height: float = 1.0, 
                                  duration: float = 8.0) -> List[Dict]:
        """Genera traiettoria a forma di 8"""
        trajectory = []
        steps = int(duration * 50)  # 50Hz
        
        for i in range(steps):
            t = i / steps
            angle = 2 * math.pi * t
            
            # Parametrica per figura 8
            x = width * math.sin(angle)
            y = height * math.sin(angle) * math.cos(angle)
            z = 0.7  # Altitudine costante
            
            trajectory.append({
                'timestamp': t * duration,
                'target_position': np.array([x, y, z]),
                'target_orientation': np.array([0.0, 0.0, angle]),
                'velocity': np.array([
                    width * math.cos(angle) * 2 * math.pi / duration,
                    height * (math.cos(angle)**2 - math.sin(angle)**2) * 2 * math.pi / duration,
                    0.0
                ])
            })
        
        return trajectory
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalizza un angolo tra -π e π"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

class AdvancedFlightController(Node):
    """
    Controller di volo avanzato per manovre complesse
    """
    
    def __init__(self):
        super().__init__('advanced_flight_controller')
        
        # Parametri del drone
        self.drone_mass = 0.027
        self.rotor_count = 4
        self.max_thrust = 0.6  # N per rotore
        self.max_angular_velocity = 1000  # rad/s
        
        # Stato corrente
        self.current_state = FlightState(
            position=np.zeros(3),
            velocity=np.zeros(3),
            orientation=np.zeros(3),
            angular_velocity=np.zeros(3),
            timestamp=0.0
        )
        
        # Controllori
        self.position_controller = PIDController3D(kp=2.0, ki=0.1, kd=0.5)
        self.orientation_controller = PIDController3D(kp=3.0, ki=0.1, kd=0.8)
        self.velocity_controller = PIDController3D(kp=1.5, ki=0.05, kd=0.3)
        
        # Generatore di traiettorie
        self.trajectory_generator = TrajectoryGenerator()
        
        # Stato delle manovre
        self.current_maneuver = None
        self.maneuver_start_time = 0.0
        self.maneuver_trajectory = []
        self.maneuver_step = 0
        
        # Configurazioni delle manovre
        self.maneuver_configs = {
            ManeuverType.FLIP_FORWARD: ManeuverConfig(
                ManeuverType.FLIP_FORWARD, 1.0,
                {'thrust_boost': 1.2, 'angular_velocity': 2*math.pi/0.3},
                {'max_angular_velocity': 15.0, 'max_tilt': math.pi/2}
            ),
            ManeuverType.FLIP_BACKWARD: ManeuverConfig(
                ManeuverType.FLIP_BACKWARD, 1.0,
                {'thrust_boost': 1.2, 'angular_velocity': 2*math.pi/0.3},
                {'max_angular_velocity': 15.0, 'max_tilt': math.pi/2}
            ),
            ManeuverType.FLIP_LEFT: ManeuverConfig(
                ManeuverType.FLIP_LEFT, 1.0,
                {'thrust_boost': 1.2, 'angular_velocity': 2*math.pi/0.3},
                {'max_angular_velocity': 15.0, 'max_tilt': math.pi/2}
            ),
            ManeuverType.FLIP_RIGHT: ManeuverConfig(
                ManeuverType.FLIP_RIGHT, 1.0,
                {'thrust_boost': 1.2, 'angular_velocity': 2*math.pi/0.3},
                {'max_angular_velocity': 15.0, 'max_tilt': math.pi/2}
            ),
            ManeuverType.SPIRAL: ManeuverConfig(
                ManeuverType.SPIRAL, 10.0,
                {'radius': 1.0, 'height_gain': 2.0},
                {'max_velocity': 2.0, 'max_altitude': 3.0}
            ),
            ManeuverType.FIGURE_8: ManeuverConfig(
                ManeuverType.FIGURE_8, 8.0,
                {'width': 2.0, 'height': 1.0},
                {'max_velocity': 1.5, 'max_radius': 2.0}
            )
        }
        
        # Setup QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.rotor_commands_pub = self.create_publisher(
            Float32MultiArray,
            '/crazyflie/advanced_rotor_commands',
            qos_profile
        )
        
        self.maneuver_status_pub = self.create_publisher(
            String,
            '/crazyflie/maneuver_status',
            qos_profile
        )
        
        # Subscribers
        self.odometry_sub = self.create_subscription(
            Odometry,
            '/crazyflie/odometry',
            self.odometry_callback,
            qos_profile
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/crazyflie/imu',
            self.imu_callback,
            qos_profile
        )
        
        # Command subscribers
        self.maneuver_command_sub = self.create_subscription(
            String,
            '/crazyflie/maneuver_command',
            self.maneuver_command_callback,
            qos_profile
        )
        
        self.trajectory_command_sub = self.create_subscription(
            MultiDOFJointTrajectory,
            '/crazyflie/trajectory_command',
            self.trajectory_command_callback,
            qos_profile
        )
        
        # Timer per il controllo
        self.control_timer = self.create_timer(0.02, self.control_loop)  # 50Hz
        
        self.get_logger().info("Advanced Flight Controller initialized")
    
    def odometry_callback(self, msg: Odometry):
        """Callback per odometria"""
        self.current_state.position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
        self.current_state.velocity = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])
        
        # Converti quaternione in euler
        q = msg.pose.pose.orientation
        self.current_state.orientation = self._quaternion_to_euler(q.x, q.y, q.z, q.w)
        
        self.current_state.timestamp = self.get_clock().now().nanoseconds / 1e9
    
    def imu_callback(self, msg: Imu):
        """Callback per IMU"""
        self.current_state.angular_velocity = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
    
    def maneuver_command_callback(self, msg: String):
        """Callback per comandi di manovra"""
        try:
            command = json.loads(msg.data)
            maneuver_type = ManeuverType(command.get('maneuver_type', 'hover'))
            
            self.execute_maneuver(maneuver_type, command.get('parameters', {}))
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid JSON in maneuver command: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing maneuver command: {e}")
    
    def trajectory_command_callback(self, msg: MultiDOFJointTrajectory):
        """Callback per comandi di traiettoria personalizzata"""
        # Converti traiettoria ROS2 in formato interno
        trajectory = []
        for point in msg.points:
            trajectory.append({
                'timestamp': point.time_from_start.nanosec / 1e9,
                'target_position': np.array([
                    point.transforms[0].translation.x,
                    point.transforms[0].translation.y,
                    point.transforms[0].translation.z
                ]),
                'target_orientation': self._quaternion_to_euler(
                    point.transforms[0].rotation.x,
                    point.transforms[0].rotation.y,
                    point.transforms[0].rotation.z,
                    point.transforms[0].rotation.w
                )
            })
        
        self.execute_custom_trajectory(trajectory)
    
    def execute_maneuver(self, maneuver_type: ManeuverType, parameters: Dict = None):
        """Esegue una manovra specifica"""
        if self.current_maneuver is not None:
            self.get_logger().warn("Maneuver already in progress")
            return
        
        config = self.maneuver_configs.get(maneuver_type)
        if not config:
            self.get_logger().error(f"Unknown maneuver type: {maneuver_type}")
            return
        
        # Merge parametri
        if parameters:
            config.parameters.update(parameters)
        
        self.current_maneuver = maneuver_type
        self.maneuver_start_time = self.current_state.timestamp
        self.maneuver_step = 0
        
        # Genera traiettoria
        if maneuver_type in [ManeuverType.FLIP_FORWARD, ManeuverType.FLIP_BACKWARD, 
                           ManeuverType.FLIP_LEFT, ManeuverType.FLIP_RIGHT]:
            direction = maneuver_type.value.split('_')[1]
            self.maneuver_trajectory = self.trajectory_generator.generate_flip_trajectory(
                direction, config.duration
            )
        elif maneuver_type == ManeuverType.SPIRAL:
            self.maneuver_trajectory = self.trajectory_generator.generate_spiral_trajectory(
                config.parameters.get('radius', 1.0),
                config.parameters.get('height_gain', 2.0),
                config.duration
            )
        elif maneuver_type == ManeuverType.FIGURE_8:
            self.maneuver_trajectory = self.trajectory_generator.generate_figure8_trajectory(
                config.parameters.get('width', 2.0),
                config.parameters.get('height', 1.0),
                config.duration
            )
        else:
            self.maneuver_trajectory = []
        
        self.get_logger().info(f"Starting maneuver: {maneuver_type.value}")
    
    def execute_custom_trajectory(self, trajectory: List[Dict]):
        """Esegue una traiettoria personalizzata"""
        if self.current_maneuver is not None:
            self.get_logger().warn("Maneuver already in progress")
            return
        
        self.current_maneuver = ManeuverType.CUSTOM_TRAJECTORY
        self.maneuver_start_time = self.current_state.timestamp
        self.maneuver_step = 0
        self.maneuver_trajectory = trajectory
        
        self.get_logger().info("Starting custom trajectory")
    
    def control_loop(self):
        """Loop principale di controllo"""
        if self.current_maneuver is not None:
            self._execute_maneuver_control()
        else:
            self._execute_hover_control()
        
        # Pubblica comandi
        self._publish_rotor_commands()
        self._publish_maneuver_status()
    
    def _execute_maneuver_control(self):
        """Controllo durante una manovra"""
        if not self.maneuver_trajectory:
            self._end_maneuver()
            return
        
        elapsed_time = self.current_state.timestamp - self.maneuver_start_time
        
        # Trova il punto corrente nella traiettoria
        current_point = None
        for i, point in enumerate(self.maneuver_trajectory):
            if point['timestamp'] >= elapsed_time:
                current_point = point
                self.maneuver_step = i
                break
        
        if current_point is None:
            # Manovra completata
            self._end_maneuver()
            return
        
        # Applica controllo
        if 'target_position' in current_point:
            position_error = current_point['target_position'] - self.current_state.position
            position_cmd = self.position_controller.update(position_error, self.current_state.timestamp)
        else:
            position_cmd = np.zeros(3)
        
        if 'target_orientation' in current_point:
            orientation_error = current_point['target_orientation'] - self.current_state.orientation
            # Normalizza errori
            for i in range(3):
                while orientation_error[i] > math.pi:
                    orientation_error[i] -= 2 * math.pi
                while orientation_error[i] < -math.pi:
                    orientation_error[i] += 2 * math.pi
            
            orientation_cmd = self.orientation_controller.update(orientation_error, self.current_state.timestamp)
        else:
            orientation_cmd = np.zeros(3)
        
        # Boost di spinta per manovre acrobatiche
        thrust_boost = current_point.get('thrust_boost', 1.0)
        
        # Calcola comandi dei rotori
        self._calculate_advanced_rotor_commands(position_cmd, orientation_cmd, thrust_boost)
    
    def _execute_hover_control(self):
        """Controllo di hover normale"""
        # Target di hover
        target_position = np.array([0.0, 0.0, 0.7])
        target_orientation = np.array([0.0, 0.0, 0.0])
        
        # Calcola errori
        position_error = target_position - self.current_state.position
        orientation_error = target_orientation - self.current_state.orientation
        
        # Normalizza errori di orientazione
        for i in range(3):
            while orientation_error[i] > math.pi:
                orientation_error[i] -= 2 * math.pi
            while orientation_error[i] < -math.pi:
                orientation_error[i] += 2 * math.pi
        
        # Calcola comandi
        position_cmd = self.position_controller.update(position_error, self.current_state.timestamp)
        orientation_cmd = self.orientation_controller.update(orientation_error, self.current_state.timestamp)
        
        # Calcola comandi dei rotori
        self._calculate_advanced_rotor_commands(position_cmd, orientation_cmd, 1.0)
    
    def _calculate_advanced_rotor_commands(self, position_cmd: np.ndarray, 
                                         orientation_cmd: np.ndarray, thrust_boost: float = 1.0):
        """Calcola comandi avanzati dei rotori"""
        # Forza di spinta totale con boost
        total_thrust = (position_cmd[2] + self.drone_mass * 9.81) * thrust_boost
        
        # Coppie per roll, pitch, yaw
        roll_torque = orientation_cmd[0]
        pitch_torque = orientation_cmd[1]
        yaw_torque = orientation_cmd[2]
        
        # Matrice di allocazione dei rotori (più sofisticata)
        allocation_matrix = np.array([
            [1, 1, 1, 1],           # Thrust
            [0.046, -0.046, -0.046, 0.046],  # Roll
            [0.046, 0.046, -0.046, -0.046],  # Pitch
            [1, -1, 1, -1]          # Yaw
        ])
        
        # Vettore dei comandi
        commands = np.array([total_thrust, roll_torque, pitch_torque, yaw_torque])
        
        # Calcola le velocità dei rotori con limiti di sicurezza
        try:
            rotor_velocities = np.linalg.solve(allocation_matrix, commands)
            
            # Applica limiti di sicurezza
            for i in range(len(rotor_velocities)):
                rotor_velocities[i] = np.clip(rotor_velocities[i], 0, self.max_angular_velocity)
                
        except np.linalg.LinAlgError:
            # Fallback se la matrice è singolare
            rotor_velocities = np.array([total_thrust/4] * 4)
        
        # Salva i comandi per la pubblicazione
        self.rotor_commands = []
        for i in range(self.rotor_count):
            thrust = max(0.0, min(1.0, rotor_velocities[i] / self.max_angular_velocity))
            self.rotor_commands.append({
                'rotor_id': i,
                'thrust': thrust,
                'angular_velocity': rotor_velocities[i]
            })
    
    def _end_maneuver(self):
        """Termina la manovra corrente"""
        self.current_maneuver = None
        self.maneuver_trajectory = []
        self.maneuver_step = 0
        
        # Reset dei controllori
        self.position_controller.reset()
        self.orientation_controller.reset()
        self.velocity_controller.reset()
        
        self.get_logger().info("Maneuver completed")
    
    def _publish_rotor_commands(self):
        """Pubblica i comandi dei rotori"""
        if not hasattr(self, 'rotor_commands'):
            return
        
        msg = Float32MultiArray()
        for cmd in self.rotor_commands:
            msg.data.append(cmd['thrust'])
            msg.data.append(cmd['angular_velocity'])
        
        self.rotor_commands_pub.publish(msg)
    
    def _publish_maneuver_status(self):
        """Pubblica lo stato della manovra"""
        status = {
            'maneuver_type': self.current_maneuver.value if self.current_maneuver else 'none',
            'progress': self.maneuver_step / len(self.maneuver_trajectory) if self.maneuver_trajectory else 0.0,
            'elapsed_time': self.current_state.timestamp - self.maneuver_start_time if self.current_maneuver else 0.0,
            'position': self.current_state.position.tolist(),
            'orientation': self.current_state.orientation.tolist()
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self.maneuver_status_pub.publish(msg)
    
    def _quaternion_to_euler(self, x: float, y: float, z: float, w: float) -> np.ndarray:
        """Converte quaternione in angoli di Eulero"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return np.array([roll, pitch, yaw])

class PIDController3D:
    """Controller PID 3D per posizione, orientazione e velocità"""
    
    def __init__(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = np.zeros(3)
        self.previous_error = np.zeros(3)
        self.last_time = 0.0
    
    def update(self, error: np.ndarray, current_time: float) -> np.ndarray:
        """Aggiorna il controller PID"""
        if self.last_time == 0.0:
            self.last_time = current_time
            return np.zeros(3)
        
        dt = current_time - self.last_time
        if dt <= 0:
            return np.zeros(3)
        
        # Calcola il termine integrale
        self.integral += error * dt
        
        # Calcola il termine derivativo
        derivative = (error - self.previous_error) / dt
        
        # Calcola l'output
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        # Aggiorna lo stato
        self.previous_error = error.copy()
        self.last_time = current_time
        
        return output
    
    def reset(self):
        """Resetta il controller"""
        self.integral = np.zeros(3)
        self.previous_error = np.zeros(3)
        self.last_time = 0.0

def main(args=None):
    rclpy.init(args=args)
    
    controller = AdvancedFlightController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 