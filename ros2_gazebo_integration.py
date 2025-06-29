import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped, Vector3
from sensor_msgs.msg import Imu, BatteryState
from std_msgs.msg import Float32MultiArray, Bool, String
from nav_msgs.msg import Odometry
import numpy as np
import math
import time
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
import threading
import queue

@dataclass
class RotorCommand:
    """Comando per un singolo rotore"""
    rotor_id: int
    thrust: float  # 0.0 - 1.0
    angular_velocity: float  # rad/s

@dataclass
class DroneState:
    """Stato completo del drone"""
    position: np.ndarray  # [x, y, z]
    velocity: np.ndarray  # [vx, vy, vz]
    orientation: np.ndarray  # [roll, pitch, yaw] in radians
    angular_velocity: np.ndarray  # [wx, wy, wz]
    battery_level: float
    timestamp: float

class CrazyflieRotorController(Node):
    """
    Controller ROS2 per il controllo a livello di rotori del Crazyflie
    Permette manovre avanzate come flip, hover, e controllo di precisione
    """
    
    def __init__(self):
        super().__init__('crazyflie_rotor_controller')
        
        # Parametri del drone
        self.drone_mass = 0.027  # kg
        self.rotor_count = 4
        self.rotor_positions = np.array([
            [0.046, 0.046, 0],   # Front Right
            [-0.046, 0.046, 0],  # Front Left
            [-0.046, -0.046, 0], # Back Left
            [0.046, -0.046, 0]   # Back Right
        ])
        
        # Coefficienti di spinta e coppia
        self.thrust_coefficient = 1.0e-6  # N/(rad/s)^2
        self.torque_coefficient = 1.0e-8  # Nm/(rad/s)^2
        
        # Stato corrente del drone
        self.current_state = DroneState(
            position=np.zeros(3),
            velocity=np.zeros(3),
            orientation=np.zeros(3),
            angular_velocity=np.zeros(3),
            battery_level=100.0,
            timestamp=0.0
        )
        
        # Comandi dei rotori
        self.rotor_commands = [RotorCommand(i, 0.0, 0.0) for i in range(self.rotor_count)]
        self.target_position = np.array([0.0, 0.0, 0.7])
        self.target_orientation = np.array([0.0, 0.0, 0.0])
        
        # Controllori PID
        self.position_pid = PIDController(kp=2.0, ki=0.1, kd=0.5)
        self.orientation_pid = PIDController(kp=3.0, ki=0.1, kd=0.8)
        
        # Flag per manovre speciali
        self.flip_in_progress = False
        self.flip_start_time = 0.0
        self.flip_duration = 1.0  # secondi
        
        # Setup QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.rotor_commands_pub = self.create_publisher(
            Float32MultiArray, 
            '/crazyflie/rotor_commands', 
            qos_profile
        )
        
        self.drone_status_pub = self.create_publisher(
            String,
            '/crazyflie/status',
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
        
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/crazyflie/battery',
            self.battery_callback,
            qos_profile
        )
        
        # Command subscribers
        self.position_cmd_sub = self.create_subscription(
            PoseStamped,
            '/crazyflie/position_command',
            self.position_command_callback,
            qos_profile
        )
        
        self.flip_cmd_sub = self.create_subscription(
            String,
            '/crazyflie/flip_command',
            self.flip_command_callback,
            qos_profile
        )
        
        # Timer per il controllo
        self.control_timer = self.create_timer(0.02, self.control_loop)  # 50Hz
        
        # Thread per calcoli complessi
        self.calculation_queue = queue.Queue()
        self.calculation_thread = threading.Thread(target=self._calculation_worker, daemon=True)
        self.calculation_thread.start()
        
        self.get_logger().info("Crazyflie Rotor Controller initialized")
    
    def odometry_callback(self, msg: Odometry):
        """Callback per i dati di odometria"""
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
        
        # Converti quaternione in euler angles
        q = msg.pose.pose.orientation
        self.current_state.orientation = self._quaternion_to_euler(q.x, q.y, q.z, q.w)
        
        self.current_state.timestamp = self.get_clock().now().nanoseconds / 1e9
    
    def imu_callback(self, msg: Imu):
        """Callback per i dati IMU"""
        self.current_state.angular_velocity = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
    
    def battery_callback(self, msg: BatteryState):
        """Callback per lo stato della batteria"""
        self.current_state.battery_level = msg.percentage * 100.0
    
    def position_command_callback(self, msg: PoseStamped):
        """Callback per comandi di posizione"""
        self.target_position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        
        q = msg.pose.orientation
        self.target_orientation = self._quaternion_to_euler(q.x, q.y, q.z, q.w)
        
        self.get_logger().info(f"New position target: {self.target_position}")
    
    def flip_command_callback(self, msg: String):
        """Callback per comandi di flip"""
        if msg.data == "flip_forward":
            self.execute_flip("forward")
        elif msg.data == "flip_backward":
            self.execute_flip("backward")
        elif msg.data == "flip_left":
            self.execute_flip("left")
        elif msg.data == "flip_right":
            self.execute_flip("right")
    
    def execute_flip(self, direction: str):
        """Esegue un flip nella direzione specificata"""
        if self.flip_in_progress:
            self.get_logger().warn("Flip already in progress")
            return
        
        self.flip_in_progress = True
        self.flip_start_time = self.current_state.timestamp
        self.flip_direction = direction
        
        self.get_logger().info(f"Starting {direction} flip")
    
    def control_loop(self):
        """Loop principale di controllo"""
        if self.flip_in_progress:
            self._execute_flip_control()
        else:
            self._execute_normal_control()
        
        # Pubblica i comandi dei rotori
        self._publish_rotor_commands()
        
        # Pubblica lo stato del drone
        self._publish_drone_status()
    
    def _execute_normal_control(self):
        """Controllo normale di posizione e orientazione"""
        # Calcola errori
        position_error = self.target_position - self.current_state.position
        orientation_error = self.target_orientation - self.current_state.orientation
        
        # Normalizza errori di orientazione
        for i in range(3):
            while orientation_error[i] > math.pi:
                orientation_error[i] -= 2 * math.pi
            while orientation_error[i] < -math.pi:
                orientation_error[i] += 2 * math.pi
        
        # Calcola comandi PID
        position_cmd = self.position_pid.update(position_error, self.current_state.timestamp)
        orientation_cmd = self.orientation_pid.update(orientation_error, self.current_state.timestamp)
        
        # Converti in comandi dei rotori
        self._calculate_rotor_commands(position_cmd, orientation_cmd)
    
    def _execute_flip_control(self):
        """Controllo durante il flip"""
        elapsed_time = self.current_state.timestamp - self.flip_start_time
        progress = elapsed_time / self.flip_duration
        
        if progress >= 1.0:
            # Flip completato
            self.flip_in_progress = False
            self.get_logger().info("Flip completed")
            return
        
        # Calcola il profilo del flip
        if self.flip_direction == "forward":
            target_roll = math.pi * 2 * progress
            target_pitch = 0.0
        elif self.flip_direction == "backward":
            target_roll = -math.pi * 2 * progress
            target_pitch = 0.0
        elif self.flip_direction == "left":
            target_roll = 0.0
            target_pitch = -math.pi * 2 * progress
        elif self.flip_direction == "right":
            target_roll = 0.0
            target_pitch = math.pi * 2 * progress
        
        # Normalizza target_roll e target_pitch
        while target_roll > math.pi:
            target_roll -= 2 * math.pi
        while target_pitch > math.pi:
            target_pitch -= 2 * math.pi
        
        # Calcola comandi per il flip
        orientation_error = np.array([target_roll, target_pitch, 0.0]) - self.current_state.orientation
        orientation_cmd = self.orientation_pid.update(orientation_error, self.current_state.timestamp)
        
        # Durante il flip, mantieni l'altitudine
        position_cmd = np.array([0.0, 0.0, 0.0])  # Solo controllo verticale
        
        self._calculate_rotor_commands(position_cmd, orientation_cmd)
    
    def _calculate_rotor_commands(self, position_cmd: np.ndarray, orientation_cmd: np.ndarray):
        """Calcola i comandi dei rotori dai comandi di posizione e orientazione"""
        # Forza di spinta totale (per il controllo verticale)
        total_thrust = position_cmd[2] + self.drone_mass * 9.81
        
        # Coppie per roll, pitch, yaw
        roll_torque = orientation_cmd[0]
        pitch_torque = orientation_cmd[1]
        yaw_torque = orientation_cmd[2]
        
        # Matrice di allocazione dei rotori
        # [FR, FL, BL, BR] per thrust e [FR, FL, BL, BR] per torques
        allocation_matrix = np.array([
            [1, 1, 1, 1],           # Thrust
            [0.046, -0.046, -0.046, 0.046],  # Roll
            [0.046, 0.046, -0.046, -0.046],  # Pitch
            [1, -1, 1, -1]          # Yaw
        ])
        
        # Vettore dei comandi
        commands = np.array([total_thrust, roll_torque, pitch_torque, yaw_torque])
        
        # Calcola le velocità dei rotori
        try:
            rotor_velocities = np.linalg.solve(allocation_matrix, commands)
        except np.linalg.LinAlgError:
            # Fallback se la matrice è singolare
            rotor_velocities = np.array([total_thrust/4] * 4)
        
        # Converti in comandi di thrust (0-1)
        for i in range(self.rotor_count):
            # Assicurati che i valori siano nel range valido
            thrust = max(0.0, min(1.0, rotor_velocities[i] / 1000.0))  # Normalizza
            self.rotor_commands[i].thrust = thrust
            self.rotor_commands[i].angular_velocity = rotor_velocities[i]
    
    def _publish_rotor_commands(self):
        """Pubblica i comandi dei rotori"""
        msg = Float32MultiArray()
        for cmd in self.rotor_commands:
            msg.data.append(cmd.thrust)
            msg.data.append(cmd.angular_velocity)
        
        self.rotor_commands_pub.publish(msg)
    
    def _publish_drone_status(self):
        """Pubblica lo stato del drone"""
        status = f"pos:{self.current_state.position},orient:{self.current_state.orientation},battery:{self.current_state.battery_level:.1f}"
        if self.flip_in_progress:
            status += f",flip:{self.flip_direction}"
        
        msg = String()
        msg.data = status
        self.drone_status_pub.publish(msg)
    
    def _quaternion_to_euler(self, x: float, y: float, z: float, w: float) -> np.ndarray:
        """Converte quaternione in angoli di Eulero (roll, pitch, yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return np.array([roll, pitch, yaw])
    
    def _calculation_worker(self):
        """Worker thread per calcoli complessi"""
        while True:
            try:
                task = self.calculation_queue.get(timeout=1.0)
                # Esegui calcoli complessi qui se necessario
                self.calculation_queue.task_done()
            except queue.Empty:
                continue

class PIDController:
    """Controller PID semplice"""
    
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
    
    controller = CrazyflieRotorController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 