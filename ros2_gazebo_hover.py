# ... tutte le importazioni all'inizio (come nel tuo script) ...
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import PoseStamped
import json
import time
import random
import math

class GazeboHoverController(Node):
    def __init__(self):
        super().__init__('gazebo_hover_controller')
        self.target_altitude = 0.7
        self.current_altitude = 0.0
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        self.hover_omega = 339
        self.kp = 200
        self.override_until = 0
        self.manual_rotor_cmd = None

        self.MAX_ALTITUDE = 6
        self.MAX_CLIMB_INCREMENT = 1.0  # Limite massimo per il climb in una volta

        # Flip
        self.flip_active = False
        self.flip_direction = None
        self.flip_start_time = 0
        self.flip_duration = 1.5
        self.flip_timeout = 3.0
        self.flip_start_roll = 0.0
        self.flip_start_pitch = 0.0
        self.flip_target_roll = 0.0
        self.flip_target_pitch = 0.0
        self.flip_phase = 1

        # PID
        self.pid_roll = {'kp': 15.0, 'ki': 0.1, 'kd': 3.0, 'prev_error': 0.0, 'integral': 0.0}
        self.pid_pitch = {'kp': 15.0, 'ki': 0.1, 'kd': 3.0, 'prev_error': 0.0, 'integral': 0.0}

        self.rotor_pub = self.create_publisher(Float32MultiArray, '/sim_crazyflie/rotor_speeds', 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/model_pose', self.pose_callback, 10)
        self.cmd_sub = self.create_subscription(String, '/hover_cmd', self.cmd_callback, 10)
        self.create_timer(0.05, self.control_loop)

        self.recovering = False
        self.recover_start_time = 0
        self.recover_duration = 1.0  # secondi

        self.led_effect = None
        self.last_led_effect = None
        self.last_led_effect_red_printed = False
        self.sudden_drop_original_target = None  # Per ripristinare la quota dopo il drop

    def pose_callback(self, msg):
        self.current_altitude = msg.pose.position.z
        q = msg.pose.orientation
        x, y, z, w = q.x, q.y, q.z, q.w
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = max(min(t2, 1.0), -1.0)
        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        self.current_roll = roll
        self.current_pitch = pitch
        self.current_yaw = yaw

    def cmd_callback(self, msg):
        try:
            cmd = json.loads(msg.data)
            if 'target_altitude' in cmd:
                self.target_altitude = float(cmd['target_altitude'])
                self.get_logger().info(f"Nuova quota target: {self.target_altitude}m")
            if 'rotor_speeds' in cmd:
                self.manual_rotor_cmd = [float(x) for x in cmd['rotor_speeds']]
                self.override_until = time.time() + 1.5
                self.get_logger().info(f"Override manuale rotori: {self.manual_rotor_cmd}")
            if 'flip' in cmd:
                direction = cmd['flip'].lower()
                self.start_boost_flip(direction)
            if 'sudden_drop' in cmd and cmd['sudden_drop']:
                if not self.recovering:
                    self.start_sudden_drop()
                else:
                    self.get_logger().info("Sudden drop ignorato: drone in recovery")
        except Exception as e:
            self.get_logger().error(f"Errore parsing comando: {e}")

    def start_boost_flip(self, direction):
        if self.flip_active:
            self.get_logger().warn("Flip già in corso, ignorato")
            return
        self.flip_active = True
        self.flip_type = 'boost'
        self.flip_direction = direction
        self.flip_start_time = time.time()
        self.flip_duration = 0.5
        self.get_logger().info(f"[FLIP BOOST] {direction} iniziato")

    def start_sudden_drop(self):
        if hasattr(self, 'sudden_active') and self.sudden_active:
            return
        drop = random.uniform(0.15, 0.4)
        # Salva la quota target originale solo se non già in drop
        if self.sudden_drop_original_target is None:
            self.sudden_drop_original_target = self.target_altitude
        new_target = max(0.1, self.target_altitude - drop)
        print(f"[SUDDEN DROP] Target altitude: {self.target_altitude:.2f} -> {new_target:.2f} (-{drop:.2f}m)")
        self.target_altitude = new_target
        self.sudden_active = True
        self.sudden_type = 'drop'
        self.sudden_start_time = time.time()
        self.sudden_duration = 0.7
        print("[LED] ROSSO: SUDDEN DROP!")
        self.get_logger().info("[LED] ROSSO: SUDDEN DROP!")

    def pid_update(self, pid, error, dt):
        pid['integral'] += error * dt
        pid['integral'] = max(-10.0, min(10.0, pid['integral']))
        derivative = (error - pid['prev_error']) / dt if dt > 0 else 0.0
        output = pid['kp'] * error + pid['ki'] * pid['integral'] + pid['kd'] * derivative
        pid['prev_error'] = error
        return output

    def normalize_angle_difference(self, target, current):
        diff = target - current
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff

    def set_led_effect(self, effect):
        if self.led_effect != effect:
            self.led_effect = effect
            print(f"[LED EFFECT] {effect}")
            self.get_logger().info(f"[LED EFFECT] {effect}")
        # Se RED, stampa sempre
        if effect.startswith("LED: RED"):
            self.last_led_effect_red_printed = True
        else:
            self.last_led_effect_red_printed = False

    def print_led_red_if_needed(self):
        if hasattr(self, 'led_effect') and self.led_effect and self.led_effect.startswith("LED: RED"):
            print("[LED EFFECT] LED: RED (ALERT)")
            self.get_logger().info("[LED EFFECT] LED: RED (ALERT)")

    def control_loop(self):
        now = time.time()
        msg = Float32MultiArray()
        # CLAMP ALTITUDINE ASSOLUTO (PRIMA DI USARLA)
        if self.target_altitude > self.MAX_ALTITUDE:
            print(f"[WARN] target_altitude clamped da {self.target_altitude:.2f} a {self.MAX_ALTITUDE}")
            self.target_altitude = self.MAX_ALTITUDE

        # --- SUDDEN DROP RANDOM ---
        # Probabilità ≈ 1 ogni 30s: 1/600 ≈ 0.0017
        if not (hasattr(self, 'sudden_active') and self.sudden_active) and not (self.flip_active and getattr(self, 'flip_type', None) == 'geometric') and not self.recovering:
            if random.random() < 0.0017:
                self.start_sudden_drop()
        if hasattr(self, 'sudden_active') and self.sudden_active:
            elapsed = now - self.sudden_start_time
            self.set_led_effect("alert_flashing_red")
            if self.sudden_type == 'drop':
                msg.data = [0.0, 0.0, 0.0, 0.0]
            if elapsed > self.sudden_duration:
                self.sudden_active = False
                self.set_led_effect("adjusting_yellow")
                # Ripristina la quota target originale se era stata salvata
                if self.sudden_drop_original_target is not None:
                    self.target_altitude = self.sudden_drop_original_target
                    self.sudden_drop_original_target = None
            self.rotor_pub.publish(msg)
            return
        if self.flip_active and getattr(self, 'flip_type', None) == 'geometric':
            elapsed = now - self.flip_start_time
            progress = min(elapsed / self.flip_duration, 1.0)
            self.set_led_effect("flip_flashing_blue")
            smooth = 0.5 * (1 - math.cos(progress * math.pi))
            roll_ref = self.flip_start_roll + smooth * (self.flip_target_roll - self.flip_start_roll)
            pitch_ref = self.flip_start_pitch + smooth * (self.flip_target_pitch - self.flip_start_pitch)
            dt = 0.05
            # BOOST PID e omega durante il flip
            pid_roll = {'kp': 60.0, 'ki': 0.2, 'kd': 8.0, 'prev_error': self.pid_roll['prev_error'], 'integral': self.pid_roll['integral']}
            pid_pitch = {'kp': 60.0, 'ki': 0.2, 'kd': 8.0, 'prev_error': self.pid_pitch['prev_error'], 'integral': self.pid_pitch['integral']}
            roll_err = self.normalize_angle_difference(roll_ref, self.current_roll)
            pitch_err = self.normalize_angle_difference(pitch_ref, self.current_pitch)
            roll_cmd = self.pid_update(pid_roll, roll_err, dt)
            pitch_cmd = self.pid_update(pid_pitch, pitch_err, dt)
            error = self.target_altitude - self.current_altitude
            omega = self.hover_omega + self.kp * error + 800  # BOOST di 800
            omega = max(0, min(3000, omega))
            rotors = [
                omega - roll_cmd - pitch_cmd,
                omega + roll_cmd - pitch_cmd,
                omega - roll_cmd + pitch_cmd,
                omega + roll_cmd + pitch_cmd
            ]
            msg.data = [float(max(0, min(3000, r))) for r in rotors]
            self.get_logger().info(f"[FLIP GEOM] roll_ref: {roll_ref:.2f}, pitch_ref: {pitch_ref:.2f}, rotors: {msg.data}")
            if progress >= 1.0:
                self.flip_active = False
                self.recovering = True
                self.recover_start_time = now
                self.set_led_effect("adjusting_yellow")
            self.rotor_pub.publish(msg)
            return
        elif self.flip_active:
            elapsed = now - self.flip_start_time
            omega_base = 1000
            omega_boost = 2500
            # Ordine rotori: [front_left, front_right, rear_left, rear_right]
            if self.flip_direction == 'forward':
                if elapsed < 0.3:
                    cmd = [omega_base, omega_base, omega_boost, omega_boost]
                elif elapsed < 0.5:
                    cmd = [omega_boost, omega_boost, omega_base, omega_base]
                else:
                    self.flip_active = False
                    self.recovering = True
                    self.recover_start_time = now
                    self.get_logger().info("Flip boost completato, inizio auto-livellamento")
                    cmd = None
            elif self.flip_direction == 'backward':
                if elapsed < 0.3:
                    cmd = [omega_boost, omega_boost, omega_base, omega_base]
                elif elapsed < 0.5:
                    cmd = [omega_base, omega_base, omega_boost, omega_boost]
                else:
                    self.flip_active = False
                    self.recovering = True
                    self.recover_start_time = now
                    self.get_logger().info("Flip boost completato, inizio auto-livellamento")
                    cmd = None
            elif self.flip_direction == 'left':
                if elapsed < 0.3:
                    cmd = [omega_base, omega_boost, omega_base, omega_boost]
                elif elapsed < 0.5:
                    cmd = [omega_boost, omega_base, omega_boost, omega_base]
                else:
                    self.flip_active = False
                    self.recovering = True
                    self.recover_start_time = now
                    self.get_logger().info("Flip boost completato, inizio auto-livellamento")
                    cmd = None
            elif self.flip_direction == 'right':
                if elapsed < 0.3:
                    cmd = [omega_boost, omega_base, omega_boost, omega_base]
                elif elapsed < 0.5:
                    cmd = [omega_base, omega_boost, omega_base, omega_boost]
                else:
                    self.flip_active = False
                    self.recovering = True
                    self.recover_start_time = now
                    self.get_logger().info("Flip boost completato, inizio auto-livellamento")
                    cmd = None
            else:
                self.flip_active = False
                self.get_logger().warn(f"Direzione flip sconosciuta: {self.flip_direction}")
                cmd = None
            if self.flip_active and cmd is not None:
                msg.data = [float(x) for x in cmd]
                self.get_logger().info(f"[FLIP BOOST] Rotor speeds: {msg.data}")
            elif not self.flip_active and not self.recovering:
                error = self.target_altitude - self.current_altitude
                omega = self.hover_omega + self.kp * error
                omega = max(0, min(3000, omega))
                msg.data = [float(omega)] * 4
                self.manual_rotor_cmd = None
        elif self.recovering:
            self.set_led_effect("adjusting_yellow")
            roll_error = -self.current_roll
            pitch_error = -self.current_pitch
            roll_gain = 800  # più aggressivo
            pitch_gain = 800
            error = self.target_altitude - self.current_altitude
            omega = self.hover_omega + self.kp * error
            omega = max(0, min(3000, omega))
            rotors = [
                omega - roll_gain * roll_error - pitch_gain * pitch_error,
                omega + roll_gain * roll_error - pitch_gain * pitch_error,
                omega - roll_gain * roll_error + pitch_gain * pitch_error,
                omega + roll_gain * roll_error + pitch_gain * pitch_error
            ]
            msg.data = [float(max(0, min(3000, r))) for r in rotors]
            self.get_logger().info(f"[RECOVERY] roll_err: {roll_error:.3f}, pitch_err: {pitch_error:.3f}, omega: {omega:.1f}, rotors: {msg.data}")
            # Esci solo se roll e pitch sono davvero piccoli
            if abs(roll_error) < 0.05 and abs(pitch_error) < 0.05:
                self.recovering = False
                self.set_led_effect("stable_green")
            self.rotor_pub.publish(msg)
            return
        elif self.manual_rotor_cmd and now < self.override_until:
            self.set_led_effect("adjusting_yellow")
            msg.data = [float(x) for x in self.manual_rotor_cmd]
            self.get_logger().info(f"[MANUALE] Rotor speeds: {msg.data}")
        else:
            # --- LOGICA LED E STAMPA OMEGA IN HOVER ---
            error = self.target_altitude - self.current_altitude
            abs_error = abs(error)
            if abs_error < 0.03:
                self.set_led_effect("LED: GREEN (HOLD)")
            elif abs_error < 0.12:
                self.set_led_effect("LED: YELLOW (NEAR TARGET)")
            else:
                self.set_led_effect("LED: RED (ALERT)")
            self.print_led_red_if_needed()
            omega = self.hover_omega + self.kp * error
            omega = max(0, min(3000, omega))
            msg.data = [float(omega)] * 4
            self.get_logger().info(f"[AUTO] Alt: {self.current_altitude:.2f}m, omega: {omega:.1f}")
            self.manual_rotor_cmd = None
        self.rotor_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GazeboHoverController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()