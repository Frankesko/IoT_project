import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import PoseStamped, Twist
import json
import time
import random
import math
from scipy.spatial.transform import Rotation as R

class GazeboHoverController(Node):
    def __init__(self):
        #setting nodo ROS2 e variabili di stato
        super().__init__('gazebo_hover_controller')
        self.target_altitude = 0.7  #quota target di default (m)
        self.current_altitude = 0.0  #quota attuale
        
        self.current_roll = 0.0      #roll attuale (rad) (inclinazione destra/sinistra)
        self.current_pitch = 0.0     #pitch attuale (rad) (inclinazione avanti/indietro)
        self.current_yaw = 0.0       #yaw attuale (rad) (orientamento)

        self.current_linear_vel = [0.0, 0.0, 0.0]  #velocità lineari
        self.current_angular_vel = [0.0, 0.0, 0.0] #velocità angolari

        self.hover_omega = 339       #velocità rotori per hover
        self.kp = 200                #guadagno proporzionale per controllo altitudine
        self.override_until = 0      #timeout per override manuale
        self.manual_rotor_cmd = None #comando manuale rotori

        self.MAX_ALTITUDE = 6        #quota massima consentita
        self.MAX_CLIMB_INCREMENT = 1.0 #incremento massimo salita

        # --- Parametri e stato per la manovra di flip a 360° ---
        self.flip_active = False
        self.flip_direction = None
        self.flip_start_time = 0

        #definizione fasi flip
        self.FLIP_PHASE_PREP = 0      #preparazione e salita
        self.FLIP_PHASE_ROTATE = 1    #rotazione pura
        self.FLIP_PHASE_BRAKE = 2     #frenata quando vicino a 290°
        self.FLIP_PHASE_RECOVER = 3   #recupero e stabilizzazione finale

        self.flip_phase = self.FLIP_PHASE_PREP
        self.flip_phase_start_time = 0

        #durate delle varie fasi del flip
        self.FLIP_PREP_DURATION = 0.5    
        self.FLIP_ROTATE_DURATION = 3.0  
        self.FLIP_BRAKE_DURATION = 0.8   
        self.FLIP_RECOVER_DURATION = 4.0 

        #variabili tracking rotazione accumulata
        self.flip_start_angle = 0.0
        self.flip_total_rotation = 0.0
        self.flip_prev_angle = 0.0
        self.flip_rotation_target = 2 * math.pi  #360° in radianti
        self.flip_rotation_completed = False

        #soglie per cambio fase durante il flip
        self.FLIP_BRAKE_THRESHOLD = 5.0   #soglia per iniziare frenata (~287°)
        self.FLIP_COMPLETE_THRESHOLD = 5.8  #soglia per considerare il flip completo (~332°)
        self.FLIP_OVERSHOOT_LIMIT = 6.5   #limite massimo rotazione (~372°)

        #altitudine di riferimento per il flip
        self.flip_target_altitude = 0.0
        self.flip_original_altitude = 0.0

        # --- Publisher e Subscriber ROS2 ---
        self.rotor_pub = self.create_publisher(Float32MultiArray, '/sim_crazyflie/rotor_speeds', 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/model_pose', self.pose_callback, 10)
        self.twist_sub = self.create_subscription(Twist, '/model_twist', self.twist_callback, 10)
        self.cmd_sub = self.create_subscription(String, '/hover_cmd', self.cmd_callback, 10)
        self.create_timer(0.02, self.control_loop)  #timer per il ciclo di controllo (50Hz)

        #stato per recovery e sudden drop
        self.recovering = False
        self.recover_start_time = 0
        self.recover_duration = 2.0
        self.led_effect = None
        self.sudden_drop_original_target = None
        self.current_quat = [0, 0, 0, 1]

    def pose_callback(self, msg):
        #callback per aggiornare posizione e orientamento dal topic /model_pose
        self.current_altitude = msg.pose.position.z
        q = msg.pose.orientation
        x, y, z, w = q.x, q.y, q.z, q.w
        
        #conversione quaternion a angoli di Eulero
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
        self.current_quat = [x, y, z, w]

    def twist_callback(self, msg):
        #callback per aggiornare velocità lineari e angolari dal topic /model_twist
        self.current_linear_vel = [msg.linear.x, msg.linear.y, msg.linear.z]
        self.current_angular_vel = [msg.angular.x, msg.angular.y, msg.angular.z]

    def cmd_callback(self, msg):
        #callback per ricevere comandi esterni (quota, flip, override rotori, sudden drop)
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
                self.start_flip(direction)
            if 'sudden_drop' in cmd and cmd['sudden_drop']:
                if not self.recovering:
                    self.start_sudden_drop()
                else:
                    self.get_logger().info("Sudden drop ignorato: drone in recovery")
        except Exception as e:
            self.get_logger().error(f"Errore parsing comando: {e}")

    def start_flip(self, direction):
        """Avvia la manovra di flip a 360° nella direzione specificata"""
        if self.flip_active:
            self.get_logger().warn("Flip già in corso, ignorato")
            return
        
        #controllo stabilità
        if abs(self.current_roll) > 0.2 or abs(self.current_pitch) > 0.2:
            self.get_logger().warn(f"Drone non stabile per flip - Roll: {math.degrees(self.current_roll):.1f}°, Pitch: {math.degrees(self.current_pitch):.1f}°")
            return
            
        #assicurati di avere altezza sufficiente
        if self.current_altitude < 0.5:
            self.get_logger().warn("Altitudine insufficiente per flip sicuro")
            return
            
        #inizializza flip
        self.flip_active = True
        self.flip_direction = direction
        self.flip_start_time = time.time()
        self.flip_phase = self.FLIP_PHASE_PREP
        self.flip_phase_start_time = time.time()
        self.flip_rotation_completed = False
        
        #salva altitudine originale e imposta target MOLTO più alto per flip
        self.flip_original_altitude = self.target_altitude
        self.flip_target_altitude = max(1.5, self.current_altitude + 0.5)  # Boost altitudine flip
        
        #RESET TRACKING ROTAZIONE CORRETTO
        if direction in ['forward', 'backward']:
            self.flip_start_angle = self.current_pitch
            self.flip_prev_angle = self.current_pitch
        else:  # left, right
            self.flip_start_angle = self.current_roll
            self.flip_prev_angle = self.current_roll
            
        self.flip_total_rotation = 0.0
        
        self.get_logger().info(f"[FLIP] {direction.upper()} iniziato - Target 360° completo")
        self.get_logger().info(f"[FLIP] Angolo iniziale: {math.degrees(self.flip_start_angle):.1f}°")

    def update_flip_rotation_accumulative(self):
        """Aggiorna la rotazione accumulata durante il flip, gestendo il wrap-around degli angoli"""
        if self.flip_direction in ['forward', 'backward']:
            current_angle = self.current_pitch
        else:  # left, right
            current_angle = self.current_roll
        
        #calcola differenza angolare gestendo wrap-around
        angle_diff = current_angle - self.flip_prev_angle
        
        #gestione wrap-around per evitare salti -π/+π
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        elif angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        #accumula la rotazione totale
        self.flip_total_rotation += abs(angle_diff)
        self.flip_prev_angle = current_angle
        
        degrees_rotated = math.degrees(self.flip_total_rotation)
        
        #log progress ogni 30°
        if int(degrees_rotated) % 30 == 0 and degrees_rotated > 30:
            progress = min(100, (degrees_rotated / 360) * 100)
            self.get_logger().info(f"[FLIP] Rotazione accumulata: {degrees_rotated:.0f}° ({progress:.0f}%)")
        
        #controllo soglie per fasi
        if self.flip_total_rotation >= self.FLIP_COMPLETE_THRESHOLD and not self.flip_rotation_completed:
            self.flip_rotation_completed = True
            self.get_logger().info(f"[FLIP] 360° COMPLETATO! ({degrees_rotated:.1f}°)")
        
        return degrees_rotated

    def calculate_flip_rotors(self):
        """Calcola i comandi dei rotori per ogni fase del flip a 360°"""
        msg = Float32MultiArray()
        phase_elapsed = time.time() - self.flip_phase_start_time
        degrees_rotated = self.update_flip_rotation_accumulative()
        
        if self.flip_phase == self.FLIP_PHASE_PREP:
            # FASE 1: Preparazione - Salita e stabilizzazione
            altitude_error = self.flip_target_altitude - self.current_altitude
            omega = self.hover_omega + min(500, max(100, self.kp * altitude_error))
            
            #stabilizzazione attiva
            roll_correction = -120 * self.current_roll
            pitch_correction = -120 * self.current_pitch
            
            rotors = [
                omega - roll_correction - pitch_correction,
                omega + roll_correction - pitch_correction,
                omega + roll_correction + pitch_correction,
                omega - roll_correction + pitch_correction
            ]
            
            msg.data = [float(max(250, min(1500, r))) for r in rotors]
            
            if (self.current_altitude >= self.flip_target_altitude - 0.1 or 
                phase_elapsed >= self.FLIP_PREP_DURATION):
                self.flip_phase = self.FLIP_PHASE_ROTATE
                self.flip_phase_start_time = time.time()
                if self.flip_direction in ['right', 'left']:
                    self.FLIP_ROTATE_DURATION = 1.2
                    self.FLIP_BRAKE_DURATION = 0.4
                self.get_logger().info("[FLIP] → FASE ROTAZIONE")
                
        elif self.flip_phase == self.FLIP_PHASE_ROTATE:
            # FASE 2: Rotazione controllata - continua fino a ~290°
            
            #passa a frenata quando vicino a 290°
            if self.flip_total_rotation >= self.FLIP_BRAKE_THRESHOLD:
                self.flip_phase = self.FLIP_PHASE_BRAKE
                self.flip_phase_start_time = time.time()
                self.get_logger().info(f"[FLIP] → FRENATA a {degrees_rotated:.1f}°")
            
            #controllo timeout più lungo per permettere rotazione completa
            elif phase_elapsed >= self.FLIP_ROTATE_DURATION:
                self.get_logger().warn(f"[FLIP] Timeout rotazione a {degrees_rotated:.1f}° → FRENATA")
                self.flip_phase = self.FLIP_PHASE_BRAKE
                self.flip_phase_start_time = time.time()
            
            # BOOST POTENTE per evitare cadute durante rotazione
            base_omega = 600   # BOOST SIGNIFICATIVO per mantenimento quota
            rotation_omega = 1490  # BOOST ROTAZIONE molto più potente
            
            #boost extra se sta perdendo quota
            if self.current_altitude < self.flip_target_altitude - 0.3:
                altitude_boost = 400
                self.get_logger().warn(f"[FLIP] BOOST EMERGENZA QUOTA! Alt: {self.current_altitude:.2f}m")
            else:
                altitude_boost = 0
            
            if self.flip_direction == 'forward':
                rotors = [base_omega + altitude_boost, base_omega + altitude_boost, 
                         rotation_omega + altitude_boost, rotation_omega + altitude_boost]
            elif self.flip_direction == 'backward':
                rotors = [rotation_omega + altitude_boost, rotation_omega + altitude_boost, 
                         base_omega + altitude_boost, base_omega + altitude_boost]
            elif self.flip_direction == 'left':
                rotors = [rotation_omega + altitude_boost, base_omega + altitude_boost, 
                         rotation_omega + altitude_boost, base_omega + altitude_boost]
            elif self.flip_direction == 'right':
                rotors = [base_omega + altitude_boost, rotation_omega + altitude_boost, 
                         base_omega + altitude_boost, rotation_omega + altitude_boost]
                
            msg.data = [float(r) for r in rotors]
            self.get_logger().info(f"[FLIP ROTATE] Potenza: Base={base_omega}, Rot={rotation_omega}, Boost={altitude_boost}")
            
        elif self.flip_phase == self.FLIP_PHASE_BRAKE:
            # FASE 3: Frenata progressiva fino a ~330° - BOOST MANTENUTO
            
            #condizioni per passare a recovery
            if (self.flip_total_rotation >= self.FLIP_COMPLETE_THRESHOLD or 
                phase_elapsed >= self.FLIP_BRAKE_DURATION):
                self.flip_phase = self.FLIP_PHASE_RECOVER
                self.flip_phase_start_time = time.time()
                self.get_logger().info(f"[FLIP] → RECOVERY FINALE a {degrees_rotated:.1f}°")
            
            #frenata controllata ma con BOOST per quota
            hover_base = self.hover_omega + 600  # BOOST SIGNIFICATIVO per quota
            brake_intensity = 200
            #brake_intensity = max(200, 400 - (phase_elapsed * 300))  # Frenata più forte
            
            #boost extra se perde quota durante frenata
            if self.current_altitude < self.flip_target_altitude - 0.4:
                altitude_boost = 500
                self.get_logger().warn(f"[FLIP BRAKE] BOOST EMERGENZA! Alt: {self.current_altitude:.2f}m")
            else:
                altitude_boost = 0
            
            if self.flip_direction == 'forward':
                rotors = [hover_base + brake_intensity + altitude_boost, 
                         hover_base + brake_intensity + altitude_boost, 
                         hover_base - brake_intensity + altitude_boost, 
                         hover_base - brake_intensity + altitude_boost]
            elif self.flip_direction == 'backward':
                rotors = [hover_base - brake_intensity + altitude_boost, 
                         hover_base - brake_intensity + altitude_boost, 
                         hover_base + brake_intensity + altitude_boost, 
                         hover_base + brake_intensity + altitude_boost]
            elif self.flip_direction == 'left':
                rotors = [hover_base - brake_intensity + altitude_boost, 
                         hover_base + brake_intensity + altitude_boost, 
                         hover_base - brake_intensity + altitude_boost, 
                         hover_base + brake_intensity + altitude_boost]
            elif self.flip_direction == 'right':
                rotors = [hover_base + brake_intensity + altitude_boost, 
                         hover_base - brake_intensity + altitude_boost, 
                         hover_base + brake_intensity + altitude_boost, 
                         hover_base - brake_intensity + altitude_boost]
                
            msg.data = [float(max(200, min(2500, r))) for r in rotors]  # Limiti più alti
            self.get_logger().info(f"[FLIP BRAKE] Potenza: Base={hover_base}, Brake={brake_intensity}, Boost={altitude_boost}")
                
        elif self.flip_phase == self.FLIP_PHASE_RECOVER:
            # FASE 4: Recovery finale con stabilizzazione
            altitude_error = self.flip_original_altitude - self.current_altitude
            
            #controllo emergenza altitudine
            if self.current_altitude < 0.2:
                emergency_omega = self.hover_omega + 600
                msg.data = [float(emergency_omega)] * 4
                self.get_logger().warn(f"[FLIP RECOVER] EMERGENZA Alt: {self.current_altitude:.2f}m")
            else:
                #recovery progressivo
                progress = min(1.0, phase_elapsed / 2.0)
                
                base_omega = self.hover_omega + min(300, max(-200, self.kp * altitude_error * 0.9))
                
                #correzioni assetto che crescono nel tempo
                max_correction = 800 * progress
                roll_correction = max_correction * (-self.current_roll)
                pitch_correction = max_correction * (-self.current_pitch)
                
                #anti-oscillazione
                roll_correction -= 300 * progress * self.current_angular_vel[0]
                pitch_correction -= 300 * progress * self.current_angular_vel[1]
                
                rotors = [
                    base_omega - roll_correction - pitch_correction,
                    base_omega + roll_correction - pitch_correction,
                    base_omega + roll_correction + pitch_correction,
                    base_omega - roll_correction + pitch_correction
                ]
                
                msg.data = [float(max(100, min(1800, r))) for r in rotors]
            
            #condizioni di uscita
            is_level = abs(self.current_roll) < 0.12 and abs(self.current_pitch) < 0.12
            is_stable = (abs(self.current_angular_vel[0]) < 0.4 and 
                        abs(self.current_angular_vel[1]) < 0.4)
            
            if (is_level and is_stable and phase_elapsed > 2.0) or phase_elapsed >= self.FLIP_RECOVER_DURATION:
                self.flip_active = False
                self.recovering = True
                self.recover_start_time = time.time()
                self.target_altitude = self.flip_original_altitude
                self.get_logger().info(f"[FLIP] COMPLETATO! Rotazione finale: {degrees_rotated:.1f}°")
                self.get_logger().info("[FLIP] → Passaggio a recovery normale")
        
        return msg

    def calculate_recovery_rotors(self):
        """Calcola i comandi dei rotori per la fase di recovery dopo il flip"""
        msg = Float32MultiArray()
        
        roll_error = -self.current_roll
        pitch_error = -self.current_pitch
        altitude_error = self.target_altitude - self.current_altitude
        recovery_time = time.time() - self.recover_start_time
        
        #controllo emergenza altitudine
        if self.current_altitude < 0.15:
            emergency_omega = self.hover_omega + 500
            msg.data = [float(emergency_omega)] * 4
            self.get_logger().warn(f"[RECOVERY] EMERGENZA - Alt: {self.current_altitude:.2f}m")
            return msg
        
        #base omega con controllo altitudine
        omega = self.hover_omega + min(250, max(-250, self.kp * altitude_error * 0.7))
        
        #correzioni assetto progressive
        correction_factor = min(1.0, recovery_time / 1.0)
        roll_correction = 350 * correction_factor * roll_error
        pitch_correction = 350 * correction_factor * pitch_error
        
        #smorzamento velocità angolare
        roll_correction -= 150 * correction_factor * self.current_angular_vel[0]
        pitch_correction -= 150 * correction_factor * self.current_angular_vel[1]
        
        rotors = [
            omega - roll_correction - pitch_correction,
            omega + roll_correction - pitch_correction,
            omega + roll_correction + pitch_correction,
            omega - roll_correction + pitch_correction
        ]
        
        msg.data = [float(max(150, min(1400, r))) for r in rotors]
        
        #verifica stabilità
        is_stable = abs(roll_error) < 0.08 and abs(pitch_error) < 0.08
        is_slow = abs(self.current_angular_vel[0]) < 0.25 and abs(self.current_angular_vel[1]) < 0.25
        
        if (is_stable and is_slow and recovery_time > 1.0) or recovery_time > self.recover_duration:
            self.recovering = False
            self.set_led_effect("stable_green")
            self.get_logger().info("[RECOVERY] Completato! Drone completamente stabilizzato")
        
        return msg

    def start_sudden_drop(self):
        #attiva una caduta improvvisa abbassando temporaneamente la quota target
        if hasattr(self, 'sudden_active') and self.sudden_active: 
            return
        drop = random.uniform(0.15, 0.4)
        if self.sudden_drop_original_target is None:
            self.sudden_drop_original_target = self.target_altitude
        new_target = max(0.1, self.target_altitude - drop)
        print(f"[SUDDEN DROP] Target altitude: {self.target_altitude:.2f} -> {new_target:.2f} (-{drop:.2f}m)")
        self.target_altitude = new_target
        self.sudden_active = True
        self.sudden_type = 'drop'
        self.sudden_start_time = time.time()
        self.sudden_duration = 0.7
        self.get_logger().info("[LED] ROSSO: SUDDEN DROP!")

    def set_led_effect(self, effect):
        #imposta l'effetto LED simulato (solo log)
        if self.led_effect != effect:
            self.led_effect = effect
            self.get_logger().info(f"[LED EFFECT] {effect}")

    def print_led_red_if_needed(self):
        #logga effetto LED rosso se necessario
        if hasattr(self, 'led_effect') and self.led_effect and self.led_effect.startswith("LED: RED"):
            self.get_logger().info("[LED EFFECT] LED: RED (ALERT)")
    
    def control_loop(self):
        #ciclo di controllo principale (chiamato periodicamente)
        now = time.time()
        msg = Float32MultiArray()
        
        #clamp target altitude
        if self.target_altitude > self.MAX_ALTITUDE:
            self.target_altitude = self.MAX_ALTITUDE

        #SUDDEN DROP (ridotta probabilità)
        if not hasattr(self, 'last_sudden_check'):
            self.last_sudden_check = now
            
        if not (hasattr(self, 'sudden_active') and self.sudden_active) and not self.flip_active and not self.recovering:
            if now - self.last_sudden_check > 5.0:
                self.last_sudden_check = now
                if random.random() < 0.001:
                    self.start_sudden_drop()
                    self.get_logger().info("[SUDDEN DROP] Attivato (probabilità 0.1%)")
                
        if hasattr(self, 'sudden_active') and self.sudden_active:
            elapsed = now - self.sudden_start_time
            self.set_led_effect("alert_flashing_red")
            if self.sudden_type == 'drop': 
                msg.data = [0.0, 0.0, 0.0, 0.0]
            if elapsed > self.sudden_duration:
                self.sudden_active = False
                self.set_led_effect("adjusting_yellow")
                if self.sudden_drop_original_target is not None:
                    self.target_altitude = self.sudden_drop_original_target
                    self.sudden_drop_original_target = None
                self.get_logger().info("[SUDDEN DROP] Completato, ritorno alla quota originale")
            self.rotor_pub.publish(msg)
            return

        #gestione flip
        if self.flip_active:
            #timeout di sicurezza più lungo
            if now - self.flip_start_time > 20.0:
                self.get_logger().error("[FLIP] Timeout di sicurezza! Forzando recovery")
                self.flip_active = False
                self.recovering = True
                self.recover_start_time = now
                self.target_altitude = self.flip_original_altitude
            else:
                msg = self.calculate_flip_rotors()
                self.rotor_pub.publish(msg)
                return

        #gestione recovery
        elif self.recovering:
            self.set_led_effect("adjusting_yellow")
            msg = self.calculate_recovery_rotors()
            self.rotor_pub.publish(msg)
            return

        #controllo manuale
        elif self.manual_rotor_cmd and now < self.override_until:
            self.set_led_effect("adjusting_yellow")
            msg.data = [float(x) for x in self.manual_rotor_cmd]
            self.get_logger().info(f"[MANUALE] Rotor speeds: {msg.data}")
        else:
            #hover normale
            error = self.target_altitude - self.current_altitude
            abs_error = abs(error)
            
            if abs_error < 0.03: 
                self.set_led_effect("LED: GREEN (HOLD)")
            elif abs_error < 0.12: 
                self.set_led_effect("LED: YELLOW (NEAR TARGET)")
            else: 
                self.set_led_effect("LED: RED (ALERT)")
            
            omega = self.hover_omega + self.kp * error
            omega = max(0, min(3000, omega))
            msg.data = [float(omega)] * 4
            self.get_logger().info(f"[AUTO] Alt: {self.current_altitude:.2f}m, Target: {self.target_altitude:.2f}m, Omega: {omega:.1f}")
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