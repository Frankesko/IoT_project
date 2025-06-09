# mavlink_interface.py
import time
import threading
from pymavlink import mavutil

class MAVLinkInterface:
    def __init__(self, connection_string='udp:127.0.0.1:14551'): # PX4 SITL di default
        self.connection_string = connection_string
        self.master = None
        self.running = False
        self.data_thread = None
        
        # Dati di telemetria che ci interessano
        self.last_time_boot_ms = 0
        self.current_altitude_m = 0.0  # Altitudine relativa (AMSL - home altitude)
        self.current_relative_altitude_m = 0.0 # Altitudine sopra il punto di home/decollo
        self.armed = False
        self.mode = "N/A"
        self.battery_voltage = 0.0
        self.battery_remaining_pct = 0.0
        # Aggiungi altri dati MAVLink se necessario

        self.data_lock = threading.Lock()

    def connect(self):
        try:
            print(f"MAVLink: Connecting to {self.connection_string}...")
            self.master = mavutil.mavlink_connection(self.connection_string)
            self.master.wait_heartbeat()
            print(f"MAVLink: Heartbeat from system (system {self.master.target_system} component {self.master.target_component})")
            

            self.master.mav.request_data_stream_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_POSITION, 20, 1) # 20 Hz per dati di posizione/altitudine
            self.master.mav.request_data_stream_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2, 1) # 2 Hz per stato batteria, ecc.


            self.running = True
            self.data_thread = threading.Thread(target=self._message_loop)
            self.data_thread.daemon = True
            self.data_thread.start()
            print("MAVLink: Interface started.")
            return True
        except Exception as e:
            print(f"MAVLink: Connection failed: {e}")
            self.master = None
            return False

    def disconnect(self):
        self.running = False
        if self.data_thread:
            self.data_thread.join(timeout=1.0)
        if self.master:
            self.master.close()
        print("MAVLink: Interface stopped and disconnected.")

    def _message_loop(self):
        while self.running:
            msg = self.master.recv_match(blocking=True, timeout=0.5)
            if not msg:
                continue

            msg_type = msg.get_type()
            print(f"MAVLink Rx: {msg_type}") # Debug

            with self.data_lock:
                if msg_type == 'GLOBAL_POSITION_INT':
                    # alt è AMSL in mm, relative_alt è sopra home in mm
                    self.current_altitude_m = msg.alt / 1000.0
                    self.current_relative_altitude_m = msg.relative_alt / 1000.0 
                    self.last_time_boot_ms = msg.time_boot_ms
                    # print(f"Alt: {self.current_altitude_m:.2f}m AMSL, RelAlt: {self.current_relative_altitude_m:.2f}m")
                
                elif msg_type == 'ATTITUDE': # Per vedere se è armato (da flags)
                    # Non c'è un flag di armamento diretto qui, ma HEARTBEAT lo ha
                    pass

                elif msg_type == 'HEARTBEAT':
                    self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self.mode = mavutil.mode_string_v10(msg)
                    # print(f"Armed: {self.armed}, Mode: {self.mode}")

                elif msg_type == 'SYS_STATUS':
                    self.battery_voltage = msg.voltage_battery / 1000.0 # mV -> V
                    self.battery_remaining_pct = msg.battery_remaining # Percentuale
                    # print(f"Battery: {self.battery_voltage:.2f}V, {self.battery_remaining_pct}%")
                
                # Aggiungere qui il parsing di altri messaggi MAVLink se necessario

    def get_telemetry(self):
        """Restituisce un dizionario con i dati di telemetria più recenti."""
        with self.data_lock:
            current_sim_time = self.last_time_boot_ms / 1000.0 if self.last_time_boot_ms > 0 else time.time()
            return {
                'sim_time': current_sim_time, # Useremo il tempo reale per ora, o un sim_time da PX4 se disponibile
                'altitude_tof': self.current_relative_altitude_m, # Usa l'altitudine relativa
                'altitude_barometer': self.current_altitude_m, # Esempio, potrebbe essere lo stesso o diverso
                'battery_voltage': self.battery_voltage,
                'battery_remaining_pct': self.battery_remaining_pct,
                'armed': self.armed,
                'mode': self.mode,
                # Aggiungi altri campi simulati se la tua FSM o logger li usano
                'temperature': 25.0, # Placeholder
                'vibration_x': 0.0, 'vibration_y': 0.0, 'vibration_z': 9.81, # Placeholders
                'signal_strength': 99, 'packet_loss_rate': 0.0 # Placeholders
            }
    

    def arm_vehicle(self):
        if self.master:
            print("MAVLink: Arming vehicle...")
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1, 0, 0, 0, 0, 0, 0) # 1 per armare
            # Attendi conferma (opzionale ma consigliato)
            # ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
            # print(f"Arm ACK: {ack}")

    def disarm_vehicle(self):
        if self.master:
            print("MAVLink: Disarming vehicle...")
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                0, 0, 0, 0, 0, 0, 0) # 0 per disarmare

    def set_mode(self, mode_name="STABILIZED"): # Esempi: STABILIZED, ALTCTL, POSCTL, OFFBOARD
        if self.master:
            # Trova l'ID della modalità
            mode_id = self.master.mode_mapping().get(mode_name.upper())
            if mode_id is not None:
                print(f"MAVLink: Setting mode to {mode_name} (ID: {mode_id})...")
                self.master.mav.set_mode_send(
                    self.master.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    mode_id)
            else:
                print(f"MAVLink: Mode '{mode_name}' not found in mapping.")

# Per testare rapidamente questo script
if __name__ == '__main__':
    mav_interface = MAVLinkInterface()
    if mav_interface.connect():
        print("MAVLink Interface connected. Receiving data...")
        try:
            for i in range(200): # Stampa dati per 20 secondi
                time.sleep(0.1)
                telemetry = mav_interface.get_telemetry()
                print(f"SimTime: {telemetry['sim_time']:.1f}, RelAlt: {telemetry['altitude_tof']:.2f}m, Armed: {telemetry['armed']}, Mode: {telemetry['mode']}, Batt: {telemetry['battery_voltage']:.2f}V ({telemetry['battery_remaining_pct']}%)")
        except KeyboardInterrupt:
            print("MAVLink test interrupted.")
        finally:
            mav_interface.disconnect()
    else:
        print("Failed to connect MAVLink interface.")