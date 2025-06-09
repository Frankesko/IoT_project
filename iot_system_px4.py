# iot_system.py (MODIFICATO PER USARE MAVLINKINTERFACE)
import json
import time
import threading
import os
import random
from typing import Dict, List, Callable, Optional

import paho.mqtt.client as mqtt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import numpy as np # LivePlotter potrebbe usarlo per min/max

from altitude_fsm import AltitudeGuardFSM
from mavlink_interface import MAVLinkInterface # <--- MODIFICA IMPORTANTE
# from drone_simulation_components import DataLogger, SensorReading # SensorReading potrebbe ancora servire per DataLogger

# Definiamo una dataclass SensorReading semplificata se DataLogger ne ha bisogno
# o adattiamo DataLogger per prendere un dizionario.
# Per ora, assumiamo che DataLogger possa essere adattato o che i campi principali siano sufficienti.
from dataclasses import dataclass # Riutilizziamo dataclass per una struttura dati semplice

@dataclass
class PX4SensorDataLog: # Struttura per il logging, simile a SensorReading
    sim_time: float
    altitude_tof: float # Useremo l'altitudine relativa da MAVLink come ToF equivalente
    battery_voltage: float
    altitude_barometer: float = 0.0 
    temperature: float = 25.0
    vibration_x: float = 0.0
    vibration_y: float = 0.0
    vibration_z: float = 0.0
    signal_strength: int = 99 # Placeholder
    packet_loss_rate: float = 0.0 # Placeholder


from drone_simulation_components import DataLogger # Assumendo che DataLogger sia ancora in quel file


class LivePlotter:
    """
    Gestisce la creazione e l'aggiornamento di un grafico Matplotlib in tempo reale
    per visualizzare l'altitudine, il target e lo stato FSM.
    """
    def __init__(self, max_points=400): # Numero massimo di punti da visualizzare nel grafico
        self.max_points = max_points
        # Deque (code a doppia estremità) per memorizzare gli ultimi N punti dati per il plot
        self.sim_time_data = deque(maxlen=max_points)      # Timestamp simulati
        self.altitude_data = deque(maxlen=max_points)      # Dati di altitudine
        self.target_altitude_data = deque(maxlen=max_points) # Dati dell'altitudine target
        self.fsm_state_plot_data = deque(maxlen=max_points)  # Dati numerici per lo stato FSM (per colorare lo sfondo)
        
        # Mappatura degli stati FSM testuali a valori numerici
        self.state_to_numeric = {
            AltitudeGuardFSM.STATE_HOLD_STABLE: 0,
            AltitudeGuardFSM.STATE_ADJUST_UP: 1,
            AltitudeGuardFSM.STATE_ADJUST_DOWN: 2,
            AltitudeGuardFSM.STATE_ALERT: 3
        }
        # Mappatura dei valori numerici degli stati ai colori per lo sfondo del grafico
        self.numeric_to_color = {
            0: 'lightgreen',    # HOLD_STABLE
            1: 'lightyellow',   # ADJUST_UP
            2: 'moccasin',      # ADJUST_DOWN 
            3: 'lightcoral'     # ALERT
        }

        # Creazione della figura e degli assi per il grafico Matplotlib
        self.fig, self.ax = plt.subplots(figsize=(14, 8)) # Dimensioni della figura
        # Creazione delle linee iniziali vuote per altitudine e target
        self.line_alt, = self.ax.plot([], [], 'b-', label='Altitude Rel (m) from PX4', linewidth=2, zorder=10) # Etichetta aggiornata
        self.line_target, = self.ax.plot([], [], 'k--', label='FSM Target Alt (m)', linewidth=1, zorder=9) # Etichetta aggiornata
        
        # Impostazione etichette, legenda, griglia e titolo del grafico
        self.ax.set_xlabel("Time (s) (from MAVLink/System)", fontsize=12) # Etichetta aggiornata
        self.ax.set_ylabel("Altitude (m)", fontsize=12)
        self.ax.legend(loc='upper right') # Posizione della legenda
        self.ax.grid(True, linestyle='--', alpha=0.7) # Griglia con stile tratteggiato e trasparenza
        self.title = self.ax.set_title("Live Altitude Data - FSM State: N/A", fontsize=12) # Titolo iniziale

        # Variabili per memorizzare lo stato corrente da visualizzare nel titolo
        self.current_fsm_state_for_title = "N/A"
        self.current_time_for_title = 0.0 # Cambiato da sim_time
        self.current_px4_mode_for_title = "N/A" # Sostituisce scenario

        self.background_patches = [] # Lista per memorizzare i rettangoli dello sfondo (per poterli rimuovere)

        self.ani = animation.FuncAnimation(self.fig, self._update_plot_data, 
                                           blit=False, interval=100, cache_frame_data=False) 
        plt.show(block=False) 
        plt.pause(0.001) 

    def add_data(self, current_time: float, altitude: float, target_altitude: float, fsm_state: str, px4_mode: str): # sim_time -> current_time, scenario -> px4_mode
        """Aggiunge nuovi dati alle deque per l'aggiornamento del grafico."""
        self.sim_time_data.append(current_time) # Rinominato internamente ma rappresenta il tempo sull'asse x
        self.altitude_data.append(altitude)
        self.target_altitude_data.append(target_altitude)
        self.fsm_state_plot_data.append(self.state_to_numeric.get(fsm_state, -1)) 
        self.current_fsm_state_for_title = fsm_state
        self.current_time_for_title = current_time
        self.current_px4_mode_for_title = px4_mode
    
    def _update_plot_data(self, frame):
        """Funzione chiamata periodicamente da FuncAnimation per aggiornare il grafico."""
        if not self.sim_time_data: 
            return self.line_alt, self.line_target, self.title 
            
        self.line_alt.set_data(self.sim_time_data, self.altitude_data)
        self.line_target.set_data(self.sim_time_data, self.target_altitude_data)
        
        fsm_s = self.current_fsm_state_for_title 
        
        if self.sim_time_data:
            min_alt_plot = min(list(self.altitude_data) + list(self.target_altitude_data) + [0.0]) - 0.2 
            max_alt_plot = max(list(self.altitude_data) + list(self.target_altitude_data) + [1.0]) + 0.2
            
            self.ax.set_xlim(max(0, self.sim_time_data[0]-1), self.sim_time_data[-1] + 5) 
            self.ax.set_ylim(min_alt_plot, max_alt_plot) 
        
        self.title.set_text(f"FSM: {fsm_s} | PX4 Mode: {self.current_px4_mode_for_title} | Alt: {self.altitude_data[-1]:.2f}m | Time: {self.current_time_for_title:.1f}s")

        for patch in self.background_patches: patch.remove()
        self.background_patches.clear()

        if len(self.sim_time_data) > 1: 
            for i in range(len(self.sim_time_data) - 1): 
                start_time = self.sim_time_data[i]
                end_time = self.sim_time_data[i+1]
                state_numeric = self.fsm_state_plot_data[i] 
                color = self.numeric_to_color.get(state_numeric, 'white') 
                
                rect = plt.Rectangle((start_time, self.ax.get_ylim()[0]), 
                                     end_time - start_time, 
                                     self.ax.get_ylim()[1] - self.ax.get_ylim()[0], 
                                     facecolor=color, alpha=0.2, zorder=1, edgecolor='none') 
                self.ax.add_patch(rect) 
                self.background_patches.append(rect) 
        
        try: 
            self.fig.canvas.draw_idle() 
        except Exception: pass 
        return self.line_alt, self.line_target, self.title, *self.background_patches

    def save_plot(self, filename="altitude_plot_px4.png"): # Nome file aggiornato
        if not self.fig: 
            print("Plotter: Figure does not exist, cannot save.")
            return

        if self.sim_time_data: 
            try: self._update_plot_data(None) 
            except Exception as e: print(f"Plotter: Error during final update before save: {e}")
        
        if hasattr(self, 'ani') and self.ani is not None:
            if hasattr(self.ani, 'event_source') and self.ani.event_source is not None:
                try: self.ani.event_source.stop(); print("Plotter: Animation stopped for saving.")
                except Exception as e: print(f"Plotter: Error stopping animation event_source: {e}")
        else: print("Plotter: Animation object 'ani' not found or already None during save.")

        try:
            self.fig.savefig(filename, dpi=200, bbox_inches='tight')
            print(f"Plot saved to {filename}")
        except Exception as e: print(f"Error saving plot: {e}")

    def close_plot(self):
        if hasattr(self, 'ani') and self.ani: self.ani.event_source.stop()
        if hasattr(self, 'fig') and self.fig: plt.close(self.fig); self.fig = None
        print("LivePlotter window closed.")


class MQTTIoTBridge:
    def __init__(self, broker_host="localhost", broker_port=1883, client_id_prefix="crazyflie_iot_px4_"): # Client ID aggiornato
        self.broker_host = broker_host; self.broker_port = broker_port
        self.client_id = f"{client_id_prefix}{random.randint(1000, 9999)}"
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, client_id=self.client_id)
        self.client.on_connect = self._on_connect; self.client.on_message = self._on_message
        self.client.on_disconnect = self._on_disconnect
        self.base_topic = "crazyflie/drone_01" # Potrebbe essere configurabile
        self.topics = {
            'telemetry': f"{self.base_topic}/telemetry", 
            'status': f"{self.base_topic}/status",      
            'alerts': f"{self.base_topic}/alerts",      
            'commands': f"{self.base_topic}/commands",  
            'diagnostics': f"{self.base_topic}/diagnostics" 
        }
        self.command_callbacks: Dict[str, Callable] = {}; self.connected = False
        self.publish_telemetry_rate_hz = 5; self.publish_status_rate_hz = 1 # Rate leggermente ridotti per non sovraccaricare
        self.publishing_active = False; self.publish_thread = None
        
    def connect(self) -> bool:
        try:
            print(f"🔌 MQTT: Attempting to connect to {self.broker_host}:{self.broker_port} as {self.client_id}")
            self.client.connect(self.broker_host, self.broker_port, 60)
            self.client.loop_start() 
            for _ in range(50): 
                if self.connected: break
                time.sleep(0.1)
            if not self.connected:
                 print(f"MQTT: Connection timed out."); self.client.loop_stop(); return False
            return True
        except Exception as e: print(f"MQTT: Connection failed: {e}"); self.connected = False; return False
    
    def disconnect(self):
        if self.publishing_active: self.stop_publishing_loop()
        if self.client: self.client.loop_stop(); self.client.disconnect()
        print("🔌 MQTT: Disconnected"); self.connected = False
    
    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.connected = True; print("MQTT: Connected successfully.")
            command_topic_filter = f"{self.topics['commands']}/#" 
            client.subscribe(command_topic_filter)
            print(f"MQTT: Subscribed to {command_topic_filter}")
        else: print(f"MQTT: Connection failed with code {rc}. Flags: {flags}"); self.connected = False
    
    def _on_disconnect(self, client, userdata, rc):
        self.connected = False
        if rc != 0: print(f"🔌 MQTT: Unexpected disconnection (rc={rc}). Will attempt to reconnect.")
        else: print("🔌 MQTT: Disconnected gracefully.")
    
    def _on_message(self, client, userdata, msg):
        try:
            topic = msg.topic; payload_str = msg.payload.decode()
            payload = json.loads(payload_str)
            if topic.startswith(self.topics['commands'] + "/"):
                command_type = topic.split('/')[-1]
                if command_type in self.command_callbacks: self.command_callbacks[command_type](payload)
                else: print(f"MQTT: Unhandled command type: {command_type} on topic {topic}")
        except json.JSONDecodeError: print(f"MQTT: Failed to decode JSON payload from {topic}: {msg.payload.decode()}")
        except Exception as e: print(f"MQTT: Error processing message from {topic}: {e}")
    
    def register_command_handler(self, command_type: str, callback: Callable):
        self.command_callbacks[command_type] = callback
    
    def publish_telemetry(self, px4_data_dict: Dict, fsm_status: Dict): # px4_data_dict da mav_interface.get_telemetry()
        if not self.connected: return False
        telemetry_payload = {
            'sim_time_s': round(px4_data_dict.get('sim_time', time.time()), 3), 
            'publish_real_time_s': round(time.time(),3),
            'altitude_tof_m': round(px4_data_dict.get('altitude_tof', 0.0), 3), # ToF è alt relativa
            'altitude_baro_m': round(px4_data_dict.get('altitude_barometer', 0.0), 3), # Baro è alt AMSL
            'battery_v': round(px4_data_dict.get('battery_voltage', 0.0), 2),
            'battery_pct': round(px4_data_dict.get('battery_remaining_pct', 0.0), 1),
            'px4_mode': px4_data_dict.get('mode', "N/A"),
            'armed_status': px4_data_dict.get('armed', False),
            'fsm': fsm_status 
        }
        return self._publish(self.topics['telemetry'], telemetry_payload)
    
    def publish_drone_status(self, px4_data_dict: Dict, fsm_state: str): # Simile a sopra, ma meno FSM
        if not self.connected: return False
        status_payload = {
            'sim_time_s': round(px4_data_dict.get('sim_time', time.time()), 3),
            'publish_real_time_s': round(time.time(),3),
            'px4_mode': px4_data_dict.get('mode', "N/A"), # Lo "scenario" è ora il modo PX4
            'armed_status': px4_data_dict.get('armed', False),
            'battery_v': round(px4_data_dict.get('battery_voltage', 0.0), 2),
            'battery_pct': round(px4_data_dict.get('battery_remaining_pct', 0.0), 1),
            'fsm_state_guard': fsm_state, # Stato della nostra FSM di guardia
            # Health score potrebbe essere ricalcolato o basato su dati PX4
            # 'health_score': self._calculate_health_score_px4(px4_data_dict) 
        }
        return self._publish(self.topics['status'], status_payload)
    
    def publish_alert_message(self, alert_type: str, alert_details: Dict, severity: str = "WARNING"):
        if not self.connected: return False
        alert_payload = {
            'real_time_s': round(time.time(),3), 
            'sim_time_at_alert_s': round(alert_details.get('sim_time_s', 0.0),3), # Tempo MAVLink dell'evento
            'alert_type': alert_type, 'severity': severity.upper(),
            'details': alert_details, 'drone_id': self.base_topic.split('/')[-1] 
        }
        return self._publish(self.topics['alerts'], alert_payload)
        
    def _publish(self, topic: str, data: Dict, qos: int = 0, retain: bool = False) -> bool: # Come prima
        if not self.connected: return False
        try:
            payload_json = json.dumps(data, default=str)
            result = self.client.publish(topic, payload_json, qos=qos, retain=retain)
            if result.rc == mqtt.MQTT_ERR_SUCCESS: return True
            else: print(f"MQTT: Publish to {topic} failed with code {result.rc}"); return False
        except TypeError as e: print(f"MQTT: JSON serialization error for topic {topic}: {e}. Data: {data}"); return False
        except RuntimeError as e: print(f"MQTT: Runtime error publishing to {topic}: {e}"); self.connected = False; return False
        except Exception as e: print(f"MQTT: Generic publish error to {topic}: {e}"); return False
    
    def start_publishing_loop(self, mav_interface_ref, fsm_ref): # Modificato per mav_interface_ref
        if self.publishing_active or not self.connected: return
        self.publishing_active = True
        self.publish_thread = threading.Thread(target=self._publishing_loop_content, args=(mav_interface_ref, fsm_ref))
        self.publish_thread.daemon = True; self.publish_thread.start()
        print("MQTT: Publishing loop for PX4 data started.")
    
    def stop_publishing_loop(self): # Come prima
        self.publishing_active = False
        if self.publish_thread and self.publish_thread.is_alive(): self.publish_thread.join(timeout=1.0)
        print("MQTT: Publishing loop stopped.")

    def _publishing_loop_content(self, mav_interface: MAVLinkInterface, fsm: AltitudeGuardFSM): # Modificato per MAVLinkInterface
        last_tel_pub, last_stat_pub = 0,0
        tel_int, stat_int = 1.0/self.publish_telemetry_rate_hz, 1.0/self.publish_status_rate_hz
        
        while self.publishing_active and self.connected:
            now = time.time() 
            px4_data = mav_interface.get_telemetry() # Prendi dati da MAVLink
            fsm_status = fsm.get_current_status()

            if px4_data and (now - last_tel_pub >= tel_int) :
                self.publish_telemetry(px4_data, fsm_status)
                last_tel_pub = now 
            
            if px4_data and (now - last_stat_pub >= stat_int):
                self.publish_drone_status(px4_data, fsm_status['state']) # Passa solo lo stato FSM
                last_stat_pub = now 
                
            time.sleep(min(tel_int, stat_int) / 5.0) 
        print(f"MQTT: PX4 Publishing loop terminato.")

class IoTDashboardData: # Come prima
    def __init__(self, max_alerts_history=20):
        self.alert_history: List[Dict] = [] 
        self.max_alerts_history = max_alerts_history
        
    def add_alert_to_dashboard(self, alert_type: str, message: str, severity: str = "WARNING", details: Optional[Dict] = None):
        timestamp = time.time()
        new_alert = {'timestamp': timestamp, 'type': alert_type, 'severity': severity.upper(), 'message': message, 'details': details or {}}
        self.alert_history.append(new_alert)
        if len(self.alert_history) > self.max_alerts_history: self.alert_history.pop(0)

    def get_recent_alerts(self, count=5) -> List[Dict]: return self.alert_history[-count:]
    def get_dashboard_summary(self) -> Dict:
        return {'timestamp': time.time(), 'active_alerts_count': len(self.alert_history), 
                'recent_alerts_preview': self.alert_history[-3:], 'status': 'Operational'}

class IoTSystemOrchestrator:
    def __init__(self, enable_live_plot=True):
        self.mav_interface = MAVLinkInterface() # USA MAVLinkInterface
        self.data_logger = DataLogger(db_path="iot_system_px4_flight_data.db") 
        self.mqtt_bridge = MQTTIoTBridge(client_id_prefix="crazyflie_iot_px4_") # Prefisso Client ID aggiornato
        self.dashboard_data_handler = IoTDashboardData()
        self.fsm = AltitudeGuardFSM() 
        
        self.system_running = False
        self.monitoring_thread = None
        self.main_loop_rate_hz = 10 # Potrebbe essere necessario un rate più basso per MAVLink se i dati non arrivano a 20Hz

        self.enable_live_plot = enable_live_plot
        self.live_plotter = None
        
    def initialize_system(self) -> bool:
        print("Initializing IoT System Orchestrator with PX4 SITL...")
        if self.enable_live_plot and self.live_plotter is None:
            self.live_plotter = LivePlotter()

        if not self.mav_interface.connect(): 
            print("Orchestrator: MAVLink connection failed. System cannot start.")
            return False

        self.mqtt_bridge.register_command_handler('set_target_altitude', self._mqtt_cmd_set_fsm_target_altitude)

        if not self.mqtt_bridge.connect():
            print("Orchestrator: MQTT connection failed. System cannot start.")
            self.mav_interface.disconnect() 
            return False
        
        print("IoT System Orchestrator initialized successfully for PX4.")
        return True
    
    def start_system(self):
        print("ENSURE PX4 SITL (make px4_sitl_default none) IS RUNNING IN A SEPARATE TERMINAL!")
        if not self.initialize_system(): return
        print("Starting Complete IoT System with PX4 SITL...")
        self.system_running = True
        
        # MAVLinkInterface.connect() è già stato chiamato in initialize_system
        self.mqtt_bridge.start_publishing_loop(self.mav_interface, self.fsm) # Passa mav_interface
        
        self.monitoring_thread = threading.Thread(target=self._orchestration_loop)
        self.monitoring_thread.daemon = True; self.monitoring_thread.start()
        print("IoT System Orchestrator with PX4 is RUNNING.")
    
    def stop_system(self):
        print("Stopping IoT System Orchestrator with PX4..."); 
        
        if self.live_plotter: 
            self.live_plotter.save_plot("final_px4_altitude_plot.png")
            self.live_plotter.close_plot() 

        self.system_running = False
        if self.monitoring_thread and self.monitoring_thread.is_alive(): self.monitoring_thread.join(timeout=1.0)
        
        self.mqtt_bridge.disconnect()
        self.mav_interface.disconnect() 
        
        fsm_log_file = self.fsm.save_log_to_csv("final_fsm_px4_log.csv") 
        if fsm_log_file: print(f"FSM Log saved: {fsm_log_file}")
        self.data_logger.close_db()
        print("IoT System Orchestrator with PX4 stopped.")
    
    def _orchestration_loop(self):
        print("Orchestration loop (PX4) started.")
        loop_interval = 1.0 / self.main_loop_rate_hz
        
        while self.system_running:
            loop_start_time = time.time()
            
            px4_telemetry = self.mav_interface.get_telemetry()
            
            if px4_telemetry:
                current_mavlink_time = px4_telemetry.get('sim_time', time.time()) # Usa tempo reale se sim_time da PX4 non c'è
                altitude_for_fsm = px4_telemetry['altitude_tof'] # Questa è l'altitudine relativa da MAVLinkInterface
                px4_mode = px4_telemetry.get('mode', 'N/A')

                previous_fsm_state = self.fsm.current_state
                self.fsm.update_altitude(altitude_for_fsm, current_mavlink_time)
                fsm_status = self.fsm.get_current_status()

                if fsm_status['alert_newly_triggered']:
                    self._handle_fsm_alert_activation(fsm_status, px4_telemetry) # Passa il dict px4_telemetry
                
                # La logica LED non controlla PX4, ma può essere loggata/inviata via MQTT
                led_effect_for_log_mqtt = "N/A"
                if fsm_status['state'] == AltitudeGuardFSM.STATE_ALERT:
                    led_effect_for_log_mqtt = "FSM_ALERT"
                elif fsm_status['state'].startswith("ADJUST"):
                    led_effect_for_log_mqtt = "FSM_ADJUSTING"
                else: 
                    led_effect_for_log_mqtt = "FSM_STABLE"

                # Adatta per DataLogger
                log_data_obj = PX4SensorDataLog(
                    sim_time=current_mavlink_time,
                    altitude_tof=altitude_for_fsm,
                    altitude_barometer=px4_telemetry.get('altitude_barometer', 0.0),
                    battery_voltage=px4_telemetry.get('battery_voltage', 0.0),
                    temperature=px4_telemetry.get('temperature', 25.0), # Potrebbe non esserci
                    vibration_x=0, vibration_y=0, vibration_z=0, # Non da SITL 'none'
                    signal_strength=99, packet_loss_rate=0.0 # Placeholder
                )
                self.data_logger.log_sensor_reading(log_data_obj, 
                                                    px4_mode, # Usa il modo PX4 come "scenario"
                                                    led_effect_for_log_mqtt)
                
                if self.live_plotter:
                    self.live_plotter.add_data(current_mavlink_time, 
                                               altitude_for_fsm, 
                                               self.fsm.target_altitude_m,
                                               fsm_status['state'],
                                               px4_mode) # Usa modo PX4 per il titolo del plot

            elapsed_time = time.time() - loop_start_time
            sleep_time = loop_interval - elapsed_time
            if sleep_time > 0: time.sleep(sleep_time)
        print("Orchestration loop (PX4) finished.")

    def _handle_fsm_alert_activation(self, fsm_status: Dict, px4_sensor_data: Dict): # px4_sensor_data è un dict
        sim_time_for_alert = px4_sensor_data.get('sim_time', time.time())
        print(f"FSM ALERT TRIGGERED! State: {fsm_status['state']}, Time: {sim_time_for_alert:.2f}s, Alt: {fsm_status['altitude']:.3f}m, Dev: {fsm_status['deviation']:.3f}m")
        print('\a', end='', flush=True) 
        if os.name == 'nt':
            try: import winsound; winsound.Beep(1000, 300)
            except ImportError: pass

        alert_details = {
            'sim_time_s': sim_time_for_alert, 
            'current_altitude_m': round(fsm_status['altitude'], 3),
            'target_altitude_m': round(fsm_status['target'], 3),
            'deviation_m': round(fsm_status['deviation'], 3),
            'fsm_state_at_alert': fsm_status['state'],
            'triggering_sensor': 'PX4_Relative_Altitude', 
            'battery_voltage_at_alert': round(px4_sensor_data.get('battery_voltage',0.0),2)
        }
        self.mqtt_bridge.publish_alert_message("FSM_PX4_ALTITUDE_ALERT", alert_details, "CRITICAL")
        self.data_logger.log_event("FSM_PX4_ALERT_TRIGGERED", alert_details, "CRITICAL")
        self.dashboard_data_handler.add_alert_to_dashboard(
            "FSM_PX4_ALTITUDE_GUARD", 
            f"Time {sim_time_for_alert:.1f}s: Alt dev: {fsm_status['deviation']:.2f}m. Drone at {fsm_status['altitude']:.2f}m.",
            "CRITICAL", 
            alert_details
        )

    def _mqtt_cmd_set_fsm_target_altitude(self, payload: Dict): # Rinominato per chiarezza
        new_target_alt = payload.get('target_altitude_m')
        if new_target_alt is not None:
            try:
                target_alt_float = float(new_target_alt)
                print(f"Orchestrator: MQTT CMD set FSM target altitude to {target_alt_float}m")
                self.fsm.set_target_altitude(target_alt_float)
                # Non inviamo comandi a PX4 per cambiare il suo target di altitudine qui
                self.data_logger.log_event("SET_FSM_TARGET_ALTITUDE_COMMAND", payload, "CONFIG")
            except ValueError: print(f"Orchestrator: Invalid target altitude value: {new_target_alt}")
        else: print("Orchestrator: Invalid set_target_altitude payload:", payload)


def run_complete_iot_system_demo(enable_plot=True):
    global orchestrator_instance 
    if enable_plot:
        orchestrator_instance = IoTSystemOrchestrator(enable_live_plot=True)
        try:
            print("\nIMPORTANT: Make sure PX4 SITL (e.g., 'make px4_sitl_default none') is running in another terminal.")
            print("AND QGroundControl is connected to see PX4 status (optional).\n")
            time.sleep(3) # Dai tempo di leggere il messaggio

            orchestrator_instance.start_system()
            if orchestrator_instance.system_running:
                print_mqtt_info_px4(orchestrator_instance) # Funzione helper aggiornata
                while orchestrator_instance.system_running:
                    if int(time.time()) % 30 == 0 and orchestrator_instance.system_running :
                        print_status_update_px4(orchestrator_instance) # Funzione helper aggiornata
                    plt.pause(0.1) 
                    if hasattr(orchestrator_instance, 'live_plotter') and \
                       orchestrator_instance.live_plotter and \
                       hasattr(orchestrator_instance.live_plotter, 'fig') and \
                       orchestrator_instance.live_plotter.fig and \
                       not plt.fignum_exists(orchestrator_instance.live_plotter.fig.number):
                        print("Plot window closed by user. Stopping system.")
                        orchestrator_instance.system_running = False 
                        break 
        except KeyboardInterrupt: print("\nUser requested system shutdown (Ctrl+C).")
        except Exception as e: print(f"UNHANDLED EXCEPTION in demo: {e}"); import traceback; traceback.print_exc()
        finally:
            print("Initiating system shutdown sequence...")
            if hasattr(orchestrator_instance, 'system_running') and orchestrator_instance.system_running:
                 orchestrator_instance.stop_system()
            elif hasattr(orchestrator_instance, 'stop_system'): 
                 orchestrator_instance.stop_system()
            plt.close('all') 
            print("Demo finished.")
    else: # Senza plot
        orchestrator = IoTSystemOrchestrator(enable_live_plot=False)
        try:
            print("\nIMPORTANT: Make sure PX4 SITL (e.g., 'make px4_sitl_default none') is running in another terminal.\n")
            time.sleep(3)
            orchestrator.start_system()
            if orchestrator.system_running:
                print_mqtt_info_px4(orchestrator)
                while orchestrator.system_running: 
                     if int(time.time()) % 30 == 0 and orchestrator.system_running :
                        print_status_update_px4(orchestrator)
                     time.sleep(1.0) 
        except KeyboardInterrupt: print("\nUser requested system shutdown (Ctrl+C).")
        except Exception as e: print(f"UNHANDLED EXCEPTION in demo: {e}"); import traceback; traceback.print_exc()
        finally:
            print("Initiating system shutdown sequence...")
            if orchestrator.system_running: orchestrator.stop_system()
            print("Demo finished.")

def print_mqtt_info_px4(orchestrator): # Aggiornata
    print("\n" + "="*30 + " PX4 ALTITUDE GUARD SYSTEM IS LIVE " + "="*30)
    print("Press Ctrl+C to stop this script (PX4 SITL & QGC must be stopped separately).")
    print(f"MQTT Telemetry on: {orchestrator.mqtt_bridge.topics['telemetry']}")
    print(f"MQTT Drone Status on: {orchestrator.mqtt_bridge.topics['status']}")
    print(f"MQTT Alerts on: {orchestrator.mqtt_bridge.topics['alerts']}")
    print(f"Send FSM target altitude command to: {orchestrator.mqtt_bridge.topics['commands']}/set_target_altitude")
    print(f"   Payload: {{\"target_altitude_m\": 0.5}}")
    print("   (Note: Other commands like anomaly injection are now manual via PX4/QGC or MAVLink tools)")
    print("="*81 + "\n")

def print_status_update_px4(orchestrator): # Aggiornata
    px4_data = orchestrator.mav_interface.get_telemetry()
    fsm_status = orchestrator.fsm.get_current_status()
    current_time_for_print = px4_data.get('sim_time', time.time()) # O `time.time()`
    
    print(f"\n--- STATUS UPDATE [{time.strftime('%H:%M:%S')} | MAVLink Time: {current_time_for_print:.1f}s] ---")
    print(f"  PX4 Mode: {px4_data.get('mode', 'N/A')}, Armed: {px4_data.get('armed', False)}")
    print(f"  FSM State: {fsm_status['state']}, FSM Target: {fsm_status['target']:.2f}m")
    print(f"  PX4 Rel Alt (FSM Input): {px4_data.get('altitude_tof', 0.0):.2f}m, FSM Deviation: {fsm_status['deviation']:.2f}m")
    if fsm_status['alert_active']: print(f"  FSM ALERT ACTIVE!")
    print(f"  Battery: {px4_data.get('battery_voltage', 0.0):.2f}V ({px4_data.get('battery_remaining_pct', 0)}%)")
    print(f"  MQTT Connected: {orchestrator.mqtt_bridge.connected}")
    print("--- END STATUS UPDATE ---")

if __name__ == "__main__":
    run_complete_iot_system_demo(enable_plot=True)