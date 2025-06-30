# iot_system.py
import json # Libreria per la manipolazione di dati JSON (usata per i payload MQTT e logging eventi)
import time # Libreria per funzionalità legate al tempo (es. sleep, timestamp)
import threading # Libreria per creare ed eseguire codice in thread separati
import os # Libreria per interagire con il sistema operativo (es. per il beep su Windows)
import random # Libreria per generare numeri casuali (usata per client_id MQTT)
from typing import Dict, List, Callable, Optional # Per type hinting, migliora leggibilità e controllo

# Importa il client MQTT dalla libreria Paho
import paho.mqtt.client as mqtt
# sqlite3 non è usato direttamente qui, ma DataLogger (importato sotto) lo usa.

# Importa le classi definite negli altri file del progetto
from altitude_fsm import AltitudeGuardFSM # La Finite State Machine
from drone_simulation_components import CrazyflieHardwareSimulator, DataLogger, SensorReading # Componenti del simulatore e logger

# Import per Matplotlib, usato per il plotting live
import matplotlib.pyplot as plt
import matplotlib.animation as animation # Per creare animazioni/plot che si aggiornano
from collections import deque # Struttura dati tipo lista con aggiunta/rimozione efficiente alle estremità

# Import for ROS2
from std_msgs.msg import Float32MultiArray, String

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
        self.line_alt, = self.ax.plot([], [], 'b-', label='Altitude ToF (m)', linewidth=2, zorder=10) # Linea altitudine
        self.line_target, = self.ax.plot([], [], 'k--', label='Target Alt (m)', linewidth=1, zorder=9) # Linea target (nera tratteggiata)
        
        # Impostazione etichette, legenda, griglia e titolo del grafico
        self.ax.set_xlabel("Sim Time (s)", fontsize=12)
        self.ax.set_ylabel("Altitude (m)", fontsize=12)
        self.ax.legend(loc='upper right') # Posizione della legenda
        self.ax.grid(True, linestyle='--', alpha=0.7) # Griglia con stile tratteggiato e trasparenza
        self.title = self.ax.set_title("Live Altitude Data - FSM State: N/A", fontsize=12) # Titolo iniziale

        # Variabili per memorizzare lo stato corrente da visualizzare nel titolo
        self.current_fsm_state_for_title = "N/A"
        self.current_sim_time_for_title = 0.0
        self.current_scenario_for_title = "N/A"

        self.background_patches = [] # Lista per memorizzare i rettangoli dello sfondo (per poterli rimuovere)

        # Crea l'oggetto animazione che chiamerà _update_plot_data periodicamente
        # blit=False è spesso più stabile su diverse piattaforme, interval in millisecondi
        self.ani = animation.FuncAnimation(self.fig, self._update_plot_data, 
                                           blit=False, interval=100, cache_frame_data=False) 
        plt.show(block=False) # Mostra la finestra del grafico senza bloccare l'esecuzione dello script
        plt.pause(0.001) # Pausa minima necessaria per far apparire la finestra su alcuni backend GUI

    def add_data(self, sim_time: float, altitude: float, target_altitude: float, fsm_state: str, scenario: str):
        """Aggiunge nuovi dati alle deque per l'aggiornamento del grafico."""
        self.sim_time_data.append(sim_time)
        self.altitude_data.append(altitude)
        self.target_altitude_data.append(target_altitude)
        self.fsm_state_plot_data.append(self.state_to_numeric.get(fsm_state, -1)) # Converte stato testuale in numerico
        # Aggiorna le variabili usate per il titolo del grafico
        self.current_fsm_state_for_title = fsm_state
        self.current_sim_time_for_title = sim_time
        self.current_scenario_for_title = scenario
    
    # Il metodo add_fsm_event non è attualmente utilizzato attivamente nel loop di orchestrazione con la colorazione dello sfondo,
    # ma è lasciato per future estensioni (es. annotazioni testuali dirette sul grafico).
    def add_fsm_event(self, sim_time: float, event_description: str, color: str = 'black', y_pos_factor: float = 0.95):
        """Aggiunge un evento FSM da annotare (attualmente non usato attivamente)."""
        if self.altitude_data: # Calcola la posizione y basata sui limiti correnti del grafico
            plot_min_y, plot_max_y = self.ax.get_ylim()
            marker_y = plot_min_y + (plot_max_y - plot_min_y) * y_pos_factor
        else: # Default se non ci sono dati
            marker_y = 0.7 * y_pos_factor
        # self.fsm_event_annotations.append({ ... }) # Commentato perché non usato

    def _update_plot_data(self, frame):
        """Funzione chiamata periodicamente da FuncAnimation per aggiornare il grafico."""
        if not self.sim_time_data: # Se non ci sono dati, non fare nulla
            return self.line_alt, self.line_target, self.title # Restituisce gli artisti da aggiornare
            
        # Aggiorna i dati delle linee di altitudine e target
        self.line_alt.set_data(self.sim_time_data, self.altitude_data)
        self.line_target.set_data(self.sim_time_data, self.target_altitude_data)
        
        fsm_s = self.current_fsm_state_for_title # Stato FSM corrente per il titolo
        
        # Auto-scala gli assi per adattarsi ai dati correnti
        if self.sim_time_data:
            # Calcola i limiti y includendo un margine e assicurando che 0 e 1 (o valori di riferimento) siano visibili
            min_alt_plot = min(list(self.altitude_data) + list(self.target_altitude_data) + [0.0]) - 0.2 
            max_alt_plot = max(list(self.altitude_data) + list(self.target_altitude_data) + [1.0]) + 0.2
            
            self.ax.set_xlim(max(0, self.sim_time_data[0]-1), self.sim_time_data[-1] + 5) # Imposta limiti x con un buffer
            self.ax.set_ylim(min_alt_plot, max_alt_plot) # Imposta limiti y
        
        # Aggiorna il titolo del grafico con le informazioni correnti
        self.title.set_text(f"FSM: {fsm_s} | Scenario: {self.current_scenario_for_title} | Alt: {self.altitude_data[-1]:.2f}m | SimTime: {self.current_sim_time_for_title:.1f}s")

        # Rimuovi i rettangoli di sfondo precedenti per ridisegnarli
        for patch in self.background_patches: patch.remove()
        self.background_patches.clear()

        # Aggiungi regioni colorate sullo sfondo per indicare lo stato FSM
        if len(self.sim_time_data) > 1: # Se ci sono almeno due punti dati
            for i in range(len(self.sim_time_data) - 1): # Itera sui segmenti temporali
                start_time = self.sim_time_data[i]
                end_time = self.sim_time_data[i+1]
                state_numeric = self.fsm_state_plot_data[i] # Stato FSM numerico all'inizio del segmento
                color = self.numeric_to_color.get(state_numeric, 'white') # Colore corrispondente
                
                # Crea un rettangolo che copre il segmento temporale e l'intera altezza del grafico
                rect = plt.Rectangle((start_time, self.ax.get_ylim()[0]), # (x,y) angolo inferiore sx
                                     end_time - start_time, # larghezza
                                     self.ax.get_ylim()[1] - self.ax.get_ylim()[0], # altezza
                                     facecolor=color, alpha=0.2, zorder=1, edgecolor='none') # Stile
                self.ax.add_patch(rect) # Aggiungi il rettangolo agli assi
                self.background_patches.append(rect) # Memorizza il riferimento al rettangolo
        
        try: 
            self.fig.canvas.draw_idle() # Richiedi un ridisegno "pigro" del canvas
        except Exception: pass # Ignora errori se la finestra viene chiusa durante il draw
        # Restituisce gli artisti modificati (necessario per blitting, anche se blit=False)
        return self.line_alt, self.line_target, self.title, *self.background_patches

    def save_plot(self, filename="altitude_plot_final.png"):
        """Salva lo stato corrente del grafico in un file immagine."""
        if not self.fig: 
            print("Plotter: Figure does not exist, cannot save.")
            return

        if self.sim_time_data: # Assicurati che l'ultimo frame sia disegnato
            try:
                self._update_plot_data(None) 
            except Exception as e:
                print(f"Plotter: Error during final update before save: {e}")
        
        # Ferma l'animazione in modo sicuro prima di salvare
        if hasattr(self, 'ani') and self.ani is not None:
            if hasattr(self.ani, 'event_source') and self.ani.event_source is not None:
                try:
                    self.ani.event_source.stop()
                    print("Plotter: Animation stopped for saving.")
                except Exception as e:
                    print(f"Plotter: Error stopping animation event_source: {e}")
        else:
            print("Plotter: Animation object 'ani' not found or already None during save.")

        try:
            # Salva la figura con una buona risoluzione (dpi) e adattando i bordi (bbox_inches)
            self.fig.savefig(filename, dpi=200, bbox_inches='tight')
            print(f"Plot saved to {filename}")
        except Exception as e:
            print(f"Error saving plot: {e}")

    def close_plot(self):
        """Chiude la finestra del grafico e ferma l'animazione."""
        # Ferma l'animazione se attiva
        if hasattr(self, 'ani') and self.ani is not None:
            if hasattr(self.ani, 'event_source') and self.ani.event_source is not None:
                try:
                    self.ani.event_source.stop()
                except Exception: pass # Ignora errori se già fermata o problematica
            self.ani = None 

        # Chiudi la figura Matplotlib se esiste
        if hasattr(self, 'fig') and self.fig is not None:
            try:
                plt.close(self.fig)
            except Exception: pass
            self.fig = None 
            print("LivePlotter window closed.")
        elif not hasattr(self, 'fig') or self.fig is None:
            print("LivePlotter: Window was already considered closed or not initialized.")


class MQTTIoTBridge:
    """
    Gestisce la comunicazione con un broker MQTT per inviare telemetria/alert
    e ricevere comandi.
    """
    def __init__(self, broker_host="localhost", broker_port=1883, client_id_prefix="crazyflie_iot_"):
        self.broker_host = broker_host; self.broker_port = broker_port
        # Genera un client_id univoco per evitare conflitti se più istanze sono eseguite
        self.client_id = f"{client_id_prefix}{random.randint(1000, 9999)}"
        # Inizializza il client MQTT specificando la versione dell'API dei callback
        # VERSION1 è usata per compatibilità con le firme dei callback attuali.
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, client_id=self.client_id)
        # Assegna i metodi di callback del client MQTT
        self.client.on_connect = self._on_connect; self.client.on_message = self._on_message
        self.client.on_disconnect = self._on_disconnect
        
        self.base_topic = "crazyflie/drone_01" # Topic MQTT di base per questo drone
        # Struttura dei topic usati
        self.topics = {
            'telemetry': f"{self.base_topic}/telemetry", 'status': f"{self.base_topic}/status", 
            'alerts': f"{self.base_topic}/alerts", 'commands': f"{self.base_topic}/commands",
            'diagnostics': f"{self.base_topic}/diagnostics"
        }
        
        self.command_callbacks: Dict[str, Callable] = {} # Dizionario per registrare handler di comandi
        self.connected = False # Flag per lo stato della connessione MQTT
        
        # Rate di pubblicazione per telemetria e status
        self.publish_telemetry_rate_hz = 10; self.publish_status_rate_hz = 1
        self.publishing_active = False; self.publish_thread = None # Per il loop di pubblicazione
        
    def connect(self) -> bool:
        """Tenta di connettersi al broker MQTT."""
        try:
            print(f"MQTT: Attempting to connect to {self.broker_host}:{self.broker_port} as {self.client_id}")
            self.client.connect(self.broker_host, self.broker_port, 60) # Timeout di 60s per la connessione
            self.client.loop_start() # Avvia un thread in background per gestire la rete MQTT
            # Attende brevemente che il callback on_connect imposti self.connected
            for _ in range(50): 
                if self.connected: break
                time.sleep(0.1)
            if not self.connected: # Se non connesso dopo il timeout
                 print(f"MQTT: Connection timed out."); self.client.loop_stop(); return False
            return True
        except Exception as e: print(f"MQTT: Connection failed: {e}"); self.connected = False; return False
    
    def disconnect(self):
        """Disconnette dal broker MQTT e ferma i loop."""
        if self.publishing_active: self.stop_publishing_loop() # Ferma il loop di pubblicazione
        if self.client: self.client.loop_stop(); self.client.disconnect() # Ferma il loop di rete e disconnette
        print("MQTT: Disconnected"); self.connected = False
    
    def _on_connect(self, client, userdata, flags, rc):
        """Callback eseguito quando la connessione al broker MQTT ha successo."""
        if rc == 0: # Codice di ritorno 0 significa successo
            self.connected = True; print("MQTT: Connected successfully.")
            command_topic_filter = f"{self.topics['commands']}/#" # Sottoscrivi a tutti i sottotopic di /commands
            client.subscribe(command_topic_filter)
            print(f"MQTT: Subscribed to {command_topic_filter}")
        else: # Errore di connessione
            print(f"MQTT: Connection failed with code {rc}. Flags: {flags}"); self.connected = False
    
    def _on_disconnect(self, client, userdata, rc):
        """Callback eseguito quando il client si disconnette dal broker."""
        self.connected = False
        if rc != 0: print(f"MQTT: Unexpected disconnection (rc={rc}). Will attempt to reconnect.") # La libreria paho tenta di riconnettere
        else: print("MQTT: Disconnected gracefully.")
    
    def _on_message(self, client, userdata, msg):
        """Callback eseguito quando viene ricevuto un messaggio MQTT sui topic sottoscritti."""
        try:
            topic = msg.topic; payload_str = msg.payload.decode() # Decodifica il payload del messaggio
            # Stampa una versione abbreviata del payload se è troppo lungo
            # print(f"MQTT Rx: {topic} -> {payload_str[:100]}{'...' if len(payload_str)>100 else ''}")
            payload = json.loads(payload_str) # Converte il payload JSON in un dizionario Python
            
            # Gestione dei comandi: estrae il tipo di comando dal topic
            if topic.startswith(self.topics['commands'] + "/"): # Se il topic è un comando
                command_type = topic.split('/')[-1] # L'ultima parte del topic è il tipo di comando
                if command_type in self.command_callbacks: # Se c'è un handler registrato
                    self.command_callbacks[command_type](payload) # Esegui l'handler
                else: print(f"MQTT: Unhandled command type: {command_type} on topic {topic}")
        except json.JSONDecodeError: print(f"MQTT: Failed to decode JSON payload from {topic}: {msg.payload.decode()}")
        except Exception as e: print(f"MQTT: Error processing message from {topic}: {e}")
    
    def register_command_handler(self, command_type: str, callback: Callable):
        """Registra una funzione di callback per un tipo specifico di comando MQTT."""
        self.command_callbacks[command_type] = callback
        # print(f"MQTT: Registered handler for command type: '{command_type}'")
    
    def publish_telemetry(self, sensor_data: SensorReading, fsm_status: Dict):
        """Pubblica dati di telemetria (sensori e stato FSM)."""
        if not self.connected: return False # Non pubblicare se non connesso
        # Costruisce il payload della telemetria
        telemetry_payload = {
            'sim_time_s': round(sensor_data.sim_time, 3), 
            'publish_real_time_s': round(time.time(),3), # Timestamp reale della pubblicazione
            'altitude_tof_m': round(sensor_data.altitude_tof, 3),
            'battery_v': round(sensor_data.battery_voltage, 2),
            'fsm': fsm_status # Includi l'intero dizionario di stato della FSM
        }
        return self._publish(self.topics['telemetry'], telemetry_payload) # Pubblica sul topic telemetria
    
    def publish_drone_status(self, sim_system_status: Dict):
        """Pubblica lo stato generale del sistema/simulatore."""
        if not self.connected: return False
        # Costruisce il payload dello stato
        status_payload = {
            'sim_time_s': sim_system_status.get('sim_time_s',0.0), 
            'publish_real_time_s': sim_system_status.get('timestamp_real', time.time()),
            'scenario': sim_system_status['scenario'],
            'physics': sim_system_status['physics'], # Stato fisico del drone
            'led_effect': sim_system_status['led_effect'], # Stato dei LED simulati
            'active_anomalies': list(sim_system_status['anomalies_active'].keys()), # Lista delle anomalie attive
            'health_score': self._calculate_health_score(sim_system_status) # Punteggio di salute calcolato
        }
        return self._publish(self.topics['status'], status_payload) # Pubblica sul topic status
    
    def publish_alert_message(self, alert_type: str, alert_details: Dict, severity: str = "WARNING"):
        """Pubblica un messaggio di alert."""
        if not self.connected: return False
        # Costruisce il payload dell'alert
        alert_payload = {
            'real_time_s': round(time.time(),3), 
            'sim_time_at_alert_s': round(alert_details.get('sim_time_s', 0.0),3), # Tempo simulato dell'evento
            'alert_type': alert_type, 'severity': severity.upper(), # Tipo e severità dell'alert
            'details': alert_details, 'drone_id': self.base_topic.split('/')[-1] # ID del drone (dall'argomento base_topic)
        }
        return self._publish(self.topics['alerts'], alert_payload) # Pubblica sul topic alerts
        
    def _publish(self, topic: str, data: Dict, qos: int = 0, retain: bool = False) -> bool:
        """Metodo helper interno per pubblicare un messaggio MQTT."""
        if not self.connected: return False
        try:
            payload_json = json.dumps(data, default=str) # Serializza il dizionario in una stringa JSON
                                                        # default=str gestisce tipi come numpy float
            result = self.client.publish(topic, payload_json, qos=qos, retain=retain) # Pubblica il messaggio
            if result.rc == mqtt.MQTT_ERR_SUCCESS: return True # Successo
            else: print(f"MQTT: Publish to {topic} failed with code {result.rc}"); return False
        except TypeError as e: print(f"MQTT: JSON serialization error for topic {topic}: {e}. Data: {data}"); return False
        except RuntimeError as e: print(f"MQTT: Runtime error publishing to {topic}: {e}"); self.connected = False; return False
        except Exception as e: print(f"MQTT: Generic publish error to {topic}: {e}"); return False
    
    def _calculate_health_score(self, sim_status: Dict) -> float:
        """Calcola un punteggio di "salute" del sistema (semplificato)."""
        score = 100.0
        # Controlla se i dati necessari sono presenti
        if 'physics' not in sim_status or 'communications' not in sim_status: return 50.0 
        
        # Penalità basate sul livello della batteria
        battery_pct = sim_status['physics'].get('battery_level', 100.0)
        if battery_pct < 10: score -= 40
        elif battery_pct < 25: score -= 20
        
        # Penalità basate sull'intensità del segnale
        signal = sim_status['communications'].get('signal_strength', 90)
        if signal < 40: score -= 25
        elif signal < 60: score -= 15
        
        # Penalità basate sulla perdita di pacchetti
        packet_loss_pct = sim_status['communications'].get('packet_loss', 0.0) * 100
        if packet_loss_pct > 15: score -= 30
        elif packet_loss_pct > 5: score -= 15
        
        # Penalità per ogni anomalia attiva
        active_anomalies_count = len(sim_status.get('anomalies_active', {}))
        score -= active_anomalies_count * 10
        
        return round(max(0.0, min(100.0, score)), 1) # Assicura che il punteggio sia tra 0 e 100
    
    def start_publishing_loop(self, simulator_ref, fsm_ref):
        """Avvia il thread che pubblica periodicamente dati telemetrici e di stato."""
        if self.publishing_active or not self.connected: return # Non avviare se già attivo o non connesso
        self.publishing_active = True
        # Crea e avvia il thread, passando riferimenti al simulatore e alla FSM
        self.publish_thread = threading.Thread(target=self._publishing_loop_content, args=(simulator_ref, fsm_ref))
        self.publish_thread.daemon = True; self.publish_thread.start()
        print("MQTT: Publishing loop started.")
    
    def stop_publishing_loop(self):
        """Ferma il thread di pubblicazione."""
        self.publishing_active = False # Segnala al loop di terminare
        if self.publish_thread and self.publish_thread.is_alive(): 
            self.publish_thread.join(timeout=1.0) # Attendi che il thread termini
        print("MQTT: Publishing loop stopped.")

    def _publishing_loop_content(self, simulator: CrazyflieHardwareSimulator, fsm: AltitudeGuardFSM):
        """Contenuto del loop di pubblicazione eseguito dal thread."""
        
        last_tel_pub = 0  
        last_stat_pub = 0 
        
        tel_int = 1.0 / self.publish_telemetry_rate_hz 
        stat_int = 1.0 / self.publish_status_rate_hz
        
        loop_counter = 0 # AGGIUNTO: Contatore per debug

        print(f"MQTTIoTBridge: Publishing loop starting... Telemetry interval: {tel_int:.3f}s, Status interval: {stat_int:.3f}s")

        while self.publishing_active and self.connected:
            try:
                now = time.time() 
                loop_counter += 1 # AGGIUNTO

                # Debug ogni N cicli
                if loop_counter % (self.publish_telemetry_rate_hz * 2) == 0: # Ogni circa 2 secondi
                    print(f"MQTTIoTBridge: Publishing loop active. Cycle: {loop_counter}, SimTime: {simulator.current_sim_time if simulator else 'N/A'}")

                current_sensor_data = None
                current_fsm_status = None
                current_sim_status = None

                # Ottieni i dati con try-except individuali per isolare problemi
                try:
                    current_sensor_data = simulator.get_latest_reading()
                except Exception as e_sensor:
                    print(f"MQTTIoTBridge: Errore in simulator.get_latest_reading(): {e_sensor}")
                    # Considera se continuare o uscire dal loop
                    continue # Salta questa iterazione

                try:
                    current_fsm_status = fsm.get_current_status()
                except Exception as e_fsm:
                    print(f"MQTTIoTBridge: Errore in fsm.get_current_status(): {e_fsm}")
                    continue

                try:
                    current_sim_status = simulator.get_system_status()
                except Exception as e_sim_status:
                    print(f"MQTTIoTBridge: Errore in simulator.get_system_status(): {e_sim_status}")
                    continue
                
                # Pubblica Telemetria
                if current_sensor_data and current_fsm_status and (now - last_tel_pub >= tel_int) :
                    # print(f"DEBUG MQTT Bridge: Publishing Telemetry. SimTime: {current_sensor_data.sim_time}") 
                    if not self.publish_telemetry(current_sensor_data, current_fsm_status):
                        print("MQTTIoTBridge: Fallimento publish_telemetry.")
                    last_tel_pub = now 
                
                # Pubblica Stato Drone
                if current_sim_status and (now - last_stat_pub >= stat_int):
                    # print(f"DEBUG MQTT Bridge: Publishing Status. SimTime: {current_sim_status.get('sim_time_s')}") 
                    if not self.publish_drone_status(current_sim_status):
                        print("MQTTIoTBridge: Fallimento publish_drone_status.")
                    last_stat_pub = now 
                
                sleep_for = min(tel_int, stat_int) / 5.0 # Leggermente più lungo, es. /5 invece di /10
                time.sleep(max(0.005, sleep_for)) # Aumentato sleep minimo

            except Exception as e_loop: 
                print(f"MQTTIoTBridge: ERRORE CRITICO nel publishing loop: {e_loop}")
                import traceback
                traceback.print_exc() 
                self.publishing_active = False # Ferma il loop in caso di errore non gestito
                # Considera di chiamare self.disconnect() o un metodo di cleanup qui
                break # Esci dal loop
        
        print(f"MQTTIoTBridge: Publishing loop terminato. Active: {self.publishing_active}, Connected: {self.connected}")

class IoTDashboardData: 
    """Classe semplificata per gestire dati per una potenziale dashboard (in memoria)."""
    def __init__(self, max_alerts_history=20):
        self.alert_history: List[Dict] = [] # Lista per memorizzare la cronologia degli alert
        self.max_alerts_history = max_alerts_history # Numero massimo di alert da tenere in memoria
        
    def add_alert_to_dashboard(self, alert_type: str, message: str, severity: str = "WARNING", details: Optional[Dict] = None):
        """Aggiunge un nuovo alert alla cronologia."""
        timestamp = time.time()
        new_alert = {'timestamp': timestamp, 'type': alert_type, 'severity': severity.upper(), 'message': message, 'details': details or {}}
        self.alert_history.append(new_alert)
        # Mantiene la cronologia entro la dimensione massima
        if len(self.alert_history) > self.max_alerts_history: self.alert_history.pop(0)
        # print(f"Dashboard Event: New {severity} alert - {alert_type}: {message}") # Stampa opzionale

    def get_recent_alerts(self, count=5) -> List[Dict]: 
        """Restituisce gli ultimi 'count' alert."""
        return self.alert_history[-count:]
    
    def get_dashboard_summary(self) -> Dict:
        """Restituisce un riassunto base per una dashboard."""
        return {'timestamp': time.time(), 'active_alerts_count': len(self.alert_history), 
                'recent_alerts_preview': self.alert_history[-3:], 'status': 'Operational'}

class IoTSystemOrchestrator:
    """
    Componente principale che orchestra tutti gli altri moduli del sistema:
    simulatore, FSM, bridge MQTT, logger, plotter.
    """
    def __init__(self, enable_live_plot=True): # Flag per abilitare/disabilitare il plot live
        # Inizializzazione dei componenti principali
        self.simulator = CrazyflieHardwareSimulator(update_rate=50) # Il simulatore hardware
        self.data_logger = DataLogger(db_path="iot_system_flight_data_liveplot.db") # Il logger su DB
        self.mqtt_bridge = MQTTIoTBridge() # Il bridge per MQTT
        self.dashboard_data_handler = IoTDashboardData() # Gestore dati base per dashboard
        self.fsm = AltitudeGuardFSM() # La Finite State Machine
        
        self.system_running = False # Flag per controllare lo stato di esecuzione del sistema
        self.monitoring_thread = None # Thread per il loop di orchestrazione
        self.main_loop_rate_hz = 20  # Frequenza del loop di monitoraggio dell'orchestratore

        self.enable_live_plot = enable_live_plot # Memorizza se il plot è abilitato
        self.live_plotter = None # Oggetto LivePlotter, inizializzato in initialize_system
        
    def initialize_system(self) -> bool:
        """Inizializza tutti i componenti del sistema."""
        print("Initializing IoT System Orchestrator...")
        # Inizializza il plotter se abilitato e non già creato
        # Questo viene fatto qui per assicurare che Matplotlib sia inizializzato nel thread principale
        if self.enable_live_plot and self.live_plotter is None:
            self.live_plotter = LivePlotter()

        # Registra gli handler per i comandi MQTT
        self.mqtt_bridge.register_command_handler('inject_anomaly', self._mqtt_cmd_inject_anomaly)
        self.mqtt_bridge.register_command_handler('set_target_altitude', self._mqtt_cmd_set_target_altitude)
        self.mqtt_bridge.register_command_handler('reset_simulation', self._mqtt_cmd_reset_simulation)
        
        # Connetti al broker MQTT
        if not self.mqtt_bridge.connect():
            print("Orchestrator: MQTT connection failed. System cannot start."); return False
        print("IoT System Orchestrator initialized successfully.")
        return True
    
    def start_system(self):
        """Avvia tutti i componenti e i loop del sistema."""
        if not self.initialize_system(): return # Inizializza prima di avviare
            
        print("Starting Complete IoT System...")
        self.system_running = True # Imposta il flag di esecuzione
        
        self.simulator.start_simulation() # Avvia il simulatore (in un thread separato)
        self.mqtt_bridge.start_publishing_loop(self.simulator, self.fsm) # Avvia il loop di pubblicazione MQTT (thread separato)
        
        # Avvia il loop di monitoraggio/orchestrazione principale in un thread separato
        self.monitoring_thread = threading.Thread(target=self._orchestration_loop)
        self.monitoring_thread.daemon = True; self.monitoring_thread.start()
        print("IoT System Orchestrator is RUNNING.")
    
    def stop_system(self):
        """Ferma tutti i componenti del sistema in modo pulito."""
        print("Stopping IoT System Orchestrator..."); 
        self.system_running = False # Segnala ai loop di terminare
        
        # Attendi che il thread di monitoraggio termini
        if self.monitoring_thread and self.monitoring_thread.is_alive(): self.monitoring_thread.join(timeout=1.0)
        
        # Ferma e disconnette gli altri componenti
        self.mqtt_bridge.disconnect()
        self.simulator.stop_simulation()
        
        # Salva il grafico finale se il plotter era attivo
        if self.live_plotter: 
            self.live_plotter.save_plot("final_altitude_simulation_plot.png")
            self.live_plotter.close_plot() 

        # Salva il log della FSM e chiudi il logger del database
        fsm_log_file = self.fsm.save_log_to_csv("final_fsm_log_liveplot.csv") # Nome del file di log FSM
        if fsm_log_file: print(f"FSM Log saved: {fsm_log_file}")
        self.data_logger.close_db()
        
        print("⏹IoT System Orchestrator stopped.")
    
    def _orchestration_loop(self):
        """Loop principale dell'orchestratore: legge sensori, aggiorna FSM, logga, plotta."""
        print("Orchestration loop started.")
        loop_interval = 1.0 / self.main_loop_rate_hz # Intervallo del loop

        while self.system_running: # Continua finché il sistema è in esecuzione
            loop_start_time = time.time() # Timestamp inizio ciclo
            
            sensor_data = self.simulator.get_latest_reading() # Prendi l'ultima lettura dal simulatore
            # Ottieni lo scenario corrente dal simulatore per il titolo del plot e log
            current_sim_scenario = self.simulator.get_system_status().get('scenario', 'N/A') 
            
            if sensor_data: # Se ci sono nuovi dati dai sensori
                # Aggiorna la FSM con l'altitudine ToF e il timestamp simulato
                self.fsm.update_altitude(sensor_data.altitude_tof, sensor_data.sim_time)
                fsm_status = self.fsm.get_current_status() # Ottieni lo stato aggiornato della FSM

                # Se l'alert FSM è appena scattato, gestisci le azioni di alert
                if fsm_status['alert_newly_triggered']:
                    self._handle_fsm_alert_activation(fsm_status, sensor_data)
                
                # Aggiorna l'effetto LED simulato basato sullo stato FSM corrente
                if fsm_status['state'] == AltitudeGuardFSM.STATE_ALERT:
                    self.simulator.set_led_effect("alert_flashing_red") 
                elif fsm_status['state'].startswith("ADJUST"): # Copre ADJUST_UP e ADJUST_DOWN
                    self.simulator.set_led_effect("adjusting_yellow")
                else: # HOLD_STABLE
                    self.simulator.set_led_effect("stable_green")

                # Logga i dati dei sensori, lo scenario corrente e l'effetto LED nel database
                sim_status_for_log = self.simulator.get_system_status() # Prende lo stato attuale per scenario e LED
                self.data_logger.log_sensor_reading(sensor_data, 
                                                    sim_status_for_log['scenario'],
                                                    sim_status_for_log['led_effect'])
                
                # Se il plotter live è abilitato, aggiungi i nuovi dati
                if self.live_plotter:
                    self.live_plotter.add_data(sensor_data.sim_time, 
                                               sensor_data.altitude_tof, 
                                               self.fsm.target_altitude_m, # Altitudine target dalla FSM
                                               fsm_status['state'],       # Stato FSM corrente
                                               current_sim_scenario)      # Scenario corrente del simulatore

            # Calcola il tempo trascorso nel ciclo e attendi per mantenere il rate del loop
            elapsed_time = time.time() - loop_start_time
            sleep_time = loop_interval - elapsed_time
            if sleep_time > 0: 
                time.sleep(sleep_time)
        print("Orchestration loop finished.")

    def _handle_fsm_alert_activation(self, fsm_status: Dict, sensor_data: SensorReading):
        """Gestisce le azioni da intraprendere quando la FSM entra nello stato ALERT."""
        # Stampa un messaggio di alert sulla console
        print(f"FSM ALERT TRIGGERED! State: {fsm_status['state']}, SimTime: {sensor_data.sim_time:.2f}s, Alt: {fsm_status['altitude']:.3f}m, Dev: {fsm_status['deviation']:.3f}m")
        
        # Emette un suono di beep (carattere ASCII BEL)
        print('\a', end='', flush=True) 
        # Suono specifico per Windows (se il modulo winsound è disponibile)
        if os.name == 'nt': # 'nt' è il nome per sistemi Windows
            try: 
                import winsound
                winsound.Beep(1000, 300) # Frequenza 1kHz, Durata 300ms
            except ImportError: 
                pass # Ignora se winsound non è disponibile

        # Prepara i dettagli dell'alert per il logging e MQTT
        alert_details = {
            'sim_time_s': sensor_data.sim_time, 
            'current_altitude_m': round(fsm_status['altitude'], 3),
            'target_altitude_m': round(fsm_status['target'], 3),
            'deviation_m': round(fsm_status['deviation'], 3),
            'fsm_state_at_alert': fsm_status['state'],
            'triggering_sensor': 'ToF', # Indica quale sensore ha probabilmente causato l'alert
            'battery_voltage_at_alert': round(sensor_data.battery_voltage,2)
        }
        # Pubblica l'alert via MQTT
        self.mqtt_bridge.publish_alert_message("FSM_ALTITUDE_GUARD_ALERT", alert_details, "CRITICAL")
        # Logga l'evento di alert nel database
        self.data_logger.log_event("FSM_ALERT_TRIGGERED", alert_details, "CRITICAL")
        # Aggiunge l'alert alla cronologia della dashboard in memoria
        self.dashboard_data_handler.add_alert_to_dashboard(
            "FSM_ALTITUDE_GUARD", 
            f"SimTime {sensor_data.sim_time:.1f}s: Alt dev: {fsm_status['deviation']:.2f}m. Drone at {fsm_status['altitude']:.2f}m.",
            "CRITICAL", 
            alert_details
        )

    def _mqtt_cmd_inject_anomaly(self, payload: Dict):
        """Handler per il comando MQTT 'inject_anomaly'."""
        anomaly_type = payload.get('type') # Tipo di anomalia da iniettare
        active = payload.get('active', True) # Se attivare (default) o disattivare l'anomalia
        duration = payload.get('duration')   # Durata opzionale dell'anomalia in secondi
        
        if anomaly_type: # Se il tipo di anomalia è specificato
            print(f"Orchestrator: MQTT CMD inject_anomaly: {anomaly_type}, active={active}, duration={duration}s")
            self.simulator.inject_anomaly(anomaly_type, active=active, duration=duration) # Chiama il metodo del simulatore
            self.data_logger.log_event("MANUAL_ANOMALY_COMMAND", payload, "INFO") # Logga il comando
        else: 
            print(f"Orchestrator: Invalid inject_anomaly command payload:", payload)

    def _mqtt_cmd_set_target_altitude(self, payload: Dict):
        """Handler per il comando MQTT 'set_target_altitude'."""
        new_target_alt = payload.get('target_altitude_m') # Nuova altitudine target dal payload
        if new_target_alt is not None:
            try:
                target_alt_float = float(new_target_alt) # Converte in float
                print(f"Orchestrator: MQTT CMD set_target_altitude to {target_alt_float}m")
                self.fsm.set_target_altitude(target_alt_float) # Imposta nella FSM
                self.simulator.set_target_altitude_sim(target_alt_float) # Imposta anche nel PID interno del simulatore
                self.data_logger.log_event("SET_TARGET_ALTITUDE_COMMAND", payload, "CONFIG") # Logga il cambio
            except ValueError: 
                print(f"Orchestrator: Invalid target altitude value in command: {new_target_alt}")
        else: 
            print("Orchestrator: Invalid set_target_altitude command payload:", payload)

    def _mqtt_cmd_reset_simulation(self, payload: Dict):
        """Handler per il comando MQTT 'reset_simulation'."""
        print("Orchestrator: MQTT CMD reset_simulation.")
        self.simulator.stop_simulation() # Ferma il simulatore
        current_fsm_target = self.fsm.target_altitude_m # Mantiene l'altitudine target corrente della FSM
        self.fsm = AltitudeGuardFSM(target_altitude=current_fsm_target) # Reiniziializza la FSM
        
        # Se il plotter è attivo, pulisci i suoi dati
        if self.live_plotter: 
            self.live_plotter.sim_time_data.clear()
            self.live_plotter.altitude_data.clear()
            self.live_plotter.target_altitude_data.clear()
            # self.live_plotter.fsm_event_annotations.clear() # Se si usassero annotazioni testuali
        
        time.sleep(0.1) # Breve pausa
        self.simulator.start_simulation() # Riavvia il simulatore (ripartirà da sim_time = 0)
        self.data_logger.log_event("SIMULATION_RESET_COMMAND", payload or {}, "CONTROL") # Logga il reset


def run_complete_iot_system_demo(enable_plot=True):
    """Funzione principale per avviare e gestire la demo completa del sistema IoT."""
    global orchestrator_instance # Usa una variabile globale per l'istanza dell'orchestratore
                                 # in modo che il loop principale possa accedervi per fermarla.
                                 # Non è la pratica migliore in assoluto, ma semplice per una demo script.

    if enable_plot: # Se il plot è abilitato
        orchestrator_instance = IoTSystemOrchestrator(enable_live_plot=True) # Crea istanza con plot
        try:
            orchestrator_instance.start_system() # Avvia il sistema (inclusa inizializzazione plotter)
            if orchestrator_instance.system_running:
                print_mqtt_info(orchestrator_instance) # Stampa informazioni MQTT
                
                # Loop principale per mantenere l'applicazione attiva e il grafico Matplotlib responsivo
                while orchestrator_instance.system_running:
                    # Stampa uno status update periodico basato sul tempo reale
                    if int(time.time()) % 30 == 0 and orchestrator_instance.system_running :
                        print_status_update(orchestrator_instance)
                    
                    plt.pause(0.1) # Pausa breve per permettere a Matplotlib di processare eventi GUI e aggiornare il plot
                    # Controlla se la finestra del grafico è stata chiusa dall'utente
                    if not plt.fignum_exists(orchestrator_instance.live_plotter.fig.number):
                        print("Plot window closed by user. Stopping system.")
                        orchestrator_instance.system_running = False # Segnala al sistema di fermarsi
                        break # Esce dal loop
                
        except KeyboardInterrupt: print("\nUser requested system shutdown (Ctrl+C).")
        except Exception as e: print(f"UNHANDLED EXCEPTION in demo: {e}"); import traceback; traceback.print_exc()
        finally: # Blocco eseguito sempre, per pulizia
            print("Initiating system shutdown sequence...")
            # Assicurati che stop_system venga chiamato solo se l'istanza e il sistema erano attivi
            if hasattr(orchestrator_instance, 'system_running') and orchestrator_instance.system_running:
                 orchestrator_instance.stop_system()
            elif hasattr(orchestrator_instance, 'stop_system'): # Se l'inizializzazione è avvenuta parzialmente
                 orchestrator_instance.stop_system()
            plt.close('all') # Chiude tutte le finestre Matplotlib rimanenti
            print("Demo finished.")
    else: # Esecuzione senza plot live
        orchestrator = IoTSystemOrchestrator(enable_live_plot=False) # Crea istanza senza plot
        try:
            orchestrator.start_system()
            if orchestrator.system_running:
                print_mqtt_info(orchestrator)
                while orchestrator.system_running: # Loop principale più semplice senza gestione plot
                     if int(time.time()) % 30 == 0 and orchestrator.system_running :
                        print_status_update(orchestrator)
                     time.sleep(1.0) # Pausa più lunga se non c'è plot da aggiornare
        except KeyboardInterrupt: print("\nUser requested system shutdown (Ctrl+C).")
        except Exception as e: print(f"UNHANDLED EXCEPTION in demo: {e}"); import traceback; traceback.print_exc()
        finally:
            print("Initiating system shutdown sequence...")
            if orchestrator.system_running: orchestrator.stop_system()
            print("Demo finished.")

def print_mqtt_info(orchestrator): 
    """Funzione helper per stampare informazioni sui topic MQTT."""
    print("\n" + "="*30 + " SYSTEM IS LIVE " + "="*30)
    print("Press Ctrl+C to stop the simulation.")
    print(f"MQTT Telemetry on: {orchestrator.mqtt_bridge.topics['telemetry']}")
    print(f"MQTT Drone Status on: {orchestrator.mqtt_bridge.topics['status']}")
    print(f"MQTT Alerts on: {orchestrator.mqtt_bridge.topics['alerts']}")
    print(f"Send commands to: {orchestrator.mqtt_bridge.topics['commands']}/<command_type>")
    print("   Example commands (publish as JSON payload):")
    print(f"   Topic: {orchestrator.mqtt_bridge.topics['commands']}/set_target_altitude")
    print(f"   Payload: {{\"target_altitude_m\": 0.5}}")
    print(f"   Topic: {orchestrator.mqtt_bridge.topics['commands']}/inject_anomaly")
    print(f"   Payload: {{\"type\": \"false_floor\", \"active\": true, \"duration\": 10}}")
    print("="*76 + "\n")

def print_status_update(orchestrator): 
    """Funzione helper per stampare un aggiornamento periodico dello stato del sistema."""
    sim_status = orchestrator.simulator.get_system_status() # Ottieni stato simulatore
    fsm_status = orchestrator.fsm.get_current_status()     # Ottieni stato FSM
    print(f"\n--- STATUS UPDATE [{time.strftime('%H:%M:%S')} | SimTime: {sim_status.get('sim_time_s',0):.1f}s] ---")
    print(f"  Sim Scenario: {sim_status['scenario']}, Sim LEDs: {sim_status['led_effect']}")
    print(f"  FSM State: {fsm_status['state']}, Alt: {fsm_status['altitude']:.2f}m (Target: {fsm_status['target']:.2f}m)")
    if fsm_status['alert_active']: print(f"  FSM ALERT ACTIVE! Dev: {fsm_status['deviation']:.2f}m")
    print(f"  MQTT Connected: {orchestrator.mqtt_bridge.connected}")
    print("--- END STATUS UPDATE ---")


if __name__ == "__main__":
    # Questo blocco viene eseguito quando lo script iot_system.py è lanciato direttamente
    run_complete_iot_system_demo(enable_plot=True) # Avvia la demo con il plot live abilitato
    # run_complete_iot_system_demo(enable_plot=False) # Opzione per avviare senza plot (per test console/MQTT)