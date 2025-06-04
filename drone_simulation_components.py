# drone_simulation_components.py
import time # Libreria per la gestione del tempo (es. sleep, timestamp reali)
import numpy as np # Libreria per calcoli numerici efficienti, specialmente array e funzioni matematiche
import threading # Libreria per creare ed eseguire codice in thread separati (es. simulatore)
import queue # Libreria per implementare code thread-safe (usata per i dati dei sensori)
import json # Libreria per codificare e decodificare dati in formato JSON (usato nel logger)
import sqlite3 # Libreria per interagire con database SQLite (usato nel logger)
import random # Libreria per generare numeri casuali (per rumore sensori, anomalie, ecc.)
from dataclasses import dataclass, asdict # Utilità per creare classi di dati semplici e convertirle in dizionari
from typing import Dict, List, Optional # Per type hinting, migliora la leggibilità e il controllo del codice
import math # Libreria per funzioni matematiche avanzate (es. seno, pi greco)

@dataclass # Decoratore che genera automaticamente metodi come __init__, __repr__, ecc.
class SensorReading:
    """Struttura dati (dataclass) per contenere una singola lettura aggregata dei sensori."""
    sim_time: float             # Tempo simulato dall'inizio della simulazione (in secondi)
    altitude_tof: float         # Altitudine misurata dal sensore Time-of-Flight (ToF) in metri
    altitude_barometer: float   # Altitudine misurata dal barometro in metri
    battery_voltage: float      # Voltaggio della batteria in Volt
    temperature: float          # Temperatura simulata in gradi Celsius
    vibration_x: float          # Vibrazione simulata sull'asse X (m/s^2)
    vibration_y: float          # Vibrazione simulata sull'asse Y (m/s^2)
    vibration_z: float          # Vibrazione simulata sull'asse Z (m/s^2, include gravità)
    signal_strength: int        # Intensità del segnale radio simulato (es. in dBm)
    packet_loss_rate: float     # Tasso di perdita pacchetti simulato (0.0 a 1.0)

@dataclass
class DronePhysics:
    """Struttura dati per memorizzare lo stato fisico del drone simulato."""
    position_z: float = 0.7         # Posizione verticale (altitudine) attuale in metri
    velocity_z: float = 0.0         # Velocità verticale attuale in m/s
    acceleration_z: float = 0.0     # Accelerazione verticale attuale in m/s^2
    mass: float = 0.027             # Massa del drone in kg (valore tipico per Crazyflie)
    drag_coefficient: float = 0.1   # Coefficiente di resistenza aerodinamica (adimensionale, semplificato)
    motor_thrust_force: float = 0.0 # Forza di spinta netta generata dai motori in Newton
    battery_level: float = 100.0    # Livello della batteria in percentuale (0-100%)
    temperature: float = 25.0       # Temperatura operativa del drone in gradi Celsius
    integral_error_z: float = 0.0   # Termine integrale accumulato per il controller PID interno del simulatore

class CrazyflieHardwareSimulator:
    """
    Simula l'hardware e la fisica di base di un drone Crazyflie 2.1,
    con focus sulla dinamica verticale, sensori e anomalie.
    """
    def __init__(self, initial_target_altitude=0.7, update_rate=50):
        """
        Inizializza il simulatore.
        Args:
            initial_target_altitude (float): Altitudine iniziale a cui il PID interno del simulatore mirerà.
            update_rate (int): Frequenza di aggiornamento della simulazione in Hz.
        """
        self.update_rate = update_rate      # Frequenza di aggiornamento del loop di simulazione
        self.dt = 1.0 / update_rate         # Intervallo di tempo per ogni step di simulazione (delta time)
        self.target_altitude = initial_target_altitude # Altitudine target per il PID interno del simulatore

        # Inizializza lo stato fisico del drone usando la dataclass DronePhysics
        self.physics = DronePhysics(position_z=initial_target_altitude)
        
        # Parametri dei sensori simulati
        self.tof_range = (0.02, 4.0)        # Range operativo del sensore ToF (min, max metri)
        self.barometer_noise = 0.01       # Deviazione standard del rumore del barometro (metri)
        self.tof_noise = 0.005            # Deviazione standard del rumore del ToF (metri)
        
        # Stato delle comunicazioni radio simulate
        self.radio_connected = True         # Flag che indica se la radio è connessa
        self.signal_strength = 90         # Intensità del segnale (valore arbitrario, es. dBm)
        self.packet_loss_rate = 0.01      # Tasso di perdita pacchetti (1%)
        
        # Dizionario per tracciare lo stato delle anomalie simulate
        self.anomalies = {
            'low_battery': False, 'sensor_drift_tof': False,
            'radio_interference': False, 'motor_vibration': False,
            'false_floor': False, 'sudden_drop': False, 'sudden_climb': False,
        }
        self.current_led_effect = "normal" # Stato corrente dei LED simulati
        
        # Coda thread-safe per memorizzare le letture dei sensori generate
        self.sensor_data_queue = queue.Queue(maxsize=1000)
        self.running = False                # Flag per controllare l'esecuzione del thread di simulazione
        self.simulation_thread = None       # Oggetto Thread per il loop di simulazione
        
        self.current_sim_time = 0.0         # Tempo corrente della simulazione, parte da 0
        self.current_scenario = "initializing" # Scenario attivo corrente
        self.previous_scenario = ""         # Scenario precedente, per rilevare cambi di scenario

        # Dizionario per tracciare se un impulso di uno scenario specifico è già stato applicato
        self._scenario_impulse_applied_flags = {}
        
    def start_simulation(self):
        """Avvia il thread di simulazione."""
        if self.running: return # Evita di avviare più volte
            
        self.running = True
        self.current_sim_time = 0.0 
        self.current_scenario = "stabilizing_at_boot" # Imposta lo scenario iniziale
        self.previous_scenario = "initializing"
        # Resetta lo stato fisico del drone all'altitudine target e l'errore integrale del PID
        self.physics = DronePhysics(position_z=self.target_altitude, integral_error_z=0.0) 
        self._scenario_impulse_applied_flags.clear() # Pulisce i flag degli impulsi precedenti
        
        # Crea e avvia il thread che eseguirà _simulation_loop
        self.simulation_thread = threading.Thread(target=self._simulation_loop)
        self.simulation_thread.daemon = True # Il thread terminerà quando il programma principale esce
        self.simulation_thread.start()
        print("Crazyflie Hardware Simulator Started (with relative sim_time)")
        
    def stop_simulation(self):
        """Ferma il thread di simulazione."""
        self.running = False # Segnala al loop di terminare
        if self.simulation_thread:
            self.simulation_thread.join(timeout=2.0) # Attende che il thread termini (con timeout)
        print("Crazyflie Hardware Simulator Stopped")
        
    def _simulation_loop(self):
        """Loop principale della simulazione, eseguito in un thread separato."""
        real_world_loop_start_time = time.time() # Timestamp reale per il controllo del rate
        while self.running: # Continua finché self.running è True
            self.current_sim_time += self.dt # Avanza il tempo simulato
            
            old_scenario = self.current_scenario # Salva lo scenario corrente prima di aggiornarlo
            self._update_scenario(self.current_sim_time) # Aggiorna lo scenario basato sul tempo simulato
            
            # Se lo scenario è cambiato, applica effetti una-tantum (es. impulsi)
            if self.current_scenario != old_scenario:
                self._apply_scenario_entry_impulses(self.current_scenario)
                # Resetta il flag per un nuovo scenario di tipo "sudden_drop/climb"
                if self.current_scenario in ["sudden_drop_sim", "sudden_climb_sim"]:
                    self._scenario_impulse_applied_flags[self.current_scenario] = False

            self._update_physics() # Aggiorna lo stato fisico del drone
            sensor_reading = self._read_sensors(self.current_sim_time) # Genera nuove letture dei sensori
            self._update_communications() # Aggiorna lo stato delle comunicazioni simulate
            
            if sensor_reading: # Se è stata generata una lettura valida
                try:
                    self.sensor_data_queue.put_nowait(sensor_reading) # Aggiungi alla coda senza bloccare
                except queue.Full: # Se la coda è piena
                    try: 
                        self.sensor_data_queue.get_nowait() # Rimuovi l'elemento più vecchio
                        self.sensor_data_queue.put_nowait(sensor_reading) # Riprova ad aggiungere
                    except queue.Empty: pass # Improbabile, ma gestito
            
            # Mantenimento del rate di simulazione basato sul tempo reale trascorso
            real_world_loop_end_time = time.time()
            processing_time = real_world_loop_end_time - real_world_loop_start_time
            sleep_duration = self.dt - processing_time # Tempo da attendere per mantenere il rate
            if sleep_duration > 0:
                time.sleep(sleep_duration)
            # else: # Se il ciclo ha impiegato più di dt, segnala un overrun (opzionale)
            #     print(f"Sim loop overrun by {abs(sleep_duration):.4f}s. Sim time: {self.current_sim_time:.2f}s")
            real_world_loop_start_time = time.time() # Registra il tempo di inizio del prossimo ciclo

    def _apply_scenario_entry_impulses(self, new_scenario: str):
        """Applica effetti una tantum (impulsi) quando si entra in certi scenari."""
        # Controlla se l'impulso per questo specifico `new_scenario` è già stato applicato
        impulse_already_applied = self._scenario_impulse_applied_flags.get(new_scenario, False)

        if new_scenario == "sudden_drop_sim" and not impulse_already_applied:
            self.physics.position_z -= 0.6  # Applica una caduta di posizione istantanea
            self.physics.velocity_z = -2.5  # Applica una velocità istantanea verso il basso
            self.physics.integral_error_z = 0 # Resetta il termine integrale del PID
            self._scenario_impulse_applied_flags[new_scenario] = True # Segna che l'impulso è stato applicato
            print(f"SIM DEBUG: Impulse for Sudden Drop applied at sim_time {self.current_sim_time:.2f}")
        
        elif new_scenario == "sudden_climb_sim" and not impulse_already_applied:
            self.physics.position_z += 0.6  # Applica una salita di posizione istantanea
            self.physics.velocity_z = 2.5   # Applica una velocità istantanea verso l'alto
            self.physics.integral_error_z = 0 # Resetta il termine integrale del PID
            self._scenario_impulse_applied_flags[new_scenario] = True # Segna che l'impulso è stato applicato
            print(f"SIM DEBUG: Impulse for Sudden Climb applied at sim_time {self.current_sim_time:.2f}")

    def _update_scenario(self, elapsed_sim_time_s: float):
        # Salva lo scenario corrente prima di determinarne uno nuovo
        self.previous_scenario = self.current_scenario 

        # Reset anomalie automatiche (quelle non manuali)
        auto_anomalies = ['sensor_drift_tof', 'false_floor', 'low_battery', 
                        'radio_interference', 'motor_vibration', 
                        'sudden_drop', 'sudden_climb']
        for key in auto_anomalies:
            # Non resettare se è un'anomalia manuale ancora attiva
            # (cioè, se è in self.anomalies e il suo timer non è scaduto)
            is_manual_active = hasattr(self, '_manual_anomaly_timers') and \
                            key in self._manual_anomaly_timers and \
                            self._manual_anomaly_timers[key]['active']
            if not is_manual_active:
                self.anomalies[key] = False
        
        # Logica per determinare lo scenario basato su elapsed_sim_time_s
        # e applicare anomalie. Non sovrascrivere se c'è uno scenario manuale.
        is_manual_scenario_active = any(self.current_scenario.startswith(f"manual_{anom}") for anom in self.anomalies if self.anomalies.get(anom, False))


        if not is_manual_scenario_active: # Se non c'è uno scenario manuale che forza lo stato
            # Timeline degli scenari automatici
            if 0 <= elapsed_sim_time_s < 5:
                 self.current_scenario = "stabilizing_at_boot"
            elif 10 < elapsed_sim_time_s <= 15:
                self.current_scenario = "tof_sensor_drift"; self.anomalies['sensor_drift_tof'] = True
            elif 15 < elapsed_sim_time_s <= 18: self.current_scenario = "post_drift_settling"
            elif 25 < elapsed_sim_time_s <= 30: 
                self.current_scenario = "false_floor_active"; self.anomalies['false_floor'] = True
            elif 30 < elapsed_sim_time_s <= 35: self.current_scenario = "post_false_floor_settling"
            elif 45 < elapsed_sim_time_s <= 55: 
                self.current_scenario = "low_battery_sim"; self.anomalies['low_battery'] = True
            elif 55 < elapsed_sim_time_s <= 58: self.current_scenario = "post_low_battery_settling"
            elif 60 < elapsed_sim_time_s <= 65:
                self.current_scenario = "radio_interference_active"; self.anomalies['radio_interference'] = True
            elif 65 < elapsed_sim_time_s <= 68: self.current_scenario = "post_radio_settling"
            elif 75 < elapsed_sim_time_s <= 80:
                self.current_scenario = "motor_vibration_active"; self.anomalies['motor_vibration'] = True
            elif 80 < elapsed_sim_time_s <= 83: self.current_scenario = "post_vibration_settling"
            elif 85 < elapsed_sim_time_s <= 89: # Durata 4 secondi per sudden_drop
                self.current_scenario = "sudden_drop_sim"; self.anomalies['sudden_drop'] = True
            elif 89 < elapsed_sim_time_s <= 93: self.current_scenario = "post_drop_settling"
            elif 95 < elapsed_sim_time_s <= 99: # Durata 4 secondi per sudden_climb
                self.current_scenario = "sudden_climb_sim"; self.anomalies['sudden_climb'] = True
            elif 99 < elapsed_sim_time_s <= 103: self.current_scenario = "post_climb_settling"
            elif elapsed_sim_time_s > 5: # Se nessun altro scenario è attivo e siamo oltre la fase di boot
                 self.current_scenario = "normal_operation_stable"
            # Se uno scenario manuale è appena terminato (indicato da current_scenario che inizia con "manual_" ma l'anomalia non è più attiva)
            elif self.previous_scenario.startswith("manual_") and not is_manual_scenario_active:
                 self.current_scenario = "normal_operation_stable" 

        # Assicura che se lo scenario è manuale, l'anomalia corrispondente sia flaggata come attiva
        for anom_type_iter in self.anomalies:
            if self.current_scenario == f"manual_{anom_type_iter}_active":
                self.anomalies[anom_type_iter] = True
                break # Trovato lo scenario manuale, non serve continuare

    def _update_physics(self):
        """Aggiorna lo stato fisico del drone (posizione, velocità, ecc.) basato sulle forze."""
        gravity_accel = -9.81 # Accelerazione di gravità (m/s^2)
        altitude_error = self.target_altitude - self.physics.position_z # Errore rispetto all'altitudine target
        
        # Guadagni del PID interno del simulatore (possono essere affinati)
        base_kp, base_ki, base_kd = 20.0, 1.5, 12.0 # Guadagni proporzionale, integrale, derivativo
        current_kp, current_ki, current_kd = base_kp, base_ki, base_kd # Inizializza con i guadagni base

        # Indebolisce temporaneamente il PID durante gli scenari "sudden_drop" o "sudden_climb"
        # per permettere alla deviazione indotta di persistere e testare l'alert FSM.
        if self.current_scenario == "sudden_drop_sim" or self.current_scenario == "sudden_climb_sim":
            current_kp *= 0.05  # Riduci KP
            current_ki *= 0.01  # Riduci KI
            current_kd *= 0.1   # Riduci KD
        
        # Logica Anti-Windup per il termine integrale del PID
        # Controlla se la spinta non è saturata (né al minimo né al massimo)
        thrust_not_saturated_low = self.physics.motor_thrust_force > 0.01 
        thrust_not_saturated_high = self.physics.motor_thrust_force < (self.physics.mass * abs(gravity_accel) * 1.99) # Un po' sotto il max
        thrust_not_saturated = thrust_not_saturated_low and thrust_not_saturated_high

        if thrust_not_saturated: # Se la spinta non è saturata, accumula l'errore integrale
            self.physics.integral_error_z += altitude_error * self.dt
            max_integral_val = 0.3 # Limite per il termine integrale per evitare un windup eccessivo
            self.physics.integral_error_z = np.clip(self.physics.integral_error_z, -max_integral_val, max_integral_val)
        # Se la spinta è saturata e l'integrale sta "spingendo" nella stessa direzione dell'errore, riducilo
        elif np.sign(altitude_error) == np.sign(self.physics.integral_error_z) and abs(self.physics.integral_error_z) > 0.01 :
            self.physics.integral_error_z *= 0.95 # Riduzione graduale dell'integrale

        # Calcolo dell'accelerazione correttiva del PID usando i guadagni correnti (potenzialmente modificati)
        pid_correction_accel = (current_kp * altitude_error) + \
                               (current_ki * self.physics.integral_error_z) - \
                               (current_kd * self.physics.velocity_z)
        
        # Calcola la forza di spinta necessaria per compensare la gravità e applicare la correzione PID
        thrust_force_calculated = self.physics.mass * (-gravity_accel + pid_correction_accel)
        self.physics.motor_thrust_force = thrust_force_calculated # Applica la spinta calcolata
        
        # Aggiungi disturbo da vibrazione del motore, se attivo
        if self.anomalies['motor_vibration']:
            self.physics.motor_thrust_force += random.gauss(0, self.physics.mass * 0.8) 

        # Limita la forza di spinta tra un minimo (0) e un massimo (es. 2 volte il peso)
        max_thrust = self.physics.mass * abs(gravity_accel) * 2.0 
        min_thrust = 0.0
        self.physics.motor_thrust_force = np.clip(self.physics.motor_thrust_force, min_thrust, max_thrust)

        # Calcola la forza di resistenza aerodinamica
        drag_force = -self.physics.drag_coefficient * self.physics.velocity_z * abs(self.physics.velocity_z)
        # Calcola la forza netta agente sul drone
        net_force = self.physics.motor_thrust_force + (self.physics.mass * gravity_accel) + drag_force
        # Calcola l'accelerazione risultante
        self.physics.acceleration_z = net_force / self.physics.mass
        
        # Integra l'accelerazione per ottenere la nuova velocità e posizione (metodo di Eulero)
        self.physics.velocity_z += self.physics.acceleration_z * self.dt
        self.physics.position_z += self.physics.velocity_z * self.dt
            
        # Vincolo del terreno: il drone non può andare sotto una certa altitudine minima
        if self.physics.position_z < 0.01:
            self.physics.position_z = 0.01 # Imposta all'altitudine minima
            if self.physics.velocity_z < 0: self.physics.velocity_z = 0 # Ferma la caduta
            self.physics.integral_error_z = 0 # Resetta l'errore integrale se tocca terra
        
        # Modello di scaricamento della batteria
        # Lo scaricamento dipende dallo sforzo normalizzato dei motori
        normalized_thrust_effort = self.physics.motor_thrust_force / (self.physics.mass * abs(gravity_accel)) if (self.physics.mass * abs(gravity_accel)) > 0 else 0
        discharge_rate_percent_per_sec = (0.05 + abs(normalized_thrust_effort -1.0) * 0.2) # Percentuale al secondo
        self.physics.battery_level -= discharge_rate_percent_per_sec * self.dt # Riduci il livello batteria
        self.physics.battery_level = max(0, self.physics.battery_level) # Non può scendere sotto 0
        
        # Modello di temperatura semplificato
        ambient_temp = 22.0 # Temperatura ambiente
        heat_from_motors = abs(normalized_thrust_effort) * 2.0 # Il calore generato dipende dallo sforzo
        cooling_rate_factor = 0.05 # Fattore di raffreddamento
        temp_change = (heat_from_motors - cooling_rate_factor * (self.physics.temperature - ambient_temp)) * self.dt
        self.physics.temperature += temp_change # Aggiorna la temperatura
        self.physics.temperature = max(ambient_temp - 5, min(self.physics.temperature, 70)) # Limita la temperatura

    def _read_sensors(self, current_sim_time_s: float) -> Optional[SensorReading]:
        """Genera letture simulate dei sensori basate sullo stato fisico e le anomalie."""
        true_altitude = self.physics.position_z # Altitudine reale dal modello fisico
        # Lettura ToF: altitudine reale + rumore gaussiano
        tof_reading = true_altitude + random.gauss(0, self.tof_noise)
        
        # Se l'anomalia 'false_floor' è attiva, simula un oggetto sotto il drone
        if self.anomalies['false_floor']:
            simulated_object_distance_from_drone = 0.10 # L'oggetto è a 10cm dal drone
            # La lettura ToF sarà la minima tra la distanza reale dal pavimento e la distanza dall'oggetto
            tof_reading = min(tof_reading, simulated_object_distance_from_drone + random.gauss(0, self.tof_noise))
            
        # Se l'anomalia 'sensor_drift_tof' è attiva, aggiungi un drift sinusoidale
        if self.anomalies['sensor_drift_tof']:
            drift_frequency_hz = 0.2  # Frequenza del drift (es. periodo di 5 secondi)
            drift_amplitude_m = 0.03  # Ampiezza del drift (3 cm)
            drift = drift_amplitude_m * math.sin(2 * math.pi * drift_frequency_hz * current_sim_time_s)
            tof_reading += drift
            
        # Limita la lettura ToF entro il range operativo del sensore
        tof_reading = np.clip(tof_reading, self.tof_range[0], self.tof_range[1])
        
        # Lettura Barometro: altitudine reale + rumore gaussiano + piccolo effetto della velocità verticale
        baro_reading = true_altitude + random.gauss(0, self.barometer_noise)
        baro_reading -= self.physics.velocity_z * 0.005 # Effetto propwash/velocità
        
        # Voltaggio Batteria: calcolato linearmente dal livello percentuale
        battery_v_full, battery_v_empty = 4.2, 3.0 # Voltaggi tipici LiPo
        battery_voltage = battery_v_empty + (self.physics.battery_level / 100.0) * (battery_v_full - battery_v_empty)
        # Se l'anomalia 'low_battery' è attiva o il livello è critico, forza un voltaggio basso
        if self.anomalies['low_battery'] or self.physics.battery_level < 5:
            battery_voltage = battery_v_empty + 0.1 
            
        # Vibrazioni: rumore gaussiano, aumentato se 'motor_vibration' è attiva
        vibration_base_std_dev = 0.1 # Deviazione standard base
        if self.anomalies['motor_vibration']: vibration_base_std_dev = 0.8 # Aumentata per l'anomalia
            
        vib_x, vib_y = random.gauss(0, vibration_base_std_dev), random.gauss(0, vibration_base_std_dev)
        vib_z = random.gauss(9.81, vibration_base_std_dev) # Vibrazione su Z centrata attorno a g
        
        # Crea e restituisce l'oggetto SensorReading con tutti i valori simulati
        return SensorReading(
            sim_time=current_sim_time_s, 
            altitude_tof=tof_reading, altitude_barometer=baro_reading,
            battery_voltage=battery_voltage, temperature=self.physics.temperature,
            vibration_x=vib_x, vibration_y=vib_y, vibration_z=vib_z,
            signal_strength=self.signal_strength, packet_loss_rate=self.packet_loss_rate
        )
    
    def _update_communications(self):
        """Simula lo stato delle comunicazioni radio."""
        # Se l'anomalia 'radio_interference' è attiva, degrada segnale e aumenta perdita pacchetti
        if self.anomalies['radio_interference']:
            self.signal_strength = random.randint(30, 60) # Segnale debole
            self.packet_loss_rate = random.uniform(0.05, 0.25) # Alta perdita
        else: # Condizioni normali
            self.signal_strength = random.randint(80, 95) # Segnale forte
            self.packet_loss_rate = random.uniform(0.001, 0.02) # Bassa perdita
        
        # Simula disconnessioni/riconnessioni occasionali
        if not self.radio_connected and random.random() < 0.2: # 20% probabilità di riconnettersi se disconnesso
             self._reconnect_radio()
        elif self.radio_connected and random.random() < 0.0005: # 0.05% probabilità di disconnettersi se connesso
            self.radio_connected = False
            if self.running: print("Radio disconnected (simulated)") # Stampa solo se la simulazione è attiva
    
    def _reconnect_radio(self):
        """Simula la riconnessione della radio."""
        if not self.running: return # Non fare nulla se la simulazione è fermata
        self.radio_connected = True
        if self.running: print("Radio reconnected (simulated)")

    def get_latest_reading(self) -> Optional[SensorReading]:
        """Restituisce l'ultima lettura dei sensori dalla coda (non bloccante con timeout)."""
        if not self.running and self.sensor_data_queue.empty(): return None # Se fermo e vuoto, nulla da dare
        try: return self.sensor_data_queue.get(timeout=0.05) # Prova a prendere un elemento con un breve timeout
        except queue.Empty: return None # Se la coda è vuota dopo il timeout, restituisci None
    
    def get_system_status(self) -> Dict:
        """Restituisce un dizionario con lo stato completo del sistema simulato."""
        return {
            'sim_time_s': self.current_sim_time, 
            'scenario': self.current_scenario,
            'physics': asdict(self.physics), # Converte la dataclass DronePhysics in dizionario
            'communications': {
                'connected': self.radio_connected,
                'signal_strength': self.signal_strength,
                'packet_loss': self.packet_loss_rate
            },
            'led_effect': self.current_led_effect,
            'anomalies_active': {k:v for k,v in self.anomalies.items() if v}, # Dizionario delle sole anomalie attive
            'timestamp_real': time.time() # Timestamp reale per riferimento
        }
    
    def inject_anomaly(self, anomaly_type: str, active: bool = True, duration: Optional[float] = None):
        """
        Attiva o disattiva manualmente un'anomalia, opzionalmente per una durata specifica.
        Args:
            anomaly_type (str): Nome dell'anomalia (deve essere una chiave in self.anomalies).
            active (bool): True per attivare, False per disattivare.
            duration (Optional[float]): Se fornito e active=True, l'anomalia si disattiva dopo questa durata in secondi.
        """
        if anomaly_type not in self.anomalies:
            print(f"Unknown anomaly type: {anomaly_type}")
            return

        # Inizializza il dizionario dei timer per le anomalie manuali se non esiste
        if not hasattr(self, '_manual_anomaly_timers'):
            self._manual_anomaly_timers = {}

        status_msg = "activated" if active else "cleared"
        print(f"Manual Anomaly: {anomaly_type} {status_msg}.")
        
        self.anomalies[anomaly_type] = active # Imposta lo stato dell'anomalia
        if active:
            self.current_scenario = f"manual_{anomaly_type}_active" # Sovrascrive lo scenario corrente con uno manuale
            if duration: # Se è stata specificata una durata
                # Cancella un eventuale timer precedente per la stessa anomalia
                if anomaly_type in self._manual_anomaly_timers and self._manual_anomaly_timers[anomaly_type]['timer']:
                    self._manual_anomaly_timers[anomaly_type]['timer'].cancel()
                
                # Funzione che verrà eseguita allo scadere del timer
                def clear_manual_anomaly():
                    print(f"Auto-cleared manual anomaly: {anomaly_type} after {duration}s")
                    self.anomalies[anomaly_type] = False # Disattiva l'anomalia
                    # Se lo scenario era ancora quello manuale, imposta uno scenario di "settling"
                    if self.current_scenario == f"manual_{anomaly_type}_active":
                        self.current_scenario = "manual_anomaly_ended_settling" 
                    # Aggiorna lo stato del timer nel dizionario
                    if anomaly_type in self._manual_anomaly_timers:
                         self._manual_anomaly_timers[anomaly_type]['active'] = False
                         self._manual_anomaly_timers[anomaly_type]['timer'] = None

                timer = threading.Timer(duration, clear_manual_anomaly) # Crea un nuovo timer
                timer.start() # Avvia il timer
                self._manual_anomaly_timers[anomaly_type] = {'active': True, 'timer': timer} # Memorizza info sul timer
            else: # Anomalia manuale persistente (nessuna durata)
                self._manual_anomaly_timers[anomaly_type] = {'active': True, 'timer': None}
        else: # Se active è False (disattivazione manuale)
            # Cancella un eventuale timer attivo
            if anomaly_type in self._manual_anomaly_timers and self._manual_anomaly_timers[anomaly_type]['timer']:
                self._manual_anomaly_timers[anomaly_type]['timer'].cancel()
            # Se lo scenario era quello manuale, imposta uno scenario di "settling"
            if self.current_scenario == f"manual_{anomaly_type}_active":
                self.current_scenario = "manual_anomaly_cleared_settling"
            # Aggiorna lo stato del timer/anomalia nel dizionario
            if anomaly_type in self._manual_anomaly_timers:
                self._manual_anomaly_timers[anomaly_type]['active'] = False
                self._manual_anomaly_timers[anomaly_type]['timer'] = None

    def set_led_effect(self, effect_name: str):
        """Imposta lo stato simulato dei LED."""
        self.current_led_effect = effect_name

    def set_target_altitude_sim(self, new_target_alt: float):
        """Imposta una nuova altitudine target per il PID interno del simulatore."""
        print(f"Simulator: Target altitude for internal PID set to {new_target_alt}m")
        self.target_altitude = new_target_alt
        self.physics.integral_error_z = 0 # Resetta il termine integrale del PID al cambio di target


class DataLogger:
    """Gestisce il logging dei dati su un database SQLite."""
    def __init__(self, db_path="crazyflie_simulation_data.db"):
        """Inizializza il logger e il database."""
        self.db_path = db_path
        self.conn = None # Connessione al DB
        self.cursor = None # Cursore del DB
        self._connect_db() # Stabilisce la connessione
        self._init_database() # Crea le tabelle se non esistono
    
    def _connect_db(self):
        """Stabilisce la connessione al database SQLite."""
        # `check_same_thread=False` è necessario se si accede al DB da thread diversi (non ideale per scritture multiple, ma ok per questo logger)
        self.conn = sqlite3.connect(self.db_path, check_same_thread=False)
        self.cursor = self.conn.cursor()

    def _init_database(self):
        """Crea le tabelle nel database se non esistono già."""
        # Tabella per le letture dei sensori
        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS sensor_readings (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                sim_time REAL UNIQUE, altitude_tof REAL, altitude_barometer REAL, 
                battery_voltage REAL, temperature REAL, vibration_x REAL, 
                vibration_y REAL, vibration_z REAL, signal_strength INTEGER, 
                packet_loss_rate REAL, active_scenario TEXT, current_led_effect TEXT
            )
        ''') # `sim_time REAL UNIQUE` assicura che non ci siano timestamp duplicati
        # Tabella per gli eventi di sistema
        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS system_events (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                real_timestamp REAL, event_type TEXT, event_data TEXT, severity TEXT
            )
        ''')
        self.conn.commit() # Salva le modifiche al database
        print(f"Database initialized/connected: {self.db_path}")
    
    def log_sensor_reading(self, reading: SensorReading, scenario: str, led_effect: str):
        """Logga una nuova lettura dei sensori nel database."""
        if not self.conn: self._connect_db() # Riconnetti se la connessione è stata chiusa
        try:
            # Inserisce una nuova riga nella tabella sensor_readings.
            # `INSERT OR IGNORE` ignora l'inserimento se una riga con lo stesso `sim_time` (UNIQUE) esiste già.
            self.cursor.execute('''
                INSERT OR IGNORE INTO sensor_readings 
                (sim_time, altitude_tof, altitude_barometer, battery_voltage, 
                 temperature, vibration_x, vibration_y, vibration_z, 
                 signal_strength, packet_loss_rate, active_scenario, current_led_effect)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            ''', (
                reading.sim_time, reading.altitude_tof, reading.altitude_barometer,
                reading.battery_voltage, reading.temperature, reading.vibration_x,
                reading.vibration_y, reading.vibration_z, reading.signal_strength,
                reading.packet_loss_rate, scenario, led_effect
            ))
            self.conn.commit() # Salva la modifica
        except sqlite3.Error as e:
            # Stampa un errore se il logging fallisce (tranne per UNIQUE constraint che è gestito da IGNORE)
            # print(f"DB log_sensor_reading error: {e} for sim_time {reading.sim_time}")
            pass 
    
    def log_event(self, event_type: str, event_data: Dict, severity: str = "INFO"):
        """Logga un evento di sistema (es. alert FSM, comando manuale)."""
        if not self.conn: self._connect_db()
        try:
            # Inserisce un nuovo evento nella tabella system_events.
            # `json.dumps(event_data, default=str)` converte il dizionario event_data in una stringa JSON;
            # `default=str` aiuta a serializzare tipi non standard come oggetti numpy.
            self.cursor.execute('''
                INSERT INTO system_events (real_timestamp, event_type, event_data, severity)
                VALUES (?, ?, ?, ?)
            ''', (time.time(), event_type, json.dumps(event_data, default=str), severity)) 
            self.conn.commit()
        except sqlite3.Error as e: print(f"DB log_event error: {e}")

    def get_recent_data(self, minutes: int = 5, table_name: str = "sensor_readings", time_col: str = "sim_time") -> List[Dict]:
        """Recupera dati recenti dal database per analisi o dashboard."""
        if not self.conn: self._connect_db()
        
        # Stima il numero di record da recuperare basato sui minuti e sul rate di aggiornamento
        # (Usato per sensor_readings per evitare di caricare l'intero DB se molto grande)
        limit_n = minutes * 60 * self.update_rate if hasattr(self, 'update_rate') and hasattr(self, 'update_rate') else minutes * 60 * 50 
        
        if table_name == "system_events": # Per gli eventi, usa un cutoff basato sul tempo reale
            cutoff_time = time.time() - (minutes * 60)
            query = f'SELECT * FROM {table_name} WHERE {time_col} > ? ORDER BY {time_col} DESC'
            params = (cutoff_time,)
        else: # Per sensor_readings, prendi gli ultimi N record ordinati per sim_time
            query = f'SELECT * FROM {table_name} ORDER BY {time_col} DESC LIMIT ?'
            params = (limit_n,)

        try:
            self.cursor.execute(query, params)
            columns = [desc[0] for desc in self.cursor.description] # Ottieni i nomi delle colonne
            # Converte i risultati della query in una lista di dizionari e inverte l'ordine per avere i dati più vecchi prima
            return [dict(zip(columns, row)) for row in self.cursor.fetchall()][::-1] 
        except sqlite3.Error as e:
            print(f"DB get_recent_data error: {e}")
            return []

    def close_db(self):
        """Chiude la connessione al database."""
        if self.conn: 
            self.conn.close()
            self.conn = None
            self.cursor = None
            # print(f"Database connection closed: {self.db_path}")


if __name__ == "__main__":
    # Questo blocco viene eseguito solo se lo script drone_simulation_components.py è lanciato direttamente.
    # È utile per testare isolatamente il simulatore e il logger.
    def run_simulation_components_demo_updated():
        print("Starting Updated Simulation Components Demo (Simulator & Logger)")
        print("=" * 60)
        
        sim_update_rate = 10 # Rate di aggiornamento più basso per la demo per facilitare l'osservazione
        simulator = CrazyflieHardwareSimulator(update_rate=sim_update_rate) 
        logger = DataLogger(db_path="components_demo_updated.db") # Usa un DB separato per questa demo
        
        simulator.start_simulation() # Avvia il thread del simulatore
        
        try:
            demo_duration_s = 110 # Durata della fase automatica della demo
            print(f"Simulating for {demo_duration_s} seconds with various scenarios...")
            
            # Loop per consumare dati dalla coda del simulatore e loggarli
            for i in range(demo_duration_s * sim_update_rate):  
                reading = simulator.get_latest_reading() # Prende una lettura dalla coda (può essere None)
                
                if reading: # Se c'è una lettura valida
                    status = simulator.get_system_status() # Ottiene lo stato attuale del simulatore
                    logger.log_sensor_reading(reading, status['scenario'], status['led_effect']) # Logga i dati
                    
                    # Stampa uno status update ogni 2 secondi di tempo simulato
                    if i % (2 * sim_update_rate) == 0: 
                        print(f"\n--- SimTime: {reading.sim_time:.1f}s (RealTime: {status['timestamp_real']:.1f}s) ---")
                        print(f"Scenario: {status['scenario']}")
                        print(f"  ToF Alt: {reading.altitude_tof:.3f}m, Physics Alt: {status['physics']['position_z']:.3f}m")
                        print(f"  Battery: {reading.battery_voltage:.2f}V ({status['physics']['battery_level']:.1f}%)")
                        print(f"  LEDs: {status['led_effect']}")
                        if status['anomalies_active']:
                             print(f"  Active Anomalies: {list(status['anomalies_active'].keys())}")
                
                # Piccolo sleep per evitare di ciclare troppo velocemente e consumare CPU inutilmente
                # Dato che il simulatore ha il suo rate, qui è solo per il loop di consumo
                time.sleep(1.0 / sim_update_rate / 2) 

            # Dopo la simulazione automatica, inietta un'anomalia manualmente
            print("\nInjecting 'false_floor' manually for 5s...")
            simulator.inject_anomaly('false_floor', active=True, duration=5.0)
            # Lascia girare la simulazione per un altro po' per osservare l'anomalia manuale
            for _ in range(7 * sim_update_rate): # Per altri 7 secondi simulati
                reading = simulator.get_latest_reading()
                if reading:
                    status = simulator.get_system_status()
                    logger.log_sensor_reading(reading, status['scenario'], status['led_effect'])
                    print(f"Manual Anomaly Phase - SimTime: {reading.sim_time:.1f}s, Scenario: {status['scenario']}, LED: {status['led_effect']}")
                time.sleep(1.0 / sim_update_rate / 2)

        except KeyboardInterrupt: # Se l'utente interrompe con Ctrl+C
            print("\nSimulation demo stopped by user")
        finally: # Blocco eseguito sempre, sia in caso di successo che di errore/interruzione
            simulator.stop_simulation() # Ferma il simulatore
            
            # Recupera e stampa un riassunto dei dati loggati
            recent_logs = logger.get_recent_data(minutes=int(demo_duration_s/60) + 2, table_name="sensor_readings", time_col="sim_time")
            print(f"\nLogged {len(recent_logs)} sensor readings.")
            if recent_logs:
                print("First logged entry sample (sim_time):", recent_logs[0]['sim_time'] if recent_logs else "N/A")
                print("Last logged entry sample (sim_time):", recent_logs[-1]['sim_time'] if recent_logs else "N/A")
            
            logger.close_db() # Chiude la connessione al database
            
        print("\n" + "=" * 60)
        print("UPDATED SIMULATION COMPONENTS DEMO COMPLETE")
        print("=" * 60)

    run_simulation_components_demo_updated() # Esegue la funzione demo se lo script è lanciato direttamente