# altitude_fsm.py
import time # Libreria per funzionalità legate al tempo (usata indirettamente per i timestamp)
import csv # Libreria per leggere e scrivere file CSV (Comma Separated Values)
import random # Libreria per generare numeri casuali (usata nel blocco di test)

class AltitudeGuardFSM:
    """
    Finite State Machine (FSM) per monitorare l'altitudine di un drone
    e gestire transizioni di stato basate su deviazioni dal target.
    """
    # Definizione costante degli stati della FSM per evitare errori di battitura e migliorare la leggibilità
    STATE_HOLD_STABLE = "HOLD_STABLE"
    STATE_ADJUST_UP = "ADJUST_UP"
    STATE_ADJUST_DOWN = "ADJUST_DOWN"
    STATE_ALERT = "ALERT"

    def __init__(self, target_altitude=0.7, stable_threshold_cm=2.0,
                 alert_deviation_cm=10.0, alert_duration_sec=2.0):
        """
        Inizializza la FSM con i parametri di configurazione.
        Args:
            target_altitude (float): Altitudine target desiderata in metri.
            stable_threshold_cm (float): Soglia di stabilità in centimetri per lo stato HOLD_STABLE.
            alert_deviation_cm (float): Deviazione in centimetri che, se persistente, attiva l'ALERT.
            alert_duration_sec (float): Durata in secondi per cui una deviazione significativa deve persistere per l'ALERT.
        """
        self.target_altitude_m = target_altitude # Altitudine target in metri
        self.stable_threshold_m = stable_threshold_cm / 100.0 # Soglia di stabilità convertita in metri
        self.alert_deviation_m = alert_deviation_cm / 100.0   # Soglia di deviazione per alert convertita in metri
        self.alert_duration_sec = alert_duration_sec         # Durata per attivare l'alert

        self.current_state = self.STATE_HOLD_STABLE # Stato iniziale della FSM
        self.last_altitude_m = target_altitude      # Ultima altitudine registrata, inizializzata al target
        self.last_deviation_m = 0.0                 # Ultima deviazione calcolata
        self.alert_newly_triggered = False          # Flag per indicare se un alert è appena stato attivato

        self._significant_deviation_start_time = None # Timestamp di quando una deviazione significativa è iniziata
        self._last_update_timestamp_sec = 0.0         # Timestamp dell'ultimo aggiornamento (tempo simulato)

        self.log_history = [] # Lista per memorizzare la cronologia degli stati per il logging CSV

        # Stampa i parametri di inizializzazione della FSM
        print(f"FSM Initialized: Target={self.target_altitude_m}m, "
              f"Stable Threshold={self.stable_threshold_m}m (+/-{stable_threshold_cm}cm), "
              f"Alert Threshold={self.alert_deviation_m}m (+/-{alert_deviation_cm}cm) for {self.alert_duration_sec}s")

    def set_target_altitude(self, new_target_m: float):
        """
        Permette di cambiare dinamicamente l'altitudine target.
        Args:
            new_target_m (float): La nuova altitudine target in metri.
        """
        print(f"FSM: Target altitude changed from {self.target_altitude_m}m to {new_target_m}m")
        self.target_altitude_m = new_target_m
        # Se non siamo in ALERT, forza un ricalcolo dello stato con il nuovo target
        if self.current_state != self.STATE_ALERT:
            # Chiama update_altitude con l'ultima altitudine nota e l'ultimo timestamp
            # per rivalutare lo stato basato sul nuovo target.
            self.update_altitude(self.last_altitude_m, self._last_update_timestamp_sec)

    def update_altitude(self, current_altitude_m: float, current_sim_timestamp_sec: float):
        """
        Aggiorna lo stato della FSM basandosi sulla nuova lettura di altitudine e il timestamp.
        Args:
            current_altitude_m (float): Altitudine corrente misurata in metri.
            current_sim_timestamp_sec (float): Timestamp simulato corrente in secondi.
        Returns:
            str: Lo stato corrente della FSM dopo l'aggiornamento.
        """
        self.last_altitude_m = current_altitude_m # Memorizza l'altitudine corrente
        self.last_deviation_m = current_altitude_m - self.target_altitude_m # Calcola la deviazione dal target
        previous_state = self.current_state # Salva lo stato precedente per rilevare transizioni
        self.alert_newly_triggered = False # Resetta il flag di nuovo alert

        # Logga l'entrata nel metodo, prima che lo stato possa cambiare
        self._log_entry(current_sim_timestamp_sec, current_altitude_m, self.last_deviation_m, previous_state, 
                        self._significant_deviation_start_time is not None, # Se il timer di alert è attivo
                        (current_sim_timestamp_sec - self._significant_deviation_start_time) if self._significant_deviation_start_time else 0) # Tempo trascorso in deviazione significativa

        # Gestione dello stato ALERT (ha la priorità più alta)
        if abs(self.last_deviation_m) > self.alert_deviation_m: # Se la deviazione supera la soglia di alert
            if self._significant_deviation_start_time is None: # Se è la prima volta che la deviazione è significativa
                self._significant_deviation_start_time = current_sim_timestamp_sec # Avvia il timer
            # Se la deviazione significativa persiste per la durata richiesta
            elif (current_sim_timestamp_sec - self._significant_deviation_start_time) >= self.alert_duration_sec:
                if self.current_state != self.STATE_ALERT: # Se non eravamo già in ALERT
                    self.alert_newly_triggered = True # Imposta il flag di nuovo alert
                self.current_state = self.STATE_ALERT # Transizione allo stato ALERT
        else: # Se la deviazione è rientrata sotto la soglia di alert
            if self.current_state == self.STATE_ALERT: # Se eravamo in stato ALERT
                print(f"[{current_sim_timestamp_sec:.2f}s] FSM: Exiting ALERT state as deviation is now {self.last_deviation_m:.3f}m")
                # Ricalcola lo stato appropriato ora che non siamo più in alert grave
                if abs(self.last_deviation_m) <= self.stable_threshold_m:
                    self.current_state = self.STATE_HOLD_STABLE
                elif self.last_deviation_m > self.stable_threshold_m:
                    self.current_state = self.STATE_ADJUST_UP
                else: # self.last_deviation_m < -self.stable_threshold_m
                    self.current_state = self.STATE_ADJUST_DOWN
            # In ogni caso (anche se non eravamo in ALERT ma il timer era partito), resetta il timer di deviazione significativa
            self._significant_deviation_start_time = None


        # Gestione degli altri stati (ADJUST_UP, ADJUST_DOWN, HOLD_STABLE) solo se non siamo in ALERT
        if self.current_state != self.STATE_ALERT:
            if abs(self.last_deviation_m) <= self.stable_threshold_m: # Deviazione entro la soglia di stabilità
                self.current_state = self.STATE_HOLD_STABLE
            elif self.last_deviation_m > self.stable_threshold_m: # Drone troppo basso (deviazione positiva > soglia stabile)
                self.current_state = self.STATE_ADJUST_UP
            elif self.last_deviation_m < -self.stable_threshold_m: # Drone troppo alto (deviazione negativa < -soglia stabile)
                self.current_state = self.STATE_ADJUST_DOWN

        # Se c'è stata una transizione di stato, stampala e gestisci il flag di nuovo alert
        if self.current_state != previous_state:
            print(f"[{current_sim_timestamp_sec:.2f}s] FSM State: {previous_state} -> {self.current_state} "
                  f"(Alt: {current_altitude_m:.3f}m, Dev: {self.last_deviation_m:.3f}m)")
            if self.alert_newly_triggered: # Questo è già stato impostato sopra se si entra in ALERT
                print(f"[{current_sim_timestamp_sec:.2f}s] FSM: ALERT NEWLY TRIGGERED!")

        self._last_update_timestamp_sec = current_sim_timestamp_sec # Aggiorna il timestamp dell'ultimo update
        return self.current_state # Restituisce lo stato corrente aggiornato

    def get_current_status(self):
        """
        Restituisce un dizionario con lo stato corrente della FSM e dati rilevanti.
        """
        time_in_deviation_for_alert = 0.0
        # Calcola da quanto tempo la deviazione è significativa (se il timer è attivo)
        if self._significant_deviation_start_time and self._last_update_timestamp_sec:
             time_in_deviation_for_alert = self._last_update_timestamp_sec - self._significant_deviation_start_time

        return {
            'state': self.current_state,
            'altitude': self.last_altitude_m,
            'target': self.target_altitude_m,
            'deviation': self.last_deviation_m,
            'alert_active': (self.current_state == self.STATE_ALERT), # True se siamo in stato ALERT
            'alert_newly_triggered': self.alert_newly_triggered, # True se l'alert è appena scattato
            'alert_timer_active': self._significant_deviation_start_time is not None, # True se il timer per l'alert è attivo
            'time_in_deviation_for_alert': time_in_deviation_for_alert # Tempo trascorso da quando la deviazione è significativa
        }

    def _log_entry(self, timestamp, altitude, deviation, state, alert_timer_active, time_in_dev):
        """
        Aggiunge una entry alla cronologia di log interna.
        Questa cronologia viene poi usata per salvare il file CSV.
        """
        self.log_history.append({
            'timestamp': timestamp,
            'altitude_m': altitude,
            'deviation_m': deviation,
            'state': state, # Stato FSM *prima* dell'attuale valutazione in update_altitude
            'alert_timer_active': alert_timer_active,
            'time_in_deviation_for_alert': time_in_dev
        })

    def save_log_to_csv(self, filename="fsm_flight_log.csv"):
        """
        Salva la cronologia degli stati FSM in un file CSV.
        Args:
            filename (str): Nome del file CSV da creare.
        Returns:
            str or None: Il nome del file se salvato con successo, None altrimenti.
        """
        if not self.log_history: # Se non ci sono dati da loggare
            print("FSM log is empty. Nothing to save.")
            return None
        
        keys = self.log_history[0].keys() # Prende gli header dalla prima entry (assumendo che tutte abbiano le stesse chiavi)

        # Apre il file in modalità scrittura ('w')
        with open(filename, 'w', newline='') as output_file:
            dict_writer = csv.DictWriter(output_file, fieldnames=keys) # Crea un writer per dizionari
            dict_writer.writeheader() # Scrive la riga di intestazione (le chiavi)
            dict_writer.writerows(self.log_history) # Scrive tutte le righe di dati
        print(f"FSM log saved to {filename}")
        return filename
