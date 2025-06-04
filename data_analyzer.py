# data_analyzer.py
import sqlite3 # Libreria per interagire con database SQLite
import pandas as pd # Libreria per la manipolazione e analisi di dati tabulari (DataFrame)
import numpy as np # Libreria per calcoli numerici, specialmente array
import matplotlib.pyplot as plt # Libreria per creare grafici e visualizzazioni
from datetime import datetime # Libreria per gestire date e ore (non usata attivamente qui ma buona pratica)

# --- Configurazione ---
# Percorsi ai file di dati e parametri chiave del sistema/FSM
DB_PATH = "iot_system_flight_data_liveplot.db" # Path al database SQLite creato da iot_system.py
FSM_CSV_PATH = "final_fsm_log_liveplot.csv"    # Path al log CSV creato dalla FSM
TARGET_ALTITUDE = 0.7  # metri (altitudine target del drone)
STABLE_THRESHOLD_CM = 2.0 # cm (soglia di stabilità per HOLD_STABLE)
ALERT_DEVIATION_CM = 10.0 # cm (soglia di deviazione per ALERT)
ALERT_DURATION_SEC = 2.0 # secondi (durata della deviazione per triggerare ALERT)

# Conversione delle soglie in metri per coerenza con i dati
STABLE_THRESHOLD_M = STABLE_THRESHOLD_CM / 100.0
ALERT_DEVIATION_M = ALERT_DEVIATION_CM / 100.0

# Nomi degli stati FSM (devono corrispondere esattamente a quelli usati nel codice della FSM)
STATE_HOLD_STABLE = "HOLD_STABLE"
STATE_ADJUST_UP = "ADJUST_UP"
STATE_ADJUST_DOWN = "ADJUST_DOWN"
STATE_ALERT = "ALERT"


def load_sensor_data_from_db(db_path):
    """Carica i dati dei sensori dal database SQLite in un DataFrame pandas."""
    try:
        conn = sqlite3.connect(db_path) # Stabilisce una connessione al database SQLite
        # Query SQL per selezionare le colonne rilevanti dalla tabella sensor_readings, ordinate per sim_time
        query = "SELECT sim_time, altitude_tof, active_scenario FROM sensor_readings ORDER BY sim_time"
        df = pd.read_sql_query(query, conn) # Esegue la query e carica i risultati in un DataFrame pandas
        conn.close() # Chiude la connessione al database
        print(f"Caricati {len(df)} record di sensori dal DB.")
        return df # Restituisce il DataFrame
    except Exception as e:
        print(f"Errore nel caricamento dei dati dei sensori dal DB: {e}")
        return pd.DataFrame() # Restituisce un DataFrame vuoto in caso di errore

def load_fsm_log_from_csv(csv_path):
    """Carica il log FSM dal file CSV in un DataFrame pandas."""
    try:
        df_fsm = pd.read_csv(csv_path) # Legge il file CSV in un DataFrame pandas
        # Rinomina le colonne per coerenza interna allo script di analisi, se necessario.
        # 'errors='ignore'' evita errori se le colonne non esistono.
        df_fsm.rename(columns={'timestamp': 'sim_time', 'state': 'fsm_state'}, inplace=True, errors='ignore')
        print(f"Caricati {len(df_fsm)} record di log FSM dal CSV.")
        return df_fsm
    except Exception as e:
        print(f"Errore nel caricamento del log FSM dal CSV: {e}")
        return pd.DataFrame()

def analyze_time_in_states(df_fsm):
    """Analizza il tempo trascorso in ogni stato FSM."""
    # Controlla se ci sono dati sufficienti per l'analisi
    if df_fsm.empty or 'fsm_state' not in df_fsm.columns or 'sim_time' not in df_fsm.columns:
        print("Dati FSM insufficienti per l'analisi degli stati.")
        return None

    print("\n--- Analisi Tempo negli Stati FSM ---")
    
    # Calcolo accurato della durata di ogni stato basato sulla differenza di tempo tra righe consecutive.
    # Si assume che ogni riga nel log FSM rappresenti l'inizio di un nuovo stato (o un campione periodico).
    state_durations = []
    for i in range(len(df_fsm) - 1): # Itera su tutte le righe tranne l'ultima
        duration = df_fsm['sim_time'].iloc[i+1] - df_fsm['sim_time'].iloc[i] # Durata = tempo_attuale - tempo_precedente
        state = df_fsm['fsm_state'].iloc[i] # Stato attivo durante questa durata
        state_durations.append({'fsm_state': state, 'duration': duration})
    
    # Aggiunge la durata dell'ultimo stato.
    # Questa è una stima, poiché non c'è una riga successiva per calcolare la differenza.
    # Si usa la durata media dei campioni precedenti o un piccolo intervallo fisso.
    if len(df_fsm) > 0:
        avg_duration_sample = df_fsm['sim_time'].diff().mean() if len(df_fsm) > 1 else 0.1 # Calcola la differenza media tra i sim_time
        state_durations.append({
            'fsm_state': df_fsm['fsm_state'].iloc[-1], # Prende l'ultimo stato registrato
            'duration': avg_duration_sample 
        })

    df_state_durations = pd.DataFrame(state_durations) # Converte la lista di dizionari in DataFrame
    
    if not df_state_durations.empty:
        # Raggruppa per stato FSM e somma le durate per ottenere il tempo totale in ogni stato
        total_time_by_state = df_state_durations.groupby('fsm_state')['duration'].sum()
        total_sim_time = total_time_by_state.sum() # Tempo di simulazione totale basato sui log FSM
        
        if total_sim_time > 0:
            # Calcola la percentuale di tempo trascorsa in ogni stato
            percentage_time_by_state = (total_time_by_state / total_sim_time) * 100
            print("Tempo totale per stato FSM:")
            print(total_time_by_state)
            print("\nPercentuale di tempo per stato FSM:")
            print(percentage_time_by_state)

            # Crea un grafico a barre della distribuzione percentuale del tempo negli stati
            percentage_time_by_state.plot(kind='bar', figsize=(10, 6))
            plt.title('Percentuale di Tempo per Stato FSM')
            plt.ylabel('Percentuale (%)')
            plt.xlabel('Stato FSM')
            plt.xticks(rotation=45) # Ruota le etichette dell'asse x per leggibilità
            plt.tight_layout() # Aggiusta il layout per evitare sovrapposizioni
            plt.savefig("fsm_state_distribution.png") # Salva il grafico come immagine
            print("\nGrafico distribuzione stati FSM salvato come fsm_state_distribution.png")
            plt.close() # Chiude la figura per liberare memoria
            return percentage_time_by_state
        else:
            print("Tempo di simulazione totale è zero, impossibile calcolare percentuali.")
            return None
    else:
        print("Nessuna durata di stato calcolata.")
        return None


def analyze_altitude_stability(df_sensor_data, df_fsm_data):
    """Analizza la stabilità dell'altitudine, specialmente in HOLD_STABLE."""
    if df_sensor_data.empty or df_fsm_data.empty:
        print("Dati insufficienti per l'analisi della stabilità dell'altitudine.")
        return

    print("\n--- Analisi Stabilità Altitudine ---")
    
    # Ordina entrambi i DataFrame per 'sim_time' per un corretto merging
    df_fsm_data = df_fsm_data.sort_values(by='sim_time')
    df_sensor_data = df_sensor_data.sort_values(by='sim_time')
    
    # Unisce i dati dei sensori con gli stati FSM. Per ogni lettura del sensore,
    # trova l'ultimo stato FSM registrato fino a quel momento (o esattamente a quel momento).
    # 'direction='backward'' assicura che si prenda lo stato FSM più recente non successivo al sim_time del sensore.
    df_merged = pd.merge_asof(df_sensor_data, 
                              df_fsm_data[['sim_time', 'fsm_state', 'target_altitude_m']], # Seleziona solo colonne utili dal log FSM
                              on='sim_time', 
                              direction='backward')
    
    # Filtra i dati per quando la FSM era nello stato HOLD_STABLE
    # .copy() è usato per evitare SettingWithCopyWarning quando si aggiungono nuove colonne
    df_hold_stable = df_merged[df_merged['fsm_state'] == STATE_HOLD_STABLE].copy()

    if df_hold_stable.empty:
        print("Nessun dato trovato per lo stato HOLD_STABLE.")
        return

    # Calcola la deviazione dell'altitudine ToF rispetto all'altitudine target
    df_hold_stable['deviation_m'] = df_hold_stable['altitude_tof'] - df_hold_stable['target_altitude_m']
    
    # Calcola statistiche sulla stabilità
    mean_altitude_stable = df_hold_stable['altitude_tof'].mean() # Media dell'altitudine
    std_altitude_stable = df_hold_stable['altitude_tof'].std()   # Deviazione standard dell'altitudine
    max_error_stable_abs = df_hold_stable['deviation_m'].abs().max() # Massimo errore assoluto dalla target
    
    print(f"Nello stato HOLD_STABLE ({len(df_hold_stable)} campioni):")
    print(f"  Altitudine Media: {mean_altitude_stable:.4f} m")
    print(f"  Deviazione Standard Altitudine: {std_altitude_stable:.4f} m")
    print(f"  Errore Massimo Assoluto dalla Target Altitude: {max_error_stable_abs:.4f} m ({max_error_stable_abs*100:.2f} cm)")

    # Verifica la percentuale di tempo in cui la deviazione è entro la soglia di stabilità (es. +/- 2cm)
    within_stable_threshold = df_hold_stable['deviation_m'].abs() <= STABLE_THRESHOLD_M
    percentage_within_threshold = (within_stable_threshold.sum() / len(df_hold_stable)) * 100 if len(df_hold_stable) > 0 else 0
    print(f"  Percentuale di tempo in HOLD_STABLE entro ±{STABLE_THRESHOLD_CM}cm: {percentage_within_threshold:.2f}%")

    # Crea un grafico della deviazione dell'altitudine (in cm) nel tempo durante HOLD_STABLE
    plt.figure(figsize=(12, 6))
    plt.plot(df_hold_stable['sim_time'], df_hold_stable['deviation_m'] * 100, label='Deviazione Altitudine (cm)')
    plt.axhline(STABLE_THRESHOLD_CM, color='r', linestyle='--', label=f'+{STABLE_THRESHOLD_CM}cm Soglia') # Linea soglia superiore
    plt.axhline(-STABLE_THRESHOLD_CM, color='r', linestyle='--', label=f'-{STABLE_THRESHOLD_CM}cm Soglia')# Linea soglia inferiore
    plt.title('Deviazione Altitudine in Stato HOLD_STABLE')
    plt.xlabel('Tempo Simulato (s)')
    plt.ylabel('Deviazione (cm)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig("altitude_deviation_hold_stable.png")
    print("Grafico deviazione altitudine in HOLD_STABLE salvato come altitude_deviation_hold_stable.png")
    plt.close()


def analyze_transitions_and_alerts(df_fsm):
    """Analizza le transizioni di stato e gli alert."""
    if df_fsm.empty or 'fsm_state' not in df_fsm.columns:
        print("Dati FSM insufficienti per l'analisi delle transizioni.")
        return
        
    print("\n--- Analisi Transizioni e Alert FSM ---")
    
    # Conta il numero di transizioni tra stati diversi
    transitions = {}
    for i in range(len(df_fsm) - 1): # Itera su tutte le righe tranne l'ultima
        prev_state = df_fsm['fsm_state'].iloc[i] # Stato nella riga corrente
        curr_state = df_fsm['fsm_state'].iloc[i+1] # Stato nella riga successiva
        if prev_state != curr_state: # Se c'è stata una transizione
            transition_key = f"{prev_state} -> {curr_state}" # Crea una chiave per la transizione
            transitions[transition_key] = transitions.get(transition_key, 0) + 1 # Incrementa il contatore per quella transizione
            
    print("Numero di transizioni tra stati:")
    for trans, count in transitions.items():
        print(f"  {trans}: {count}")

    # Conta il numero totale di volte che si è entrati nello stato ALERT
    # Somma le transizioni da qualsiasi stato non-ALERT allo stato ALERT
    num_alerts = transitions.get(f"{STATE_ADJUST_UP} -> {STATE_ALERT}", 0) + \
                 transitions.get(f"{STATE_ADJUST_DOWN} -> {STATE_ALERT}", 0) + \
                 transitions.get(f"{STATE_HOLD_STABLE} -> {STATE_ALERT}", 0) 
    print(f"Numero totale di attivazioni ALERT: {num_alerts}")

    # Calcola la durata media dello stato ALERT
    if STATE_ALERT in df_fsm['fsm_state'].values: # Controlla se lo stato ALERT è mai apparso
        # Riutilizza la logica di calcolo della durata per stato da analyze_time_in_states
        state_durations = []
        for i in range(len(df_fsm) - 1):
            duration = df_fsm['sim_time'].iloc[i+1] - df_fsm['sim_time'].iloc[i]
            state = df_fsm['fsm_state'].iloc[i]
            state_durations.append({'fsm_state': state, 'duration': duration})
        
        df_state_durations_alerts = pd.DataFrame(state_durations)
        # Somma tutte le durate in cui lo stato era ALERT
        total_alert_duration = df_state_durations_alerts[df_state_durations_alerts['fsm_state'] == STATE_ALERT]['duration'].sum()

        if num_alerts > 0:
            avg_alert_duration = total_alert_duration / num_alerts # Calcola la media
            print(f"Durata totale in stato ALERT: {total_alert_duration:.2f} s")
            print(f"Durata media per ALERT: {avg_alert_duration:.2f} s")
        else:
            print("Nessun ALERT registrato per calcolare la durata media.")
    else:
        print("Stato ALERT non presente nei log.")

def analyze_scenario_performance(df_sensor_data, df_fsm_data):
    """Analizza come la FSM si comporta durante scenari specifici del simulatore."""
    # Controlla se ci sono dati sufficienti, in particolare la colonna 'active_scenario'
    if df_sensor_data.empty or 'active_scenario' not in df_sensor_data.columns or df_fsm_data.empty:
        print("Dati insufficienti (manca 'active_scenario' o dati FSM) per l'analisi degli scenari.")
        return

    print("\n--- Analisi Performance per Scenario ---")
    # Unisce i dati dei sensori (che contengono 'active_scenario') con gli stati FSM
    df_merged = pd.merge_asof(df_sensor_data.sort_values('sim_time'), 
                              df_fsm_data[['sim_time', 'fsm_state']].sort_values('sim_time'), 
                              on='sim_time', direction='backward')

    # Raggruppa i dati per ogni scenario attivo e analizza la distribuzione degli stati FSM
    for scenario_name, group_df in df_merged.groupby('active_scenario'):
        print(f"\nScenario: {scenario_name}")
        if group_df.empty:
            print("  Nessun dato per questo scenario.")
            continue

        # Calcola la percentuale di campioni (e quindi approssimativamente di tempo) trascorsi in ogni stato FSM durante quello scenario
        state_counts = group_df['fsm_state'].value_counts(normalize=True) * 100
        print(f"  Distribuzione stati FSM (% campioni): \n{state_counts.to_string()}")
        
        # Evidenzia se lo stato ALERT è stato attivo durante lo scenario
        if STATE_ALERT in state_counts.index and state_counts[STATE_ALERT] > 0:
            print(f"  ATTENZIONE: Stato ALERT presente durante '{scenario_name}' ({state_counts[STATE_ALERT]:.1f}% dei campioni)")
        

def main_analysis():
    """Funzione principale per eseguire tutte le analisi."""
    print(f"Inizio analisi dati da DB: {DB_PATH} e CSV: {FSM_CSV_PATH}")
    
    df_sensor = load_sensor_data_from_db(DB_PATH) # Carica dati sensori
    df_fsm = load_fsm_log_from_csv(FSM_CSV_PATH)   # Carica log FSM

    if df_fsm.empty and df_sensor.empty: # Se non ci sono dati, interrompe
        print("Nessun dato caricato. Analisi interrotta.")
        return

    # Analisi #1: Tempo negli stati
    analyze_time_in_states(df_fsm)

    # Analisi #2: Stabilità Altitudine
    # Aggiunge la colonna 'target_altitude_m' al DataFrame FSM se non presente, usando il valore di default.
    # Questo è necessario se il CSV log della FSM non include il target altitude per ogni entry.
    if 'target_altitude_m' not in df_fsm.columns:
        df_fsm['target_altitude_m'] = TARGET_ALTITUDE 
    analyze_altitude_stability(df_sensor, df_fsm)
    
    # Analisi #3: Transizioni e Alert
    analyze_transitions_and_alerts(df_fsm)

    # Analisi #4: Performance per Scenario
    analyze_scenario_performance(df_sensor, df_fsm)

    print("\n--- Analisi Completata ---")

if __name__ == "__main__":
    # Questo blocco viene eseguito solo se lo script data_analyzer.py è lanciato direttamente
    main_analysis()