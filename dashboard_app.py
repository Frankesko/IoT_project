# dashboard_app.py
import dash
from dash import dcc, html, Input, Output, State
import plotly.graph_objects as go
from collections import deque
import paho.mqtt.client as mqtt
import json
import time
import threading
import pandas as pd 

# --- Configurazione Globale ---
MAX_DATA_POINTS = 100 
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
TELEMETRY_TOPIC = "crazyflie/drone_01/telemetry"
STATUS_TOPIC = "crazyflie/drone_01/status"
ALERTS_TOPIC = "crazyflie/drone_01/alerts"
COMMAND_TOPIC_BASE = "crazyflie/drone_01/commands"

# Deque per i dati
altitude_timeseries = deque(maxlen=MAX_DATA_POINTS) 
battery_timeseries = deque(maxlen=MAX_DATA_POINTS)

# Variabili globali per lo stato
latest_fsm_state = "N/A"
latest_scenario = "N/A"
latest_sim_time = 0.0
latest_health_score = 100.0
latest_led_effect = "N/A"
recent_alerts = deque(maxlen=10) 

data_lock = threading.Lock()

# --- Connessione MQTT e Gestione Messaggi ---
def on_connect(client, userdata, flags, rc, properties=None):
    print(f"DASH: Connected to MQTT Broker with result code {rc}")
    client.subscribe(TELEMETRY_TOPIC)
    client.subscribe(STATUS_TOPIC)
    client.subscribe(ALERTS_TOPIC)
    print(f"DASH: Subscribed to {TELEMETRY_TOPIC}, {STATUS_TOPIC}, {ALERTS_TOPIC}")

def on_message(client, userdata, msg):
    global latest_fsm_state, latest_scenario, latest_sim_time, latest_health_score, latest_led_effect
    
    try:
        payload = json.loads(msg.payload.decode())
        with data_lock:
            if msg.topic == TELEMETRY_TOPIC:
                sim_time = payload.get('sim_time_s', 0)
                alt_tof = payload.get('altitude_tof_m', 0)
                fsm_info = payload.get('fsm', {})
                target_alt = fsm_info.get('target', 0.7)
                
                altitude_timeseries.append((sim_time, alt_tof, target_alt))
                
                bat_v = payload.get('battery_v', 0)
                battery_timeseries.append((sim_time, bat_v))

                latest_fsm_state = fsm_info.get('state', 'N/A')
                latest_sim_time = sim_time

            elif msg.topic == STATUS_TOPIC:
                latest_scenario = payload.get('scenario', 'N/A')
                latest_health_score = payload.get('health_score', 100.0)
                latest_led_effect = payload.get('led_effect', 'N/A')

            elif msg.topic == ALERTS_TOPIC:
                alert_msg = f"[{time.strftime('%H:%M:%S', time.localtime(payload.get('real_time_s')))}] "
                alert_msg += f"SimT: {payload.get('sim_time_at_alert_s', 0):.1f}s - {payload.get('severity', 'WARN')}: "
                alert_msg += f"{payload.get('alert_type', 'Unknown')} - Alt: {payload.get('details', {}).get('current_altitude_m', '?')}m"
                recent_alerts.append(alert_msg)

    except Exception as e:
        print(f"DASH: Error processing MQTT message: {e}")

mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message

def mqtt_thread_function():
    mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
    mqtt_client.loop_forever() 

mqtt_thread = threading.Thread(target=mqtt_thread_function)
mqtt_thread.daemon = True
mqtt_thread.start()

# --- Creazione dell'App Dash ---
app = dash.Dash(__name__, external_stylesheets=['https://codepen.io/chriddyp/pen/bWLwgP.css'])
app.title = "Crazyflie Altitude Guard Dashboard"

# --- Layout della Dashboard ---
app.layout = html.Div([
    html.H1("Crazyflie Table-Top Altitude Guard - Live Dashboard", style={'textAlign': 'center'}),
    
    dcc.Interval(
        id='interval-component-live',
        interval=500, 
        n_intervals=0
    ),

    # Riga per Status e Comandi (SENZA INIEZIONE ANOMALIE)
    html.Div(className='row', children=[
        html.Div(className='six columns', children=[ # Aumentato a six per più spazio
            html.H3("System Status"),
            html.Div([html.Strong("FSM State: "), html.Span(id='fsm-state-output', children=latest_fsm_state)]),
            html.Div([html.Strong("Current Scenario: "), html.Span(id='scenario-output', children=latest_scenario)]),
            html.Div([html.Strong("Simulated LED: "), html.Span(id='led-output', children=latest_led_effect)]),
            html.Div([html.Strong("Health Score: "), html.Span(id='health-score-output', children=f"{latest_health_score:.1f}%")]),
            html.Div([html.Strong("Last Sim Time: "), html.Span(id='sim-time-output', children=f"{latest_sim_time:.1f}s")]),
        ]),
        html.Div(className='six columns', children=[ # Aumentato a six per più spazio
            html.H3("Manual Controls"),
            html.Div([
                dcc.Input(id='target-altitude-input', type='number', placeholder='New Target Alt (m)', value=0.7, step=0.1, style={'marginRight': '10px'}),
                html.Button('Set Target Altitude', id='set-target-button', n_clicks=0)
            ], style={'marginBottom': '10px'}),
            html.Button('Reset Simulation', id='reset-simulation-button', n_clicks=0, style={'marginTop': '10px'}),
            html.Div(id='command-status-output', children="", style={'marginTop': '10px'}) 
        ]),
    ]),

    # Riga per i Grafici
    html.Div(className='row', children=[
        html.Div(className='six columns', children=[
            dcc.Graph(id='live-altitude-graph')
        ]),
        html.Div(className='six columns', children=[
            dcc.Graph(id='live-battery-graph')
        ]),
    ]),

    # Riga per Log Alert
    html.Div(className='row', children=[
        html.Div(className='twelve columns', children=[
            html.H3("Recent Alerts Log"),
            html.Pre(id='alerts-log-output', children="No alerts yet.", 
                     style={'height': '200px', 'overflowY': 'scroll', 'border': '1px solid #ccc', 'padding': '10px'})
        ])
    ]),
])

# --- Callback per Aggiornare i Grafici e i Dati Live ---
@app.callback(
    [Output('live-altitude-graph', 'figure'),
     Output('live-battery-graph', 'figure'),
     Output('fsm-state-output', 'children'),
     Output('scenario-output', 'children'),
     Output('led-output', 'children'),
     Output('health-score-output', 'children'),
     Output('sim-time-output', 'children'),
     Output('alerts-log-output', 'children')],
    [Input('interval-component-live', 'n_intervals')]
)
def update_live_data(n):
    with data_lock: 
        times_alt, alts, targets = [], [], []
        if altitude_timeseries:
            times_alt, alts, targets = zip(*altitude_timeseries)

        times_bat, bat_vs = [], []
        if battery_timeseries:
            times_bat, bat_vs = zip(*battery_timeseries)

        fig_alt = go.Figure()
        fig_alt.add_trace(go.Scatter(x=list(times_alt), y=list(alts), mode='lines', name='Altitude ToF'))
        fig_alt.add_trace(go.Scatter(x=list(times_alt), y=list(targets), mode='lines', name='Target Altitude', line=dict(dash='dash')))
        fig_alt.update_layout(title='Altitude (m) vs. Sim Time (s)', xaxis_title='Sim Time (s)', yaxis_title='Altitude (m)', margin=dict(t=50, b=50, l=50, r=30)) # Margini più piccoli
        fig_alt.update_yaxes(range=[min(alts)-0.2 if alts else 0, max(alts)+0.2 if alts else 1.5]) # Auto-range Y asse altitudine

        fig_bat = go.Figure()
        fig_bat.add_trace(go.Scatter(x=list(times_bat), y=list(bat_vs), mode='lines', name='Battery Voltage'))
        fig_bat.update_layout(title='Battery Voltage (V) vs. Sim Time (s)', xaxis_title='Sim Time (s)', yaxis_title='Voltage (V)', margin=dict(t=50, b=50, l=50, r=30))
        fig_bat.update_yaxes(range=[3.0, 4.3]) 

        alerts_text = "\n".join(reversed(list(recent_alerts))) 

    return fig_alt, fig_bat, latest_fsm_state, latest_scenario, latest_led_effect, f"{latest_health_score:.1f}%", f"{latest_sim_time:.1f}s", alerts_text

# --- Callback per Inviare Comandi MQTT ---
@app.callback(
    Output('command-status-output', 'children'),
    [Input('set-target-button', 'n_clicks'),
     Input('reset-simulation-button', 'n_clicks')], # Rimosso input da inject/clear anomaly
    [State('target-altitude-input', 'value')], # Rimosso state da anomaly-type e duration
    prevent_initial_call=True 
)
def handle_commands(n_set_target, n_reset, target_alt): # Rimosso argomenti per anomaly
    ctx = dash.callback_context
    if not ctx.triggered:
        return "No command."
    
    button_id = ctx.triggered[0]['prop_id'].split('.')[0]
    msg = ""

    if button_id == 'set-target-button' and target_alt is not None:
        try:
            payload = {'target_altitude_m': float(target_alt)}
            mqtt_client.publish(f"{COMMAND_TOPIC_BASE}/set_target_altitude", json.dumps(payload))
            msg = f"Set target altitude to {target_alt}m command sent."
        except ValueError:
            msg = "Invalid target altitude value."
    elif button_id == 'reset-simulation-button':
        mqtt_client.publish(f"{COMMAND_TOPIC_BASE}/reset_simulation", json.dumps({}))
        msg = "Reset simulation command sent."
    
    print(f"DASH CMD: {msg}")
    return msg

# --- Esecuzione dell'App ---
if __name__ == '__main__':
    print("DASH: Starting Dash app...")
    app.run(debug=True, port=8050)