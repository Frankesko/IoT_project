#!/usr/bin/env python3
"""
Test Integration Script for IoT-ROS2-Gazebo System
Tests the integration between existing IoT system and new ROS2/Gazebo extension
"""

import time
import json
import threading
import subprocess
import sys
import os
from typing import Dict, List, Optional
import paho.mqtt.client as mqtt
import numpy as np

class IntegrationTester:
    """Tester per l'integrazione IoT-ROS2-Gazebo"""
    
    def __init__(self):
        self.mqtt_client = mqtt.Client(client_id=f"integration_tester_{int(time.time())}")
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_message = self._on_mqtt_message
        
        # Stato del test
        self.test_results = {}
        self.mqtt_messages = []
        self.iot_telemetry_received = False
        self.gazebo_telemetry_received = False
        self.flip_command_sent = False
        self.flip_executed = False
        
        # Configurazione test
        self.test_timeout = 30  # secondi
        self.test_start_time = 0
        
    def _on_mqtt_connect(self, client, userdata, flags, rc):
        """Callback per connessione MQTT"""
        if rc == 0:
            print("✓ Connesso al broker MQTT")
            # Sottoscrizione ai topic di test
            topics = [
                "iot/telemetry",
                "gazebo/telemetry", 
                "crazyflie/alerts",
                "crazyflie/status"
            ]
            for topic in topics:
                client.subscribe(topic)
        else:
            print(f"✗ Errore connessione MQTT: {rc}")
    
    def _on_mqtt_message(self, client, userdata, msg):
        """Callback per messaggi MQTT"""
        try:
            payload = json.loads(msg.payload.decode('utf-8'))
            self.mqtt_messages.append({
                'topic': msg.topic,
                'payload': payload,
                'timestamp': time.time()
            })
            
            if msg.topic == "iot/telemetry":
                self.iot_telemetry_received = True
                print("✓ Telemetria IoT ricevuta")
            elif msg.topic == "gazebo/telemetry":
                self.gazebo_telemetry_received = True
                print("✓ Telemetria Gazebo ricevuta")
            elif msg.topic == "crazyflie/alerts":
                print(f"⚠ Alert ricevuto: {payload.get('alert_type', 'unknown')}")
                
        except json.JSONDecodeError:
            print(f"⚠ Messaggio MQTT non valido su {msg.topic}")
    
    def start_mqtt_client(self):
        """Avvia il client MQTT"""
        try:
            self.mqtt_client.connect("localhost", 1883, 60)
            self.mqtt_client.loop_start()
            return True
        except Exception as e:
            print(f"✗ Errore avvio MQTT: {e}")
            return False
    
    def stop_mqtt_client(self):
        """Ferma il client MQTT"""
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
    
    def test_iot_system(self) -> bool:
        """Test del sistema IoT esistente"""
        print("\n=== Test Sistema IoT ===")
        
        try:
            # Avvia il sistema IoT
            print("Avviando sistema IoT...")
            iot_process = subprocess.Popen([
                sys.executable, "iot_system.py"
            ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            
            # Attendi che si avvii
            time.sleep(5)
            
            # Verifica che sia in esecuzione
            if iot_process.poll() is None:
                print("✓ Sistema IoT avviato")
                
                # Attendi telemetria
                timeout = 10
                start_time = time.time()
                while not self.iot_telemetry_received and (time.time() - start_time) < timeout:
                    time.sleep(0.1)
                
                if self.iot_telemetry_received:
                    print("✓ Telemetria IoT funzionante")
                    self.test_results['iot_system'] = True
                else:
                    print("✗ Telemetria IoT non ricevuta")
                    self.test_results['iot_system'] = False
                
                # Termina il processo
                iot_process.terminate()
                iot_process.wait(timeout=5)
                
            else:
                print("✗ Sistema IoT non avviato")
                self.test_results['iot_system'] = False
                
        except Exception as e:
            print(f"✗ Errore test IoT: {e}")
            self.test_results['iot_system'] = False
        
        return self.test_results.get('iot_system', False)
    
    def test_ros2_gazebo_setup(self) -> bool:
        """Test del setup ROS2/Gazebo"""
        print("\n=== Test Setup ROS2/Gazebo ===")
        
        try:
            # Verifica ROS2
            result = subprocess.run([
                "ros2", "--version"
            ], capture_output=True, text=True)
            
            if result.returncode == 0:
                print("✓ ROS2 installato")
                self.test_results['ros2_installed'] = True
            else:
                print("✗ ROS2 non installato")
                self.test_results['ros2_installed'] = False
                return False
            
            # Verifica Gazebo
            result = subprocess.run([
                "gazebo", "--version"
            ], capture_output=True, text=True)
            
            if result.returncode == 0:
                print("✓ Gazebo installato")
                self.test_results['gazebo_installed'] = True
            else:
                print("✗ Gazebo non installato")
                self.test_results['gazebo_installed'] = False
                return False
            
            # Verifica workspace
            if os.path.exists("crazyflie_ws"):
                print("✓ Workspace ROS2 trovato")
                self.test_results['workspace_exists'] = True
            else:
                print("✗ Workspace ROS2 non trovato")
                self.test_results['workspace_exists'] = False
                return False
            
            return True
            
        except Exception as e:
            print(f"✗ Errore test setup: {e}")
            return False
    
    def test_maneuver_execution(self) -> bool:
        """Test dell'esecuzione di manovre"""
        print("\n=== Test Esecuzione Manovre ===")
        
        try:
            # Invia comando flip via MQTT
            flip_command = {
                "command_type": "execute_flip",
                "data": {
                    "direction": "forward"
                }
            }
            
            self.mqtt_client.publish(
                "crazyflie/commands",
                json.dumps(flip_command)
            )
            
            self.flip_command_sent = True
            print("✓ Comando flip inviato")
            
            # Attendi esecuzione
            timeout = 15
            start_time = time.time()
            while not self.flip_executed and (time.time() - start_time) < timeout:
                time.sleep(0.1)
                
                # Cerca messaggi di stato che indicano flip completato
                for msg in self.mqtt_messages:
                    if msg['topic'] == "crazyflie/status":
                        status_data = msg['payload']
                        if "flip" in str(status_data).lower():
                            self.flip_executed = True
                            break
            
            if self.flip_executed:
                print("✓ Flip eseguito con successo")
                self.test_results['maneuver_execution'] = True
            else:
                print("✗ Flip non eseguito")
                self.test_results['maneuver_execution'] = False
            
            return self.test_results.get('maneuver_execution', False)
            
        except Exception as e:
            print(f"✗ Errore test manovre: {e}")
            self.test_results['maneuver_execution'] = False
            return False
    
    def test_data_synchronization(self) -> bool:
        """Test della sincronizzazione dati"""
        print("\n=== Test Sincronizzazione Dati ===")
        
        try:
            # Verifica che entrambi i sistemi inviino telemetria
            timeout = 10
            start_time = time.time()
            
            while (not self.iot_telemetry_received or not self.gazebo_telemetry_received) and \
                  (time.time() - start_time) < timeout:
                time.sleep(0.1)
            
            if self.iot_telemetry_received and self.gazebo_telemetry_received:
                print("✓ Sincronizzazione dati funzionante")
                self.test_results['data_sync'] = True
                
                # Analizza i dati per verificare coerenza
                iot_data = None
                gazebo_data = None
                
                for msg in self.mqtt_messages:
                    if msg['topic'] == "iot/telemetry":
                        iot_data = msg['payload']
                    elif msg['topic'] == "gazebo/telemetry":
                        gazebo_data = msg['payload']
                
                if iot_data and gazebo_data:
                    print("✓ Dati IoT e Gazebo disponibili")
                    self.test_results['data_consistency'] = True
                else:
                    print("✗ Dati mancanti")
                    self.test_results['data_consistency'] = False
                    
            else:
                print("✗ Sincronizzazione dati fallita")
                self.test_results['data_sync'] = False
            
            return self.test_results.get('data_sync', False)
            
        except Exception as e:
            print(f"✗ Errore test sincronizzazione: {e}")
            self.test_results['data_sync'] = False
            return False
    
    def test_performance(self) -> bool:
        """Test delle performance"""
        print("\n=== Test Performance ===")
        
        try:
            # Misura latenza MQTT
            start_time = time.time()
            test_message = {"test": "performance", "timestamp": start_time}
            
            self.mqtt_client.publish(
                "test/performance",
                json.dumps(test_message)
            )
            
            # Attendi ricezione (se c'è un subscriber di test)
            time.sleep(0.1)
            
            latency = time.time() - start_time
            print(f"✓ Latenza MQTT: {latency*1000:.2f}ms")
            
            if latency < 0.1:  # Meno di 100ms
                self.test_results['mqtt_latency'] = True
            else:
                self.test_results['mqtt_latency'] = False
            
            # Verifica throughput
            message_count = len(self.mqtt_messages)
            test_duration = time.time() - self.test_start_time
            
            if test_duration > 0:
                throughput = message_count / test_duration
                print(f"✓ Throughput: {throughput:.1f} messaggi/sec")
                
                if throughput > 1.0:  # Almeno 1 messaggio/sec
                    self.test_results['throughput'] = True
                else:
                    self.test_results['throughput'] = False
            
            return True
            
        except Exception as e:
            print(f"✗ Errore test performance: {e}")
            return False
    
    def run_all_tests(self) -> Dict:
        """Esegue tutti i test"""
        print("🚀 Avvio Test Integrazione IoT-ROS2-Gazebo")
        print("=" * 50)
        
        self.test_start_time = time.time()
        
        # Avvia MQTT client
        if not self.start_mqtt_client():
            print("✗ Impossibile avviare MQTT client")
            return {"overall": False, "error": "MQTT connection failed"}
        
        try:
            # Esegui test
            tests = [
                ("IoT System", self.test_iot_system),
                ("ROS2/Gazebo Setup", self.test_ros2_gazebo_setup),
                ("Data Synchronization", self.test_data_synchronization),
                ("Maneuver Execution", self.test_maneuver_execution),
                ("Performance", self.test_performance)
            ]
            
            for test_name, test_func in tests:
                try:
                    result = test_func()
                    self.test_results[f"{test_name.lower().replace(' ', '_')}"] = result
                except Exception as e:
                    print(f"✗ Errore nel test {test_name}: {e}")
                    self.test_results[f"{test_name.lower().replace(' ', '_')}"] = False
            
            # Calcola risultato complessivo
            passed_tests = sum(1 for result in self.test_results.values() if result is True)
            total_tests = len(self.test_results)
            
            self.test_results['overall'] = passed_tests / total_tests > 0.7  # 70% success rate
            self.test_results['passed_tests'] = passed_tests
            self.test_results['total_tests'] = total_tests
            
        finally:
            self.stop_mqtt_client()
        
        return self.test_results
    
    def print_results(self):
        """Stampa i risultati dei test"""
        print("\n" + "=" * 50)
        print("📊 RISULTATI TEST INTEGRAZIONE")
        print("=" * 50)
        
        for test_name, result in self.test_results.items():
            if test_name in ['overall', 'passed_tests', 'total_tests']:
                continue
                
            status = "✓ PASS" if result else "✗ FAIL"
            print(f"{test_name.replace('_', ' ').title()}: {status}")
        
        print("-" * 50)
        passed = self.test_results.get('passed_tests', 0)
        total = self.test_results.get('total_tests', 0)
        overall = self.test_results.get('overall', False)
        
        print(f"Test Passati: {passed}/{total}")
        print(f"Risultato Complessivo: {'✓ PASS' if overall else '✗ FAIL'}")
        
        if overall:
            print("\n🎉 Integrazione funzionante!")
        else:
            print("\n⚠️  Problemi rilevati nell'integrazione")
        
        # Suggerimenti
        print("\n💡 Suggerimenti:")
        if not self.test_results.get('iot_system', True):
            print("- Verifica che il sistema IoT sia configurato correttamente")
        if not self.test_results.get('ros2_installed', True):
            print("- Installa ROS2 Humble")
        if not self.test_results.get('gazebo_installed', True):
            print("- Installa Gazebo")
        if not self.test_results.get('data_sync', True):
            print("- Verifica la connessione MQTT e il bridge ROS2")
        if not self.test_results.get('maneuver_execution', True):
            print("- Verifica che il controller ROS2 sia in esecuzione")

def main():
    """Funzione principale"""
    print("🧪 Test Integrazione IoT-ROS2-Gazebo")
    print("Questo script testa l'integrazione tra il sistema IoT esistente")
    print("e la nuova estensione ROS2/Gazebo per il controllo del drone.")
    print()
    
    # Verifica prerequisiti
    print("Verifica prerequisiti...")
    try:
        import paho.mqtt.client
        print("✓ paho-mqtt installato")
    except ImportError:
        print("✗ paho-mqtt non installato. Installa con: pip install paho-mqtt")
        return
    
    try:
        import numpy
        print("✓ numpy installato")
    except ImportError:
        print("✗ numpy non installato. Installa con: pip install numpy")
        return
    
    # Esegui test
    tester = IntegrationTester()
    results = tester.run_all_tests()
    
    # Stampa risultati
    tester.print_results()
    
    # Salva risultati
    with open("integration_test_results.json", "w") as f:
        json.dump(results, f, indent=2)
    
    print(f"\n📄 Risultati salvati in: integration_test_results.json")
    
    # Exit code
    sys.exit(0 if results.get('overall', False) else 1)

if __name__ == "__main__":
    main() 