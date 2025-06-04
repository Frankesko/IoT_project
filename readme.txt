Project 9: Crazyflie 2.1 (STEM Ranging Bundle) — "Table-Top Altitude Guard"

This project models and simulates a "Table-Top Altitude Guard" system for the Crazyfile 2.1 drone. The system aims to maintain a target altitude using a simulated Time-of-Flight (ToF) sensor, detect significant vertcal deviation, and trigger alerts based on a Finite State Machine (FSM).


Core features:
	Altitude control simulation: maintains the target altitude.
	Finite State Mahcine: Mangaes stated 'HOLD_STABLE', 'ADJUST_UP', 'ADJUST_DOWN', 'ALERT'.
	Falut detection and response: detects significant deviation (>10cm for more than 2 seconds) and specific fault scenarios like "false floor", "sensor drift" and "sudded drop/climb", triggering alerts.
	MQTT integration: Publishes telemetry/status/alerts.
	Live visualization and logging: matplotlib live plot, SQLite database logging and CSV FSM trace.
	Data analysis: a script to analyze simulation logs and generate performance metrics.


Prerequisites:
	python 3.7+
	pip
	MQTT Broker:
		a local or networked MQTT broker (I used Mosquitto).
	    	default configuration (localhost:1883).

Setup and installation:
	clone the repository
	install python dependencies (numpy, pandas, matplotlib, paho-mqtt)
	install MQTT broker

Running the simulation:
	run iot_system.py file
	(optional) run dashboard_app for the live dashboard (status, altitude, alerts)

Analyzing simulation data
	once the simulation is finished (just close the matlab live plotter window) you have to run the data_analyzer.py file

