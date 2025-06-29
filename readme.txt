Project 9: Crazyflie 2.1 (STEM Ranging Bundle) — "Table-Top Altitude Guard"

This project models and simulates a "Table-Top Altitude Guard" system for the Crazyfile 2.1 drone. The system aims to maintain a target altitude using a simulated Time-of-Flight (ToF) sensor, detect significant vertcal deviation, and trigger alerts based on a Finite State Machine (FSM).


Core features:
	Altitude control simulation: maintains the target altitude.
	Finite State Mahcine: Mangaes stated 'HOLD_STABLE', 'ADJUST_UP', 'ADJUST_DOWN', 'ALERT'.
	Falut detection and response: detects significant deviation (>10cm for more than 2 seconds) and specific fault scenarios like "false floor", "sensor drift" and "sudded drop/climb", triggering alerts.
	MQTT integration: Publishes telemetry/status/alerts.
	Live visualization and logging: matplotlib live plot, SQLite database logging and CSV FSM trace.
	Data analysis: a script to analyze simulation logs and generate performance metrics.

NEW: ROS2/Gazebo Extension
	Advanced rotor-level control: Direct control of individual drone rotors.
	Complex maneuvers: Flip, spiral, figure-8, and custom trajectories.
	Gazebo simulation: Realistic physics simulation with ROS2 integration.
	IoT-ROS2 bridge: Seamless integration between existing IoT system and ROS2/Gazebo.
	Real-time control: 50Hz control loop with advanced PID controllers.


Prerequisites:
	python 3.7+
	pip
	MQTT Broker:
		a local or networked MQTT broker (I used Mosquitto).
	    	default configuration (localhost:1883).

For ROS2/Gazebo Extension:
	ROS2 Humble (Ubuntu 22.04+)
	Gazebo
	Python 3.8+
	Git

Setup and installation:
	clone the repository
	install python dependencies (numpy, pandas, matplotlib, paho-mqtt)
	install MQTT broker

For ROS2/Gazebo Extension:
	run setup_ros2_gazebo.py for automatic setup
	or follow manual installation in README_EXTENSION.md

Running the simulation:
	run iot_system.py file
	(optional) run dashboard_app for the live dashboard (status, altitude, alerts)

For ROS2/Gazebo Extension:
	source crazyflie_ws/install/setup.bash
	ros2 launch crazyflie_gazebo crazyflie_simulation.launch.py

Analyzing simulation data
	once the simulation is finished (just close the matlab live plotter window) you have to run the data_analyzer.py file

Testing integration:
	run test_integration.py to test the IoT-ROS2-Gazebo integration

Advanced features:
	Flip maneuvers: Execute flips via MQTT or ROS2 topics
	Custom trajectories: Define complex flight paths
	Real-time monitoring: Monitor drone state and performance
	Performance analysis: Analyze control performance and stability

For detailed documentation on the ROS2/Gazebo extension, see README_EXTENSION.md

