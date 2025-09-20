# Rover UI

## Build
### Run the Frontend
```
cd frontend
npm i
npm run dev
```
### ROS2 Test
#### Nodes
```bash
cd ros2_ws
colcon build
source install/setup.bash
```

#### Simulated GPS Data
```
python3.12 src/sc_rover_sim/sc_rover_sim/gps_sim.py
```
#### ROSBridge
```
source /opt/ros/jazzy/setup.bash
ros2 run rosbridge_server rosbridge_websocket
```
## How It Works
The Rover UI operates through a bridge architecture connecting ROS2 and web technologies. The frontend application receives real-time data from ROS2 topics via ROSBridge, which acts as a WebSocket server that translates ROS2 messages into JSON format. This allows the web-based interface to subscribe to and display live rover telemetry data, including GPS coordinates and other sensor information, without requiring direct ROS2 integration in the browser.
