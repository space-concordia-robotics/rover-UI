# Rover UI

## Build
### Frontend
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
ros2 launch ...
```
or

```
python3.12 src/sc_rover_sim/sc_rover_sim/gps_sim.py
```
#### ROSBridge
```
source /opt/ros/jazzy/setup.bash
ros2 run rosbridge_server rosbridge_websocket
```

