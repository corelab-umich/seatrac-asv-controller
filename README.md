# TODO
- Use `sensor_msgs/BatteryState` to track SOC
- Use `nav_msgs/Odometry` to define pose and twist (position and heading)
- Create controllers
- Create sensors package and write drivers/parser for each sensor
- Develop environment block with custom wind/currents and historical wind/currents

# Dependencies
All dependencies will be installed by container.
## ROS2 Humble Hawksbill
This software is written to work with [ROS2 Humble Hawksbill](https://docs.ros.org/en/humble/index.html). 

## Foxglove
The Foxglove websocket will be automatically installed through the Dockerfile.

# Running
```
docker run -p 62001:62001/udp --rm [container_name]
```
`ros2 run asv_model asv_state`
will run the state publisher

## Foxglove
We must open a docker container that exposes its port to the network/machine
```
docker run -it -p 8765:8765 --rm [container_name] bash
```
`ros2 launch foxglove_bridge foxglove_bridge_launch.xml`
will open up the websocket and enable data to be viewed from foxglove

# Notes
Quick reference for [message types](https://github.com/ros2/common_interfaces).
