# TODO
- Use `sensor_msgs/BatteryState` to track SOC
- Use `nav_msgs/Odometry` to define pose and twist (position and heading)
- Create controllers
- Create sensors package and write drivers/parser for each sensor
- Develop environment block with custom wind/currents and historical wind/currents

# Dependencies
## Foxglove
To view robot data in Foxglove, you need set up the [foxglove websocket](https://docs.foxglove.dev/docs/connecting-to-data/ros-foxglove-bridge/).

# Running
`ros2 run asv_model asv_state`
will run the state publisher

`ros2 launch foxglove_bridge foxglove_bridge_launch.xml`
will open up the websocket and enable data to be viewed from foxglove

# Notes
Quick reference for [message types](https://github.com/ros2/common_interfaces).
