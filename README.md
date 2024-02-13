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

## Easy run
```
docker compose up --build
```
This will compile the local ros ws, build into docker containers, and launch all the services (ctrl-c to stop). For subsequent runs, `--build` can be omitted (unless there are source code changes requiring a rebuild of the docker images). 

## Individual services

The `docker-compose.yml` file contains some of the different services that can be launched. We can use `docker compose up [service name]` to run a service individually

### Advanced
```
docker run -p 62001:62001/udp --rm [container_name] bash
```
Once the bash shell is launched, we have free control to launch any ros nodes we desire. For example, 
`ros2 run asv_model asv_state`
will run the state publisher

#### Foxglove
If we want to use Foxglove, we need to launch the foxglove bridge. This is handled in the `foxglove_bridge`  service in the docker-compose.yml file, and the `docker compose run foxglove_bridge` command can be used to launch this service.

# Notes
Quick reference for [message types](https://github.com/ros2/common_interfaces).
