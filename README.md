[**r2d2** (ros2dorna2)](https://github.com/pinorobotics/r2d2) - set of ROS2 packages which provide support for [Dorna2 robotic arm](https://dorna.ai/).

**r2d2_control** package contains ROS2 Joint Trajectory Controller node for Dorna2 robotic arm.

NOTE: This is unofficial Dorna ROS2 package. For official Dorna software follow https://dorna.ai/

# Prereq

- Java 22

# Build

First build Java part:
```
gradle clean build
```

Then build ROS part:
```
cd <ROS WORKSPACE>
colcon build
source install/setup.zsh
```

# Run

**r2d2_control** package does not have own ROS launch files since it is integrated into launch files of [r2d2](https://github.com/pinorobotics/r2d2) package.

# Documentation

- [r2d2](https://github.com/pinorobotics/r2d2)
- [Java based ROS2 controller for Dorna2 robotic arm](http://pinoweb.freetzi.com/jroscontrol/r2d2/index.html)

# Usage
```
r2d2_control [ <OPTIONS> ]
```

Options:

`-dornaUrl=<string>` - URL to Dorna2 robotic arm Command Server. Default: "ws://192.168.0.3:443"

`-controllerName=<string>` - name of the Dorna2 controller as it will appear to ROS nodes. Default: "dorna2_arm_controller"

`-moveItConfigPath=<path>` - path to [r2d2_moveit_config](https://github.com/pinorobotics/r2d2_moveit_config) ROS package with MoveIt configuration of Dorna2 arm. Default: "../r2d2_moveit_config"

`-broadcastRateInMillis=<int>` - how often to publish states of Dorna2 joints. Default: 100

`-exportMetricsToElastic=<string>` - Elasticsearch URL where metrics will be emitted. Credentials can be part of the URL. Example "http://user:password@localhost:9200" (this option is ignored if "METRICS_ELASTIC_URL" environment variable is present)

`-debug=<true|false>` - print debug information and log it to r2d2-control-debug.log inside system temporary folder. Default is "false".

# Contacts

aeon_flux <aeon_flux@eclipso.ch>
