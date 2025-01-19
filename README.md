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

``` bash
ros2 launch sainsmart-simulator simulate_launch.py
```

# Contacts

aeon_flux <aeon_flux@eclipso.ch>
