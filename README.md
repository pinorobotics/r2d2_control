**r2d2** (ros2dorna2) - set of ROS2 packages which provide support for [Dorna2 robotic arm](https://dorna.ai/).

**r2d2_control** package contains ROS2 controller node for Dorna2 robotic arm. The controller is implemented in Java.

NOTE: This is unofficial Dorna ROS2 package. For official Dorna software follow https://dorna.ai/

# Prereq

- Java 22
- Gradle (exact version see in [build.gradle](build.gradle)

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
