# asl_flight2
![foxy](https://github.com/StanfordASL/asl_flight2/actions/workflows/foxy.yml/badge.svg)
![humble](https://github.com/StanfordASL/asl_flight2/actions/workflows/humble.yml/badge.svg)
![rolling](https://github.com/StanfordASL/asl_flight2/actions/workflows/rolling.yml/badge.svg)

## Environment Setup
1. Install ROS2 ([foxy](https://docs.ros.org/en/foxy/Installation.html), 
[humble](https://docs.ros.org/en/humble/Installation.html), 
or [rolling](https://docs.ros.org/en/rolling/Installation.html))
2. Install Fast-DDS-Gen following 
[this guide](https://docs.px4.io/main/en/dev_setup/fast-dds-installation.html#fast-rtps-gen) from PX4.
3. Create a ROS2 workspace

    ```sh
    mkdir ~/ros2_ws && cd ~/ros_ws
    ```
4. Pull this repository and custom dependencies
    
    ```sh
    git clone https://github.com/StanfordASL/asl_flight2.git src/asl_flight2
    vcs import src < src/asl_flight2/.github/ros2.repos
    ```

5. Install package dependencies with `rosdep`
    
    ```sh
    rosdep update
    rosdep install --from-paths src -y --ignore-src
    ```

## Build

The building process is standard as in any ROS2 package.

```sh
cd ~/ros_ws  # go to your ROS2 workspace
colcon build
```

## Development

This codebase has linter and code-style enforcement following the 
[ROS2 official standard](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html).
Pull requests can only be merged if all tests pass. This can be locally checked with

```sh
colcon test --packages-select asl_flight2
```

or to see verbose test failure reasons,

```sh
colcon test --packages-select asl_flight2 --event-handlers console_direct+
```
