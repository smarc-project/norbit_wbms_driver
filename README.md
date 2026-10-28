# Norbit WBMS Multibeam Driver

This repository provides a ROS 2 driver for the Norbit WBMS multibeam sonar, which communicates with the driver via TCP connections. It allows for real-time configuration, control, data acquisition and visualization from the sonar, and includes supports for bathymetry, water column and snippet data. The repository contains two ROS 2 packages:

- **`norbit_wbms_driver`**: A Python package that implements the sonar drivers, data parsers and visualizers.
- **`sonar_interfaces`**: A C++ package containing custom ROS 2 message definitions used across the sonar system.

## Recent change to Watercolumn message
The data type of the raw watercolumn data has been changed from Image to Float32MultiArray. If you have problems playing older rosbags from Lolo (before 2026-11) try ckecking out an earlier version.

## Package Structure
### `norbit_wbms_driver`
Contains the main ROS 2 nodes for interfacing with the sonar over TCP.
Each node is responsible for one type of communication or processing task.
The settings driver and parsers each connect to the sonar using a different TCP port.

| Node                  | Role                      |  Port | Status     |
|-----------------------|---------------------------|--------------|------------|
| `wbms_driver`         | Sonar configuration       | 2209        | ✅ Implemented |
| `bathymetry_parser`   | Bathymetric data stream   | 2210        | ✅ Implemented |
| `watercolumn_parser`  | Water column data stream  | 2211        | ✅ Implemented |
| `snippet_parser`      | Snippet data stream       | 2212        | ⏳ Planned |
| `bathymetry_visualizer` | Bathymetry visualization     | —            | 🛠 In Progress |
| `watercolumn_visualizer` | Water column data visualization         | —            | 🛠 In Progress    |
| `snippet_visualizer`  | Snippet data visualization | —            | ⏳ Planned |

### `norbit_wbms_interfaces`
Contains custom ROS2 messages definitions for the different sonar data streams.

|Message  | Status | Comments |
|---------|--------|----------|
|BathymetryBeam|✅ Implemented | One single beam of a Bathymetry message |
|Bathymetry|✅ Implemented | |
|WaterColumn| ✅ Implemented| |
|Snippet| ⏳ Planned |
|Topics| ⏳ Planned | Topic names (constants) for the driver |

## Dependencies

This driver has been tested with the following environment:

- [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
- [Ubuntu 22.04 LTS](https://releases.ubuntu.com/jammy/)
- Python 3.10.12

## Installation

Clone the repository into your ROS 2 workspace and build:

```bash
cd ~/ros2_ws/src
git clone https://github.com/smarc-project/norbit_wbms_driver.git
colcon build && source install/setup.bash
```

## Usage

### Change sonar configurations using `wbms_driver`
The available parameters, meanings and default settings can be found at `driver_node/config/wbms_default.yaml`.
1. Start the sonar configuration node with default parameters:
```bash
ros2 run norbit_wbms_driver wbms_driver --ros-args \
  --params-file ros2_ws/src/norbit_wbms_driver/driver_node/config/wbms_default.yaml 
```
2. Change individual parameters via command line:
```bash
# ros2 param set wbms_driver [param-name] [param-value]
# e.g. use set_mode parameter to change the sonar mode to 0 (forward)
ros2 param set wbms_driver set_mode 0
```
3. Load a new parameter file via command line:
```bash
ros2 param load wbms_driver config/wbms_default.yaml
```
### Parse data streams and visualize results
There are three possible data streams from the Norbit WBMS sonar: bathymetry, water column and snippet.
1. Parse data stream:
```bash
# ros2 run norbit_wbms_driver [stream]_parser
# e.g. run bathymetry parser
ros2 run norbit_wbms_driver bathymetry_parser
```
2. Real-time visualization:
```bash
ros2 run norbit_wbms_driver [stream]_visualizer
# e.g. run bathymetry visualizer
ros2 run norbit_wbms_driver bathymetry_visualizer
```
