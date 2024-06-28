# Change Log
## V2.1.0
### Developer Warning
- ***#include<px4_cmd/vehicle_external_command.hpp> change to #include<px4_cmd/vehicle_external_command.h>***

### Feature Improvement
- improve python API
- update examples

### Fix Bug
- realsense_camera Model can not publish image topics properly
- update docs


## V2.0.0 - first Release
### Main Features
- Generate your px4 simulation launch file with specified setting easily:
  - vehicles: `iris`, `typhoon`, `standard plane`, support ***multiple vehicles*** up to any numbers you want.
  - sensors: currently support `RGB camera`, `Depth camera`, `Realsense D435i`
  - worlds: sitl_gazebo_folder/worlds
  - other settings: sensors further setting, etc.

- Simple GroundContorller for simulation:
  - basic operations: take off, change px4 mode, etc.
  - Real-time Data Visulization
  - Control vehicle with custom position / velocity command
  - Multiple vehicles control support
  - Image Topic Visulization
  - Runing ROS node
  - Provide external command API for user-defined command by other ROS Node
