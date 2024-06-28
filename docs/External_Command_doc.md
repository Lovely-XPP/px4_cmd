# External Command API Usage

## Installation

``````
catkin_make
``````

## Usage for Your ROS Package

If you want to use your own external command (e.g vision control) in your ros package, here is the instruction.

### CmakeLists.txt

```cmake
find_package(catkin REQUIRED COMPONENTS
	px4_cmd
)
```

### package.xml

```xml
<build_depend>px4_cmd</build_depend>
<build_export_depend>px4_cmd</build_export_depend>
<exec_depend>px4_cmd</exec_depend>
```

### Usage in source code

#### C++

```cpp
// introduce px4_cmd message
#include <px4_cmd/Command.h>

// simulation / hardware for single or multiple vehicle(s)
#include <px4_cmd/vehicle_external_command.h>

// init class
vehicle_external_command ext_cmd();

// start 
ext_cmd.start();
// if multiple vehicle, need to input node name
// topic name: /${node_name}/mavros/xxxxx
ext_cmd.start(node_name);

// set poistion 
// x: position in x axis
// y: position in y axis
// z: position in z axis
// yaw (optional): command for yaw
// frame: 
// Global Frame: px4_cmd::Command::ENU 
//  Body  Frame: px4_cmd::Command::BODY
ext_cmd.set_position(x, y, z, frame);
ext_cmd.set_position(x, y, z, yaw, frame);

// set velocity
// vx: velocity in x axis
// vy: velocity in y axis
// vz: velocity in z axis
// yaw (optional): command for yaw
// frame: 
// Global Frame: px4_cmd::Command::ENU 
//  Body  Frame: px4_cmd::Command::BODY
ext_cmd.set_velocity(vx, vy, vz, frame);
ext_cmd.set_velocity(vx, vy, vz, yaw, frame);

// set velocity with height
// vx: velocity in x axis
// vy: velocity in y axis
// z:  position in z axis
// yaw (optional): command for yaw
// frame (this mode only support Global Frame): 
// Global Frame: px4_cmd::Command::ENU 
ext_cmd.set_velocity_with_height(x, y, z, frame);
ext_cmd.set_velocity_with_height(x, y, z, yaw, frame);

// shutdown
ext_cmd.shutdown();
```

#### python

```python
# introduce px4_cmd message
from px4_cmd.msg import Command

# simulation / hardware for single vehicle 
from px4_cmd.single_vehicle_external_command import single_vehicle_external_command

# init class
ext_cmd = vehicle_external_command()

# start 
ext_cmd.start()
# if multiple vehicle, need to input node name
# topic name: /${node_name}/mavros/xxxxx
ext_cmd.start(node_name)

''' set poistion 
x: position in x axis
y: position in y axis
z: position in z axis
yaw (optional): command for yaw
frame: 
 Global Frame: px4_cmd::Command::ENU 
  Body  Frame: px4_cmd::Command::BODY
'''
ext_cmd.set_position(x, y, z, frame)
ext_cmd.set_position(x, y, z, yaw, frame)

''' set velocity
vx: velocity in x axis
vy: velocity in y axis
vz: velocity in z axis
yaw (optional): command for yaw
frame: 
 Global Frame: px4_cmd::Command::ENU 
  Body  Frame: px4_cmd::Command::BODY
'''
ext_cmd.set_velocity(vx, vy, vz, frame);
ext_cmd.set_velocity(vx, vy, vz, yaw, frame);

''' set velocity with height
vx: velocity in x axis
vy: velocity in y axis
z:  position in z axis
yaw (optional): command for yaw
frame (this mode only support Global Frame): 
 Global Frame: px4_cmd::Command::ENU 
'''
ext_cmd.set_velocity_with_height(x, y, z, frame)
ext_cmd.set_velocity_with_height(x, y, z, yaw, frame)

# shutdown
ext_cmd.shutdown()
```

## Examples

### Single & Muti-Vehicle Simualtion

#### C++

Code: examples/vehicle_sim.cpp

#### python

Code: examples/vehicle_sim.py
