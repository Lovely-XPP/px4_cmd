# External Command API Usage

## Installation

``````
catkin_make install
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

// simulation / hardware for single vehicle 
#include <px4_cmd/template/single_vehicle_external_command.hpp>

// init class
single_vehicle_external_command ext_cmd();

// start 
ext_cmd.start();

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


// simulation for muti-vehicles (this mode can not use in hardware)
#include <px4_cmd/template/vehicle_external_command.hpp>
// init class - node_name in rostopic: /{node_name}/mavros/****
vehicle_external_command ext_cmd(node_name);

// start 
ext_cmd.start();

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
ext_cmd = single_vehicle_external_command()

# start 
ext_cmd.start();

''' set poistion 
x: position in x axis
y: position in y axis
z: position in z axis
yaw (optional): command for yaw
frame: 
 Global Frame: px4_cmd::Command::ENU 
  Body  Frame: px4_cmd::Command::BODY
'''
ext_cmd.set_position(x, y, z, frame);
ext_cmd.set_position(x, y, z, yaw, frame);

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
ext_cmd.set_velocity_with_height(x, y, z, frame);
ext_cmd.set_velocity_with_height(x, y, z, yaw, frame);

# shutdown
ext_cmd.shutdown();


# simulation for muti-vehicles (this mode can not use in hardware)
from px4_cmd.vehicle_external_command import vehicle_external_command
# init class - node_name in rostopic: /{node_name}/mavros/****
ext_cmd = vehicle_external_command(node_name);

# start 
ext_cmd.start();

''' set poistion 
x: position in x axis
y: position in y axis
z: position in z axis
yaw (optional): command for yaw
frame: 
 Global Frame: px4_cmd::Command::ENU 
  Body  Frame: px4_cmd::Command::BODY
'''
ext_cmd.set_position(x, y, z, frame);
ext_cmd.set_position(x, y, z, yaw, frame);

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
ext_cmd.set_velocity_with_height(x, y, z, frame);
ext_cmd.set_velocity_with_height(x, y, z, yaw, frame);

# shutdown
ext_cmd.shutdown();
```

## Examples

### Single Vehicle Simulation

#### C++

Code: examples/single_vehicle_sim.cpp

#### python

Code: examples/single_vehicle_sim.py

### Muti-Vehicle Simualtion

#### C++

Code: examples/single_vehicle_sim.cpp

#### python

Code: examples/single_vehicle_sim.py