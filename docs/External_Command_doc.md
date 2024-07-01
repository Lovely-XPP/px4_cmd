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

### Source code

#### C++

```cpp
// introduce px4_cmd message
#include <px4_cmd/Command.h>

// simulation / hardware for single or multiple vehicle(s)
#include <px4_cmd/vehicle_external_command.h>

// init class
vehicle_external_command ext_cmd();

// start node
// if multiple vehicle, need to input node name
// topic name: /${node_name}/mavros/xxxxx
// if single vehicle, no need to input node name
ext_cmd.start();
ext_cmd.start(node_name);

// get information for uav
// Example to get initial position for uav
ext_cmd->init_x;
ext_cmd->init_y;
ext_cmd->init_z;

// Example to get position (m) / attitude (deg) / velocity (m/s) / angle_rate (deg/s) for uavs
ext_cmd->position[0]; // get x position of uav
ext_cmd->velocity[1]; // get y velocity of uav
ext_cmd->angle_rate[1]; // get x angle rate of uav
ext_cmd->attitude[0]; // get x attitude (Row) of uav
ext_cmd->attitude[1]; // get y attitude (Pitch) of uav
ext_cmd->attitude[2]; // get z attitude (Yaw) of uav
ext_cmd->quaternion; // get quaternion of uav (tf::Quaternion)
// get element of quaternion
ext_cmd->quaternion.x();
ext_cmd->quaternion.y();
ext_cmd->quaternion.z();
ext_cmd->quaternion.w();

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

// setting custom command
// cmd custom command, more detail in next section
void set_custom_command(CustomCommand cmd);

// setting vehicle to hover mode
// yaw (optional): command for yaw
void set_hover();
void set_hover(double yaw);

// set sync lock with bool ptr
// sync_lock bool ptr
void set_sync_lock(bool *sync_lock_);

// reset sync lock to nullptr
void reset_sync_lock();

// shutdown
ext_cmd.shutdown();
```

#### python

```python
# introduce px4_cmd message
from px4_cmd.msg import Command

# simulation / hardware for single vehicle 
from px4_cmd.vehicle_external_command import vehicle_external_command

# init class
ext_cmd = vehicle_external_command()

# start node
# if multiple vehicle, need to input node name
# topic name: /${node_name}/mavros/xxxxx
# if single vehicle, no need to input node name
ext_cmd.start()
ext_cmd.start(node_name)

# get information for uav
# Example to get initial position for uav
ext_cmd.init_x
ext_cmd.init_y
ext_cmd.init_z

# Example to get position (m) / attitude (deg) / velocity (m/s) / angle_rate (deg/s) for uavs
ext_cmd.position[0] # get x position of uav
ext_cmd.velocity[1] # get y velocity of uav
ext_cmd.angle_rate[1] # get x angle rate of uav
ext_cmd.attitude[0] # get x attitude (Row) of uav
ext_cmd.attitude[1] # get y attitude (Pitch) of uav
ext_cmd.attitude[2] # get z attitude (Yaw) of uav
ext_cmd.quaternion # get quaternion of uav (quaternion format: [x, y, z, w])


''' set poistion 
x: position in x axis
y: position in y axis
z: position in z axis
yaw (optional): command for yaw
frame: 
 Global Frame: px4_cmd.Command.ENU 
  Body  Frame: px4_cmd.Command.BODY
'''
ext_cmd.set_position(x, y, z, yaw, frame)

"""setting velocity command in 3 axis for vehicle
Args:
    vx (float): desire velocity in x axis
    vy (float): desire velocity in y axis
    vz (float): desire velocity in z axis
    yaw (float | None, optional): desire yaw command. Defaults to None.
    frame (int, optional): velocity in which frame, px4_cmd.Command.ENU / px4_cmd.Command.BODY. Defaults to Command.ENU.
"""
ext_cmd.set_velocity(vx, vy, vz, yaw, frame)

"""setting velocity command in 2 axis with height command for vehicle

Args:
    vx (float): desire velocity in x axis
    vy (float): desire velocity in y axis
    z (float): desire height
    yaw (float | None, optional): desire yaw command. Defaults to None.
    frame (int, optional): velocity in which frame, px4_cmd.Command.ENU / px4_cmd.Command.BODY. Defaults to Command.ENU.
"""
ext_cmd.set_velocity_with_height(x, y, z, yaw, frame)

"""setting custom command

Args:
    cmd (CustomCommand): custom command struct, more detail in next section
"""  
ext_cmd.set_velocity_with_height(cmd)

"""setting vehicle to hover mode

Args:
    yaw (float | None, optional): desire yaw command. Defaults to None.
"""
ext_cmd.set_hover(yaw)

# shutdown
ext_cmd.shutdown()
```

## Additional Information for Custom Command
Custom Command provide more flexible way for user to give control command for uav. Support combination commands are shown as follow:

### Custom Command Struct
#### C++
```C++
// custom command mode
enum CommandMode
{
    TargetLocal = 1,   // local frame with position / velocity / accelerate setpoint
    TargetGlobal = 2,  // global frame with position / velocity setpoint (lat / lon / alt)
    TargetAttitude = 3 // Attitude control with thrust
};

// FixWing only support position setpoint in local & global mode, and need speicify position mode
enum FixWingPositionMode : unsigned long
{
    // Gliding Mode.
    // This configures TECS to prioritize airspeed over altitude in order to make the vehicle glide when there is no thrust (i.e. pitch is controlled to regulate airspeed).
    GlidingMode = 292,
    // Takeoff Mode
    TakeoffMode = 4096,
    // Land Mode
    LandMode = 8192,
    // Loiter Mode (fly a circle centred on setpoint).
    LoiterMode = 12288,
    // Idle Mode (zero throttle, zero roll / pitch).
    IdleMode = 16384
};

// Custom command for vehicle with px4
struct CustomCommand
{
    // command mode: local / global / attitude
    CommandMode mode;                
    // FixWing only support position setpoint in local & global mode, and need speicify position mode
    FixWingPositionMode fw_mode;     
    // frame id in px4_cmd message
    int frame_id;                    
    // position setpoint, xyz for local, lat lon alt for global
    double position[3];
    // velocity setpoint
    double velocity[3];
    // accelerate setpoint
    double accelerate[3];
    // flag for accelerate to force
    bool force_flag;
    // yaw setpoint
    double yaw;
    // yaw rate setpoint
    double yaw_rate;
    // attitude setpoint [quaternion {xyzw}]
    double attitude[4];
    // attitude rate setpoint
    double attitude_rate[3];
    // thrust setpoint
    double thrust = NAN;             
};
```

#### python
```python
class CommandMode(Enum):
    TargetLocal = 1,   # local frame with position / velocity / accelerate setpoint
    TargetGlobal = 2,  # global frame with position / velocity setpoint (lat / lon / alt)
    TargetAttitude = 3 # Attitude control with thrust

class FixWingPositionMode(Enum):
    # Gliding Mode.
    # This configures TECS to prioritize airspeed over altitude in order to make the vehicle glide when there is no thrust(i.e. pitch is controlled to regulate airspeed).
    GlidingMode = 292,
    # Takeoff Mode
    TakeoffMode = 4096,
    # Land Mode
    LandMode = 8192,
    # Loiter Mode(fly a circle centred on setpoint).
    LoiterMode = 12288,
    # Idle Mode(zero throttle, zero roll / pitch).
    IdleMode = 16384

class CustomCommand:
    def __init__(self):
        # command mode: local / global / attitude
        self.mode: CommandMode = CommandMode.TargetLocal
        # FixWing only support position setpoint in local & global mode, and need specify position mode
        self.fw_mode: FixWingPositionMode = FixWingPositionMode
        # frame id in px4_cmd message
        self.frame_id: int = 0 
        # position setpoint, xyz for local, lat lon alt for global
        self.position: List[int] = [NaN, NaN, NaN]
        # velocity setpoint
        self.velocity: List[int] = [NaN, NaN, NaN]
        # accelerate setpoint
        self.accelerate: List[int] = [NaN, NaN, NaN]
        # flage for accelerate to force
        self.force_flag: bool = False
        # yaw serpoint
        self.yaw: float = NaN
        # yaw rate setpoint
        self.yaw_rate: float = NaN
        # attitude setpoint [quaternion {xyzw}]
        self.attitude: List[float] = [NaN, NaN, NaN, NaN]
        # attitude rate setpoint
        self.attitude_rate: List[int] = [NaN, NaN, NaN]
        # thrust setpoint
        self.thrust: float = NaN

```

## Examples

### Single Vehicle Simualtion

#### C++

Code: examples/single_external_cmd.cpp

Exe: CMake target: px4_cmd_example_single

#### python

Code & Exe: examples/single_external_cmd.py

### Multiple Vehicles Simualtion

#### C++

Code: examples/multiple_external_cmd.cpp

Exe: CMake target: px4_cmd_example_multiple

#### python

Code & Exe: examples/multiple_external_cmd.py
