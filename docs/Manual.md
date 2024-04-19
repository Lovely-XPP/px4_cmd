# Manual

## Support Mode
```c++
"Refresh Status"  //刷新状态
"MANUAL"          //手动
"OFFBOARD"        //外部控制
"STABILIZED"      //自稳
"POSCTL"          //位置控制
"ALTCTL"          //高度控制
"AUTO.LAND"       //自动降落
"AUTO.RTL"        //自动返航
"Arm"             //解除锁定
"DisArm"          //锁定
```

## Support Command

### Hover

#### Idle

Initial Mode for Vehicle.

#### Hover
Hover at the setpoint.

#### Takeoff

You can set desire height for takeoff.

#### Move

<div style="align: center">

| Frame | SubCommand |
| :---: | ----------- |
|   ENU    | Position (XYZ) [m]<br>Velocity (XY) [m/s] + Height (Z) [m]<br>Velocity (XYZ) [m/s]<br>Relative Position (XYZ) [m] |
|   Body   | Velocity (XYZ) [m/s] |

</div>

And support `yaw command [deg]` input for both frames.

#### External Command
Using `single_vehicle_external_command.hpp` as API.

### Fix Wing

#### Idle

Initial Mode for Vehicle.

#### Loiter

Loiter at the setpoint, you can also set the radius of loiter cycle.

#### Takeoff

For fix wing, you need to set x,y,z position for takeoff.

#### Move

<div style="align: center">

| Frame | SubCommand |
| :---: | ----------- |
|   ENU    | Position (XYZ) [m]   |
|   Body   | Position (XYZ) [m] |

</div>

#### External Command
Using `single_vehicle_external_command.hpp` as API.

## Required Packages
```
PX4-Autopilot
Mavros
```
You can follow the instructions on the [Wiki page](https://github.com/Lovely-XPP/PX4_cmd/wiki).