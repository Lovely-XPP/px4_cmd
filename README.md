# PX4 command

## Introduction
PX4 command sent via terminal (based on mavlink), for whom wants to control vehicle by themselves, e.g. setting points.

## Support Command

### Idle

Initial Mode for Vehicle.

### Takeoff

You can set desire height for takeoff.

### Move

<div style="align: center">

| Frame | SubCommand |
| :---: | ----------- |
|   ENU    | Position (XYZ) [m]<br>Velocity (XY) [m/s] + Height (Z) [m]<br>Velocity (XYZ) [m/s]<br>Relative Position (XYZ) [m] |
|   Body   | Velocity (XYZ) [m/s] |

</div>

And support `yaw command [deg]` input for both frames.


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

## Todo

- [x] Mode Change
- [x] Set Command 
- [x] Send Command to PX4
- [x] Support Video Recieving

## Required Packages
```
PX4-Autopilot
Mavros
```
You can follow the instructions on the [Wiki page](https://github.com/Lovely-XPP/PX4_cmd/wiki).

## Installation
```bash
# Create Catkin Workspace in home
mkdir -p ~/px4_ws/src 
cd catkin_ws/src
catkin_init_workspace
# Clone this Repo
git clone https://github.com/Lovely-XPP/PX4_cmd.git
# make
cd ..
catkin_make
# Add Source for the project
echo "source ~/px4_ws/devel/setup.bash" >> ~/.bashrc
# update terminal
source ~/.bashrc
```

## Run Simulation
```bash
bash ~/px4_ws/src/px4_cmd/src/sh/sim.sh
```

## About Offboard Mode
If you have problem for changing mode to Offboard Mode, please check the offical instruction:

> Note for Offboard Mode
> - This mode requires position or pose/attitude information - e.g. GPS, optical flow, visual-inertial odometry, mocap, etc.
> - RC control is disabled except to change modes.
> - The vehicle must be armed before this mode can be engaged.
> - The vehicle must be already be receiving a stream of target setpoints (>2Hz) before this mode can be engaged.
> - The vehicle will exit the mode if target setpoints are not received at a rate of > 2Hz.
> - Not all coordinate frames and field values allowed by MAVLink are supported.

More details in [https://docs.px4.io/main/en/flight_modes/offboard.html](https://docs.px4.io/main/en/flight_modes/offboard.html).

## About PX4 V1.13.0 or Newer Version

***Tips: If you are using PX4 V1.13.0 or newer, you can not set to Offboard Mode in simulation, because Offboard Mode can not be enabled without RC signal in newer version.***

### Relevant Code
`PX4-Autopilot/src/modules/commander/state_machine_helper.cpp: 632`

```cpp
else if (status.rc_signal_lost && !(param_com_rcl_except & RCLossExceptionBits::RCL_EXCEPT_OFFBOARD)) {
    // Only RC is lost
    enable_failsafe(status, old_failsafe, mavlink_log_pub, event_failsafe_reason_t::no_rc);
    set_link_loss_nav_state(status, armed, status_flags, internal_state, rc_loss_act, param_com_rcl_act_t);
}
```

Therefore, even if we provide stable Offboard Command (publish to `/mavros/setpoints_raw/local`, > 2 Hz), we still can not enable the Offboard Mode if We do not have RC signal.

### Temporary Solution
If you don't want to connet a RC controller for simulation, you can delete the code shown above, but it ***ONLY FOR Simulation USE, DO NOT For Real Vehicle USE.***



## Credits
- [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot)
- [PX4 Guide](https://docs.px4.io/main)
