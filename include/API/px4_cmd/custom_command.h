// Copyright (c) 2023 易鹏 中山大学航空航天学院
// Copyright (c) 2023 Peng Yi, Sun Yat-Sen University, School of Aeronautics and Astronautics

#ifndef CUSTOM_COMMAND_H
#define CUSTOM_COMMAND_H
#include <cmath>
#include <px4_cmd/Command.h>

/// @brief custom command mode
enum CommandMode
{
    TargetLocal = 1,   // local frame with position / velocity / accelerate setpoint
    TargetGlobal = 2,  // global frame with position / velocity setpoint (lat / lon / alt)
    TargetAttitude = 3 // Attitude control with thrust
};

/// @brief  FixWing only support position setpoint in local & global mode, and need speicify position mode
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

/// @brief Custom command for vehicle with px4
struct CustomCommand
{
    // command mode: local / global / attitude
    CommandMode mode;
    // FixWing only support position setpoint in local & global mode, and need specify position mode
    FixWingPositionMode fw_mode;     
    // frame id in px4_cmd message
    int frame_id;                    
    // position setpoint, xyz for local, lat lon alt for global
    double position[3] = {
        NAN,
        NAN,
        NAN
    };
    // velocity setpoint
    double velocity[3] = {
        NAN,
        NAN,
        NAN
    };
    // accelerate setpoint
    double accelerate[3] = {
        NAN,
        NAN,
        NAN
    };
    // flag for accelerate to force
    bool force_flag = false;
    // yaw setpoint
    double yaw = NAN;
    // yaw rate setpoint
    double yaw_rate = NAN;
    // attitude setpoint [quaternion {xyzw}]
    double attitude[4] = {
        NAN,
        NAN,
        NAN,
        NAN
    };
    // attitude rate setpoint
    double attitude_rate[3] = {
        NAN,
        NAN,
        NAN
    };
    // thrust setpoint
    double thrust = NAN;             
};


/// @brief reset custom command to default value (NAN), everytime generate custom command need this operate
/// @param cmd
void reset_custom_command(CustomCommand &cmd);

/// @brief transfer px4 command message to custom command struct
/// @param msg px4 command message
/// @param cmd custom command struct
void px4_msg_to_custom_command(const px4_cmd::Command &msg, CustomCommand &cmd);

/// @brief transfrom custom command struct to px4 command message
/// @param msg px4 command message
/// @param cmd custom command struct
void custom_command_to_px4_msg(const CustomCommand &cmd, px4_cmd::Command &msg);

/// @brief check custom command if correct
/// @param cmd [in] custom command
/// @param vehicle_type [in] Vehicle type in px4_cmd message : Multicopter / FixWing
/// @param error [out] error inforamtion string
/// @return if custom command input correct, true - correct, false - incorrect
bool check_custom_command(CustomCommand cmd, int vehicle_type, std::string &error);

#endif