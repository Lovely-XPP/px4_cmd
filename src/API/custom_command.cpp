// Copyright (c) 2023 易鹏 中山大学航空航天学院
// Copyright (c) 2023 Peng Yi, Sun Yat-Sen University, School of Aeronautics and Astronautics

#include <px4_cmd/custom_command.h>

void reset_custom_command(CustomCommand &cmd)
{
    // reset values to NAN
    for (size_t i = 0; i < 3; i++)
    {
        cmd.position[i] = NAN;
        cmd.velocity[i] = NAN;
        cmd.accelerate[i] = NAN;
        cmd.attitude_rate[i] = NAN;
    }
    for (size_t i = 0; i < 4; i++)
    {
        cmd.attitude[i] = NAN;
    }
    cmd.force_flag = false;
    cmd.yaw = NAN;
    cmd.yaw_rate = NAN;
    cmd.thrust = NAN;
}

void px4_msg_to_custom_command(const px4_cmd::Command &msg, CustomCommand &cmd)
{
    // new command
    CustomCommand generate_cmd;

    // mode
    switch (msg.Custom_Command_Mode)
    {
        case px4_cmd::Command::Custom_Command_TargetLocal:
        {
            generate_cmd.mode = CommandMode::TargetLocal;
            break;
        }
        case px4_cmd::Command::Custom_Command_TargetGlobal:
        {
            generate_cmd.mode = CommandMode::TargetGlobal;
            break;
        }
        case px4_cmd::Command::Custom_Command_TargetAttitude:
        {
            generate_cmd.mode = CommandMode::TargetAttitude;
            break;
        }
    }

    // frame
    generate_cmd.frame_id = msg.Move_frame;

    // convert data
    int id = 0;

    // position convert [id 0 - 2]
    for (size_t i = 0; i < 3; i++)
    {
        generate_cmd.position[i] = msg.custom_cmd[i];
    }

    // velocity convert [id 3 - 5]
    id += 3;
    for (size_t i = 0; i < 3; i++)
    {
        generate_cmd.velocity[i] = msg.custom_cmd[i + id];
    }

    // accelerate convert [id 6 - 8]
    id += 3;
    for (size_t i = 0; i < 3; i++)
    {
        generate_cmd.accelerate[i] = msg.custom_cmd[i + id];
    }

    // Force flag [id 9]
    id += 3;
    generate_cmd.force_flag = msg.custom_cmd[id];

    // yaw convert [id 10]
    id += 1;
    generate_cmd.yaw = msg.custom_cmd[id];

    // yaw rate convert [id 11]
    id += 1;
    generate_cmd.yaw_rate = msg.custom_cmd[id];

    // Attitude (quaternion) convert [id 12 - 15]
    id += 1;
    for (size_t i = 0; i < 4; i++)
    {
        generate_cmd.attitude[i] = msg.custom_cmd[i + id];
    }

    // Attitude (quaternion) convert [id 16 - 18]
    id += 4;
    for (size_t i = 0; i < 3; i++)
    {
        generate_cmd.attitude_rate[i] = msg.custom_cmd[i + id];
    }

    // Thrust convert [id 19]
    id += 3;
    generate_cmd.thrust = msg.custom_cmd[id];

    // update command
    cmd = generate_cmd;
}

void custom_command_to_px4_msg(const CustomCommand &cmd, px4_cmd::Command &msg)
{
    // init
    px4_cmd::Command external_cmd;

    // basic info
    external_cmd.Mode = px4_cmd::Command::Move;
    external_cmd.Move_frame = cmd.frame_id;
    external_cmd.Move_mode = px4_cmd::Command::Custom_Command;
    // transform data
    // mode
    switch (cmd.mode)
    {
        case CommandMode::TargetLocal:
        {
            external_cmd.Custom_Command_Mode = px4_cmd::Command::Custom_Command_TargetLocal;
            break;
        }
        case CommandMode::TargetGlobal:
        {
            external_cmd.Custom_Command_Mode = px4_cmd::Command::Custom_Command_TargetGlobal;
            break;
        }
        case CommandMode::TargetAttitude:
        {
            external_cmd.Custom_Command_Mode = px4_cmd::Command::Custom_Command_TargetAttitude;
            break;
        }
    }

    // custom command - [position] 3 [velocity] 3 [accelerate] 3 [Forceflag Yaw YawRate] 3 [Attitude (quaternion) {xyzw}] 4 [BodyRate] 3 [Thrust] 1
    int id = 0;

    // position
    for (size_t i = 0; i < 3; i++)
    {
        external_cmd.custom_cmd[i + id] = cmd.position[i];
    }

    // velocity
    id += 3;
    for (size_t i = 0; i < 3; i++)
    {
        external_cmd.custom_cmd[i + id] = cmd.velocity[i];
    }

    // accelerate
    id += 3;
    for (size_t i = 0; i < 3; i++)
    {
        external_cmd.custom_cmd[i + id] = cmd.accelerate[i];
    }

    // Force flag
    id += 3;
    external_cmd.custom_cmd[id] = cmd.force_flag;

    // yaw
    id += 1;
    external_cmd.custom_cmd[id] = cmd.yaw;

    // yaw rate
    id += 1;
    external_cmd.custom_cmd[id] = cmd.yaw_rate;

    // attitude
    id += 1;
    for (size_t i = 0; i < 4; i++)
    {
        external_cmd.custom_cmd[i + id] = cmd.attitude[i];
    }

    // attitude rate
    id += 4;
    for (size_t i = 0; i < 3; i++)
    {
        external_cmd.custom_cmd[i + id] = cmd.attitude_rate[i];
    }

    // thrust
    id += 3;
    external_cmd.custom_cmd[id] = cmd.thrust;
    // end transform

    // update message command
    msg = external_cmd;
}

bool check_custom_command(CustomCommand cmd, int vehicle_type, std::string &error)
{
    error.clear();
    const std::string MODE_ERROR = "[Error] Custom Command Mode Error!";
    const std::string XY_NOT_PAIR = "[Error] Custom Command X - Y Command need pair!";
    const std::string Z_NOT_EXSIT = "[Error] Z Command Not Exist!";
    const std::string XYZ_NOT_COMBINE = "[Error] Current Mode Not Support position + velocity + accelerate combine command!";
    const std::string ATTITUDE_MODE_ERROR = "[Error] TargetAttitude Mode only support Attitude + Thrust or Attitude Rate + Thrust command!";
    const std::string FW_MODE_ERROR = "[Error] FixWing Only support Position Command in Current Mode!";
    const std::string FW_TYPEMASK_ERROR = "[Error] FixWing Position TypeMask Error in Current Mode!";
    const std::string FRAME_ERROR = "[Error] Frame Is Not Supported in Current Command Mode";
    const std::string FRAME_NOSET_ERROR = "[Error] Frame Is Not Set!";
    const std::string BODY_FRAME_WITH_POSITION_ERROR = "[Error] Body Frame Is Not Supported Position Setpoint";
    switch (cmd.mode)
    {
        case CommandMode::TargetLocal:
        {
            // Not Set Frame
            if (isnan(cmd.frame_id))
            {
                error = FRAME_NOSET_ERROR;
                return false;
            }

            // Not Support Frame
            if (cmd.frame_id == px4_cmd::Command::GLOBAL)
            {
                error = FRAME_ERROR;
                return false;
            }

            // fixwing only support position input
            if (vehicle_type == px4_cmd::Command::FixWing)
            {
                if (isnan(cmd.position[0]) || isnan(cmd.position[1]) || isnan(cmd.position[2]))
                {
                    error = FW_MODE_ERROR;
                    return false;
                }

                // typemask check
                if (isnanl(cmd.fw_mode))
                {
                    error = FW_TYPEMASK_ERROR;
                    return false;
                }
                switch (cmd.fw_mode)
                {
                    case FixWingPositionMode::GlidingMode:
                        break;

                    case FixWingPositionMode::IdleMode:
                        break;

                    case FixWingPositionMode::LandMode:
                        break;

                    case FixWingPositionMode::LoiterMode:
                        break;

                    case FixWingPositionMode::TakeoffMode:
                        break;

                    default:
                    {
                        error = FW_TYPEMASK_ERROR;
                        return false;
                    }
                }

                return true;
            }

            // x - y command need pair
            if (isnan(cmd.accelerate[0]) ^ isnan(cmd.accelerate[1]))
            {
                error = XY_NOT_PAIR;
                return false;
            }
            if (isnan(cmd.velocity[0]) ^ isnan(cmd.velocity[1]))
            {
                error = XY_NOT_PAIR;
                return false;
            }
            if (isnan(cmd.position[0]) ^ isnan(cmd.position[1]))
            {
                error = XY_NOT_PAIR;
                return false;
            }

            // z command need exist
            if (isnan(cmd.accelerate[2]) && isnan(cmd.velocity[2]) && isnan(cmd.position[2]))
            {
                error = Z_NOT_EXSIT;
                return false;
            }

            // Body frame can not with Position Command
            if ((cmd.frame_id == px4_cmd::Command::BODY) && ((!isnan(cmd.position[0])) || (!isnan(cmd.position[1])) || (!isnan(cmd.position[2]))))
            {
                error = BODY_FRAME_WITH_POSITION_ERROR;
                return false;
            }

            break;
        }

        case CommandMode::TargetGlobal:
        {
            // Not Set Frame
            if (isnan(cmd.frame_id))
            {
                error = FRAME_NOSET_ERROR;
                return false;
            }

            // Not Support Frame
            if (cmd.frame_id == px4_cmd::Command::ENU)
            {
                error = FRAME_ERROR;
                return false;
            }

            // fixwing only support position input
            if (vehicle_type == px4_cmd::Command::FixWing)
            {
                if (isnan(cmd.position[0]) || isnan(cmd.position[1]) || isnan(cmd.position[2]))
                {
                    error = FW_MODE_ERROR;
                    return false;
                }

                // typemask check
                if (isnanl(cmd.fw_mode))
                {
                    error = FW_TYPEMASK_ERROR;
                    return false;
                }
                switch (cmd.fw_mode)
                {
                    case FixWingPositionMode::IdleMode:
                        break;

                    case FixWingPositionMode::LandMode:
                        break;

                    case FixWingPositionMode::LoiterMode:
                        break;

                    case FixWingPositionMode::TakeoffMode:
                        break;

                    default:
                    {
                        error = FW_TYPEMASK_ERROR;
                        return false;
                    }
                }

                return true;
            }
            // x - y pair need pair
            if (isnan(cmd.accelerate[0]) ^ isnan(cmd.accelerate[1]))
            {
                error = XY_NOT_PAIR;
                return false;
            }
            if (isnan(cmd.velocity[0]) ^ isnan(cmd.velocity[1]))
            {
                error = XY_NOT_PAIR;
                return false;
            }
            if (isnan(cmd.position[0]) ^ isnan(cmd.position[1]))
            {
                error = XY_NOT_PAIR;
                return false;
            }

            // z command need exist
            if (isnan(cmd.accelerate[0]) && isnan(cmd.velocity[1]) && isnan(cmd.position[2]))
            {
                error = Z_NOT_EXSIT;
                return false;
            }

            // not support p + v + a mode
            bool p_exist = !((isnan(cmd.position[0])) && (isnan(cmd.position[1])) && (isnan(cmd.position[2])));
            bool v_exist = !((isnan(cmd.velocity[0])) && (isnan(cmd.velocity[1])) && (isnan(cmd.velocity[2])));
            bool a_exist = !((isnan(cmd.accelerate[0])) && (isnan(cmd.accelerate[1])) && (isnan(cmd.accelerate[2])));
            if ((p_exist || v_exist) && a_exist)
            {
                error = XYZ_NOT_COMBINE;
                return false;
            }

            // Body frame can not with Position Command
            if ((cmd.frame_id == px4_cmd::Command::BODY) && ((!isnan(cmd.position[0])) || (!isnan(cmd.position[1])) || (!isnan(cmd.position[2]))))
            {
                error = BODY_FRAME_WITH_POSITION_ERROR;
                return false;
            }

            break;
        }

        case CommandMode::TargetAttitude:
        {
            bool mode1 = (isnan(cmd.attitude[0]) || isnan(cmd.attitude[1]) || isnan(cmd.attitude[2]) || isnan(cmd.attitude[3]));
            bool mode2 = (isnan(cmd.attitude_rate[0]) || isnan(cmd.attitude_rate[1]) || isnan(cmd.attitude_rate[2]));

            // only support mode1 and mode2
            if ((mode1 && mode2) || isnan(cmd.thrust))
            {
                error = ATTITUDE_MODE_ERROR;
                return false;
            }

            // only support one mode
            if (!mode1 && !mode2)
            {
                error = ATTITUDE_MODE_ERROR;
                return false;
            }

            break;
        }

        default:
        {
            std::cout << MODE_ERROR << std::endl;
            return false;
        }
    }
    return true;
}