# -*- coding: utf-8 -*-
# Copyright (c) 2023 易鹏 中山大学航空航天学院
# Copyright (c) 2023 Peng Yi, Sun Yat-Sen University, School of Aeronautics and Astronautics

from enum import Enum
from typing import List, Optional
from px4_cmd.msg import Command
from numpy import NaN, isnan

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

def reset_custom_command() -> CustomCommand:
    """
    reset custom command to default value (NAN), everytime generate custom command need this operate
    """
    return CustomCommand()

def px4_msg_to_custom_command(msg: Command) -> CustomCommand:
    """transfer px4 command message to custom command struct

    Args:
        msg (Command): px4 command message

    Returns:
        CustomCommand: transformed custom command struct
    """
    # new command
    generate_cmd = CustomCommand()

    # mode
    if (msg.Custom_Command_Mode == Command.Custom_Command_TargetLocal):
        generate_cmd.mode = CommandMode.TargetLocal
    elif (msg.Custom_Command_Mode == Command.Custom_Command_TargetGlobal):
        generate_cmd.mode = CommandMode.TargetGlobal
    elif (msg.Custom_Command_Mode == Command.Custom_Command_TargetAttitude):
        generate_cmd.mode = CommandMode.TargetAttitude
    
    # frame
    generate_cmd.frame_id = msg.Move_frame

    # convert data
    id: int = 0
    # position convert [id 0 - 2]
    for i in range(3):
        generate_cmd.position[i] = msg.custom_cmd[i]

    # velocity convert [id 3 - 5]
    id += 3
    for i in range(3):
        generate_cmd.velocity[i] = msg.custom_cmd[i + id]

    # accelerate convert [id 6 - 8]
    id += 3
    for i in range(3):
        generate_cmd.accelerate[i] = msg.custom_cmd[i + id]

    # Force flag [id 9]
    id += 3
    generate_cmd.force_flag = msg.custom_cmd[id]

    # yaw convert [id 10]
    id += 1
    generate_cmd.yaw = msg.custom_cmd[id]

    # yaw rate convert [id 11]
    id += 1
    generate_cmd.yaw_rate = msg.custom_cmd[id]

    # Attitude (quaternion) convert [id 12 - 15]
    id += 1
    for i in range(4):
        generate_cmd.attitude[i] = msg.custom_cmd[i + id]

    # Attitude (quaternion) convert [id 16 - 18]
    id += 4
    for i in range(3):
        generate_cmd.attitude_rate[i] = msg.custom_cmd[i + id]

    # Thrust convert [id 19]
    id += 3
    generate_cmd.thrust = msg.custom_cmd[id]

    # return command
    return generate_cmd


def custom_command_to_px4_msg(cmd: CustomCommand) -> Command:
    """transfrom custom command struct to px4 command message

    Args:
        cmd (CustomCommand): custom command struct 

    Returns:
        Command: transformed px4 command message
    """
    # new command
    external_cmd = Command()

    # basic info
    external_cmd.Mode = Command.Move
    external_cmd.Move_frame = cmd.frame_id
    external_cmd.Move_mode = Command.Custom_Command
    # transform data
    # mode
    if (cmd.mode == CommandMode.TargetLocal):
        external_cmd.Custom_Command_Mode = Command.Custom_Command_TargetLocal
    elif (cmd.mode == CommandMode.TargetGlobal):
        external_cmd.Custom_Command_Mode = Command.Custom_Command_TargetGlobal
    elif (cmd.mode == CommandMode.TargetAttitude):
        external_cmd.Custom_Command_Mode = Command.Custom_Command_TargetAttitude
    
    # custom command - [position] 3 [velocity] 3 [accelerate] 3 [Forceflag Yaw YawRate] 3 [Attitude (quaternion) {xyzw}] 4 [BodyRate] 3 [Thrust] 1
    id: int = 0

    # position
    for i in range(3):
        external_cmd.custom_cmd[i + id] = cmd.position[i]

    # velocity
    id += 3
    for i in range(3):
        external_cmd.custom_cmd[i + id] = cmd.velocity[i]

    # accelerate
    id += 3
    for i in range(3):
        external_cmd.custom_cmd[i + id] = cmd.accelerate[i]

    # Force flag
    id += 3
    external_cmd.custom_cmd[id] = cmd.force_flag

    # yaw
    id += 1
    external_cmd.custom_cmd[id] = cmd.yaw

    # yaw rate
    id += 1
    external_cmd.custom_cmd[id] = cmd.yaw_rate

    # attitude
    id += 1
    for i in range(4):
        external_cmd.custom_cmd[i + id] = cmd.attitude[i]

    # attitude rate
    id += 4
    for i in range(3):
        external_cmd.custom_cmd[i + id] = cmd.attitude_rate[i]

    # thrust
    id += 3
    external_cmd.custom_cmd[id] = cmd.thrust
    # end transform

    # return message command
    return external_cmd


def check_custom_command(cmd: CustomCommand, vehicle_type: int = Command.Multicopter) -> Optional[str]:
    """check custom command if correct

    Args:
        cmd (CustomCommand): custom command struct
        vehicle_type (int, optional): Vehicle type in px4_cmd message : Command.Multicopter / Command.FixWing. Defaults to Command.Multicopter.

    Returns:
        str | None: error inforamtion string, if no error return None
    """
    # init error str
    error = None
    MODE_ERROR = "[Error] Custom Command Mode Error!"
    XY_NOT_PAIR = "[Error] Custom Command X - Y Command need pair!"
    Z_NOT_EXSIT = "[Error] Z Command Not Exist!"
    XYZ_NOT_COMBINE = "[Error] Current Mode Not Support position + velocity + accelerate combine command!"
    ATTITUDE_MODE_ERROR = "[Error] TargetAttitude Mode only support Attitude + Thrust or Attitude Rate + Thrust command!"
    FW_MODE_ERROR = "[Error] FixWing Only support Position Command in Current Mode!"
    FW_TYPEMASK_ERROR = "[Error] FixWing Position TypeMask Error in Current Mode!"
    FRAME_ERROR = "[Error] Frame Is Not Supported in Current Command Mode"
    FRAME_NOSET_ERROR = "[Error] Frame Is Not Set!"
    BODY_FRAME_WITH_POSITION_ERROR = "[Error] Body Frame Is Not Supported Position Setpoint"

    # mode switch
    if (cmd.mode == CommandMode.TargetLocal):
        # Not Set Frame
        if (isnan(cmd.frame_id)):
            error = FRAME_NOSET_ERROR
            return error
        
        # Not Support Frame
        if (cmd.frame_id == Command.GLOBAL):
            error = FRAME_ERROR
            return error
        
        # fixwing only support position input
        if (vehicle_type == Command.FixWing):
            if (isnan(cmd.position[0]) or isnan(cmd.position[1]) or isnan(cmd.position[2])):
                error = FW_MODE_ERROR
                return error
            
            # typemask check
            if (isnan(cmd.fw_mode)):
                error = FW_TYPEMASK_ERROR
                return error
            if (cmd.fw_mode != FixWingPositionMode.GlidingMode or 
                cmd.fw_mode != FixWingPositionMode.IdleMode or
                cmd.fw_mode != FixWingPositionMode.LandMode or
                cmd.fw_mode != FixWingPositionMode.LoiterMode or
                cmd.fw_mode != FixWingPositionMode.TakeoffMode):
                error = FW_TYPEMASK_ERROR
                return error
            
            return error
        
        #  x - y command need pair
        if (isnan(cmd.accelerate[0]) ^ isnan(cmd.accelerate[1])):
            error = XY_NOT_PAIR
            return error
        if (isnan(cmd.velocity[0]) ^ isnan(cmd.velocity[1])):
            error = XY_NOT_PAIR
            return error
        if (isnan(cmd.position[0]) ^ isnan(cmd.position[1])):
            error = XY_NOT_PAIR
            return error

        # z command need exist
        if (isnan(cmd.accelerate[2]) and isnan(cmd.velocity[2]) and isnan(cmd.position[2])):
            error = Z_NOT_EXSIT
            return error

        # Body frame can not with Position Command
        if ((cmd.frame_id == Command.BODY) and ((not isnan(cmd.position[0])) or (not isnan(cmd.position[1])) or (not isnan(cmd.position[2])))):
            error = BODY_FRAME_WITH_POSITION_ERROR
            return error
        
        return error

    elif (cmd.mode == CommandMode.TargetGlobal):
        # Not Set Frame
        if (isnan(cmd.frame_id)):
            error = FRAME_ERROR
            return error
        
        # Not Support Frame
        if (cmd.frame_id == Command.ENU):
            error = FRAME_ERROR
            return error
        
        # fixwing only support position input
        if (vehicle_type == Command.FixWing):
            if (isnan(cmd.position[0]) or isnan(cmd.position[1]) or isnan(cmd.position[2])):
                error = FW_MODE_ERROR
                return error

            # typemask check
            if (isnan(cmd.fw_mode)):
                error = FW_TYPEMASK_ERROR
                return error
            if (cmd.fw_mode != FixWingPositionMode.GlidingMode or
                cmd.fw_mode != FixWingPositionMode.IdleMode or
                cmd.fw_mode != FixWingPositionMode.LandMode or
                cmd.fw_mode != FixWingPositionMode.LoiterMode or
                    cmd.fw_mode != FixWingPositionMode.TakeoffMode):
                error = FW_TYPEMASK_ERROR
                return error

            return error

        #  x - y command need pair
        if (isnan(cmd.accelerate[0]) ^ isnan(cmd.accelerate[1])):
            error = XY_NOT_PAIR
            return error
        if (isnan(cmd.velocity[0]) ^ isnan(cmd.velocity[1])):
            error = XY_NOT_PAIR
            return error
        if (isnan(cmd.position[0]) ^ isnan(cmd.position[1])):
            error = XY_NOT_PAIR
            return error

        # z command need exist
        if (isnan(cmd.accelerate[2]) and isnan(cmd.velocity[2]) and isnan(cmd.position[2])):
            error = Z_NOT_EXSIT
            return error
        
        # not support p + v + a mode
        p_exist: bool = not ((isnan(cmd.position[0])) and (isnan(cmd.position[1])) and (isnan(cmd.position[2])))
        v_exist: bool = not ((isnan(cmd.velocity[0])) and (isnan(cmd.velocity[1])) and (isnan(cmd.velocity[2])))
        a_exist: bool = not ((isnan(cmd.accelerate[0])) and (isnan(cmd.accelerate[1])) and (isnan(cmd.accelerate[2])))
        if ((p_exist or v_exist) and a_exist):
            error = XYZ_NOT_COMBINE
            return error
        
        # Body frame can not with Position Command
        if ((cmd.frame_id == Command.BODY) and ((not isnan(cmd.position[0])) or (not isnan(cmd.position[1])) or (not isnan(cmd.position[2])))):
            error = BODY_FRAME_WITH_POSITION_ERROR
            return error

        return error
    
    elif (cmd.mode == CommandMode.TargetAttitude):
        mode1: bool = (isnan(cmd.attitude[0]) or isnan(cmd.attitude[1]) or isnan(cmd.attitude[2]) or isnan(cmd.attitude[3]))
        mode2: bool = (isnan(cmd.attitude_rate[0]) or isnan(cmd.attitude_rate[1]) or isnan(cmd.attitude_rate[2]))

        # only support mode1 and mode2
        if ((mode1 and mode2) or isnan(cmd.thrust)):
            error = ATTITUDE_MODE_ERROR
            return error
        
        # only support one mode
        if (not mode1 and not mode2):
            error = ATTITUDE_MODE_ERROR
            return error
    
    else:
        error = MODE_ERROR
        return error
    
    return error