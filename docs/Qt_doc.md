# Qt for PX4 Command Usage

## Installation

```bash
catkin_make
```

## Generator - Generate Launch File

### run

```bash
rosrun px4_cmd generator
```

### Usage
- Vehicle Type: Vehicle Model defined in PX4-Autopilot
- Sensor Type: Add sensor for Vehicle
- World Files: World Files in PX4-Autopilot
- Topic Type: 
   - uav_{ID} (Recommanded): e.g. uav_1, uav_2, ...
   - {Vehicle Type}_{ID}: e.g. iris_1, plane_2, ...
- Initial Pose: initial pose for model
   - x: initial position in x axis
   - y: initial position in y axis
   - z: initial position in z axis
   - R: initial eular angle in x axis
   - P: initial eular angle in y axis
   - Y: initial eular angle in z axis
- Gazebo GUI: if open gazebo GUI
- Output Dir: output directory for launch file
- Simulation Dir: Simulation directory, which includes log, PX4 setting etc.
   - Default (Optional): if true, default directory: ~/.ros/
- Add Vehicle: add vehicle of setting 
- Del Vehicle: deleta a vehicle chosen in the table
- Clear Vehcles: clear all vehicles in the table
- Sensors Setting: Set the parameter of all sensors type
- Load Launch: Load launch file generated by the programme before
- Generate Launch: Generate launch file in output dir.

## Monitor: Information of Simulation Vehicles
### run
```bash
rosrun px4_cmd monitor
```


## Controller: Control Simulation Vehicles
### run
```bash
rosrun px4_cmd controller
```