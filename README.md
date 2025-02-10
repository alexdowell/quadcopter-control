# quadcopter-control
# Angle and Rate Controller

## Overview
This project implements a **PID controller** for **quadcopter attitude control**, specifically for **angular rates** and **pitch/roll angles**. The controller utilizes sensor data from an **AHRS (Attitude and Heading Reference System)** and **IMU (Inertial Measurement Unit)** to regulate vehicle motion using **ROS (Robot Operating System)**.

## Features
- 🛠️ **PID control** for:
  - Yaw rate
  - Pitch and roll angles
  - Angular rates for pitch and roll  
- 🎛 **Butterworth filtering** for sensor data noise reduction  
- 🔄 **Complementary filter** for sensor fusion  
- 🎮 **RC input scaling** for real-time user control  
- 📊 **Real-time logging** of control outputs  
- 🚨 **Failsafe mechanism** for motor shutdown  

## System Requirements
- **Python 3**
- **ROS (Robot Operating System)**
- **NumPy**
- **me457common ROS package** (for message definitions)

## Installation
Run the following commands:

    git clone https://github.com/alexdowell/your-repo-name.git
    cd your-repo-name
    pip install numpy rospy

## Usage
1. **Start ROS**:

       roscore

2. **Run the controller**:

       python3 controller.py

3. The script will:
   - Subscribe to **IMU & AHRS data** from ROS topics  
   - Process **RC inputs** for control  
   - Compute **PID control outputs**  
   - Publish **servo commands** to control the vehicle  

## ROS Topics
| Topic          | Type   | Description            |
|----------------|--------|------------------------|
| `/imupub`      | IMU    | IMU sensor data        |
| `/madgwickpub` | AHRS   | Attitude estimation    |
| `/rcpub`       | RC     | RC input values        |
| `/servocmd`    | Servo  | Motor control commands |

## Controller Details
### Control Gains
- **Yaw Rate:** `Kp = 0.5`, `Ki = 0.875`, `Kd = 0.0075`
- **Pitch/Roll Rate:** `Kp = 0.0666`, `Ki = 0`, `Kd = 0.0005`
- **Pitch/Roll Angle:** `Kp = 0.5442`, `Kd = 0.0017`

### Filtering
- **Low-pass Butterworth filter** for noise reduction  
- **High-pass filter** for drift correction  
- **Moving average filter** for additional smoothing  

## Safety Features
- **Kill switch** (RC Channel 4) to immediately stop motors  
- **Failsafe on shutdown**: Ensures all motor outputs are set to `0`  

## Logging
The script logs all control values to CSV in:

    /home/pi/catkin_ws/src/autophatros/scripts/logfiles/Alex/

Log format: `log_N_controller.csv` (auto-incrementing)

## Example Output
    throttle_SP: 85.0 angle_w: 1.234 rate_w: 1.567
    yaw_rate: 0.453 pitch: -0.567 roll: 0.789

## License
This project is open-source and can be used for research and development.
