# Eye-on-base UR robot - camera calibration package 
This package exposes a functionality to calibrate the ur robot with camera using AprilTag and ur_rtde in the eye-on-base setting.

## How to use
1. Mount AprilTag on the robot end-effector
2. Update `param/april_tag.yaml` file with the parameters of the used AprilTag
3. Update `param/defaults.param.yaml` file with the desired robot ip, prefix and save path
4. Set robot position such that AprilTag is visible to the camera
5. Launch calibration 
   ```
   ros2 launch robot_camera_calibration robot_camera_calibration.launch.py \ config_param_file:=./src/external/robot_camera_calibration/param/defaults.param.yaml
   ```