This directory is empty in the git repository.

`scripts/install_env.sh` and `scripts/clone_third_party.sh` clone these upstream projects into it:

- [rslidar_sdk](https://github.com/RoboSense-LiDAR/rslidar_sdk)
- [rslidar_msg](https://github.com/RoboSense-LiDAR/rslidar_msg)
- [ros2_calib](https://github.com/ika-rwth-aachen/ros2_calib)

Do not commit the clones. The E1R driver settings live in `src/e1r_calib_bringup/config/e1r_online.yaml`.
