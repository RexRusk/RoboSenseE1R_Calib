# RoboSense E1R + RGB

Publish the E1R point cloud with the RoboSense SDK, calibrate a fixed lidar-to-camera extrinsic with [ros2_calib](https://github.com/ika-rwth-aachen/ros2_calib), and draw the current scan in a ground-aligned world frame.

Upstream trees under `src/third_party` are not modified. `rslidar_msg` is the message package the SDK needs in the same workspace. This package is `src/e1r_calib_bringup`.

Tested on Ubuntu 22.04 and ROS 2 Humble. The install script follows the machine architecture.

## Requirements

- Ubuntu 22.04
- A RoboSense E1R on the same subnet
- A USB camera, if you want images or the extrinsic

Factory lidar address: `192.168.1.200`. MSOP `6699`, DIFOP `7788`. Set the computer to `192.168.1.102/255.255.0.0`. Only one program may listen on UDP 6699. Stop RSView, or any other driver, before launching this one.

## Install

`src/third_party` is empty in this repository. The install script clones the RoboSense SDK, `rslidar_msg`, and `ros2_calib`, then builds them with this package.

```bash
git clone <repository-url> RoboSenseE1R_Calib
cd RoboSenseE1R_Calib
bash scripts/install_env.sh
```

The build does not create an `install` directory in the checkout. It writes `~/e1r_calib/install` on the Linux machine. In every new shell:

```bash
source ~/e1r_calib/install/setup.bash
source ~/e1r_calib/venv/bin/activate
```

If Humble is already installed, `bash scripts/finish_setup.sh` clones the upstream trees, builds, and creates the Python environment.

## View the cloud

The launch sets `ROS_LOCALHOST_ONLY=1` so RViz and the driver on this computer find each other.

Lidar frame. The cloud stays in `rslidar`. The grid is the lidar XY plane, not the floor. Topic: `/rslidar_points`.

```bash
ros2 launch e1r_calib_bringup calib_capture.launch.py use_camera:=false
```

World frame. The grid is the ground. Looking down lays the floor on that grid. Topic: `/rslidar_points_world`. Fixed frame: `base`.

```bash
ros2 launch e1r_calib_bringup calib_capture.launch.py frame:=world use_camera:=false
```

Pitch and roll come from the ground each frame. Yaw is the horizontal motion between frames. The pivot stays at zero until the ground and one wall have both been seen from several angles. The terminal prints `pivot` when that happens.

RViz on this OpenGL setup draws **Flat Squares**. Style **Points** can show an empty view.

## Camera on Linux

On Linux the camera is read with `v4l2`, which is the launch default. Use a mode the camera actually lists. The packaged intrinsics are for 1024×576, and the yaml must match `image_width` and `image_height`.

```bash
ros2 launch e1r_calib_bringup calib_capture.launch.py \
  camera_transport:=v4l2 \
  video_device:=/dev/video0 \
  image_width:=1024 image_height:=576
```

`/camera/image_raw` and `/camera/camera_info` are published. With the default extrinsic, points that project into the image are colored on `/rslidar_points_color`. The world-frame view still shows `/rslidar_points_world` by intensity.

Find the device with `v4l2-ctl --list-devices`.

## Intrinsics

The checked-in USB camera used a 4×6 inner-corner board and 1024×576 images. Square size does not change the intrinsics. For another camera, solve a new yaml at the resolution you will publish:

```bash
python scripts/zhang_calib.py \
  --images "/path/to/WIN_*.jpg" \
  --corners 4x6 \
  --output src/e1r_calib_bringup/config/usb_camera_1024x576.yaml
```

`scripts/zhang_fixk3.py` is the same command with k3 fixed at 0. A yaml is valid only for the camera and resolution that produced it. Rebuild after replacing a file under `config/`, then pass it with `camera_info_url:=file:///absolute/path.yaml`.

## Record a calibration bag

Lock the gimbal. The camera and the lidar must see the same corners.

```bash
ros2 launch e1r_calib_bringup calib_capture.launch.py \
  camera_transport:=v4l2 \
  record:=true use_rviz:=false
```

The bag is `~/e1r_calib/bags/calib_<timestamp>/` (`.mcap` and `metadata.yaml`). Topics: `/rslidar_points`, `/camera/image_raw`, `/camera/camera_info`.

## Extrinsic

```bash
bash scripts/open_ros2_calib.sh
```

In the GUI: LiDAR-to-Camera, open the `.mcap`, select those three topics, and mark at least four correspondences. More than ten, not all on one plane, is better. The exported rigid transform is valid only for the locked pose in that bag.

## Limitations

Recovering a fixed point from the difference between two pitch and yaw views is not implemented. The world frame uses the ground normal for pitch, roll, and height, and frame-to-frame alignment for yaw. The pivot is not solved from those two views.

A robot dog carrying the gimbal is not handled well. The ground fit and the yaw alignment both assume the base is still. While the dog moves, points in the world frame shake.

## Layout

| Path | Role |
|---|---|
| `src/third_party/rslidar_sdk` | RoboSense ROS 2 driver |
| `src/third_party/rslidar_msg` | Messages required to build the driver |
| `src/third_party/ros2_calib` | Offline extrinsic GUI |
| `src/e1r_calib_bringup` | E1R config, launch, world-frame node |
| `scripts/install_env.sh` | Apt packages, clone, build, Python env |
| `scripts/build.sh` | `colcon` into `~/e1r_calib` |
