#!/usr/bin/env python3
"""Intel RealSense D435i — IMU bring-up with Madgwick orientation fusion.

Runs the RealSense node directly (like c1.launch.py runs sllidar) streaming its
6-DOF IMU (accel + gyro) as a single united topic, then runs imu_filter_madgwick
to add an orientation quaternion. Output: sensor_msgs/Imu on /imu/data.

Notes / gotchas baked into the defaults:
  * The D435i IMU has NO magnetometer (BMI055 = accel + gyro only), so Madgwick
    runs with use_mag:=false. Roll/pitch are gravity-referenced and stable;
    absolute yaw is NOT observable and will slowly drift. Fuse this with wheel/
    lidar odometry (e.g. robot_localization) for a usable heading.
  * unite_imu_method=2 (linear_interpolation) merges the separate accel (~63 Hz)
    and gyro (~200 Hz) streams into one /camera/camera/imu topic. Without it the
    wrapper publishes /accel/sample and /gyro/sample separately and Madgwick has
    nothing to subscribe to.
  * Depth / color / pointcloud are OFF by default — this file is for IMU bring-up
    and keeps USB3/CPU load low on the Pi. floor_scan.launch.py turns depth +
    pointcloud on. Direct Node + a params dict is used (not an rs_launch include)
    so dotted params like pointcloud.enable are set deterministically.

The base_link -> camera_link transform comes from the URDF via
robot_state_publisher, NOT from here. The RealSense node only publishes its own
internal camera/IMU frames.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Node namespace='camera' + name='camera' -> topics under /camera/camera/...
IMU_TOPIC = "/camera/camera/imu"


def generate_launch_description():
    enable_depth = LaunchConfiguration("enable_depth", default="false")
    enable_color = LaunchConfiguration("enable_color", default="false")
    enable_pointcloud = LaunchConfiguration("enable_pointcloud", default="false")

    realsense = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="camera",
        name="camera",
        output="screen",
        parameters=[{
            # Frame prefix MUST match the URDF sensor_d435i macro (name="d435i")
            # so the cloud/IMU frames (d435i_*_optical_frame) connect to base_link
            # via robot_state_publisher. The default "camera" left them in a
            # separate TF tree, so depth_floor_scan could not transform obstacles
            # into base_link and /camera/scan stayed empty.
            "camera_name": "d435i",
            # Low depth res/rate: floor scan needs only coarse ~8Hz depth; cuts CPU.
            "depth_module.depth_profile": "424x240x6",   # 15 fps cost 55% of a core (realsense+floor_scan) for a 0.2 m/s robot
            # Low color res/rate for the same reason: enough for a teleop/monitor
            # view, cheap on USB + CPU. Compressed transport only costs CPU
            # while something actually subscribes.
            "rgb_camera.color_profile": "424x240x15",
            "enable_gyro": True,
            "enable_accel": True,
            # 0-None, 1-copy, 2-linear_interpolation. 2 = one fused imu topic.
            "unite_imu_method": 2,
            "enable_depth": ParameterValue(enable_depth, value_type=bool),
            "enable_color": ParameterValue(enable_color, value_type=bool),
            "enable_infra1": False,
            "enable_infra2": False,
            # NOTE: this ARM/Pi build names the pointcloud filter "pointcloud__neon_"
            # (NEON-optimised), so the real param is pointcloud__neon_.enable. We set
            # both names so the launch also works on a plain x86 "pointcloud" build;
            # the realsense node silently ignores the one it doesn't declare.
            "pointcloud.enable": ParameterValue(enable_pointcloud, value_type=bool),
            "pointcloud__neon_.enable": ParameterValue(enable_pointcloud, value_type=bool),
            # Still emit cloud points where there's no color texture.
            # hardware-reset the camera at startup: the D435i firmware wedges
            # (depth stream opens but "Frames didn't arrived" forever) and
            # survives node restarts -- only a reset clears it (2026-06-10).
            "initial_reset": True,
            "pointcloud.allow_no_texture_points": True,
            "pointcloud__neon_.allow_no_texture_points": True,
            # 16UC1 depth cannot be JPEG-compressed; whitelist transports so a
            # mis-configured RViz 'compressed' subscription cannot spam
            # "[16UC1] is not a color format" — only raw + compressedDepth exist.
            "camera.depth.image_rect_raw.enable_pub_plugins": [
                "image_transport/raw", "image_transport/compressedDepth"],
        }],
    )

    madgwick = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter_madgwick",
        output="screen",
        parameters=[{
            "use_mag": False,            # D435i has no magnetometer
            "world_frame": "enu",        # REP-103 (x east, y north, z up)
            "publish_tf": False,         # orientation only, don't fight the URDF TF
        }],
        remappings=[
            ("imu/data_raw", IMU_TOPIC),
            ("imu/data", "/imu/data"),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("enable_depth", default_value=enable_depth),
        DeclareLaunchArgument("enable_color", default_value=enable_color),
        DeclareLaunchArgument("enable_pointcloud", default_value=enable_pointcloud),
        realsense,
        madgwick,
    ])
