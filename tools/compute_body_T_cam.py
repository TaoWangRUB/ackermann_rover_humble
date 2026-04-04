#!/usr/bin/env python3
"""
Compute correct VINS body_T_cam by replicating the official T265 ROS driver's
exact coordinate conversion logic from t265_realsense_node.cpp and
base_realsense_node.cpp.

The T265 driver:
1. Gets SDK extrinsics via sensor.get_extrinsics_to(pose_base_profile)
2. Converts rotation: Q_ros = Q_opt * Q_sdk * Q_opt^-1
   where Q_opt = rpy(M_PI/2, 0.0, -M_PI/2)  [T265-specific, different from D4xx]
3. Converts translation: ros_x=sdk_z, ros_y=-sdk_x, ros_z=-sdk_y
   (done in publish_static_tf)
4. Publishes: base_frame -> sensor_frame  (4x4 with Q_ros, t_ros)
5. Publishes: sensor_frame -> sensor_optical_frame  (pure rotation Q_opt)

For VINS body_T_cam, we need: T(imu_optical_frame -> fisheye_optical_frame)
"""

import numpy as np
from scipy.spatial.transform import Rotation

# ============================================================
# Hardware extrinsics from rs-enumerate-devices -c
# Convention: p_to = R * p_from + t
# ============================================================

# Gyro -> Fisheye1 (SDK coords)
R_gyro_to_fish1_sdk = np.array([
    [-0.999949, -0.00511504,  0.00871749],
    [ 0.00508425, -0.999981, -0.00355086],
    [ 0.00873548, -0.00350636,  0.999956 ]
])
t_gyro_to_fish1_sdk = np.array([0.0107, -0.0001, -0.0001])

# Gyro -> Fisheye2 (SDK coords)
R_gyro_to_fish2_sdk = np.array([
    [-0.999978, -0.00311985,  0.00592826],
    [ 0.00309844, -0.999989, -0.00361781],
    [ 0.00593948, -0.00359937,  0.999976 ]
])
t_gyro_to_fish2_sdk = np.array([-0.0528, 0.0002, 0.0003])

# Gyro -> Pose (SDK coords)
R_gyro_to_pose_sdk = np.array([
    [-1.0,  0.0,  0.0],
    [ 0.0,  1.0,  0.0],
    [ 0.0,  0.0, -1.0]
])
t_gyro_to_pose_sdk = np.array([-0.021, 0.0, 0.0])

# ============================================================
# Derive Fish1->Pose and Fish2->Pose extrinsics (what the driver uses)
# The driver calls: sensor.get_extrinsics_to(pose_base_profile)
# ============================================================

R_fish1_to_pose_sdk = R_gyro_to_pose_sdk @ np.linalg.inv(R_gyro_to_fish1_sdk)
t_fish1_to_pose_sdk = -R_gyro_to_pose_sdk @ np.linalg.inv(R_gyro_to_fish1_sdk) @ t_gyro_to_fish1_sdk + t_gyro_to_pose_sdk

R_fish2_to_pose_sdk = R_gyro_to_pose_sdk @ np.linalg.inv(R_gyro_to_fish2_sdk)
t_fish2_to_pose_sdk = -R_gyro_to_pose_sdk @ np.linalg.inv(R_gyro_to_fish2_sdk) @ t_gyro_to_fish2_sdk + t_gyro_to_pose_sdk

print("=== SDK Extrinsics (sensor.get_extrinsics_to(pose)) ===")
print(f"Fish1->Pose R:\n{R_fish1_to_pose_sdk}")
print(f"Fish1->Pose t: {t_fish1_to_pose_sdk}")
print(f"Fish2->Pose R:\n{R_fish2_to_pose_sdk}")
print(f"Fish2->Pose t: {t_fish2_to_pose_sdk}")
print(f"Gyro->Pose R:\n{R_gyro_to_pose_sdk}")
print(f"Gyro->Pose t: {t_gyro_to_pose_sdk}")

# ============================================================
# T265 driver coordinate conversion (from t265_realsense_node.cpp)
# ============================================================

# T265-specific optical quaternion: rpy(M_PI/2, 0.0, -M_PI/2)
R_opt = Rotation.from_euler('xyz', [np.pi/2, 0.0, -np.pi/2]).as_matrix()
print(f"\nR_opt (T265 optical, rpy=pi/2,0,-pi/2):\n{R_opt}")

def convert_sdk_to_ros_tf(R_sdk, t_sdk):
    """
    Replicate the T265 driver's calcAndPublishStaticTransform for non-POSE.
    
    Q_ros = Q_opt * Q_sdk * Q_opt^-1  (rotation conjugation)
    t_ros = (sdk_z, -sdk_x, -sdk_y)   (from publish_static_tf)
    Published as: base_frame -> sensor_frame
    """
    R_ros = R_opt @ R_sdk @ R_opt.T
    t_ros = np.array([t_sdk[2], -t_sdk[0], -t_sdk[1]])
    return R_ros, t_ros

print("\n=== Driver TF: base_frame -> sensor_frame (non-optical) ===")

R_base_fish1, t_base_fish1 = convert_sdk_to_ros_tf(R_fish1_to_pose_sdk, t_fish1_to_pose_sdk)
print(f"base->fish1_frame R:\n{R_base_fish1}")
print(f"base->fish1_frame t: {t_base_fish1}")

R_base_fish2, t_base_fish2 = convert_sdk_to_ros_tf(R_fish2_to_pose_sdk, t_fish2_to_pose_sdk)
print(f"base->fish2_frame R:\n{R_base_fish2}")
print(f"base->fish2_frame t: {t_base_fish2}")

R_base_gyro, t_base_gyro = convert_sdk_to_ros_tf(R_gyro_to_pose_sdk, t_gyro_to_pose_sdk)
print(f"base->gyro_frame R:\n{R_base_gyro}")
print(f"base->gyro_frame t: {t_base_gyro}")

# ============================================================
# Build full 4x4 TF chain: imu_optical -> fisheye_optical
# ============================================================

def make_tf44(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

T_base_fish1 = make_tf44(R_base_fish1, t_base_fish1)
T_fish1_fish1opt = make_tf44(R_opt, np.zeros(3))

T_base_fish2 = make_tf44(R_base_fish2, t_base_fish2)
T_fish2_fish2opt = make_tf44(R_opt, np.zeros(3))

T_base_gyro = make_tf44(R_base_gyro, t_base_gyro)
T_gyro_gyroopt = make_tf44(R_opt, np.zeros(3))

# Full chain: base -> sensor_optical
T_base_fish1opt = T_base_fish1 @ T_fish1_fish1opt
T_base_fish2opt = T_base_fish2 @ T_fish2_fish2opt
T_base_gyroopt = T_base_gyro @ T_gyro_gyroopt

print("\n=== Full TF: base_frame -> optical_frame (4x4) ===")
print(f"T_base_fish1opt:\n{T_base_fish1opt}")
print(f"T_base_fish2opt:\n{T_base_fish2opt}")
print(f"T_base_imuopt:\n{T_base_gyroopt}")

# ============================================================
# VINS body_T_cam: imu_optical -> fisheye_optical
# p_imuopt = body_T_cam * p_fishopt
# ============================================================

T_imuopt_fish1opt = np.linalg.inv(T_base_gyroopt) @ T_base_fish1opt
T_imuopt_fish2opt = np.linalg.inv(T_base_gyroopt) @ T_base_fish2opt

print("\n" + "="*60)
print("=== VINS body_T_cam (imu_optical -> fisheye_optical) ===")
print("="*60)
print(f"\nbody_T_cam0 (imu_opt -> fish1_opt):\n{T_imuopt_fish1opt}")
print(f"\nbody_T_cam1 (imu_opt -> fish2_opt):\n{T_imuopt_fish2opt}")

# ============================================================
# Compare with raw SDK (currently in config)
# ============================================================

print("\n" + "="*60)
print("=== Raw SDK extrinsic (currently in config) ===")
print("="*60)
T_raw_cam0 = make_tf44(R_gyro_to_fish1_sdk, t_gyro_to_fish1_sdk)
T_raw_cam1 = make_tf44(R_gyro_to_fish2_sdk, t_gyro_to_fish2_sdk)
print(f"\nRaw Gyro->Fish1:\n{T_raw_cam0}")
print(f"\nRaw Gyro->Fish2:\n{T_raw_cam1}")

print(f"\nDiff cam0 (correct - raw):\n{T_imuopt_fish1opt - T_raw_cam0}")
print(f"\nDiff cam1 (correct - raw):\n{T_imuopt_fish2opt - T_raw_cam1}")

# ============================================================
# Compare with URDF chain (approximate)
# ============================================================
print("\n" + "="*60)
print("=== URDF chain (approximate, from _t265.urdf.xacro) ===")
print("="*60)

R_link_gyro_urdf = Rotation.from_euler('xyz', [0, 0, np.pi]).as_matrix()
t_link_gyro_urdf = np.array([0, 0.021, 0])
R_gyro_imuopt_urdf = Rotation.from_euler('xyz', [np.pi/2, 0, -np.pi/2]).as_matrix()

R_link_fish1_urdf = Rotation.from_euler('xyz', [-np.pi, 0, -np.pi]).as_matrix()
t_link_fish1_urdf = np.array([0, 0.032, 0])
R_fish1_fish1opt_urdf = Rotation.from_euler('xyz', [np.pi/2, 0, -np.pi/2]).as_matrix()

R_link_fish2_urdf = Rotation.from_euler('xyz', [-np.pi, 0, -np.pi]).as_matrix()
t_link_fish2_urdf = np.array([0, -0.032, 0])
R_fish2_fish2opt_urdf = Rotation.from_euler('xyz', [np.pi/2, 0, -np.pi/2]).as_matrix()

T_link_imuopt_urdf = make_tf44(R_link_gyro_urdf, t_link_gyro_urdf) @ make_tf44(R_gyro_imuopt_urdf, np.zeros(3))
T_link_fish1opt_urdf = make_tf44(R_link_fish1_urdf, t_link_fish1_urdf) @ make_tf44(R_fish1_fish1opt_urdf, np.zeros(3))
T_link_fish2opt_urdf = make_tf44(R_link_fish2_urdf, t_link_fish2_urdf) @ make_tf44(R_fish2_fish2opt_urdf, np.zeros(3))

T_imuopt_fish1opt_urdf = np.linalg.inv(T_link_imuopt_urdf) @ T_link_fish1opt_urdf
T_imuopt_fish2opt_urdf = np.linalg.inv(T_link_imuopt_urdf) @ T_link_fish2opt_urdf

print(f"\nURDF body_T_cam0:\n{T_imuopt_fish1opt_urdf}")
print(f"\nURDF body_T_cam1:\n{T_imuopt_fish2opt_urdf}")

print(f"\nDiff (hardware - URDF) cam0:\n{T_imuopt_fish1opt - T_imuopt_fish1opt_urdf}")
print(f"\nDiff (hardware - URDF) cam1:\n{T_imuopt_fish2opt - T_imuopt_fish2opt_urdf}")

# ============================================================
# YAML-ready output
# ============================================================
np.set_printoptions(precision=8, suppress=True)
print("\n" + "="*60)
print("=== YAML-ready body_T_cam (for VINS config) ===")
print("="*60)
print("\nbody_T_cam0:")
for i in range(4):
    row = T_imuopt_fish1opt[i]
    print(f"   {row[0]:.8f}, {row[1]:.8f}, {row[2]:.8f}, {row[3]:.8f},")

print("\nbody_T_cam1:")
for i in range(4):
    row = T_imuopt_fish2opt[i]
    print(f"   {row[0]:.8f}, {row[1]:.8f}, {row[2]:.8f}, {row[3]:.8f},")
