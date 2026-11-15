// Basic sensor data types shared by every module (docs/design.md §5).
#ifndef MINI_VIO_TYPES_H_
#define MINI_VIO_TYPES_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <opencv2/core.hpp>
#include <variant>

namespace mini_vio {

// Nanoseconds, same as EuRoC. Converted to seconds only when handed to GTSAM.
using Timestamp = std::int64_t;

struct ImuData {
    Timestamp t = 0;
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();   // rad/s, body (IMU) frame
    Eigen::Vector3d accel = Eigen::Vector3d::Zero();  // m/s², specific force (about +g upward at rest)
};

enum class CameraId : std::uint8_t { kLeft = 0, kRight = 1 };

// One image from one camera. SensorSync pairs left and right (FR-2).
struct CameraImage {
    Timestamp t = 0;
    CameraId camera = CameraId::kLeft;
    cv::Mat image;  // 8-bit grayscale, raw (not undistorted or rectified)
};

// What a DatasetReader emits, in time order.
using SensorEvent = std::variant<ImuData, CameraImage>;

// Calibration of one camera as given by the dataset, before rectification.
struct RawCameraCalibration {
    Eigen::Isometry3d T_B_C = Eigen::Isometry3d::Identity();  // camera pose in the body frame
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    Eigen::Vector4d distortion = Eigen::Vector4d::Zero();  // radial-tangential: k1, k2, p1, p2
    int width = 0;
    int height = 0;
    double rate_hz = 0.0;
};

struct RawStereoCalibration {
    RawCameraCalibration left;   // cam0
    RawCameraCalibration right;  // cam1
};

// IMU noise as continuous-time densities (EuRoC imu0/sensor.yaml).
struct ImuNoiseParams {
    Eigen::Isometry3d T_B_I = Eigen::Isometry3d::Identity();  // IMU pose in the body frame (identity in EuRoC)
    double rate_hz = 0.0;
    double gyro_noise_density = 0.0;   // rad/s/√Hz
    double gyro_random_walk = 0.0;     // rad/s²/√Hz
    double accel_noise_density = 0.0;  // m/s²/√Hz
    double accel_random_walk = 0.0;    // m/s³/√Hz
};

}  // namespace mini_vio

#endif  // MINI_VIO_TYPES_H_
