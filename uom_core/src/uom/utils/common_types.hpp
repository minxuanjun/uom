#pragma once

#include <vector>
#include <Eigen/Eigen>
#include <sophus/se3.hpp>

/// Timestamp related
using Timestamp = int64_t;
static constexpr double ns_to_ms = 1e-6;  // Nanosecond to millisecond conversion
static constexpr double ms_to_ns = 1e6;   // millisecond to Nanosecond conversion
static constexpr double ns_to_s  = 1e-9;  // Nanosecond to second conversion
static constexpr double s_to_ns  = 1e9;   // Second to nanosecond conversion

using Eigen::Vector2f;
using Eigen::Vector3f;
using Eigen::Vector4f;
using Eigen::Quaternionf;

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::Quaterniond;
using Vector6d = Eigen::Matrix<double, 6, 1>;

using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Matrix;
using Eigen::MatrixXd;

using Eigen::Matrix2f;
using Eigen::Matrix3f;
using Eigen::Matrix4f;

using Sophus::SO3d;
using Sophus::SE3d;

template<typename T>
using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;