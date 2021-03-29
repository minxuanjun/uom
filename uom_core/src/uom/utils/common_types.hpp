#pragma once

#include <vector>
#include <Eigen/Eigen>
#include <sophus/se3.hpp>

using Eigen::Vector2f;
using Eigen::Vector3f;

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::Quaterniond;

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