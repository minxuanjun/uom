#pragma once

#include <vector>
#include <Eigen/Eigen>

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

template<typename T>
using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;