#pragma once

#include <ostream>
#include "uom/utils/common_types.hpp"


struct State
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// Timestamp (second)
    double t;

    /// Position and Velocity in Global frame
    Vector3d p, v;

    /// Orientation in Global frame. aka. \mathbf{R}_{world}_{imu}
    Quaterniond q;

    /// Bias for accelerator and gyroscope in Body frame
    Vector3d ba, bg;


    /// \brief Timestamp will be set to zero. Pose will be set to identity. Bias will be set to zero.
    inline void set_identity()
    {
        t = 0;
        p.setZero();
        v.setZero();
        q.setIdentity();
        ba.setZero();
        bg.setZero();
    }


    Matrix4d transformation() const
    {
        Matrix4d pose = Matrix4d::Identity();
        pose.topLeftCorner<3, 3>() = q.toRotationMatrix();
        pose.block<3, 1>(0, 3) = p;
        return pose;
    }


    /// \brief For print to console or generate out stream
    inline void print(std::ostream& ss, bool out_all = false) const
    {
        ss << t << " " << p.x() << " " << p.y() << " " << p.z() << " "
           << q.x() << " " << q.y() << " " << q.z() << " " << q.w();

        if (out_all)
        {
            ss << " "
               << v.x() << " " << v.y() << " " << v.z() << " "
               << ba.x() << " " << ba.y() << " " << ba.z() << " "
               << bg.x() << " " << bg.y() << " " << bg.z();
        }
    }
};