#pragma once

#include <ostream>
#include "uom/utils/common_types.hpp"


struct State
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double t;

    Vector3d p;

    Vector3d v;

    Quaterniond q;

    Vector3d ba;

    Vector3d bg;

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