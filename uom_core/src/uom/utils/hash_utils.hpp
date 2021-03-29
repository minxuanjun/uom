#pragma once

#include <pcl/point_types.h>


template <typename T>
struct hash_eigen
{
    std::size_t operator ()(T const& matrix) const
    {
        size_t seed = 0;
        for (int i = 0; i < (int)matrix.size(); i++)
        {
            auto elem = *(matrix.data() + i);
            seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 +
                    (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};


namespace std
{
    template <>
    class hash<pcl::PointXYZI>
    {
    public :
        size_t operator ()(const pcl::PointXYZI& p) const
        {
            return ((std::hash<float>()(p.x) ^ (std::hash<float>()(p.y) << 1)) >> 1) ^ (std::hash<float>()(p.z) << 1);
        }
    };
};
