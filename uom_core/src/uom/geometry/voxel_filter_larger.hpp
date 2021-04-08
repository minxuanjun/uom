#pragma once

#include <pcl/filters/voxel_grid.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/filters/passthrough.h>


namespace pcl
{

    template <typename PointT>
    class VoxelGridLarge : public pcl::VoxelGrid<PointT>
    {
    protected:

        using Base = pcl::VoxelGrid<PointT>;
        using PointCloud = typename Base::PointCloud;

        void applyFilter(PointCloud& output) override
        {
            // Has the input dataset been set already?
            if (!this->input_)
            {
                PCL_WARN ("[pcl::%s::applyFilter] No input dataset given!\n", this->getClassName().c_str());
                output.width = output.height = 0;
                output.points.clear();
                return;
            }

            // Copy the header (and thus the frame_id) + allocate enough space for points
            output.height = 1;                    // downsampling breaks the organized structure
            output.is_dense = true;                 // we filter out invalid points

            Eigen::Vector4f min_p, max_p;
            // Get the minimum and maximum dimensions
            if (!this->filter_field_name_.empty())
            { // If we don't want to process the entire cloud...
                getMinMax3D<PointT>(this->input_, *this->indices_, this->filter_field_name_,
                                    static_cast<float> (this->filter_limit_min_),
                                    static_cast<float> (this->filter_limit_max_), min_p, max_p,
                                    this->filter_limit_negative_);
            }
            else
            {
                getMinMax3D<PointT>(*this->input_, *this->indices_, min_p, max_p);
            }

            // Check that the leaf size is not too small, given the size of the data
            std::int64_t dx = static_cast<std::int64_t>((max_p[0] - min_p[0]) * this->inverse_leaf_size_[0]) + 1;
            std::int64_t dy = static_cast<std::int64_t>((max_p[1] - min_p[1]) * this->inverse_leaf_size_[1]) + 1;
            std::int64_t dz = static_cast<std::int64_t>((max_p[2] - min_p[2]) * this->inverse_leaf_size_[2]) + 1;

            if ((dx * dy * dz) > static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max()))
            {
                typename PointCloud::Ptr cloud1(new PointCloud), cloud2(new PointCloud);
                PointCloud cloud1f, cloud2f;
                pcl::PassThrough<PointT> pass;

                if (dx > dy && dx > dz)
                {
                    pass.setInputCloud(this->input_);
                    pass.setFilterFieldName("x");
                    pass.setFilterLimits(min_p[0], min_p[0] + (max_p[0] - min_p[0]) / 2);
                    pass.filter(*cloud1);

                    pass.setFilterLimitsNegative(true);
                    pass.filter(*cloud2);
                }
                else if (dy > dx && dy > dz)
                {
                    pass.setInputCloud(this->input_);
                    pass.setFilterFieldName("y");
                    pass.setFilterLimits(min_p[1], min_p[1] + (max_p[1] - min_p[1]) / 2);
                    pass.filter(*cloud1);

                    pass.setFilterLimitsNegative(true);
                    pass.filter(*cloud2);
                }
                else
                {
                    pass.setInputCloud(this->input_);
                    pass.setFilterFieldName("z");
                    pass.setFilterLimits(min_p[2], min_p[2] + (max_p[2] - min_p[2]) / 2);
                    pass.filter(*cloud1);

                    pass.setFilterLimitsNegative(true);
                    pass.filter(*cloud2);
                }

                VoxelGridLarge voxelGridLarge = *this;
                voxelGridLarge.setInputCloud(cloud1);
                voxelGridLarge.filter(cloud1f);

                voxelGridLarge.setInputCloud(cloud2);
                voxelGridLarge.filter(cloud2f);

                output = cloud1f + cloud2f;
                return;
            }

            // Compute the minimum and maximum bounding box values
            this->min_b_[0] = static_cast<int> (std::floor(min_p[0] * this->inverse_leaf_size_[0]));
            this->max_b_[0] = static_cast<int> (std::floor(max_p[0] * this->inverse_leaf_size_[0]));
            this->min_b_[1] = static_cast<int> (std::floor(min_p[1] * this->inverse_leaf_size_[1]));
            this->max_b_[1] = static_cast<int> (std::floor(max_p[1] * this->inverse_leaf_size_[1]));
            this->min_b_[2] = static_cast<int> (std::floor(min_p[2] * this->inverse_leaf_size_[2]));
            this->max_b_[2] = static_cast<int> (std::floor(max_p[2] * this->inverse_leaf_size_[2]));

            // Compute the number of divisions needed along all axis
            this->div_b_ = this->max_b_ - this->min_b_ + Eigen::Vector4i::Ones();
            this->div_b_[3] = 0;

            // Set up the division multiplier
            this->divb_mul_ = Eigen::Vector4i(1, this->div_b_[0], this->div_b_[0] * this->div_b_[1], 0);

            // Storage for mapping leaf and pointcloud indexes
            std::vector<cloud_point_index_idx> index_vector;
            index_vector.reserve(this->indices_->size());

            // If we don't want to process the entire cloud, but rather filter points far away from the viewpoint first...
            if (!this->filter_field_name_.empty())
            {
                // Get the distance field index
                std::vector<pcl::PCLPointField> fields;
                int distance_idx = pcl::getFieldIndex<PointT>(this->filter_field_name_, fields);
                if (distance_idx == -1)
                    PCL_WARN ("[pcl::%s::applyFilter] Invalid filter field name. Index is %d.\n",
                              this->getClassName().c_str(), distance_idx);

                // First pass: go over all points and insert them into the index_vector vector
                // with calculated idx. Points with the same idx value will contribute to the
                // same point of resulting CloudPoint
                for (std::vector<int>::const_iterator it = this->indices_->begin(); it != this->indices_->end(); ++it)
                {
                    if (!this->input_->is_dense)
                    {
                        // Check if the point is invalid
                        if (!std::isfinite(this->input_->points[*it].x) ||
                            !std::isfinite(this->input_->points[*it].y) ||
                            !std::isfinite(this->input_->points[*it].z))
                        {
                            continue;
                        }
                    }

                    // Get the distance value
                    const std::uint8_t* pt_data = reinterpret_cast<const std::uint8_t*> (&this->input_->points[*it]);
                    float distance_value = 0;
                    memcpy(&distance_value, pt_data + fields[distance_idx].offset, sizeof(float));

                    if (this->filter_limit_negative_)
                    {
                        // Use a threshold for cutting out points which inside the interval
                        if ((distance_value < this->filter_limit_max_) && (distance_value > this->filter_limit_min_))
                        {
                            continue;
                        }
                    }
                    else
                    {
                        // Use a threshold for cutting out points which are too close/far away
                        if ((distance_value > this->filter_limit_max_) || (distance_value < this->filter_limit_min_))
                        {
                            continue;
                        }
                    }

                    int ijk0 = static_cast<int> (std::floor(this->input_->points[*it].x * this->inverse_leaf_size_[0]) -
                                                 static_cast<float> (this->min_b_[0]));
                    int ijk1 = static_cast<int> (std::floor(this->input_->points[*it].y * this->inverse_leaf_size_[1]) -
                                                 static_cast<float> (this->min_b_[1]));
                    int ijk2 = static_cast<int> (std::floor(this->input_->points[*it].z * this->inverse_leaf_size_[2]) -
                                                 static_cast<float> (this->min_b_[2]));

                    // Compute the centroid leaf index
                    int idx = ijk0 * this->divb_mul_[0] + ijk1 * this->divb_mul_[1] + ijk2 * this->divb_mul_[2];
                    index_vector.emplace_back(static_cast<unsigned int> (idx), *it);
                }
            }
                // No distance filtering, process all data
            else
            {
                // First pass: go over all points and insert them into the index_vector vector
                // with calculated idx. Points with the same idx value will contribute to the
                // same point of resulting CloudPoint
                for (std::vector<int>::const_iterator it = this->indices_->begin(); it != this->indices_->end(); ++it)
                {
                    if (!this->input_->is_dense)
                    {
                        // Check if the point is invalid
                        if (!std::isfinite(this->input_->points[*it].x) ||
                            !std::isfinite(this->input_->points[*it].y) ||
                            !std::isfinite(this->input_->points[*it].z))
                        {
                            continue;
                        }
                    }

                    int ijk0 = static_cast<int> (std::floor(this->input_->points[*it].x * this->inverse_leaf_size_[0]) -
                                                 static_cast<float> (this->min_b_[0]));
                    int ijk1 = static_cast<int> (std::floor(this->input_->points[*it].y * this->inverse_leaf_size_[1]) -
                                                 static_cast<float> (this->min_b_[1]));
                    int ijk2 = static_cast<int> (std::floor(this->input_->points[*it].z * this->inverse_leaf_size_[2]) -
                                                 static_cast<float> (this->min_b_[2]));

                    // Compute the centroid leaf index
                    int idx = ijk0 * this->divb_mul_[0] + ijk1 * this->divb_mul_[1] + ijk2 * this->divb_mul_[2];
                    index_vector.emplace_back(static_cast<unsigned int> (idx), *it);
                }
            }

            // Second pass: sort the index_vector vector using value representing target cell as index
            // in effect all points belonging to the same output cell will be next to each other
            std::sort(index_vector.begin(), index_vector.end(), std::less<cloud_point_index_idx>());

            // Third pass: count output cells
            // we need to skip all the same, adjacent idx values
            unsigned int total = 0;
            unsigned int index = 0;
            // first_and_last_indices_vector[i] represents the index in index_vector of the first point in
            // index_vector belonging to the voxel which corresponds to the i-th output point,
            // and of the first point not belonging to.
            std::vector<std::pair<unsigned int, unsigned int> > first_and_last_indices_vector;
            // Worst case size
            first_and_last_indices_vector.reserve(index_vector.size());
            while (index < index_vector.size())
            {
                unsigned int i = index + 1;
                while (i < index_vector.size() && index_vector[i].idx == index_vector[index].idx)
                    ++i;
                if (i - index >= this->min_points_per_voxel_)
                {
                    ++total;
                    first_and_last_indices_vector.emplace_back(index, i);
                }
                index = i;
            }

            // Fourth pass: compute centroids, insert them into their final position
            output.points.resize(total);
            if (this->save_leaf_layout_)
            {
                try
                {
                    // Resizing won't reset old elements to -1.  If this->leaf_layout_ has been used previously, it needs to be re-initialized to -1
                    std::uint32_t new_layout_size = this->div_b_[0] * this->div_b_[1] * this->div_b_[2];
                    //This is the number of elements that need to be re-initialized to -1
                    std::uint32_t reinit_size = std::min(static_cast<unsigned int> (new_layout_size),
                                                         static_cast<unsigned int> (this->leaf_layout_.size()));
                    for (std::uint32_t i = 0; i < reinit_size; i++)
                    {
                        this->leaf_layout_[i] = -1;
                    }
                    this->leaf_layout_.resize(new_layout_size, -1);
                }
                catch (std::bad_alloc&)
                {
                    throw PCLException("VoxelGrid bin size is too low; impossible to allocate memory for layout",
                                       "voxel_grid.hpp", "applyFilter");
                }
                catch (std::length_error&)
                {
                    throw PCLException("VoxelGrid bin size is too low; impossible to allocate memory for layout",
                                       "voxel_grid.hpp", "applyFilter");
                }
            }

            index = 0;
            for (const auto& cp : first_and_last_indices_vector)
            {
                // calculate centroid - sum values from all input points, that have the same idx value in index_vector array
                unsigned int first_index = cp.first;
                unsigned int last_index = cp.second;

                // index is centroid final position in resulting PointCloud
                if (this->save_leaf_layout_)
                {
                    this->leaf_layout_[index_vector[first_index].idx] = index;
                }

                //Limit downsampling to coords
                if (!this->downsample_all_data_)
                {
                    Eigen::Vector4f centroid(Eigen::Vector4f::Zero ());

                    for (unsigned int li = first_index; li < last_index; ++li)
                        centroid += this->input_->points[index_vector[li].cloud_point_index].getVector4fMap();

                    centroid /= static_cast<float> (last_index - first_index);
                    output.points[index].getVector4fMap() = centroid;
                }
                else
                {
                    CentroidPoint<PointT> centroid;

                    // fill in the accumulator with leaf points
                    for (unsigned int li = first_index; li < last_index; ++li)
                        centroid.add(this->input_->points[index_vector[li].cloud_point_index]);

                    centroid.get(output.points[index]);
                }

                ++index;
            }
            output.width = static_cast<std::uint32_t> (output.points.size());
        }
    };

}