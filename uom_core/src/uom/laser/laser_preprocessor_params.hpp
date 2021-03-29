#pragma once

#include "uom/utils/math_utils.hpp"
#include "uom/utils/params_base.hpp"


struct CloudFeatureExtractorParams : ParamsBase
{

    /// Average time interval between points. Used to assign accurate timestamp.
    double time_interval_pts = 1.0e-5; // 10us = 1e-5

    /// Range filtering
    float min_allow_dist = 1.0;

    /// Intensity Threshold. eq.(4)
    float min_sigma = 7e-4;

    /// Threshold for rejecting points nearing to the fringe of the FoV (eq.2)
    float max_near_fov = std::tan( 17.0 * deg2rad);

    /// Minimal incident angle between the laser ray and the local plane around the measured point (degree)
    float min_incident_angle = 10.0;

    float threshold_surf_curvature = 0.005;
    float threshold_corn_curvature = 0.001;
    float threshold_corn_reflectivity = 0.05;

    /// Half number of neighborhood used to compute curvature
    size_t curvature_ssd_size = 2;


    void parse_yaml(const std::string& file_path) override
    {
        YamlParser parser(file_path);

        parser.get_param("time_interval_pts", time_interval_pts);
        parser.get_param("min_allow_dist", min_allow_dist);
        parser.get_param("min_sigma", min_sigma);
        parser.get_param("min_incident_angle", min_incident_angle);
        parser.get_param("curvature_ssd_size", curvature_ssd_size);
        parser.get_param("threshold_surf_curvature", threshold_surf_curvature);
        parser.get_param("threshold_corn_curvature", threshold_corn_curvature);
        parser.get_param("threshold_corn_reflectivity", threshold_corn_reflectivity);

        parser.get_param("max_circle_edge_reject", max_near_fov);
        CHECK_GT(max_near_fov, 0);
        CHECK_LT(max_near_fov, 80);
        max_near_fov = std::tan( max_near_fov * deg2rad);
    }


    void print() override
    {
        std::stringstream out;

        ParamsBase::print
        (
            // param name
            "CloudFeatureExtractorParams",
            // output
            out,
            // input
            "curvature_ssd_size", curvature_ssd_size,
            "min_allow_dist", min_allow_dist,
            "min_sigma", min_sigma,
            "min_incident_angle", min_incident_angle,
            "time_interval_pts", time_interval_pts,
            "threshold_surf_curvature", threshold_surf_curvature,
            "threshold_corn_curvature", threshold_corn_curvature,
            "threshold_corn_reflectivity", threshold_corn_reflectivity
        );

        LOG(INFO) << out.str();
    }

};