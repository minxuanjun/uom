#pragma once

#include "uom/utils/params_base.hpp"


struct LaserOdometryParams : ParamsBase
{

    std::size_t windows_size = 11;
    std::size_t max_num_iter = 15;
    std::size_t scan_match_cnt = 3;

    float ds_surf_size = 0.05;
    float ds_surf_map_size = 0.1;

    float ds_edge_size = 0.05;
    float ds_edge_map_size = 0.1;


    void parse_yaml(const std::string& file_path) override
    {
        YamlParser parser(file_path);

        parser.get_param("windows_size", windows_size);
        parser.get_param("max_num_iter", max_num_iter);
        parser.get_param("scan_match_cnt", scan_match_cnt);
        parser.get_param("ds_surf_size", ds_surf_size);
        parser.get_param("ds_surf_map_size", ds_surf_map_size);
        parser.get_param("ds_edge_size", ds_edge_size);
        parser.get_param("ds_edge_map_size", ds_edge_map_size);
    }


    void print() override
    {
        std::stringstream out;

        ParamsBase::print
            (
                // param name
                "LaserOdometryParams",
                // output
                out,
                // input
                "ds_edge_size", ds_edge_size,
                "ds_edge_map_size", ds_edge_map_size,
                "ds_surf_size", ds_surf_size,
                "ds_surf_map_size", ds_surf_map_size,
                "max_num_iter", max_num_iter,
                "windows_size: ", windows_size,
                "scan_match_cnt", scan_match_cnt
            );

        LOG(INFO) << out.str();
    }
};