#pragma once

#include "uom/utils/params_base.hpp"
#include "uom/utils/common_types.hpp"


struct StaticInitializerParams : ParamsBase
{

    /// If true we will wait for a "jerk". This will check if we have had a large enough jump in our acceleration.
    /// If we have then we will use the period of time before this jump to initialize the state.
    bool wait_for_jerk = true;

    /// Amount of time we will initialize over (seconds)
    double window_length = 0.75;

    /// Variance threshold on our acceleration to be classified as moving
    double excite_threshold = 1.0;


    void parse_yaml(const std::string& file_path) override
    {
        YamlParser parser(file_path);

        // Wait for jerk setting
        parser.get_param("wait_for_jerk", wait_for_jerk);

        // Window length setting.
        parser.get_param("window_length", window_length);

        // Wait for jerk setting
        parser.get_param("excite_threshold", excite_threshold);
    }


    void print() override
    {
        std::stringstream out;

        ParamsBase::print
        (
            // param name
            "StaticInitializerParams",
            // output
            out,
            // input
            "excite_threshold", excite_threshold,
            "wait_for_jerk: ", wait_for_jerk,
            "window_length: ", (std::to_string(window_length) + "s")
        );

        LOG(INFO) << out.str();
    }

};