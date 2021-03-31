#pragma once

#include "uom/utils/params_base.hpp"
#include "uom/utils/common_types.hpp"


struct StaticInitializerParams : ParamsBase
{

    /// If true we will wait for a "jerk". This will check if we have had a large enough jump in our acceleration.
    /// If we have then we will use the period of time before this jump to initialize the state.
    bool wait_for_jerk = true;

    /// Amount of time we will initialize over (nanoseconds)
    Timestamp window_length = 0.75 * s_to_ns;

    /// Variance threshold on our acceleration to be classified as moving
    double excite_threshold = 1.0;


    void parse_yaml(const std::string& file_path) override
    {
        YamlParser parser(file_path);

        // Wait for jerk setting
        parser.get_param("wait_for_jerk", wait_for_jerk);

        // Window length setting. We read with second unit then convert it to nanosecond.
        double window_length_s; // second.
        parser.get_param("window_length", window_length_s);
        window_length = window_length_s * s_to_ns;

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
            "window_length: ", window_length
        );

        LOG(INFO) << out.str();
    }

};