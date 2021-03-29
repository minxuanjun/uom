#include <string>

#include "uom/vision/camera_params.hpp"

std::ostream& operator <<(std::ostream& os, const DistortionModel& model)
{
    switch (model)
    {
        case DistortionModel::NONE        : os << "NONE";   break;
        case DistortionModel::RADTAN      : os << "RADTAN"; break;
        case DistortionModel::EQUIDISTANT : os << "EQUIDISTANT"; break;
    }
    return os;
}


DistortionModel string_to_distortion_model(const std::string& distortion_model)
{
    std::string lower_case_distortion_model = distortion_model;

    std::transform(lower_case_distortion_model.begin(),
                   lower_case_distortion_model.end(),
                   lower_case_distortion_model.begin(),
                   ::tolower);

    if (lower_case_distortion_model == std::string("none"))
    {
        return DistortionModel::NONE;
    }
    else if ((lower_case_distortion_model == std::string("plumb_bob")) ||
             (lower_case_distortion_model == std::string("radial-tangential")) ||
             (lower_case_distortion_model == std::string("radtan")))
    {
        return DistortionModel::RADTAN;
    }
    else if (lower_case_distortion_model == std::string("equidistant"))
    {
        return DistortionModel::EQUIDISTANT;
    }
    else
    {
        throw std::runtime_error("Unrecognized distortion model for pinhole camera."
                                 "Valid pinhole distortion model options are 'none', 'radtan', 'equidistant'.");
    }
}