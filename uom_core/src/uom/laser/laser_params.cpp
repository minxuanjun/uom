#include "uom/laser/laser_params.hpp"

std::ostream& operator <<(std::ostream& os, const LaserModel& model)
{
    switch (model)
    {
        case LaserModel::MID40       : os << "Mid40";   break;
    }
    return os;
}


LaserModel string_to_laser_model(const std::string& laser_model)
{
    std::string lower_case_distortion_model = laser_model;

    std::transform(lower_case_distortion_model.begin(),
                   lower_case_distortion_model.end(),
                   lower_case_distortion_model.begin(),
                   ::tolower);

    if (lower_case_distortion_model == std::string("mid40"))
    {
        return LaserModel::MID40;
    }
    else
    {
        throw std::runtime_error("Unrecognized laser model." "Valid laser model options are 'mid40'.");
    }
}