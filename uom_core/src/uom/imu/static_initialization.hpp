#pragma once

#include <deque>
#include <tuple>

#include "uom/imu/state.hpp"
#include "uom/imu/imu_data.hpp"
#include "uom/imu/imu_params.hpp"
#include "uom/imu/static_initialization_params.hpp"

#include "uom/utils/macros.hpp"
#include "uom/utils/math_utils.hpp"


/**
 * @brief Used for initializing bias and rotation of inertial device. We assume the device is static.
 */
class StaticInitializer
{

public:

    POINTER_TYPEDEFS(StaticInitializer)


    /// ctor
    StaticInitializer(StaticInitializerParams options = StaticInitializerParams(),
                      ImuParams imu_params = ImuParams()): n_gravity_(imu_params.n_gravity), options_(options){}

    ~StaticInitializer() = default;


    /**
     * @brief Stores incoming inertial readings
     * @param imu_data_vec IMU readings vector
     */
    void add_measurement(const ImuDataVector& imu_data_vec);


    /**
     * @brief Try to initialize the system using just the imu
     * @param[out] out_state Navigation State at initialization.
     * @return True if we have successfully initialized our system
     */
    bool initialize(State& out_state);


private:

    std::tuple<Vector3d, Vector3d, double> compute_avg_var(const ImuDataVector& imu_data_vec);

    /// Set from ImuParams. Gravity vector
    Vector3d n_gravity_;

    /// Settings of StaticInitializer
    StaticInitializerParams options_;

    /// Our history of IMU messages (time, angular, linear)
    std::deque<ImuData> imu_buffer_;

};
