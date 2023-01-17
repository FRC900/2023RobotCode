#include "ros_control_boilerplate/frc_robot_interface.h"

using namespace ctre::phoenix::led;

namespace ros_control_boilerplate {
void FRCRobotInterface::candle_read_thread(
    std::shared_ptr<ctre::phoenix::led::CANdle> candle,
    std::shared_ptr<hardware_interface::candle::CANdleHWState> state,
    std::shared_ptr<std::mutex> mutex,
    std::unique_ptr<Tracer> tracer,
    double poll_frequency
) {
    // Set thread name in linux
    #ifdef __linux__
        std::stringstream thread_name{"candle_read_"};

        if (pthread_setname_np(pthread_self(), thread_name.str().c_str()))
        {
            ROS_ERROR_STREAM("Error setting thread name " << thread_name.str() << " " << errno);
        }
    #endif
    // Slight delay so the CAN bus doesn't implode from too many requests
    ros::Duration(2.7 + state->getDeviceID() * 0.07).sleep();
    
    // Loop updating the state every <poll_frequency>
    for (ros::Rate rate(poll_frequency); ros::ok(); rate.sleep()) {
		tracer->start("cancoder read main_loop");
        // Store the CANdle's configuration
        CANdleConfiguration config;
        // Get the configuration, and store it in config
        candle->GetAllConfigs(config);

        // Lock the state mutex
        std::lock_guard<std::mutex> _(*mutex);
        // Update the state using the config variable
        state->setBrightness(config.brightnessScalar);
        state->showStatusLEDWhenActive(config.statusLedOffWhenActive);
        state->setEnabled(config.v5Enabled);
    }
}
}
