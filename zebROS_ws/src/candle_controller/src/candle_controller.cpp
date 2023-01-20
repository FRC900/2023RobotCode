#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <ctre_interfaces/candle_command_interface.h>
#include <string>
#include <optional>
#include <candle_controller_msgs/Colour.h>
#include <realtime_tools/realtime_buffer.h>
#include <pluginlib/class_list_macros.h>

using namespace hardware_interface::candle;

namespace candle_controller {
class CANdleController : public controller_interface::Controller<CANdleCommandInterface> {
public:
    CANdleController() {}

    bool init(
        hardware_interface::RobotHW* hw,
        ros::NodeHandle& root_nh,
        ros::NodeHandle& controller_nh
    ) {
        CANdleCommandInterface* const candle_command_interface = hw->get<CANdleCommandInterface>();
        
        std::string candle_name;
        if (!controller_nh.getParam("name", candle_name)) {
            ROS_ERROR("Cannot initialize candle! Failed to read the 'name' field.");
            return false;
        }

        this->candle_handle = candle_command_interface->getHandle(candle_name);

        this->colour_service = controller_nh.advertiseService("candle_colour", &CANdleController::colourCallback, this);
        return true;
    }

    void starting(const ros::Time&) {}

    void update(const ros::Time&, const ros::Duration&) {
        const Colour colour = *(this->colour_buffer.readFromRT());
        const Animation animation = *(this->animation_buffer.readFromRT());
    }

    void stopping(const ros::Time&) {}

private:
    CANdleCommandHandle candle_handle;

    ros::ServiceServer colour_service;

    struct Colour : CANdleColour {
        ros::Time time;

        Colour() {}
    };
    struct Animation {
        CANdleAnimation* animation;
        ros::Time time;

        Animation() {}
    };

    realtime_tools::RealtimeBuffer<Colour> colour_buffer;
    realtime_tools::RealtimeBuffer<Animation> animation_buffer;

    bool colourCallback(candle_controller_msgs::Colour::Request& req, candle_controller_msgs::Colour::Response& res) {
        if (this->isRunning()) {
            Colour colour;
            colour.red = req.red;
            colour.green = req.green;
            colour.blue = req.blue;
            colour.white = req.white;
            colour.time = ros::Time::now();

            return true;
        } else {
              ROS_ERROR_STREAM("Can't accept new commands. CANdleController is not running.");
              return false;
        }
    }
};
} // namespace candle_controller

PLUGINLIB_EXPORT_CLASS(candle_controller::CANdleController, controller_interface::ControllerBase)
