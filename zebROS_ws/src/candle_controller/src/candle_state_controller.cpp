#include <ros/ros.h>
#include <candle_controller_msgs/State.h>
#include <controller_interface/controller.h>
#include <ctre_interfaces/candle_state_interface.h>

namespace candle_controller {
class CANdleStateController: public controller_interface::Controller<CANdleStateInterface> {
public:
    bool init(
        CANdleStateInterface* state_interface,
        ros::NodeHandle& root_nh,
        ros::NodeHandle& controller_nh
    ) override {

    }

private:
}
}