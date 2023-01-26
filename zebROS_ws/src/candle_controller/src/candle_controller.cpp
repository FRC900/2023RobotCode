#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <ctre_interfaces/candle_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <pluginlib/class_list_macros.h>

#include <string>
#include <optional>

#include <candle_controller_msgs/Brightness.h>
#include <candle_controller_msgs/Colour.h>
#include <candle_controller_msgs/Animation.h>

using namespace hardware_interface::candle;

CANdleAnimationType convertAnimationType(int type) {
    switch (type) {
        case 0: {
            return CANdleAnimationType::ColourFlow;
        }
        case 1: {
            return CANdleAnimationType::Fire;
        }
        case 2: {
            return CANdleAnimationType::Larson;
        }
        case 3: {
            return CANdleAnimationType::Rainbow;
        }
        case 4: {
            return CANdleAnimationType::RGBFade;
        }
        case 5: {
            return CANdleAnimationType::SingleFade;
        }
        case 6: {
            return CANdleAnimationType::Strobe;
        }
        case 7: {
            return CANdleAnimationType::Twinkle;
        }
        case 8: {
            return CANdleAnimationType::TwinkleOff;
        }
        default: {
            ROS_ERROR_STREAM("Invalid int to convert to CANdleAnimationType!");
        }
    }
}
CANdleAnimationClass getAnimationClass(CANdleAnimationType type) {
    switch (type) {
        case CANdleAnimationType::ColourFlow: {
            return CANdleAnimationClass::BaseTwo;
        }
        case CANdleAnimationType::Fire: {
            return CANdleAnimationClass::BaseStandard;
        }
        case CANdleAnimationType::Larson: {
            return CANdleAnimationClass::BaseTwo;
        }
        case CANdleAnimationType::Rainbow: {
            return CANdleAnimationClass::BaseStandard;
        }
        case CANdleAnimationType::RGBFade: {
            return CANdleAnimationClass::BaseStandard;
        }
        case CANdleAnimationType::SingleFade: {
            return CANdleAnimationClass::BaseTwo;
        }
        case CANdleAnimationType::Strobe: {
            return CANdleAnimationClass::BaseTwo;
        }
        case CANdleAnimationType::Twinkle: {
            return CANdleAnimationClass::BaseTwo;
        }
        case CANdleAnimationType::TwinkleOff: {
            return CANdleAnimationClass::BaseTwo;
        }
        default: {
            ROS_ERROR_STREAM("Faild to match CANdleAnimationType to get CANdleAnimationClass!");
        }
    }
}

namespace candle_controller {
class CANdleController : public controller_interface::Controller<CANdleCommandInterface> {
public:
    CANdleController() {}

    bool init(
        CANdleCommandInterface* candle_command_interface,
        ros::NodeHandle& /*root_nh*/,
        ros::NodeHandle& controller_nh
    ) override {
        std::string candle_name;
        if (!controller_nh.getParam("name", candle_name)) {
            ROS_ERROR("Cannot initialize candle! Failed to read the 'name' field.==========");
            return false;
        }

        this->candle_handle = candle_command_interface->getHandle(candle_name);

        this->colour_service = controller_nh.advertiseService("colour", &CANdleController::colourCallback, this);
        this->brightness_service = controller_nh.advertiseService("brightness", &CANdleController::brightnessCallback, this);
        this->animation_service = controller_nh.advertiseService("animation", &CANdleController::animationCallback, this);
        return true;
    }

    void starting(const ros::Time&) override {}

    void update(const ros::Time&, const ros::Duration&) override {
        const LEDs leds = *(this->led_buffer.readFromRT());
        const Animation animation = *(this->animation_buffer.readFromRT());
        const Brightness brightness = *(this->brightness_buffer.readFromRT());

        // Brightness isn't exclusive to colours/animations, so it gets special treatment
        if (brightness.time > this->last_write) {
            ROS_INFO_STREAM("Writing new brightness to CANdle");
            this->candle_handle->setBrightness(brightness.brightness);
        }

        if (leds.time > this->last_write || animation.time > this->last_write) {
            if (leds.time > animation.time) {
                ROS_INFO_STREAM("Writing LED group to CANdle");
                this->last_write = leds.time;
                this->candle_handle->setLEDGroup(leds.group);
            } else {
                ROS_INFO_STREAM("Writing new animation to CANdle");
                this->last_write = animation.time;
                this->candle_handle->setAnimation(animation.animation);
            }
        }

        // Only overwrite last_write after the leds and animations have applied, since brightness isn't exclusive to them
        if (brightness.time > this->last_write) {
            this->last_write = brightness.time;
        }
    }

    void stopping(const ros::Time&) override {}

private:
    struct LEDs {
        ros::Time time;
        LEDGroup group;

        LEDs() {}
    };

    struct Animation {
        CANdleAnimation animation;
        ros::Time time;

        Animation() {}
    };

    struct Brightness {
        double brightness;
        ros::Time time;

        Brightness() {}
    };

    CANdleCommandHandle candle_handle;
    ros::ServiceServer colour_service;
    ros::ServiceServer brightness_service;
    ros::ServiceServer animation_service;
    ros::Time last_write;

    realtime_tools::RealtimeBuffer<LEDs> led_buffer;
    realtime_tools::RealtimeBuffer<Animation> animation_buffer;
    realtime_tools::RealtimeBuffer<Brightness> brightness_buffer;

    bool colourCallback(candle_controller_msgs::Colour::Request& req, candle_controller_msgs::Colour::Response&) {
        if (this->isRunning()) {
            LEDs leds;
            leds.group = LEDGroup(req.start, req.count, req.red, req.green, req.blue, req.white);
            leds.time = ros::Time::now();

            this->led_buffer.writeFromNonRT(leds);

            return true;
        } else {
            ROS_ERROR_STREAM("Can't accept new commands. CANdleController is not running.");
            return false;
        }
    }

    bool brightnessCallback(candle_controller_msgs::Brightness::Request& req, candle_controller_msgs::Brightness::Response&) {
        if (this->isRunning()) {
            Brightness brightness;
            brightness.brightness = req.brightness;
            brightness.time = ros::Time::now();

            this->brightness_buffer.writeFromNonRT(brightness);

            return true;
        } else {
            ROS_ERROR_STREAM("Can't accept new commands. CANdleController is not running.");
            return false;
        }
    }

    bool animationCallback(candle_controller_msgs::Animation::Request& req, candle_controller_msgs::Animation::Response&) {
        if (this->isRunning()) {
            Animation animation;
            animation.time = ros::Time::now();
            CANdleAnimationType animation_type = convertAnimationType(req.animation_type);
            CANdleAnimationClass animation_class = getAnimationClass(animation_type);

            switch (animation_class) {
                // Base Standard animation
                case CANdleAnimationClass::BaseStandard: {
                    animation.animation = CANdleAnimation(
                        req.speed,
                        req.start,
                        req.count,
                        animation_type,
                        req.brightness,
                        req.reversed,
                        req.param4,
                        req.param5
                    );
                }
                // Base Two animation
                case CANdleAnimationClass::BaseTwo: {
                    animation.animation = CANdleAnimation(
                        req.speed,
                        req.start,
                        req.count,
                        animation_type,
                        req.red,
                        req.blue,
                        req.green,
                        req.white,
                        req.direction
                    );
                }
            }

            ROS_INFO_STREAM("Setting CANdle animation to animation with speed |" 
                            << animation.animation.speed << "| // start |" << animation.animation.start);
            
            this->animation_buffer.writeFromNonRT(animation);

            return true;
        } else {
            ROS_ERROR_STREAM("Can't accept new commands. CANdleController is not running.");
            return false;
        }
    }
};
} // namespace candle_controller

PLUGINLIB_EXPORT_CLASS(candle_controller::CANdleController, controller_interface::ControllerBase)
