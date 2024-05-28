#include <ctre/phoenix/led/CANdle.h>
#include <ctre/phoenix/led/ColorFlowAnimation.h>
#include <ctre/phoenix/led/FireAnimation.h>
#include <ctre/phoenix/led/LarsonAnimation.h>
#include <ctre/phoenix/led/RainbowAnimation.h>
#include <ctre/phoenix/led/RgbFadeAnimation.h>
#include <ctre/phoenix/led/SingleFadeAnimation.h>
#include <ctre/phoenix/led/StrobeAnimation.h>
#include <ctre/phoenix/led/TwinkleAnimation.h>
#include <ctre/phoenix/led/TwinkleOffAnimation.h>
#include "ctre_interfaces/candle_command_interface.h"
#include "ros_control_boilerplate/candle_device.h"

static ctre::phoenix::led::ColorFlowAnimation::Direction convertCANdleDirection(int direction);
static std::unique_ptr<ctre::phoenix::led::Animation> convertAnimation(hardware_interface::candle::Animation animation);

#define safeCANdleCall(error_code, call_string) \
    SIMFLAG ? true : safeCall(error_code, call_string)

template <bool SIMFLAG>
CANdleDevice<SIMFLAG>::CANdleDevice(const std::string &name_space,
                                    const int joint_index,
                                    const std::string &joint_name,
                                    const int candle_id,
                                    const std::string &can_bus,
                                    const bool local_hardware,
                                    const bool local_update)
    : CTREV5Device(name_space, "CANdle", joint_name, candle_id)
    , local_hardware_{local_hardware}
    , local_update_{local_update}
    , state_{std::make_unique<hardware_interface::candle::CANdleHWState>(candle_id)}
    , command_{std::make_unique<hardware_interface::candle::CANdleHWCommand>()}
{
    ROS_INFO_STREAM_NAMED("frc_robot_interface",
                          "Loading joint " << joint_index << "=" << joint_name <<
                          (local_update_ ? " local" : " remote") << " update, " <<
                          (local_hardware_ ? "local" : "remote") << " hardware" <<
                          " as CANdle " << candle_id << " on bus " << can_bus);

    if constexpr (!SIMFLAG)
    {
        if (local_hardware_)
        {
            candle_ = std::make_unique<ctre::phoenix::led::CANdle>(candle_id, can_bus);
        }
    }
}

template <bool SIMFLAG>
CANdleDevice<SIMFLAG>::~CANdleDevice() = default;

template <bool SIMFLAG>
void CANdleDevice<SIMFLAG>::registerInterfaces(hardware_interface::candle::CANdleStateInterface &state_interface,
                                      hardware_interface::candle::CANdleCommandInterface &command_interface,
                                      hardware_interface::candle::RemoteCANdleStateInterface &remote_state_interface) const
{
    ROS_INFO_STREAM("FRCRobotInterface: Registering interface for CANdle : " << getName() << " at CAN id " << getId());

    hardware_interface::candle::CANdleStateHandle state_handle(getName(), state_.get());
    state_interface.registerHandle(state_handle);

    hardware_interface::candle::CANdleCommandHandle command_handle(state_handle, command_.get());
    command_interface.registerHandle(command_handle);

    if (!local_update_)
    {
        hardware_interface::candle::CANdleWritableStateHandle remote_handle(getName(), state_.get());
        remote_state_interface.registerHandle(remote_handle);
    }
}

// Used to clear out previous references to these LEDs.
// Clear any animations which reference the LEDs and set
// the LEDs to off. Used to provide a clean slate for new
// writes to the LEDs, so only one value to them is active
// at any given point in time.
template <bool SIMFLAG>
void CANdleDevice<SIMFLAG>::clearLEDs(size_t start, size_t count)
{
    const size_t group_max = start + count;
    for (size_t led_id = start; led_id < group_max; led_id++) {
        const std::optional<hardware_interface::candle::LED> led = state_->getLED(led_id);
        if (led.has_value()) {
            const auto animation_id = led->getAnimationID();
            if (animation_id.has_value()) {
                safeCANdleCall(candle_->ClearAnimation(*animation_id), "candle_->ClearAnimation");

                hardware_interface::candle::Animation existing_animation = state_->getAnimation(*animation_id).value();
                safeCANdleCall(
                candle_->SetLEDs(
                    0,
                    0,
                    0,
                    0,
                    existing_animation.start,
                    existing_animation.count
                ),
                "candle_->SetLEDs");

                command_->clearCurrentAnimation(existing_animation);
                state_->clearAnimation(*animation_id);
            }
        }
    }
}

template <bool SIMFLAG>
void CANdleDevice<SIMFLAG>::write(const ros::Time &/*time*/, const ros::Duration &/*period*/)
{
    if (!local_hardware_)
    {
        return;
    }

    // For each Command property, check if it changed
    //  if it did change, update the actual CANdle, and update the state

    // Brightness
    if (double brightness; command_->brightnessChanged(brightness)) {
        if (safeCANdleCall(
            candle_->ConfigBrightnessScalar(brightness),
            "candle_->ConfigBrightnessScalar"
        )) {
            ROS_INFO_STREAM("CANdle " << getName() << " : Set brightness to " << brightness);
            state_->setBrightness(brightness);
        } else {
            command_->resetBrightnessChanged();
        }
    }

    // Show status LED when active
    if (bool status_led_when_active; command_->statusLEDWhenActiveChanged(status_led_when_active)) {
        if (safeCANdleCall(
            candle_->ConfigStatusLedState(!status_led_when_active),
            "candle_->ConfigStatusLedState"
        )) {
            ROS_INFO_STREAM("CANdle " << getName() << " : Set status led when active to " << status_led_when_active);
            state_->setStatusLEDWhenActive(status_led_when_active);
        } else {
            command_->resetStatusLEDWhenActiveChanged();
        }
    }

    // Enabled
    if (bool enabled; command_->enabledChanged(enabled)) {
        if (safeCANdleCall(
            candle_->configV5Enabled(enabled),
            "candle_->configV5Enabled"
        )) {
            ROS_INFO_STREAM("CANdle " << getName() << " : Set enabled to " << enabled);
            state_->setEnabled(enabled);
        } else {
            command_->resetEnabledChanged();
        }
    }

    // Stop animations
    if (bool stop_animations; command_->stopAnimationsChanged(stop_animations)) {
        size_t max_animations = 8;
        if constexpr (!SIMFLAG)
        {
            max_animations = (size_t)candle_->GetMaxSimultaneousAnimationCount();
        }
        for (size_t i = 0; i < max_animations; i++) {
            safeCANdleCall(candle_->ClearAnimation(i), "candle_->ClearAnimation");
        }

        command_->drainAnimations();
        // state_->setMaxAnimations(max_animations);
        state_->clearAnimations();
    }

    // Animation
    if (std::vector<hardware_interface::candle::Animation> candle_animations;
        command_->animationsChanged(candle_animations)) {
        for (const hardware_interface::candle::Animation& candle_animation : candle_animations) {
            // Clear any old animations for the range of LEDs this animation will cover
            clearLEDs(candle_animation.start, candle_animation.count);

            auto animation_slot = state_->getNextAnimationSlot();
            if (animation_slot == hardware_interface::candle::MAX_ANIMATION_SLOTS) {
                ROS_ERROR_STREAM("Failed to set CANdle animation - no available slots!");
                continue;
            }
            ROS_INFO_STREAM("Setting animation slot " << animation_slot);
            if (safeCANdleCall(
                candle_->Animate(*convertAnimation(candle_animation), animation_slot),
                "candle_->Animate"
            )) {
                ROS_INFO_STREAM("CANdle " << getName() << " : Changed its animation");
                state_->setAnimation(candle_animation);
            } else {
                // Remove the failed animation from the list the command interface
                // thinks is active then re-submit the failed animation so we retry
                // it next update period
                command_->clearCurrentAnimation(candle_animation);
                command_->setAnimation(candle_animation);
            }
        }
    }

    // LEDs
    std::vector<hardware_interface::candle::LEDGroup> led_groups;
    if (command_->ledGroupsChanged(led_groups)) {
        for (hardware_interface::candle::LEDGroup group : led_groups) {
            // Clear any old animations for the range of LEDs this animation will cover
            clearLEDs(group.start, group.count);

            if (safeCANdleCall(
                candle_->SetLEDs(
                    group.colour.red,
                    group.colour.green,
                    group.colour.blue,
                    group.colour.white,
                    group.start,
                    group.count
                ),
                "candle_->SetLEDs"
            )) {
                for (size_t led_id = group.start; led_id < static_cast<size_t>(group.start + group.count); led_id++) {
                    state_->setLED(led_id, group.colour);
                }

                ROS_INFO_STREAM("CANdle " << getName() << " : Changed colours");
            } else {
                // On an error, just re-queue the failed LED group
                command_->setLEDGroup(group);
            }
        }
    }
}

using namespace hardware_interface::candle;
using namespace ctre::phoenix::led;
static ColorFlowAnimation::Direction convertCANdleDirection(int direction) {
    switch (direction) {
        case 0:
            return ColorFlowAnimation::Direction::Forward;
        case 1:
            return ColorFlowAnimation::Direction::Backward;
        default:
            ROS_ERROR_STREAM("Invalid int to convert to CANdle direction! Defaulting to Forwards.");
            return ColorFlowAnimation::Direction::Forward;
    }
}

static std::unique_ptr<ctre::phoenix::led::Animation> convertAnimation(hardware_interface::candle::Animation animation) {
    Colour colour = animation.colour;
    switch (animation.type) {
        case AnimationType::Fire: {
            return std::make_unique<FireAnimation>(
                animation.brightness,
                animation.speed,
                animation.count,
                animation.param4,
                animation.param5,
                animation.reversed,
                animation.start
            );
        }
        case AnimationType::Rainbow: {
            return std::make_unique<RainbowAnimation>(
                animation.brightness,
                animation.speed,
                animation.count,
                animation.reversed,
                animation.start
            );
        }
        case AnimationType::RGBFade: {
            return std::make_unique<RgbFadeAnimation>(
                animation.brightness,
                animation.speed,
                animation.count,
                animation.start
            );
        }
        case AnimationType::ColourFlow: {
            return std::make_unique<ColorFlowAnimation>(
                colour.red,
                colour.green,
                colour.blue,
                colour.white,
                animation.speed,
                animation.count,
                convertCANdleDirection(animation.direction),
                animation.start
            );
        }
        case AnimationType::Larson:
        {
            return std::make_unique<LarsonAnimation>(
                colour.red,
                colour.green,
                colour.blue,
                colour.white,
                animation.speed,
                animation.count,
                // TODO: Store Bounce mode and Size arguments in animation class
                LarsonAnimation::BounceMode::Front,
                2,
                animation.start
            );
        }
        case AnimationType::SingleFade:
        {
            return std::make_unique<SingleFadeAnimation>(
                colour.red,
                colour.green,
                colour.blue,
                colour.white,
                animation.speed,
                animation.count,
                animation.start
            );
        }
        case AnimationType::Strobe:
        {
            return std::make_unique<StrobeAnimation>(
                colour.red,
                colour.green,
                colour.blue,
                colour.white,
                animation.speed,
                animation.count,
                animation.start
            );
        }
        case AnimationType::Twinkle:
        {
            return std::make_unique<TwinkleAnimation>(
                colour.red,
                colour.green,
                colour.blue,
                colour.white,
                animation.speed,
                animation.count,
                // TODO: Store actual Divider value
                TwinkleAnimation::TwinklePercent::Percent100,
                animation.start
            );
        }
        case AnimationType::TwinkleOff:
        {
            return std::make_unique<TwinkleOffAnimation>(
                colour.red,
                colour.green,
                colour.blue,
                colour.white,
                animation.speed,
                animation.count,
                // TODO: Store actual Divider value
                TwinkleOffAnimation::TwinkleOffPercent::Percent100,
                animation.start
            );
        }
        default: {
            ROS_ERROR_STREAM("Invalid animation type in " << __FUNCTION__ << " " << static_cast<int>(animation.type));
            return {};
        }
    }
}
