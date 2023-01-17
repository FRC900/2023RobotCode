#include "ros_control_boilerplate/candle_convert.h"

using namespace hardware_interface::candle;
using namespace ctre::phoenix::led;

namespace candle_convert {

ColorFlowAnimation::Direction convertCANdleDirection(int direction) {
    switch (direction) {
        case 0:
            return ColorFlowAnimation::Direction::Forward;
        case 1:
            return ColorFlowAnimation::Direction::Backward;
    }
}

BaseStandardAnimation convertBaseStandardAnimation(CANdleAnimation* animation) {
    bool reversed = animation->getReversed();
    double brightness = animation->getBrightness();

    switch (animation->type) {
        case CANdleAnimationType::Fire: {
            return FireAnimation(
                brightness,
                animation->speed,
                animation->count,
                animation->getParam4(),
                animation->getParam5(),
                reversed,
                animation->start
            );
        }
        case CANdleAnimationType::Rainbow: {
            return RainbowAnimation(
                brightness,
                animation->speed,
                animation->count,
                reversed,
                animation->start
            );
        }
        case CANdleAnimationType::RGBFade: {
            return RgbFadeAnimation(
                brightness,
                animation->speed,
                animation->count,
                animation->start
            );
        }
    }
}
BaseTwoSizeAnimation convertBaseTwoAnimation(CANdleAnimation* animation) {
    CANdleColour colour = animation->getColour();
    switch (animation->type) {
        case CANdleAnimationType::ColourFlow: {
            return ColorFlowAnimation(
                colour.red,
                colour.green,
                colour.blue,
                colour.white,
                animation->speed,
                animation->count,
                convertCANdleDirection(animation->getDirection())
            );
        }
        case CANdleAnimationType::Larson:
        {
            return LarsonAnimation(
                colour.red,
                colour.green,
                colour.blue,
                colour.white,
                animation->speed,
                animation->count,
                // TODO: Store Bounce mode and Size arguments in animation class
                LarsonAnimation::BounceMode::Front,
                2,
                animation->start
            );
        }
        case CANdleAnimationType::SingleFade:
        {
            return SingleFadeAnimation(
                colour.red,
                colour.green,
                colour.blue,
                colour.white,
                animation->speed,
                animation->count,
                animation->start
            );
        }
        case CANdleAnimationType::Strobe:
        {
            return StrobeAnimation(
                colour.red,
                colour.green,
                colour.blue,
                colour.white,
                animation->speed,
                animation->count,
                animation->start
            );
        }
        case CANdleAnimationType::Twinkle:
        {
            return TwinkleAnimation(
                colour.red,
                colour.green,
                colour.blue,
                colour.white,
                animation->speed,
                animation->count,
                // TODO: Store actual Divider value
                TwinkleAnimation::TwinklePercent::Percent100,
                animation->start
            );
        }
        case CANdleAnimationType::TwinkleOff:
        {
            return TwinkleOffAnimation(
                colour.red,
                colour.green,
                colour.blue,
                colour.white,
                animation->speed,
                animation->count,
                // TODO: Store actual Divider value
                TwinkleOffAnimation::TwinkleOffPercent::Percent100,
                animation->start
            );
        }
    }
}

} // namespace candle_convert
