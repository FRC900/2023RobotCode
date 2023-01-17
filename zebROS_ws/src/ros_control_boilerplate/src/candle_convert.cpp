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

void convertCANdleAnimation(CANdleAnimation* animation, Animation& result) {
    switch (animation->type)
    {
        case CANdleAnimationType::ColourFlow: {
            CANdleColour colour = animation->getColour();
            result = ColorFlowAnimation(
                colour.red,
                colour.green,
                colour.blue,
                colour.white,
                animation->getSpeed(),
                animation->getLEDCount(),
                convertCANdleDirection(animation->getDirection())
            );
            break;
        }
        case CANdleAnimationType::Fire: {
            result = FireAnimation(
                animation->getBrightness(),
                animation->getSpeed(),
                animation->getLEDCount(),
                animation->getParam4(),
                animation->getParam5(),
                animation->getReversed(),
                animation->getLEDStart()
            );
            break;
        }
        case CANdleAnimationType::Larson: {
            CANdleColour colour = animation->getColour();
            result = LarsonAnimation(
                colour.red,
                colour.green,
                colour.blue,
                colour.white,
                animation->getSpeed(),
                animation->getLEDCount(),
                // TODO: Store Bounce mode and Size arguments in animation class
                LarsonAnimation::BounceMode::Front,
                2,
                animation->getLEDStart()
            );
            break;
        }
        case CANdleAnimationType::Rainbow: {
            result = RainbowAnimation(
                animation->getBrightness(),
                animation->getSpeed(),
                animation->getLEDCount(),
                animation->getReversed(),
                animation->getLEDStart()
            );
            break;
        }
        case CANdleAnimationType::RGBFade: {
            result = RgbFadeAnimation(
                animation->getBrightness(),
                animation->getSpeed(),
                animation->getLEDCount(),
                animation->getLEDStart()
            );
            break;
        }
        case CANdleAnimationType::SingleFade: {
            CANdleColour colour = animation->getColour();
            result = SingleFadeAnimation(
                colour.red,
                colour.green,
                colour.blue,
                colour.white,
                animation->getSpeed(),
                animation->getLEDCount(),
                animation->getLEDStart()
            );
            break;
        }
        case CANdleAnimationType::Strobe: {
            CANdleColour colour = animation->getColour();
            result = StrobeAnimation(
                colour.red,
                colour.green,
                colour.blue,
                colour.white,
                animation->getSpeed(),
                animation->getLEDCount(),
                animation->getLEDStart()
            );
            break;
        }
        case CANdleAnimationType::Twinkle: {
            CANdleColour colour = animation->getColour();
            result = TwinkleAnimation(
                colour.red,
                colour.green,
                colour.blue,
                colour.white,
                animation->getSpeed(),
                animation->getLEDCount(),
                // TODO: Store actual Divider value
                TwinkleAnimation::TwinklePercent::Percent100,
                animation->getLEDStart()
            );
            break;
        }
        case CANdleAnimationType::TwinkleOff: {
            CANdleColour colour = animation->getColour();
            result = TwinkleOffAnimation(
                colour.red,
                colour.green,
                colour.blue,
                colour.white,
                animation->getSpeed(),
                animation->getLEDCount(),
                // TODO: Store actual Divider value
                TwinkleOffAnimation::TwinkleOffPercent::Percent100,
                animation->getLEDStart()
            );
            break;
        }
    }
}

} // namespace candle_convert
