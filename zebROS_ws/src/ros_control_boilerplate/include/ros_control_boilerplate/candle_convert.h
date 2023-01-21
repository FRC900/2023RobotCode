#pragma once

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
#include "ctre_interfaces/candle_state_interface.h"

using namespace ctre::phoenix::led;
using namespace hardware_interface::candle;

namespace candle_convert {

ColorFlowAnimation::Direction convertCANdleDirection(int direction);
BaseStandardAnimation convertBaseStandardAnimation(CANdleAnimation animation);
BaseTwoSizeAnimation convertBaseTwoAnimation(CANdleAnimation animation);

} // namespace candle_convert
