#include "ctre_interfaces/candle_state_interface.h"

namespace hardware_interface {
namespace candle {

CANdleColour::CANdleColour(int red, int green, int blue, int white) {
    this->red = red;
    this->green = green;
    this->blue = blue;
    this->white = white;
}
CANdleColour::CANdleColour() {}
bool CANdleColour::operator!=(const CANdleColour& rhs) {
    return !(this->red == rhs.red && this->green == rhs.green && this->blue == rhs.blue && this->white == rhs.white);
}
bool CANdleColour::operator==(const CANdleColour& rhs) {
    return (
        this->red == rhs.red &&
        this->green == rhs.green &&
        this->blue == rhs.blue &&
        this->white == rhs.white
    );
}

CANdleAnimation::CANdleAnimation(double speed, int start, int count, CANdleAnimationType type, double brightness, bool reversed, double param4, double param5) {
    this->speed = speed;
    this->start = start;
    this->count = count;
    this->type = type;
    this->class_type = CANdleAnimationClass::BaseStandard;
    this->brightness = brightness;
    this->reversed = reversed;
    this->param4 = param4;
    this->param5 = param5;
}
CANdleAnimation::CANdleAnimation(double speed, int start, int count, CANdleAnimationType type, int red, int green, int blue, int white, int direction) {
    this->speed = speed;
    this->start = start;
    this->count = count;
    this->type = type;
    this->class_type = CANdleAnimationClass::BaseTwo;
    this->colour = CANdleColour(red, green, blue, white);
    this->direction = direction;
}
CANdleAnimation::CANdleAnimation() {}

bool CANdleAnimation::operator==(const CANdleAnimation& rhs) {
    if (this->class_type == rhs.class_type) {
        if (this->class_type == CANdleAnimationClass::BaseStandard) {
            return (
                (this->speed == rhs.speed) &&
                (this->start == rhs.start) &&
                (this->count == rhs.count) &&
                (this->brightness == rhs.brightness) &&
                (this->param4 == rhs.param4) &&
                (this->param5 == rhs.param5) &&
                (this->type == rhs.type)
            );
        } else {
            return (
                (this->speed == rhs.speed) &&
                (this->start == rhs.start) &&
                (this->count == rhs.count) &&
                (this->colour == rhs.colour) &&
                (this->direction == rhs.direction) &&
                (this->type == rhs.type)
            );
        }
    } else {
        return false;
    }
}
bool CANdleAnimation::operator!=(const CANdleAnimation& rhs) {
    return !(this->operator==(rhs));
}

CANdleHWState::CANdleHWState(int id) :
    device_id{id},
    brightness{1},
    leds(8),
    animation()
{
    for (int i = 0; i < 8; i++) {
        this->leds[i] = CANdleColour(255, 255, 255, 255);
    }
}

void CANdleHWState::setLED(int id, CANdleColour led) {
    this->leds[id] = led;
}
CANdleColour CANdleHWState::getLED(int id) {
    return this->leds[id];
}

void CANdleHWState::setBrightness(double brightness) {
    this->brightness = brightness;
}
double CANdleHWState::getBrightness() {
    return this->brightness;
}

void CANdleHWState::setStatusLEDWhenActive(bool show) {
    this->show_led_when_active = show;
}
bool CANdleHWState::getStatusLEDWhenActive() {
    return this->show_led_when_active;
}

void CANdleHWState::setEnabled(bool enabled) {
    this->enabled = enabled;
}
bool CANdleHWState::getEnabled() {
    return this->enabled;
}

void CANdleHWState::setAnimation(CANdleAnimation animation) {
    this->animation.emplace(animation);
}
std::optional<CANdleAnimation>& CANdleHWState::getAnimation() {
    return this->animation;
}

int CANdleHWState::getDeviceID() const {
    return this->device_id;
}

} // namespace candle
} // namespace hardware_interface