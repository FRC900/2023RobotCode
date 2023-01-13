#include "ctre_interfaces/candle_state_interface.h"

namespace hardware_interface {
namespace candle {

CANdleAnimation::CANdleAnimation(int id, double speed, int start, int count, CANdleAnimationType animation) {
    this->id = id;
    this->speed = speed;
    this->start = start;
    this->count = count;
    this->animation = animation;
}
CANdleAnimation::CANdleAnimation() {}
BaseTwoCANdleAnimation::BaseTwoCANdleAnimation(int id, double speed, int start, int count, CANdleAnimationType animation, int red, int green, int blue, int white, int direction)
: CANdleAnimation(id, speed, start, count, animation)
{
    this->red = red;
    this->green = green;
    this->blue = blue;
    this->white = white;
    this->direction = direction;
}
BaseStandardCANdleAnimation::BaseStandardCANdleAnimation(int id, double speed, int start, int count, CANdleAnimationType animation, double brightness, bool reversed, double param4, double param5)
: CANdleAnimation(id, speed, start, count, animation)
{
    this->brightness = brightness;
    this->reversed = reversed;
    this->param4 = param4;
    this->param5 = param5;
}

LED::LED(int id, int red, int green, int blue) {
    this->id = id;
    this->red = red;
    this->green = green;
    this->blue = blue;
}
LED::LED() {}

CANdleHWState::CANdleHWState(int id) :
    device_id{id},
    brightness{1},
    leds(8),
    animation()
{
    for (int i = 0; i < 8; i++) {
        this->leds[i] = LED(i, 255, 255, 255);
    }
}

void CANdleHWState::setLED(int id, int red, int green, int blue) {
    LED led = this->leds[id];
    led.red = red;
    led.green = green;
    led.blue = blue;
}
LED CANdleHWState::getLED(int id) {
    return this->leds[id];
}

void CANdleHWState::setBrightness(double brightness) {
    this->brightness = brightness;
}
double CANdleHWState::getBrightness() {
    return this->brightness;
}

void CANdleHWState::showStatusLEDWhenActive(bool show) {
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
    this->animation = animation;
}
CANdleAnimation CANdleHWState::getAnimation() {
    return this->animation;
}

} // namespace candle
} // namespace hardware_interface