#include "ctre_interfaces/candle_state_interface.h"

namespace hardware_interface {
namespace candle {

Colour::Colour(int red, int green, int blue, int white) {
    this->red = red;
    this->green = green;
    this->blue = blue;
    this->white = white;
}
Colour::Colour() {}
bool Colour::operator!=(const Colour& rhs) {
    return !(this->red == rhs.red && this->green == rhs.green && this->blue == rhs.blue && this->white == rhs.white);
}
bool Colour::operator==(const Colour& rhs) {
    return (
        this->red == rhs.red &&
        this->green == rhs.green &&
        this->blue == rhs.blue &&
        this->white == rhs.white
    );
}

Animation::Animation(double speed, int start, int count, AnimationType type, double brightness, bool reversed, double param4, double param5) {
    this->speed = speed;
    this->start = start;
    this->count = count;
    this->type = type;
    this->class_type = AnimationClass::BaseStandard;
    this->brightness = brightness;
    this->reversed = reversed;
    this->param4 = param4;
    this->param5 = param5;
}
Animation::Animation(double speed, int start, int count, AnimationType type, int red, int green, int blue, int white, int direction) {
    this->speed = speed;
    this->start = start;
    this->count = count;
    this->type = type;
    this->class_type = AnimationClass::BaseTwo;
    this->colour = Colour(red, green, blue, white);
    this->direction = direction;
}
Animation::Animation() {}

bool Animation::operator==(const Animation& rhs) {
    if (this->class_type == rhs.class_type) {
        if (this->class_type == AnimationClass::BaseStandard) {
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
bool Animation::operator!=(const Animation& rhs) {
    return !(this->operator==(rhs));
}

CANdleHWState::CANdleHWState(int id) :
    device_id{id},
    brightness{1},
    leds(8)
{
    for (int i = 0; i < 8; i++) {
        this->leds[i] = Colour(255, 255, 255, 255);
    }
}

void CANdleHWState::setLED(int id, Colour colour) {
    this->leds[id] = LED(colour);
}
void CANdleHWState::setLED(int id, int animation_id) {
    this->leds[id] = LED(animation_id);
}
void CANdleHWState::setLEDOff(int id) {
    this->leds[id] = LED();
}
std::optional<LED> CANdleHWState::getLED(size_t id) {
    if (id < this->leds.size()) {
        return std::optional<LED>(this->leds[id]);
    } else {
        return std::nullopt;
    }
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

void CANdleHWState::setAnimation(Animation animation) {
    size_t animation_id = this->getNextAnimationSlot();
    this->animations[animation_id].emplace(animation);
    for (size_t led_id = animation.start; led_id < (size_t)(animation.start + animation.count); led_id++) {
        this->setLED(led_id, animation_id);
    }
}
void CANdleHWState::clearAnimation(int id) {
    if (this->animations[id].has_value()) {
        Animation animation = this->animations[id].value();
        for (size_t led_id = animation.start; led_id < (size_t)(animation.start + animation.count); led_id++) {
            this->setLEDOff(led_id);
        }
        //this->animations.erase(animations.begin() + id);
        this->animations[id].reset();
    }
}
std::optional<Animation> CANdleHWState::getAnimation(size_t id) {
    if (id < 8) {
        return std::optional<Animation>{this->animations[id]};
    }

    return std::optional<Animation>{std::nullopt};
}
size_t CANdleHWState::getNextAnimationSlot() {
    //return this->animations.size();
    for (size_t animation_id = 0; animation_id < 8; animation_id++) {
        if (!(this->animations[animation_id].has_value())) {
            return animation_id;
        }
    }
    ROS_ERROR_STREAM("Failed to get available animation slot!");
    return 9;
}

int CANdleHWState::getDeviceID() const {
    return this->device_id;
}

} // namespace candle
} // namespace hardware_interface