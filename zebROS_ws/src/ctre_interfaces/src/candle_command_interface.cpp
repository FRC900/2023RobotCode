#include "ctre_interfaces/candle_command_interface.h"

namespace hardware_interface {
namespace candle {

LEDGroup::LEDGroup(int start, int count, int red, int green, int blue) {
    this->start = start;
    this->count = count;
    this->red = red;
    this->green = green;
    this->blue = blue;
}
LEDGroup::LEDGroup() {}

CANdleHWCommand::CANdleHWCommand(int device_id) :
    device_id{device_id},
    led_groups(10),
    brightness{1},
    brightness_changed{false},
    show_status_led_when_active{true},
    status_led_changed{false},
    enabled{true},
    enabled_changed{false}
{}

int CANdleHWCommand::getDeviceID() {
    return this->device_id;
}

void CANdleHWCommand::setLEDGroup(LEDGroup leds) {
    this->led_groups.push_back(leds);
}
LEDGroup CANdleHWCommand::getLEDGroup(int id) {
    return this->led_groups[id];
}
bool CANdleHWCommand::ledGroupChanged(vector<LEDGroup>& groups) {
    if (this->led_groups.size() > 0) {
        for (LEDGroup group : this->led_groups) {
            groups.push_back(group);
        }
        this->led_groups.clear();
        return true;
    }
    return false;
}

void CANdleHWCommand::setBrightness(double brightness) {
    this->brightness = brightness;
    this->brightness_changed = true;
}
double CANdleHWCommand::getBrightness() {
    return this->brightness;
}
bool CANdleHWCommand::brightnessChanged(double& brightness) {
    if (this->brightness_changed) {
        this->brightness_changed = false;
        brightness = this->brightness;
        return true;
    }
    return false;
}

void CANdleHWCommand::showStatusLEDWhenActive(bool show) {
    this->show_status_led_when_active = show;
    this->status_led_changed = true;
}
bool CANdleHWCommand::getStatusLEDWhenActive() {
    return this->show_status_led_when_active;
}
bool CANdleHWCommand::statusLEDWhenActiveChanged(bool& show) {
    if (this->status_led_changed) {
        this->status_led_changed = false;
        show = this->show_status_led_when_active;
        return true;
    }
    return false;
}

void CANdleHWCommand::setEnabled(bool enabled) {
    this->enabled = enabled;
    this->enabled_changed = true;
}
bool CANdleHWCommand::getEnabled() {
    return this->enabled;
}
bool CANdleHWCommand::enabledChanged(bool& enabled) {
    if (this->enabled_changed) {
        this->enabled_changed = false;
        enabled = this->enabled;
        return true;
    }
    return false;
}

void CANdleHWCommand::setAnimation(CANdleAnimation* animation) {
    this->animation = animation;
    this->animation_changed = true;
}
CANdleAnimation* CANdleHWCommand::getAnimation() {
    return this->animation;
}
bool CANdleHWCommand::animationChanged(CANdleAnimation* animation) {
    if (this->animation_changed) {
        this->animation_changed = false;
        animation = this->animation;
        return true;
    }
    return false;
}

}
}
