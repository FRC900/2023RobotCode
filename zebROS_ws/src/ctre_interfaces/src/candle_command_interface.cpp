#include "ctre_interfaces/candle_command_interface.h"

namespace hardware_interface {
namespace candle {

LEDGroup::LEDGroup(int start, int count, CANdleColour colour) {
    this->start = start;
    this->count = count;
    this->colour = colour;
}
LEDGroup::LEDGroup() {}

CANdleHWCommand::CANdleHWCommand() :
    led_groups(10),
    brightness{1},
    brightness_changed{false},
    show_status_led_when_active{true},
    status_led_changed{false},
    enabled{true},
    enabled_changed{false},
    animation{nullptr},
    animation_changed{false}
{}

void CANdleHWCommand::setLEDGroup(LEDGroup leds) {
    this->led_groups.push_back(leds);
}
LEDGroup* CANdleHWCommand::getLEDGroup(int id) {
    return &(this->led_groups[id]);
}
vector<LEDGroup>& CANdleHWCommand::getAllLEDGroups() {
    return this->led_groups;
}
void CANdleHWCommand::removeLEDGroup(int id) {
    this->led_groups.erase(this->led_groups.begin() + id);
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
void CANdleHWCommand::resetBrightnessChanged() {
    this->brightness_changed = true;
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
void CANdleHWCommand::resetStatusLEDWhenActiveChanged() {
    this->status_led_changed = true;
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
void CANdleHWCommand::resetEnabledChanged() {
    this->enabled_changed = true;
}

void CANdleHWCommand::setAnimation(CANdleAnimation* animation) {
    this->animation = animation;
    this->animation_changed = true;
}
CANdleAnimation* CANdleHWCommand::getAnimation() {
    return this->animation;
}
bool CANdleHWCommand::animationChanged(CANdleAnimation*& animation) {
    if (this->animation_changed) {
        this->animation_changed = false;
        animation = this->animation;
        return true;
    }
    return false;
}
void CANdleHWCommand::resetAnimationChanged() {
    this->animation_changed = true;
}

}
}
