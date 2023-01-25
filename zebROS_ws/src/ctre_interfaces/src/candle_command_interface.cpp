#include "ctre_interfaces/candle_command_interface.h"

namespace hardware_interface {
namespace candle {

LEDGroup::LEDGroup(int start, int count, int red, int green, int blue, int white)
:
    CANdleColour(red, green, blue, white)
{
    this->start = start;
    this->count = count;
}
LEDGroup::LEDGroup() {}

CANdleHWCommand::CANdleHWCommand() :
    brightness{1},
    brightness_changed{false},
    show_status_led_when_active{true},
    status_led_changed{false},
    enabled{true},
    enabled_changed{false},
    animation_changed{false}
{}

void CANdleHWCommand::setLEDGroup(LEDGroup leds)
{
    size_t max = leds.start + leds.count;
    for (size_t i = leds.start; i < max; i++) {
        if (i < this->leds.size()) {
            this->leds[i] = CANdleColour(
                leds.red,
                leds.green,
                leds.blue,
                leds.white
            );
        } else {
            this->leds.emplace_back(CANdleColour(
                leds.red,
                leds.green,
                leds.blue,
                leds.white
            ));
        }
    }
    this->leds_changed = true;
}
bool CANdleHWCommand::ledGroupChanged(vector<LEDGroup>& groups) {
    groups.clear();

    if (this->leds_changed) {
        std::optional<CANdleColour> previous_colour;
        for (size_t i = 0; i < this->leds.size(); i++) {
            std::optional<CANdleColour> led = this->leds[i];

            if (led.has_value()) {
                if (!previous_colour.has_value() || *previous_colour != *led) {
                    groups.emplace_back(LEDGroup(
                        i,
                        1,
                        led->red,
                        led->green,
                        led->blue,
                        led->white
                    ));
                } else {
                    groups.back().count += 1;
                }
            }

            previous_colour = led;
        }
        this->leds_changed = false;
        return true;
    }
    return false;
}
void CANdleHWCommand::resetLEDGroupChanged(LEDGroup& group) {
    this->setLEDGroup(group);
}
void CANdleHWCommand::drainLEDGroups() {
    this->leds.clear();
}

void CANdleHWCommand::setBrightness(double brightness) {
    if (this->brightness != brightness) {
        this->brightness = brightness;
        this->brightness_changed = true;
    }
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

void CANdleHWCommand::setStatusLEDWhenActive(bool show) {
    if (this->show_status_led_when_active != show) {
        this->show_status_led_when_active = show;
        this->status_led_changed = true;
    }
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
    if (this->enabled != enabled) {
        this->enabled = enabled;
        this->enabled_changed = true;
    }
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

void CANdleHWCommand::setAnimation(CANdleAnimation animation) {
    if (this->animation != animation) {
        this->animation = animation;
        this->animation_changed = true;
    }
}
CANdleAnimation& CANdleHWCommand::getAnimation() {
    return this->animation;
}
bool CANdleHWCommand::animationChanged(CANdleAnimation& animation) {
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
