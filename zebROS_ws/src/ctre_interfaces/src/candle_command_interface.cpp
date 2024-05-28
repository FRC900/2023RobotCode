#include "ctre_interfaces/candle_command_interface.h"

namespace hardware_interface::candle {

CANdleHWCommand::CANdleHWCommand()
{
    for (size_t i = 0; i < 8; i++) {
        this->leds.emplace_back(Colour(255, 255, 255, 255));
    }
}

void CANdleHWCommand::setLEDGroup(const LEDGroup& leds)
{
    size_t max = leds.start + leds.count;
    for (size_t i = leds.start; i < max; i++) {
        if (i < this->leds.size()) {
            this->leds[i] = Colour(
                leds.colour.red,
                leds.colour.green,
                leds.colour.blue,
                leds.colour.white
            );
        } else {
            if ((i - this->leds.size()) > 1) {
                this->leds.resize(i);
            }

            this->leds.emplace_back(Colour(
                leds.colour.red,
                leds.colour.green,
                leds.colour.blue,
                leds.colour.white
            ));
        }
    }
    this->leds_changed = true;
}
bool CANdleHWCommand::ledGroupsChanged(std::vector<LEDGroup>& groups) {
    groups.clear();

    if (this->leds_changed) {
        std::optional<Colour> previous_colour;
        for (size_t i = 0; i < this->leds.size(); i++) {
            std::optional<Colour> led = this->leds[i];

            if (led.has_value()) {
                // If the previous colour is none, or the colour has changed, start a new group
                if (!previous_colour.has_value() || previous_colour.value() != led.value()) {
                    groups.emplace_back(
                        i,
                        1,
                        Colour(
                            led->red,
                            led->green,
                            led->blue,
                            led->white
                        )
                    );
                // If the colour matches the last colour, just increase the number of LEDs in that group
                } else {
                    groups.back().count += 1;
                }
            }

            previous_colour = led;
        }
        this->leds_changed = false;
        this->leds.clear();
        return true;
    }
    return false;
}
void CANdleHWCommand::drainLEDGroups() {
    this->leds.clear();
}

void CANdleHWCommand::setBrightness(double brightness) {
    if (this->brightness != brightness) {
        ROS_INFO_STREAM("Setting CANdle brightness to: " << brightness);
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

void CANdleHWCommand::setAnimation(const Animation &animation) {
    // Check if the animation is already in the list to
    // update this time
    for (Animation& existing_animation : this->animations) {
        if (animation == existing_animation) {
            ROS_INFO_STREAM("Identical animation detected. Not updating");
            return;
        }
    }

    // Check if the animation matches an existing one already
    // written to hardware. This will prevent restarting the
    // animation if it's already running.
    for (const Animation& a : this->current_animations) {
        if (a == animation) {
            ROS_INFO_STREAM("New animation matching existing animation. Not updating");
            return;
        }
    }

    this->animations.push_back(animation);
    this->animation_changed = true;

    // Previously checked for an exact match, so here we
    // can just check start and size and if there's an
    // existing animation with the same start and size,
    // overwrite the rest of the info with the requested animation
    for (size_t i = 0; i < this->current_animations.size(); i++) {
        if ((this->current_animations[i].start == animation.start) &&
            (this->current_animations[i].count == animation.count)) {
            this->current_animations[i] = animation;
            return;
        }
    }
    this->current_animations.push_back(animation);
}
bool CANdleHWCommand::animationsChanged(std::vector<Animation>& animations) {
    if (this->animation_changed) {
        this->animation_changed = false;
        animations = this->animations;
        this->animations.clear();
        return true;
    }
    return false;
}
void CANdleHWCommand::drainAnimations() {
    this->animations.clear();
}

// Clear an animation from the list of currently active animations
// Do this by passing the animation config rather than the index
// since I'm not sure if the indexes will get out of sync between
// here and the state interface
void CANdleHWCommand::clearCurrentAnimation(const Animation &animation) {
    for (size_t i = 0; i < this->current_animations.size(); i++) {
        if (this->current_animations[i] == animation) {
            this->current_animations.erase(this->current_animations.begin() + i);
            return;
        }
    }
}

void CANdleHWCommand::stopAnimations() {
    this->stop_animations = true;
    this->current_animations.clear();
}
bool CANdleHWCommand::stopAnimationsChanged(bool& stop) {
    if (this->stop_animations) {
        stop = this->stop_animations;
        this->stop_animations = false;
        return true;
    }
    return false;
}

} // namespace hardware_interface::candle