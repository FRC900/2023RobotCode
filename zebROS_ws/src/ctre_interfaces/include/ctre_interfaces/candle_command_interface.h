#pragma once

#include "ctre_interfaces/candle_state_interface.h"
#include "state_handle/command_handle.h"
#include <vector>
#include <optional>

namespace hardware_interface {
namespace candle {

struct LEDGroup : CANdleColour {
    // Start LED
    int start;
    // Number of LEDs this group covers
    int count;
    // Colour is handled by CANdleColour superclass

    // Constructor
    LEDGroup(int start, int count, int red, int green, int blue, int white);
    // Blank constructor for arrays and vectors
    LEDGroup();
};

class CANdleHWCommand {
    public:
        // Constructor
        CANdleHWCommand();

        // Set colour of LEDs
        void setLEDGroup(LEDGroup leds);
        bool ledGroupChanged(std::vector<LEDGroup>& groups);
        void drainLEDGroups();

        // Set brightness of LEDs
        void setBrightness(double brightness);
        double getBrightness();
        bool brightnessChanged(double& brightness);
        void resetBrightnessChanged();

        // If the status LED should be shown when running
        void setStatusLEDWhenActive(bool show);
        bool getStatusLEDWhenActive();
        bool statusLEDWhenActiveChanged(bool& show);
        void resetStatusLEDWhenActiveChanged();

        // If the CANdle is enabled
        void setEnabled(bool enabled);
        bool getEnabled();
        bool enabledChanged(bool& enabled);
        void resetEnabledChanged();

        // The CANdle's animation
        void setAnimation(CANdleAnimation animation);
        CANdleAnimation& getAnimation();
        bool animationChanged(CANdleAnimation& animation);
        void resetAnimationChanged();

    private:
        // LED groups to be written
        std::vector<std::optional<CANdleColour>> leds;
        bool leds_changed;
        // Brightness of LEDs
        double brightness;
        bool brightness_changed;
        // Status LED when active
        bool show_status_led_when_active;
        bool status_led_changed;
        // If the CANdle is enabled
        bool enabled;
        bool enabled_changed;
        // The CANdle's animation
        CANdleAnimation animation;
        bool animation_changed;
};


typedef CommandHandle<CANdleHWCommand, CANdleHWState, CANdleStateHandle> CANdleCommandHandle;
class CANdleCommandInterface : public HardwareResourceManager<CANdleCommandHandle, ClaimResources> {};

} // namespace candle
} // namespace hardware_interface
