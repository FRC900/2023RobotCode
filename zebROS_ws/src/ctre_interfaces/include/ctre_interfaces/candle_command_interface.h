#ifndef CANDLE_COMMAND_INTERFACE_INC__
#define CANDLE_COMMAND_INTERFACE_INC__

#include "ctre_interfaces/candle_state_interface.h"
#include "state_handle/command_handle.h"
#include <optional>
#include <variant>
#include <vector>

namespace hardware_interface::candle {

struct LEDGroup {
    // Start LED
    int start;
    // Number of LEDs this group covers
    int count;
    // Colour to set the LEDs to
    Colour colour;

    // Constructor
    LEDGroup(int start, int count, Colour colour) :
        start{start},
        count{count},
        colour{colour}
    {}
    // Blank constructor for arrays and vectors
    LEDGroup() = default;
};

class CANdleHWCommand {
    public:
        // Constructor
        CANdleHWCommand();

        CANdleHWCommand(const CANdleHWCommand& other) = delete;
        CANdleHWCommand(CANdleHWCommand&& other) noexcept = delete;
        CANdleHWCommand& operator=(const CANdleHWCommand& other) = delete;
        CANdleHWCommand& operator=(CANdleHWCommand&& other) noexcept = delete;
        virtual ~CANdleHWCommand() = default;

        // Set colour of LEDs
        void setLEDGroup(const LEDGroup& leds);
        bool ledGroupsChanged(std::vector<LEDGroup>& groups);
        void drainLEDGroups();

        // Set brightness of LEDs
        void setBrightness(const double brightness);
        double getBrightness();
        bool brightnessChanged(double& brightness);
        void resetBrightnessChanged();

        // If the status LED should be shown when running
        void setStatusLEDWhenActive(const bool show);
        bool getStatusLEDWhenActive();
        bool statusLEDWhenActiveChanged(bool& show);
        void resetStatusLEDWhenActiveChanged();

        // If the CANdle is enabled
        void setEnabled(const bool enabled);
        bool getEnabled();
        bool enabledChanged(bool& enabled);
        void resetEnabledChanged();

        // The CANdle's animation
        void setAnimation(const Animation &animation);
        bool animationsChanged(std::vector<Animation>& animation);
        void drainAnimations();

        void clearCurrentAnimation(const Animation &animation);

        // Stop CANdle animations
        void stopAnimations();
        bool stopAnimationsChanged(bool& stop);

    private:
        // LEDs to be written
        std::vector<std::optional<Colour>> leds;
        bool leds_changed{true};
        // Brightness of LEDs
        double brightness{1.0};
        bool brightness_changed{true};
        // Status LED when active
        bool show_status_led_when_active{true};
        bool status_led_changed{true};
        // If the CANdle is enabled
        bool enabled{true};
        bool enabled_changed{true};
        // Animations to be written
        std::vector<Animation> animations;
        bool animation_changed{false};

        // Animations already written - used to prevent duplicate
        // writes of already-programmed animations
        std::vector<Animation> current_animations;

        // If we should reset all the animations
        bool stop_animations{true};
};


using CANdleCommandHandle = CommandHandle<CANdleHWCommand, CANdleHWState, CANdleStateHandle>;
class CANdleCommandInterface : public HardwareResourceManager<CANdleCommandHandle, ClaimResources> {};

} // namespace hardware_interface::candle

#endif