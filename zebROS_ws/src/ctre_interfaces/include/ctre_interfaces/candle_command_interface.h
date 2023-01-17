#pragma once

#include "ctre_interfaces/candle_state_interface.h"
#include "state_handle/command_handle.h"
#include <vector>

using namespace std;

namespace hardware_interface {
namespace candle {

struct LEDGroup {
    // Start LED
    int start;
    // Number of LEDs this group covers
    int count;
    // Colour to set for these LEDs
    int red;
    int green;
    int blue;

    // Constructor
    LEDGroup(int start, int count, int red, int green, int blue);
    // Blank constructor for arrays and vectors
    LEDGroup();
};

class CANdleHWCommand {
    public:
        // Constructor
        CANdleHWCommand(int device_id);
        // Get CAN ID
        int getDeviceID();

        // Set colour of LEDs
        void setLEDGroup(LEDGroup leds);
        LEDGroup getLEDGroup(int id);
        bool ledGroupChanged(vector<LEDGroup>& groups);

        // Set brightness of LEDs
        void setBrightness(double brightness);
        double getBrightness();
        bool brightnessChanged(double& brightness);

        // If the status LED should be shown when running
        void showStatusLEDWhenActive(bool show);
        bool getStatusLEDWhenActive();
        bool statusLEDWhenActiveChanged(bool& show);

        // If the CANdle is enabled
        void setEnabled(bool enabled);
        bool getEnabled();
        bool enabledChanged(bool& enabled);

        // The CANdle's animation
        void setAnimation(CANdleAnimation* animation);
        CANdleAnimation* getAnimation();
        bool animationChanged(CANdleAnimation* animation);

    private:
        // CAN ID
        int device_id;
        // LED groups to be written
        vector<LEDGroup> led_groups;
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
        CANdleAnimation* animation;
        bool animation_changed;
};


typedef CommandHandle<CANdleHWCommand, CANdleHWState, CANdleStateHandle> CANdleCommandHandler;
class CANdleCommandInterface : public HardwareResourceManager<CANdleCommandHandler, ClaimResources> {};

} // namespace candle
} // namespace hardware_interface
