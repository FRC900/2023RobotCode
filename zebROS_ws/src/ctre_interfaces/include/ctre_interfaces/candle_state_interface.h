#pragma once

#include <vector>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include "state_handle/state_handle.h"
#include <optional>

namespace hardware_interface {
namespace candle {

// An enum for all the animations the CANdle can play
enum class CANdleAnimationType {
    ColourFlow,
    Fire,
    Larson,
    Rainbow,
    RGBFade,
    SingleFade,
    Strobe,
    Twinkle,
    TwinkleOff
};
enum class CANdleAnimationClass {
    BaseStandard,
    BaseTwo
};

// A CANdle colour
struct CANdleColour {
    int red;
    int green;
    int blue;
    int white;

    // Constructor
    CANdleColour(int red, int green, int blue, int white);
    CANdleColour();

    bool operator!=(const CANdleColour& rhs);
    bool operator==(const CANdleColour& rhs);
};

// An animation for the CANdle to play
struct CANdleAnimation {
    // Animation speed
    double speed;
    // Initial LED to animate
    int start;
    // Number of LEDs to animation
    int count;
    // The animation to play
    CANdleAnimationType type;
    // The animation class type
    CANdleAnimationClass class_type;

    // For Base Two animations
    // Animation colour
    CANdleColour colour;
    // Animation direction
    int direction;

    // For Base Standard animations
    // Animation brightness
    double brightness;
    // If the animation is reversed
    bool reversed;
    // Extra params
    double param4;
    double param5;

    // Constructor for BaseStandard animations
    CANdleAnimation(double speed, int start, int count, CANdleAnimationType type, double brightness, bool reversed, double param4, double param5);
    // Constructor for BaseTwo animations
    CANdleAnimation(double speed, int start, int count, CANdleAnimationType type, int red, int green, int blue, int white, int direction);
    // Blank constructor/null
    CANdleAnimation();

    // Comparison methods
    bool operator==(const CANdleAnimation& rhs);
    bool operator!=(const CANdleAnimation& rhs);
};

class CANdleHWState {
    public:
        // Constructor and method to get device ID
        CANdleHWState(int id);
        int getDeviceID() const;

        // Set the colour of an LED
        void setLED(int id, CANdleColour led);
        CANdleColour getLED(int id);

        // Set the brightness of the CANdle's LEDs
        void setBrightness(double brightness);
        double getBrightness();

        // Show status LED when the CANdle is being controlled
        void setStatusLEDWhenActive(bool show);
        bool getStatusLEDWhenActive();

        // If the CANdle is enabled
        void setEnabled(bool enabled);
        bool getEnabled();

        // The CANdle's animation
        void setAnimation(CANdleAnimation animation);
        std::optional<CANdleAnimation>& getAnimation();

    private:
        // The CAN ID of this CANdle
        int device_id;
        // All of the LED groups to colour
        std::vector<CANdleColour> leds;
        // The brightness of the LEDs in the CANdle, from 0->1
        double brightness;
        // If the status LED should be on when the CANdle is being controlled
        bool show_led_when_active;
        // If the CANdle is enabled
        bool enabled;
        // The currently playing CANdle animation
        std::optional<CANdleAnimation> animation;
};


typedef StateHandle<const CANdleHWState> CANdleStateHandle;
typedef StateHandle<CANdleHWState>       CANdleWritableStateHandle;
class CANdleStateInterface : public HardwareResourceManager<CANdleStateHandle> {};
class RemoteCANdleStateInterface : public HardwareResourceManager<CANdleWritableStateHandle> {};

} // namespace candle
} // namespace hardware_interface
