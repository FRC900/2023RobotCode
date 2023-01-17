#pragma once

#include <string>
#include <vector>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include "state_handle/state_handle.h"

using namespace std;

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

// A CANdle colour
struct CANdleColour {
    int red;
    int green;
    int blue;
    int white;

    // Constructor
    CANdleColour(int red, int green, int blue, int white);
};

// An animation for the CANdle to play
struct CANdleAnimation {
    // Animation ID
    int id;
    // Animation speed
    double speed;
    // Initial LED to animate
    int start;
    // Number of LEDs to animation
    int count;
    // The animation to play
    CANdleAnimationType type;

    // Constructor
    CANdleAnimation(int id, double speed, int start, int count, CANdleAnimationType type);
    // Blank constructor/null
    CANdleAnimation();

    // For all animations
    int getID();
    double getSpeed();
    int getLEDStart();
    int getLEDCount();
    CANdleAnimationType getType();
    // For base2 animations
    virtual CANdleColour getColour() = 0;
    virtual int getDirection() = 0;
    // For base standard animations
    virtual double getBrightness() = 0;
    virtual bool getReversed() = 0;
    virtual double getParam4() = 0;
    virtual double getParam5() = 0;
};
// Base2 animations have configurable colour and direction
struct BaseTwoCANdleAnimation : CANdleAnimation {
    // Animation colour
    CANdleColour colour;
    // Animation direction
    int direction;

    // Constructor
    BaseTwoCANdleAnimation(int id, double speed, int start, int count, CANdleAnimationType type, int red, int green, int blue, int white, int direction);
    // Overriden methods
    virtual CANdleColour getColour() override;
    virtual int getDirection() override;
};
// BaseStandard animations have configurable brightness, reversable direction, and 2 custom params
struct BaseStandardCANdleAnimation : CANdleAnimation {
    // Animation brightness
    double brightness;
    // If the animation should be reversed
    bool reversed;
    // Extra, configurable params
    double param4;
    double param5;

    // Constructor
    BaseStandardCANdleAnimation(int id, double speed, int start, int count, CANdleAnimationType type, double brightness, bool reversed, double param4, double param5);
    // Overriden methods
    virtual double getBrightness() override;
    virtual bool getReversed() override;
    virtual double getParam4() override;
    virtual double getParam5() override;
};

// An LED in the CANdle, and it's current colour
struct LED {
    // The LED's number (0->7)
    int id;
    // The RGB colour values
    int red;
    int green;
    int blue;

    // Constructor
    LED(int id, int red, int green, int blue);
    // Blank constructor for arrays/vectors
    LED();
};

class CANdleHWState {
    public:
        // Constructor and method to get device ID
        CANdleHWState(int id);
        int getDeviceID() const;

        // Set the colour of an LED
        void setLED(int id, int red, int green, int blue);
        LED getLED(int id);

        // Set the brightness of the CANdle's LEDs
        void setBrightness(double brightness);
        double getBrightness();

        // Show status LED when the CANdle is being controlled
        void showStatusLEDWhenActive(bool show);
        bool getStatusLEDWhenActive();

        // If the CANdle is enabled
        void setEnabled(bool enabled);
        bool getEnabled();

        // The CANdle's animation
        void setAnimation(CANdleAnimation* animation);
        CANdleAnimation* getAnimation();

    private:
        // The CAN ID of this CANdle
        int device_id;
        // All of the LED groups to colour
        vector<LED> leds;
        // The brightness of the LEDs in the CANdle, from 0->1
        double brightness;
        // If the status LED should be on when the CANdle is being controlled
        bool show_led_when_active;
        // If the CANdle is enabled
        bool enabled;
        // The currently playing CANdle animation
        CANdleAnimation* animation;
};


typedef StateHandle<const CANdleHWState> CANdleStateHandle;
typedef StateHandle<CANdleHWState>       CANdleWritableStateHandle;
class CANdleStateInterface : public HardwareResourceManager<CANdleStateHandle> {};
class RemoteCANdleStateInterface : public HardwareResourceManager<CANdleWritableStateHandle> {};

} // namespace candle
} // namespace hardware_interface
