#pragma once

#include <hardware_interface/internal/hardware_resource_manager.h>
#include "state_handle/state_handle.h"
#include <optional>
#include <variant>
#include <vector>

namespace hardware_interface::candle {

static constexpr size_t MAX_ANIMATION_SLOTS = 8;

// An enum for all the animations the CANdle can play
enum class AnimationType {
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
enum class AnimationClass {
    BaseStandard,
    BaseTwo
};

// A CANdle colour
struct Colour {
    int red{0};
    int green{0};
    int blue{0};
    int white{0};

    // Constructor
    Colour(int red, int green, int blue, int white);
    Colour() = default;

    bool operator!=(const Colour& rhs) const;
    bool operator==(const Colour& rhs) const;
};

// An animation for the CANdle to play
struct Animation {
    // Animation speed
    double speed{0.0};
    // Initial LED to animate
    int start{0};
    // Number of LEDs to animation
    int count{0};
    // The animation to play
    AnimationType type{AnimationType::ColourFlow};
    // The animation class type
    AnimationClass class_type{AnimationClass::BaseStandard};

    // For Base Two animations
    // Animation colour
    Colour colour{};
    // Animation direction
    int direction{0};

    // For Base Standard animations
    // Animation brightness
    double brightness{1.0};
    // If the animation is reversed
    bool reversed{false};
    // Extra params
    double param4{0.0};
    double param5{0.0};

    // Constructor for BaseStandard animations
    Animation(double speed, int start, int count, AnimationType type, double brightness, bool reversed, double param4, double param5);
    // Constructor for BaseTwo animations
    Animation(double speed, int start, int count, AnimationType type, int red, int green, int blue, int white, int direction);
    // Blank constructor/null
    Animation() = default;

    // Comparison methods
    bool operator==(const Animation& rhs) const;
    bool operator!=(const Animation& rhs) const;
};

// An LED on the CANdle
class LED {
    public:
        explicit LED(size_t animation_id) :
            display{animation_id}
        {
        }
        explicit LED(Colour colour) :
            display{colour}
        {
        }

        std::optional<Colour> getColour() const {
            return std::holds_alternative<Colour>(display) ? std::optional<Colour>(std::get<Colour>(this->display)) : std::nullopt;
        }
        std::optional<int> getAnimationID() const {
            return std::holds_alternative<size_t>(display) ? std::optional<size_t>(std::get<size_t>(this->display)) : std::nullopt;
        }

        void setColour(Colour colour) {
            this->display = colour;
        }
        void setAnimationID(size_t animation_id) {
            this->display = animation_id;
        }

    private:
        std::variant<Colour, size_t> display;
};

class CANdleHWState {
    public:
        // Constructor and method to get device ID
        explicit CANdleHWState(int id);
        CANdleHWState(const CANdleHWState& other) = delete;
        CANdleHWState(CANdleHWState&& other) noexcept = delete;
        CANdleHWState& operator=(const CANdleHWState& other) = delete;
        CANdleHWState& operator=(CANdleHWState&& other) noexcept = delete;
        virtual ~CANdleHWState() = default;

        int getDeviceID() const;

        // Set the colour of an LED
        void setLED(size_t id, Colour colour);
        void setLED(size_t id, size_t animation_id);
        void setLEDOff(size_t id);
        std::optional<LED> getLED(size_t id) const;

        size_t getLEDCount() const;

        // Set the brightness of the CANdle's LEDs
        void setBrightness(double brightness);
        double getBrightness() const;

        // Show status LED when the CANdle is being controlled
        void setStatusLEDWhenActive(bool show);
        bool getStatusLEDWhenActive() const;

        // If the CANdle is enabled
        void setEnabled(bool enabled);
        bool getEnabled() const;

        // The CANdle's animation
        void setAnimation(const Animation& animation);
        void clearAnimation(size_t id);
        void clearAnimations();
        void setMaxAnimations(size_t max);
        size_t getAnimationCount(void) const;
        std::optional<Animation> getAnimation(size_t id) const;
        size_t getNextAnimationSlot();

    private:
        // The CAN ID of this CANdle
        int device_id;
        // All of the LED groups to colour
        std::vector<std::optional<LED>> leds;
        // The brightness of the LEDs in the CANdle, from 0->1
        double brightness{1.0};
        // If the status LED should be on when the CANdle is being controlled
        bool show_led_when_active{false};
        // If the CANdle is enabled
        bool enabled{false};
        // The CANdle's animations
        std::vector<std::optional<Animation>> animations;
};


using CANdleStateHandle = StateHandle<const CANdleHWState>;
using CANdleWritableStateHandle = StateHandle<CANdleHWState>;
class CANdleStateInterface : public HardwareResourceManager<CANdleStateHandle> {};
class RemoteCANdleStateInterface : public HardwareResourceManager<CANdleWritableStateHandle, ClaimResources> {};

} // namespace hardware_interface::candle
