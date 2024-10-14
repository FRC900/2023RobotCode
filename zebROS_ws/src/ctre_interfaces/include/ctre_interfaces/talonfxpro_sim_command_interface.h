#ifndef TALONFXPRO_SIM_COMMAND_INTERFACE_INC__
#define TALONFXPRO_SIM_COMMAND_INTERFACE_INC__

#include "ctre_interfaces/talonfxpro_state_interface.h"
#include "state_handle/command_handle.h"

// TODO : there are various Get methods in the TalonFXProSimState class that are not used in the sim code
namespace hardware_interface::talonfxpro
{
    class TalonFXProSimCommand
    {
    public:
        TalonFXProSimCommand(void);
        void setSupplyVoltage(const double supply_voltage);
        double getSupplyVoltage(void) const;
        bool supplyVoltageChanged(double &supply_voltage);
        void resetSupplyVoltage(void);

        void setForwardLimit(const bool forward_limit);
        bool getForwardLimit(void) const;
        bool forwardLimitChanged(bool &forward_limit);
        void resetForwardLimit(void);

        void setReverseLimit(const bool reverse_limit);
        bool getReverseLimit(void) const;
        bool reverseLimitChanged(bool &reverse_limit);
        void resetReverseLimit(void);

        void setRawRotorPosition(const double rotor_position);
        double getRawRotorPosition(void) const;
        bool rawPositionChanged(double &rotor_position);
        void resetRawRotorPosition(void);

        void setAddRotorPosition(const double add_rotor_position);
        double getAddRotorPosition(void) const;
        bool addPositionChanged(double &add_rotor_position);
        void resetAddRotorPosition(void);

        void setRotorVelocity(const double rotor_velocity);
        double getRotorVelocity(void) const;
        bool rotorVelocityChanged(double &rotor_velocity);
        void resetRotorVelocity(void);

        void setRotorAcceleration(const double acceleration);
        double getRotorAcceleration(void) const;
        bool accelerationChanged(double &acceleration);
        void resetRotorAcceleration(void);

    private:
        double supply_voltage_{0.0};
        bool supply_voltage_changed_{false};
        bool forward_limit_{false};
        bool forward_limit_changed_{false};
        bool reverse_limit_{false};
        bool reverse_limit_changed_{false};
        double rotor_position_{0.0};
        bool rotor_position_changed_{false};
        double add_rotor_position_{0.0};
        bool add_rotor_position_changed_{false};
        double rotor_velocity_{0.0};
        bool rotor_velocity_changed_{false};
        double rotor_acceleration_{0.0};
        bool rotor_acceleration_changed_{false};
    };

    // Handle - used by each controller to get, by name of the
    // corresponding joint, an interface with which to send commands
    // to a TalonFXPro simulator
    using TalonFXProSimCommandHandle = CommandHandle<TalonFXProSimCommand, TalonFXProHWState, TalonFXProStateHandle>;

    // Use ClaimResources here since we only want 1 controller
    // to be able to access a given TalonFXPro at any particular time
    // TODO : is this true?  For sim stuff, do we need to worry about this?
    class TalonFXProSimCommandInterface : public HardwareResourceManager<TalonFXProSimCommandHandle, ClaimResources> {};
}  // namespace hardware_interface::talonfxpro

#endif // TALONFXPRO_SIM_COMMAND_INTERFACE_INC__
