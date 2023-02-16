#ifndef INC_PIGEON2_CONTROLLER_INTERFACE_H__
#define INC_PIGEON2_CONTROLLER_INTERFACE_H__

#include <atomic>
#include <mutex>
#include "ddr_updater/ddr_updater.h"
#include "pigeon2_interface/pigeon2_command_interface.h"

namespace pigeon2_controller_interface
{
class Pigeon2CIParams : public ddr_updater::DDRUpdater
{
	public:
		Pigeon2CIParams(ros::NodeHandle n);

		void setVelocityMeasPeriod(int velocity_meas_period, bool update_dynamic = true);
		void setVelocityMeasWindow(int velocity_meas_window, bool update_dynamic = true);
		void setAbsoluteSensorRange(int absolute_sensor_range, bool update_dynamic = true);
		void setMagnetOffset(double magnet_offset, bool update_dynamic = true);
		void setInitializationStrategy(int initialization_strategy, bool update_dynamic = true);
		void setFeedbackCoefficient(double feedback_coefficient, bool update_dynamic = true);
		void setUnitString(const std::string &unit_string, bool update_dynamic = true);
		void setTimeBase(int time_base, bool update_dynamic = true);
		void setDirection(bool direction, bool update_dynamic = true);
		void setSensorDataStatusFramePeriod(int sensor_data_status_frame_period, bool update_dynamic = true);
		void setVBatAndFaultsStatusFramePeriod(int vbat_and_faults_status_frame_period, bool update_dynamic = true);
		void setConversionFactor(double conversion_factor, bool update_dynamic = true);

		hardware_interface::pigeon2::SensorVelocityMeasPeriod     getVelocityMeasPeriod(void) const;
		int                                                        getVelocityMeasWindow(void) const;
		hardware_interface::pigeon2::AbsoluteSensorRange          getAbsoluteSensorRange(void) const;
		double                                                     getMagnetOffset(void) const;
		hardware_interface::pigeon2::SensorInitializationStrategy getInitializationStrategy(void) const;
		double                                                     getFeedbackCoefficient(void) const;
		std::string                                                getUnitString(void);
		hardware_interface::pigeon2::SensorTimeBase               getTimeBase(void) const;
		double                                                     getDirection(void) const;
		int                                                        getSensorDataStatusFramePeriod(void) const;
		int                                                        getVBatAndFaultsStatusFramePeriod(void) const;
		double                                                     getConversionFactor(void) const;

	private:
		std::atomic<hardware_interface::pigeon2::SensorVelocityMeasPeriod>     velocity_meas_period_;
		std::atomic<int>                                                        velocity_meas_window_;
		std::atomic<hardware_interface::pigeon2::AbsoluteSensorRange>          absolute_sensor_range_;
		std::atomic<double>                                                     magnet_offset_;
		std::atomic<hardware_interface::pigeon2::SensorInitializationStrategy> initialization_strategy_;
		std::atomic<double>                                                     feedback_coefficient_;
		std::mutex                                                              unit_string_mutex_;
		std::string                                                             unit_string_;
		std::atomic<hardware_interface::pigeon2::SensorTimeBase>               time_base_;
		std::atomic<bool>                                                       direction_;
		std::atomic<int>                                                        sensor_data_status_frame_period_;
		std::atomic<int>                                                        vbat_and_faults_status_frame_period_;
		std::atomic<double>                                                     conversion_factor_;

		const std::map<std::string, int> velocity_measurement_period_enum_map_ =
		{
			{"1Ms", static_cast<int>(hardware_interface::pigeon2::SensorVelocityMeasPeriod::Sensor_Period_1Ms)},
			{"2Ms", static_cast<int>(hardware_interface::pigeon2::SensorVelocityMeasPeriod::Sensor_Period_2Ms)},
			{"5Ms", static_cast<int>(hardware_interface::pigeon2::SensorVelocityMeasPeriod::Sensor_Period_5Ms)},
			{"10Ms", static_cast<int>(hardware_interface::pigeon2::SensorVelocityMeasPeriod::Sensor_Period_10Ms)},
			{"20Ms", static_cast<int>(hardware_interface::pigeon2::SensorVelocityMeasPeriod::Sensor_Period_20Ms)},
			{"25Ms", static_cast<int>(hardware_interface::pigeon2::SensorVelocityMeasPeriod::Sensor_Period_25Ms)},
			{"50Ms", static_cast<int>(hardware_interface::pigeon2::SensorVelocityMeasPeriod::Sensor_Period_50Ms)},
			{"100Ms", static_cast<int>(hardware_interface::pigeon2::SensorVelocityMeasPeriod::Sensor_Period_100Ms)}
		};
		const std::map<std::string, int> absolute_sensor_range_enum_map_ =
		{
			{"Unsigned_0_to_360", static_cast<int>(hardware_interface::pigeon2::AbsoluteSensorRange::Unsigned_0_to_360)},
			{"Signed_PlusMinus180", static_cast<int>(hardware_interface::pigeon2::AbsoluteSensorRange::Signed_PlusMinus180)}
		};
		const std::map<std::string, int> sensor_initialization_strategy_enum_map_ =
		{
			{"BootToAbsolutePosition", static_cast<int>(hardware_interface::pigeon2::SensorInitializationStrategy::BootToAbsolutePosition)},
			{"BootToZero", static_cast<int>(hardware_interface::pigeon2::SensorInitializationStrategy::BootToZero)}
		};
		const std::map<std::string, int> sensor_time_base_enum_map_ =
		{
			{"Per100Ms_Legacy", static_cast<int>(hardware_interface::pigeon2::SensorTimeBase::Per100Ms_Legacy)},
			{"PerSecond", static_cast<int>(hardware_interface::pigeon2::SensorTimeBase::PerSecond)},
			{"PerMinute", static_cast<int>(hardware_interface::pigeon2::SensorTimeBase::PerMinute)}
		};

		template <typename T>
		void readIntoScalar(ros::NodeHandle &n, const std::string &name, std::atomic<T> &scalar)
		{
			T val;
			if (n.getParam(name, val))
				scalar = val;
		}

		template <typename T>
		bool readIntoEnum(ros::NodeHandle &n,
				const std::string &param_name,
				const std::map<std::string, int> &mymap,
				std::atomic<T>&out) const
		{
			std::string param_str;
			if (!n.getParam(param_name, param_str))
				return true;

			const auto it = mymap.find(param_str);
			if (it != mymap.cend())
			{
				out = static_cast<T>(it->second);
				return true;
			}
			ROS_ERROR_STREAM("Could not convert param_name "
					<< param_name
					<< " param_str="
					<< param_str
					<< " didn't match list of valid types");
			return false;
		}
};
class Pigeon2ControllerInterface
{
	public:
		Pigeon2ControllerInterface(ros::NodeHandle &n, const std::string &joint_name, hardware_interface::pigeon2::Pigeon2CommandHandle handle);

		void update(void);
		void setPosition(double new_position);
		void setPositionToAbsolute(void);
		void setClearStickyFaults(void);

		void setVelocityMeasPeriod(hardware_interface::pigeon2::SensorVelocityMeasPeriod velocity_meas_period);
		void setVelocityMeasWindow(int velocity_meas_window);
		void setAbsoluteSensorRange(hardware_interface::pigeon2::AbsoluteSensorRange absolute_sensor_range);
		void setMagnetOffset(double magnet_offset);
		void setInitializationStrategy(hardware_interface::pigeon2::SensorInitializationStrategy initialization_strategy);
		void setFeedbackCoefficient(double feedback_coefficient);
		void setUnitString(const std::string &unit_string);
		void setTimeBase(hardware_interface::pigeon2::SensorTimeBase time_base);
		void setDirection(bool direction);
		void setSensorDataStatusFramePeriod(int sensor_data_status_frame_period);
		void setVBatAndFaultsStatusFramePeriod(int vbat_and_faults_status_frame_period);
		void setConversionFactor(double conversion_factor);

	private:
		Pigeon2CIParams                                   params_;
		hardware_interface::pigeon2::Pigeon2CommandHandle handle_;
		std::mutex                                          set_position_mutex_;
		bool                                                set_position_;
		double                                              new_position_;
		std::atomic<bool>                                   set_position_to_absolute_;
		std::atomic<bool>                                   clear_sticky_faults_;
};

} // namespace pigeon2_controller_interfac

#endif