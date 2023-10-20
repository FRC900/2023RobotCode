#include "ros_control_boilerplate/frcrobot_gazebosim_interface.h"

#include "ros_control_boilerplate/devices.h"

namespace frcrobot_control
{
	bool FRCRobotGazeboSimInterface::initSim(const std::string &robot_namespace,
											 ros::NodeHandle model_nh,
											 gazebo::physics::ModelPtr parent_model,
											 const urdf::Model *const urdf_model,
											 std::vector<transmission_interface::TransmissionInfo> transmissions)
	{
		ros::NodeHandle not_used;
		FRCRobotSimInterface::init(model_nh, not_used);
		e_stop_active_ = false;
		last_e_stop_active_ = false;

		for (auto &d : devices_)
		{
			d->gazeboSimInit(model_nh);
		}

#if 0
		for (size_t i = 0; i < can_ctre_mc_names_.size(); i++)
		{
			ROS_INFO_STREAM_NAMED("frcrobot_gazebosim_interface",
								  "Connecting to gazebo : CTRE MC joint" << i << "=" << can_ctre_mc_names_[i] << (can_ctre_mc_local_updates_[i] ? " local" : " remote") << " update, " << (can_ctre_mc_local_hardwares_[i] ? "local" : "remote") << " hardware"
																		 << " as " << (can_ctre_mc_is_talon_[i] ? "TalonSRX" : "VictorSPX")
																		 << " CAN id " << can_ctre_mc_can_ids_[i]);

			if (can_ctre_mc_local_hardwares_[i])
			{
				gazebo::physics::JointPtr joint = parent_model->GetJoint(can_ctre_mc_names_[i]);
				if (!joint)
				{
					ROS_WARN_STREAM_NAMED("frcrobot_gazebosim_interface", "This robot has a joint named \"" << can_ctre_mc_names_[i]
																											<< "\" which is not in the gazebo model.");
				}
				sim_joints_ctre_mcs_.push_back(joint);
			}
			else
			{
				ROS_INFO_STREAM("   skipping non-local hardware");
				sim_joints_ctre_mcs_.push_back(nullptr);
			}
		}

		for (size_t i = 0; i < num_solenoids_; i++)
		{
			ROS_INFO_STREAM_NAMED("frcrobot_gazebosim_interface",
								  "Connecting to gazebo : solenoid joint " << i << "=" << solenoid_names_[i] << (solenoid_local_updates_[i] ? " local" : " remote") << " update, " << (solenoid_local_hardwares_[i] ? "local" : "remote") << " hardware "
																		   << " as Solenoid " << solenoid_ids_[i]);

			if (solenoid_local_hardwares_[i])
			{
				gazebo::physics::JointPtr joint = parent_model->GetJoint(solenoid_names_[i]);
				if (!joint)
				{
					ROS_WARN_STREAM_NAMED("frcrobot_gazebosim_interface", "This robot has a joint named \"" << solenoid_names_[i]
																											<< "\" which is not in the gazebo model.");
				}
				sim_joints_solenoids_.push_back(joint);
			}
			else
			{
				ROS_INFO_STREAM("   skipping non-local hardware");
				sim_joints_solenoids_.push_back(nullptr);
			}
		}
		for (size_t i = 0; i < num_double_solenoids_; i++)
		{
			ROS_INFO_STREAM_NAMED("frcrobot_sim_interface",
								  "Connecting to gazebo : double solenoid " << i << "=" << double_solenoid_names_[i] << (double_solenoid_local_updates_[i] ? " local" : " remote") << " update, " << (double_solenoid_local_hardwares_[i] ? "local" : "remote") << " hardware "
																			<< " as Double Solenoid forward " << double_solenoid_forward_ids_[i] << " reverse " << double_solenoid_reverse_ids_[i]);

			if (double_solenoid_local_hardwares_[i])
			{
				gazebo::physics::JointPtr joint = parent_model->GetJoint(double_solenoid_names_[i]);
				if (!joint)
				{
					ROS_WARN_STREAM_NAMED("frcrobot_gazebosim_interface", "This robot has a joint named \"" << double_solenoid_names_[i]
																											<< "\" which is not in the gazebo model.");
				}
				sim_joints_double_solenoids_.push_back(joint);
			}
			else
			{
				ROS_INFO_STREAM("   skipping non-local hardware");
				sim_joints_double_solenoids_.push_back(nullptr);
			}
		}
#endif

		// get physics engine type
#if GAZEBO_MAJOR_VERSION >= 8
		gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->Physics();
#else
		gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->GetPhysicsEngine();
#endif
		physics_type_ = physics->GetType();
		if (physics_type_.empty())
		{
			ROS_WARN_STREAM_NAMED("frcrobot_gazebosim_interface", "No physics type found.");
		}

		ROS_INFO("FRCRobotGazeboSim Ready.");

		return true;
	}

	void FRCRobotGazeboSimInterface::readSim(ros::Time time, ros::Duration period)
	{
		FRCRobotSimInterface::read(time, period);
		for (auto &d : devices_)
		{
			d->gazeboSimRead(time, period, *read_tracer_);
		}
#if 0
		for (size_t i = 0; i < num_can_ctre_mcs_; i++)
		{
			if (sim_joints_ctre_mcs_[i] && can_ctre_mc_local_hardwares_[i])
			{
				// Gazebo has an interesting API...
#if GAZEBO_MAJOR_VERSION >= 8
				const double position = sim_joints_ctre_mcs_[i]->Position(0);
#else
				const double position = sim_joints_ctre_mcs_[i]->GetAngle(0).Radian();
#endif
				auto &ts = talon_state_[i];
				ts.setPosition(position);
				if (ts.getTalonMode() != hardware_interface::TalonMode_MotionMagic)
					ts.setSpeed(sim_joints_ctre_mcs_[i]->GetVelocity(0));

#if 0
				if (joint_types_[j] == urdf::Joint::PRISMATIC)
				{
					joint_position_[j] = position;
				}
				else
				{
					joint_position_[j] += angles::shortest_angular_distance(joint_position_[j],
							position);
				}
				joint_velocity_[j] = sim_joints_ctre_mcs_[j]->GetVelocity(0);
#endif
			}
		}
		// solenoid and double solenoid state is set in FRCRobotSimInterface::write()
#endif
	}

	void FRCRobotGazeboSimInterface::writeSim(ros::Time time, ros::Duration period)
	{
		FRCRobotSimInterface::write(time, period);
		for (auto &d : devices_)
		{
			d->gazeboSimWrite(time, period, *read_tracer_);
		}
#if 0
		for (size_t i = 0; i < num_can_ctre_mcs_; i++)
		{
			if (sim_joints_ctre_mcs_[i] && can_ctre_mc_local_hardwares_[i])
			{
				auto &ts = talon_state_[i];
				hardware_interface::TalonMode simulate_mode = ts.getTalonMode();
				double position;
				double velocity;
				bool set_position = false;
				bool set_velocity = false;
				if (simulate_mode == hardware_interface::TalonMode_Position)
				{
					// Assume instant velocity
					position = ts.getSetpoint();
					set_position = true;
					velocity = 0;
					set_velocity = true;
				}
				else if (simulate_mode == hardware_interface::TalonMode_Velocity)
				{
					// Assume instant acceleration for now
					set_position = false;
					velocity = ts.getSetpoint();
					set_velocity = true;
				}
				else if (simulate_mode == hardware_interface::TalonMode_MotionMagic)
				{
					position = ts.getPosition();
					set_position = true;
					velocity = ts.getSpeed();
					set_velocity = true;
				}
				else if (simulate_mode == hardware_interface::TalonMode_Disabled)
				{
					set_position = false;
					velocity = 0;
					set_velocity = true;
				}
				else if (simulate_mode == hardware_interface::TalonMode_PercentOutput)
				{
					position = ts.getPosition();
					set_position = true;
					velocity = ts.getSpeed();
					set_velocity = true;
				}
#if 0
				ROS_INFO_STREAM("talon " << can_ctre_mc_names_[i] <<
						" setpoint=" << ts.getSetpoint() <<
						" pos=" << position <<
						" set_position=" << set_position <<
						" velocity=" << velocity <<
						" set_velocity=" << set_velocity <<
						" e_stop_active_=" << e_stop_active_);
#endif
				if (set_position)
				{
					sim_joints_ctre_mcs_[i]->SetPosition(0, position, true);
				}
				if (set_velocity)
				{
					// if (physics_type_.compare("ode") == 0)
					//{
					//		sim_joints_ctre_mcs_[i]->SetParam("vel", 0, e_stop_active_ ? 0 : velocity);
					// }
					// else
					{
						sim_joints_ctre_mcs_[i]->SetVelocity(0, e_stop_active_ ? 0 : velocity);
					}
				}
			}
		}
		for (size_t i = 0; i < num_solenoids_; i++)
		{
			if (sim_joints_solenoids_[i] && solenoid_local_hardwares_[i])
			{
				const double sign = solenoid_state_[i] ? 1.0 : -1.0;
				sim_joints_solenoids_[i]->SetForce(0, sign * 5.0);
			}
		}
		for (size_t i = 0; i < num_double_solenoids_; i++)
		{
			if (sim_joints_double_solenoids_[i] && double_solenoid_local_hardwares_[i])
			{
				const double sign = double_solenoid_state_[i] ? 1.0 : -1.0;
				sim_joints_double_solenoids_[i]->SetForce(0, sign * 5.0);
			}
		}
#endif
	}
} // namespace
