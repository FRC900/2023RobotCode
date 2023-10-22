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
			d->gazeboSimInit(model_nh, parent_model);
		}

#if 0

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
	}

	void FRCRobotGazeboSimInterface::writeSim(ros::Time time, ros::Duration period)
	{
		FRCRobotSimInterface::write(time, period);
		for (auto &d : devices_)
		{
			d->gazeboSimWrite(time, period, *read_tracer_, e_stop_active_);
		}
#if 0
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
