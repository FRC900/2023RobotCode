#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Translation2d.h"
#include "frc/kinematics/SwerveDriveOdometry.h"
#include <ros/ros.h>
#include <talon_state_msgs/TalonState.h>
#include <utility>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <angles/angles.h>

template <size_t WHEELCOUNT>
class ROSSwerveKinematics
{
    public:
        // The swerve module locations relative to the robot center.
        wpi::array<frc::Translation2d, WHEELCOUNT> moduleLocations{wpi::empty_array};

        // The WPILib swerve kinematics object
        frc::SwerveDriveKinematics<WHEELCOUNT> m_kinematics{moduleLocations};

        // The latest talon states
        talon_state_msgs::TalonState latest_talon_states;

        // Maps talon names to indices in latest_talon_states for optimization
        std::map<std::string, size_t> index_map;

        // Offsets for each module
        std::array<double, WHEELCOUNT> offsets;

        // Names of the speed and steering talons
        std::array<std::string, WHEELCOUNT> speed_names;
        std::array<std::string, WHEELCOUNT> steering_names;

        // Wheel radius and encoder to rotations ratio
        double wheel_radius = 0.050; // meters
        double encoder_to_rotations = 0.148148148; // 1/6.75

        // Subscriber for talon states
        ros::Subscriber talon_state_sub;

        // Get the state of a module
        frc::SwerveModuleState getState(size_t module_index)
        {
            // get the index of the speed and steering talons
            size_t steer_index, speed_index = 0;

            if (auto index = index_map.find(steering_names[module_index]); index != index_map.end())
            {
                steer_index = index->second;
            }
            else
            {
                for (size_t i = 0; i < latest_talon_states.name.size(); i++)
                {
                    if (latest_talon_states.name[i] == steering_names[module_index])
                    {
                        index_map[steering_names[module_index]] = i;
                        steer_index = i;
                    }
                }
            }

            if (auto index = index_map.find(speed_names[module_index]); index != index_map.end())
            {
                speed_index = index->second;
            }
            else
            {
                for (size_t i = 0; i < latest_talon_states.name.size(); i++)
                {
                    if (latest_talon_states.name[i] == speed_names[module_index])
                    {
                        index_map[speed_names[module_index]] = i;
                        speed_index = i;
                    }
                }
            }

            // calculate angle and velocity
            double angle = angles::normalize_angle(latest_talon_states.position[steer_index] - offsets[module_index]);
            double velocity = latest_talon_states.speed[speed_index] * wheel_radius * encoder_to_rotations;

            // return calculated state in WPILib format
            return frc::SwerveModuleState{units::meters_per_second_t(velocity), frc::Rotation2d(units::radian_t(angle))};
        }

        // Publisher for twists
        ros::Publisher twist_pub;

        // Publish latest twist from talon states
        void publish_twist()
        {
            // array to store states in WPILib format
            wpi::array<frc::SwerveModuleState, WHEELCOUNT> states{wpi::empty_array};

            // get states for each module
            for (size_t i = 0; i < WHEELCOUNT; i++)
            {
                states[i] = getState(i);
                double v = states[i].speed.value();
                double theta = states[i].angle.Radians().value();
            }

            // calculate chassis speeds (WPILib magic)
            frc::ChassisSpeeds speeds = m_kinematics.ToChassisSpeeds(states);

            // publish twist
            geometry_msgs::TwistWithCovarianceStamped twist_msg;
            twist_msg.header.stamp = ros::Time::now();
            twist_msg.header.frame_id = "base_link";
            // x = y and y = -x in the talon swerve drive controller, so copying that here
            // and it seems to work
            twist_msg.twist.twist.linear.x = speeds.vy.value();
            twist_msg.twist.twist.linear.y = -speeds.vx.value();
            twist_msg.twist.twist.angular.z = speeds.omega.value();

            twist_msg.twist.covariance = boost::array<double, 36>{0.0001, 0, 0, 0, 0, 0,
                                          0, 0.0001, 0, 0, 0, 0,
                                          0, 0, 0.0001, 0, 0, 0,
                                          0, 0, 0, 0, 0, 0,
                                          0, 0, 0, 0, 0, 0,
                                          0, 0, 0, 0, 0, 0};

            twist_pub.publish(twist_msg);
        }

        // talon state callback
        void talon_state_callback(const talon_state_msgs::TalonState::ConstPtr &msg)
        {
            // Store latest talon states
            latest_talon_states = *msg;

            // Call function to calculate and publish twist
            publish_twist();
        }

        // Get wheel names from rosparam, given a node handle and a parameter name
        bool getWheelNames(ros::NodeHandle &controller_nh,
                           const std::string &wheel_param,
                           std::array<std::string, WHEELCOUNT> &wheel_names)
        {
            // copied from swerve_drive_controller.cpp
            XmlRpc::XmlRpcValue wheel_list;
            if (!controller_nh.getParam(wheel_param, wheel_list))
            {
                ROS_ERROR_STREAM("Couldn't retrieve wheel param '" << wheel_param << "'.");
                return false;
            }

            if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
            {
                if (wheel_list.size() == 0)
                {
                    ROS_ERROR_STREAM("Wheel param '" << wheel_param << "' is an empty list");
                    return false;
                }
                if (wheel_list.size() != WHEELCOUNT)
                {
                    ROS_ERROR_STREAM("Wheel param size (" << wheel_list.size() <<
                                     " != WHEELCOUNT (" << WHEELCOUNT << ")." );
                    return false;
                }

                for (int i = 0; i < wheel_list.size(); ++i)
                {
                    if (wheel_list[i].getType() != XmlRpc::XmlRpcValue::TypeString)
                    {
                        ROS_ERROR_STREAM("Wheel param '" << wheel_param << "' #" << i <<
                                         " isn't a string.");
                        return false;
                    }
                }

                for (int i = 0; i < wheel_list.size(); ++i)
                {
                    wheel_names[i] = static_cast<std::string>(wheel_list[i]);
                }
            }
            else
            {
                ROS_ERROR_STREAM("Wheel param '" << wheel_param <<
                                 "' is neither a list of strings nor a string.");
                return false;
            }
            return true;
        }

        bool read_params()
        {
            // node handle with the /frcrobot_jetson/swerve_drive_controller namespace
            ros::NodeHandle params_nh("/frcrobot_jetson/swerve_drive_controller");

            // Get wheel names from the parameter server
            if (!getWheelNames(params_nh, "speed", speed_names) or
                    !getWheelNames(params_nh, "steering", steering_names))
            {
                return false;
            }

            // Basic sanity checks
            if (speed_names.size() != steering_names.size())
            {
                ROS_ERROR_STREAM("#speed (" << speed_names.size() << ") != " <<
                                 "#steering (" << steering_names.size() << ").");
                return false;
            }
            if (speed_names.size() != WHEELCOUNT)
            {
                ROS_ERROR_STREAM("#speed (" << speed_names.size() << ") != " <<
                                 "WHEELCOUNT (" << WHEELCOUNT << ").");
                return false;
            }

            // read swerve drive module locations from rosparam
            XmlRpc::XmlRpcValue wheel_coords;

            if (!params_nh.getParam("wheel_coords", wheel_coords))
            {
                ROS_ERROR("talon_swerve_drive_controller : could not read wheel_coords");
                return false;
            }
            if (wheel_coords.getType() != XmlRpc::XmlRpcValue::TypeArray )
            {
                ROS_ERROR("talon_swerve_drive_controller : param 'wheel_coords' is not a list");
                return false;
            }
            if (wheel_coords.size() != WHEELCOUNT)
            {
                ROS_ERROR_STREAM("talon_swerve_drive_controller : param 'wheel_coords' is not correct length (expecting WHEELCOUNT = " << WHEELCOUNT << ")");
                return false;
            }
            for (int i = 0; i < wheel_coords.size(); ++i)
            {
                if (wheel_coords[i].getType() != XmlRpc::XmlRpcValue::TypeArray )
                {
                    ROS_ERROR("talon_swerve_drive_controller : param wheel_coords[%d] is not a list", i);
                    return false;
                }
                if (wheel_coords[i].size() != 2)
                {
                    ROS_ERROR("talon_swerve_drive_controller: param wheel_coords[%d] is not a pair", i);
                    return false;
                }
                if (	wheel_coords[i][0].getType() != XmlRpc::XmlRpcValue::TypeDouble ||
                        wheel_coords[i][1].getType() != XmlRpc::XmlRpcValue::TypeDouble)
                {
                    ROS_ERROR("talon_swerve_drive_controller : param wheel_coords[%d] is not a pair of doubles", i);
                    return false;
                }
                double x = wheel_coords[i][0];
                double y = wheel_coords[i][1];
                moduleLocations[i] = frc::Translation2d(units::meter_t(x), units::meter_t(y));
            }

            // Initialize kinematics object
            m_kinematics = frc::SwerveDriveKinematics<4>(moduleLocations);

            // read offsets using nh_params, see the 2023_swerve_drive.yaml file for how these are encoded
            for (size_t i = 0; i < WHEELCOUNT; i++)
            {
                ros::NodeHandle nh(params_nh, steering_names[i]);
                if (!nh.getParam("offset", offsets[i]))
                {
                    ROS_ERROR_STREAM("Can not read offset for " << steering_names[i]);
                    return false;
                }
            }

            // now that we've retrieved offsets, overwrite speed and steering names with joint names
            for (size_t i = 0; i < WHEELCOUNT; i++)
            {
                ros::NodeHandle nh_steer(params_nh, steering_names[i]);
                if (!nh_steer.getParam("joint", steering_names[i]))
                {
                    ROS_ERROR_STREAM("Can not read joint name for " << steering_names[i]);
                    return false;
                }
                ros::NodeHandle nh_speed(params_nh, speed_names[i]);
                if (!nh_speed.getParam("joint", speed_names[i]))
                {
                    ROS_ERROR_STREAM("Can not read joint name for " << speed_names[i]);
                    return false;
                }
            }

            // Load wheel radius and encoder to rotations
            if (!params_nh.getParam("wheel_radius", wheel_radius))
            {
                ROS_ERROR_STREAM("Cannot read wheel_radius");
                return false;
            }
            if (!params_nh.getParam("ratio_encoder_to_rotations", encoder_to_rotations))
            {
                ROS_ERROR_STREAM("Cannot read ratio_encoder_to_rotations");
                return false;
            }

            return true;
        }

        bool init()
        {
            // set up subscriber with callback to store latest talon states in latest_talon_states variable
            ros::NodeHandle nh;

            // Try to read parameters
            if (!read_params())
            {
                return false;
            }

            // subscribe to talon states
            talon_state_sub = nh.subscribe<talon_state_msgs::TalonState>("/frcrobot_jetson/talon_states", 1, &ROSSwerveKinematics::talon_state_callback, this);

            // create publisher to publish twist
            twist_pub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("/frcrobot_jetson/swerve_drive_odom/twist", 1); // sure copilot great topic name

            return true;
        }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wpilib_swerve_odom");

    // create kinematics node object
    ROSSwerveKinematics<4> swerve_kinematics;

    // initialize kinematics node
    if (!swerve_kinematics.init())
    {
        ROS_ERROR_STREAM("Initialization failed");
        exit(-1);
    }

    // spin forever
    ros::spin();
}