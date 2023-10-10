#include "frc/geometry/Rotation2d.h"
#include "frc/kinematics/SwerveDriveOdometry.h"
#include <ros/ros.h>
#include <talon_state_msgs/TalonState.h>
#include <utility>
#include <geometry_msgs/TwistStamped.h>

/*
// Read wheel rotation from encoder.  Subtract previous position
// to get the motion since the last odom update. Convert from rotation
// to linear distance using wheel radius and drive ratios
const double new_wheel_rot = speed_joints_[k].getPosition();
const double delta_rot     = new_wheel_rot - last_wheel_rot_[k];
const double dist          = delta_rot * wheel_radius_ * driveRatios_.encodertoRotations;

// Get the offset-corrected steering angle
const double steer_angle = swerveC_->getWheelAngle(k, steer_angles[k]);
// And then average it with the last reading - moving the full
// wheel rot distance in a direction of the average of the start
// and end steer directions for this timestep is an approximation
// of the actual direction. It ignores changes in speed of the
// steer motor, but it is the best we can do given the info we have.
const double average_steer_angle = angle_midpoint(last_wheel_angle_[k], steer_angle);

// delta_pos of the wheel in x,y - decompose vector into x&y components
const Eigen::Vector2d delta_pos{dist * cos(average_steer_angle), dist * sin(average_steer_angle)};
*/

// Locations for the swerve drive modules relative to the robot center.
frc::Translation2d m_frontLeftLocation{0.250825_m, 0.250825_m};
frc::Translation2d m_frontRightLocation{0.250825_m, -0.250825_m};
frc::Translation2d m_backLeftLocation{-0.250825_m, 0.250825_m};
frc::Translation2d m_backRightLocation{-0.250825_m, -0.250825_m};

// Creating my kinematics object using the module locations.
frc::SwerveDriveKinematics<4> m_kinematics = frc::SwerveDriveKinematics<4>{
    m_frontLeftLocation, m_frontRightLocation,
    m_backLeftLocation, m_backRightLocation};

talon_state_msgs::TalonState latest_talon_states;
std::map<std::string, std::pair<size_t, size_t>> index_map; // <drive_index, steer_index>
std::map<std::string, double> offsets;

// hardcoded for now, should load from 2023_swerve_drive.yaml
double wheel_radius = 0.050; // meters
double encoder_to_rotations = 0.148148148; // 1/6.75

frc::SwerveModuleState getState(std::string module_id) {
    size_t drive_index, steer_index = 0;
    if (auto indices = index_map.find(module_id); indices != index_map.end()) {
        drive_index = indices->second.first;
        steer_index = indices->second.second;
    } else {
        for (size_t i = 0; i < latest_talon_states.name.size(); i++) {
            if (latest_talon_states.name[i].find(module_id + "_") != std::string::npos) {
                if (latest_talon_states.name[i].find("drive") != std::string::npos) {
                    drive_index = i;
                } else if (latest_talon_states.name[i].find("angle") != std::string::npos) {
                    steer_index = i;
                }
            }
        }
        index_map[module_id] = std::make_pair(drive_index, steer_index);
    }

    double angle = latest_talon_states.position[steer_index] - offsets[module_id];
    double velocity = latest_talon_states.speed[drive_index] * wheel_radius * encoder_to_rotations;

    return frc::SwerveModuleState{units::meters_per_second_t{velocity}, frc::Rotation2d(units::radian_t(angle))};
}

// talon state callback
void talon_state_callback(const talon_state_msgs::TalonState::ConstPtr& msg)
{
    latest_talon_states = *msg;
}

int main(int argc, char** argv)
{
    // ros init node
    ros::init(argc, argv, "wpilib_swerve_odom");

    // set up subscriber with callback to store latest talon states in latest_talon_states variable
    ros::NodeHandle nh;

    ros::Subscriber talon_state_sub = nh.subscribe<talon_state_msgs::TalonState>("/frcrobot_jetson/talon_states", 1, talon_state_callback);

    // create publisher to publish twist
    ros::Publisher twist_pub = nh.advertise<geometry_msgs::TwistStamped>("/frcrobot_jetson/swerve_drive_odom/twist", 1); // sure copilot great topic name

    // spin
    ros::Rate loop_rate(50);
    while (ros::ok()) {
        // get latest talon states
        ros::spinOnce();

        // get latest swerve module states
        frc::SwerveModuleState front_left = getState("fl");
        frc::SwerveModuleState front_right = getState("fr");
        frc::SwerveModuleState back_left = getState("bl");
        frc::SwerveModuleState back_right = getState("br");

        // get latest twist
        frc::ChassisSpeeds speeds = m_kinematics.ToChassisSpeeds(front_left, front_right, back_left, back_right);

        // publish twist
        geometry_msgs::TwistStamped twist_msg;
        twist_msg.header.stamp = ros::Time::now();
        twist_msg.twist.linear.x = speeds.vx.to<double>();
        twist_msg.twist.linear.y = speeds.vy.to<double>();
        twist_msg.twist.angular.z = speeds.omega.to<double>();
        twist_pub.publish(twist_msg);

        loop_rate.sleep();
    }
}