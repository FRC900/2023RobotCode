#include "frc/geometry/Rotation2d.h"
#include "frc/kinematics/SwerveDriveOdometry.h"


int main(void)
{
    // Copy-pasta from here wpimath/src/test/native/cpp/kinematics/SwerveDriveOdometryTest.cpp
    // Also see /home/ubuntu/.2023RobotCode.readonly/allwpilib/wpimath/src/main/native/include/frc/estimator/SwerveDrivePoseEstimator.h
    // Fix to match our robot (maybe even read from swerve params)
    // Locations for the swerve drive modules relative to the robot center.
    frc::Translation2d m_frontLeftLocation{0.381_m, 0.381_m};
    frc::Translation2d m_frontRightLocation{0.381_m, -0.381_m};
    frc::Translation2d m_backLeftLocation{-0.381_m, 0.381_m};
    frc::Translation2d m_backRightLocation{-0.381_m, -0.381_m};

    // Creating my kinematics object using the module locations.
    frc::SwerveDriveKinematics<4> m_kinematics{
        m_frontLeftLocation, m_frontRightLocation,
        m_backLeftLocation, m_backRightLocation};

    // Fill these with info from talon states?
    // Not sure how they're different from starting pose.
    frc::SwerveModulePosition m_frontLeft{0_m, 0_rad};
    frc::SwerveModulePosition m_frontRight{0_m, 0_rad};
    frc::SwerveModulePosition m_backLeft{0_m, 0_rad};
    frc::SwerveModulePosition m_backRight{0_m, 0_rad};
    // Creating my odometry object from the kinematics object. Here,
    // our starting pose is 5 meters along the long end of the field and in the
    // center of the field along the short end, facing forward.
    frc::SwerveDriveOdometry<4> m_odometry{m_kinematics,
                                           frc::Rotation2d{static_cast<units::radian_t>(0)},
                                           {m_frontLeft,
                                            m_frontRight,
                                            m_backLeft,
                                            m_backRight},
                                           frc::Pose2d{5_m, 13.5_m, 0_rad}};

    frc::SwerveModulePosition fl{18.85_m, 90_deg};
    frc::SwerveModulePosition fr{42.15_m, 26.565_deg};
    frc::SwerveModulePosition bl{18.85_m, -90_deg};
    frc::SwerveModulePosition br{42.15_m, -26.565_deg};

    frc::SwerveModulePosition zero{};

    m_odometry.ResetPosition(0_rad, {zero, zero, zero, zero}, frc::Pose2d{});
    auto pose = m_odometry.Update(90_deg, {fl, fr, bl, br});
}