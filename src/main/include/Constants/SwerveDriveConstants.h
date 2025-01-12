#pragma once

#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/frequency.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>

#define PI 3.14159265358979323846

enum class ActiveDriveController { OfficialDriver, TestControls };

namespace SwerveDriveConstants {
    // SET ACTIVE DRIVER HERE
    constexpr ActiveDriveController kActiveController {ActiveDriveController::TestControls};

    constexpr int kGyroID {13};
    constexpr double kRadiansToDegreesMultiplier {180.0 / PI};

    // Coordinate plane distance in meters to each swerve drive
    // This has x-positive as forward, y-positive as left
    constexpr auto kXDistanceFromCenter {0.2921_m};
    constexpr auto kYDistanceFromCenter {0.2921_m};
    constexpr auto kWheelbase {0.2921_m * 2.0};
    constexpr auto kTrackwidth {0.2921_m * 2.0};

    constexpr units::degree_t kGyroMountPoseYaw {0.0};

    constexpr auto kAmpAlignTarget = 90.0_deg;

    // TODO: Get the angles for all of them
    constexpr auto kMidFieldY = 4.05;  // Meters
    constexpr auto kBlueSourceAlignTargetTop = 120.0_deg;
    constexpr auto kRedSourceAlignTargetTop = 60.0_deg;
    constexpr auto kBlueSourceAlignTargetBottom = 120.0_deg;
    constexpr auto kRedSourceAlignTargetRightBottom = 60.0_deg;

    constexpr frc::Pose2d kBlueResetPose {8.0_m, 4.023_m, frc::Rotation2d{0.0_deg}};
    constexpr frc::Pose2d kRedResetPose {9.55_m, 4.023_m, frc::Rotation2d{180.0_deg}};

    constexpr auto kRefreshRate = units::hertz_t {100};

    static frc::SwerveDriveKinematics<4> kKinematics {
        frc::Translation2d{+SwerveDriveConstants::kXDistanceFromCenter, +SwerveDriveConstants::kYDistanceFromCenter},
        frc::Translation2d{+SwerveDriveConstants::kXDistanceFromCenter, -SwerveDriveConstants::kYDistanceFromCenter},
        frc::Translation2d{-SwerveDriveConstants::kXDistanceFromCenter, +SwerveDriveConstants::kYDistanceFromCenter},
        frc::Translation2d{-SwerveDriveConstants::kXDistanceFromCenter, -SwerveDriveConstants::kYDistanceFromCenter}
    };
    constexpr auto kMaxSpeed {5.0_mps};
    constexpr auto kMaxAcceleration {5.0_mps_sq}; // TODO
    constexpr units::radians_per_second_t kMaxAngularVelocity {4.0 * PI};
    constexpr units::radians_per_second_squared_t kMaxAngularAcceleration {2.0 * PI};

    // SysID robot characterization values -- **varies by robot**
    // constexpr double kSRot {0.125}; // 0.408_V
    // constexpr auto kv {3.206 * 1_V * 1_s / 1_m};
    // constexpr auto ka {3.409 * 1_V * 1_s * 1_s / 1_m};

    // These are for robot rotation, not wheel rotation
    constexpr double kSRot {0.125}; // 0.408_V

    // These are for robot rotation, not wheel rotation
    constexpr double kPRot {0.005};
    constexpr double kIRot {0.0}; // constexpr double kIRot {0.00025};
    constexpr double kDRot {0.0};

    constexpr double kSlowModeDriveMultiplier {0.35};
    constexpr double kSwerveRotationTolerance {0.5};
}