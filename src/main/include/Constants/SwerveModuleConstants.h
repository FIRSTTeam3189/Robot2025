#pragma once

#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <units/time.h>
#include <units/frequency.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/current.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>

#define PI 3.14159265358979323846

namespace SwerveModuleConstants {
    // Sensor IDs for motors + encoders - labeled on robot
    constexpr int kFrontLeftDriveID {1};
    constexpr int kFrontLeftAngleID {2};

    constexpr int kFrontRightDriveID {3};
    constexpr int kFrontRightAngleID {4};

    constexpr int kBackLeftDriveID {5};
    constexpr int kBackLeftAngleID {6};

    constexpr int kBackRightDriveID {7};
    constexpr int kBackRightAngleID {8};

    constexpr int kFrontLeftCANcoderID {9};
    constexpr int kFrontRightCANcoderID {10};
    constexpr int kBackLeftCANcoderID {11};
    constexpr int kBackRightCANcoderID {12};

    // Swerve angle offsets -- difference between actual degrees heading and absolute degree values
    // TODO not figured out yet
    constexpr units::turn_t kFrontLeftOffset {0.921143};
    constexpr units::turn_t kFrontRightOffset {-0.351562};
    constexpr units::turn_t kBackLeftOffset {0.115479}; 
    constexpr units::turn_t kBackRightOffset {0.332031};

     // Motor + sensor inversions
    constexpr bool kDriveMotorInverted = false;
    constexpr bool kAngleMotorInverted = false;
    constexpr bool kCANcoderInverted = false;

    constexpr units::volt_t kMaxVoltage {10.0};
    // Current limits -- allows continuous operation at certain amps, or a peak of greater amps for <threshold time
    constexpr auto kAngleContinuousCurrentLimit = units::ampere_t{25.0};
    constexpr auto kAnglePeakCurrentLimit = units::ampere_t{40.0};
    constexpr auto kAnglePeakCurrentDuration = 0.1_s; // seconds
    constexpr bool kAngleEnableCurrentLimit = true;
    
    constexpr auto kDriveContinuousCurrentLimit = units::ampere_t{35.0};
    constexpr auto kDrivePeakCurrentLimit = units::ampere_t{60.0};
    constexpr auto kDrivePeakCurrentDuration = 0.1_s;
    constexpr bool kDriveEnableCurrentLimit = true;

    // Encoder sensor range
    // constexpr auto kCANcoderSensorRange = ctre::phoenix6::signals::AbsoluteSensorRangeValue::Signed_PlusMinusHalf;
    constexpr auto kCANcoderDiscontinuityPoint = units::turn_t{0.5}; // same as +- half from before

    // Motor neutral modes -- what they do when no power is applied
    constexpr auto kDriveNeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    constexpr auto kAngleNeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    constexpr double kDriveNeutralDeadband = 0.02;

    // TalonFX Remote Sensor Type Settings
    constexpr int kRotorSensor{0};
    constexpr int kRemoteCANcoder{1};
    constexpr int kRemotePigeon2_Yaw{2};
    constexpr int kRemotePigeon2_Pitch{3};
    constexpr int kRemotePigeon2_Roll{4};
    constexpr int kFusedCANcoder{5};
    constexpr int kSyncCANcoder{6};

    // Even though probably not using FF, keep constants in case
    constexpr double kPDrive {0.0};
    constexpr double kIDrive {0.0};
    constexpr double kDDrive {0.0};
    constexpr double kVDrive {0.0};
    constexpr double kSDrive {0.0};

    constexpr double kPAngle {0.0};
    constexpr double kIAngle {0.0};
    constexpr double kDAngle {0.0};
    constexpr double kVAngle {0.0};
    constexpr double kSAngle {0.0};

    constexpr auto kMaxSpeed {5.0_mps}; // TODO
    constexpr double kDEGToRAD {57.2957795131};
    constexpr double kWheelRadiusInches {2.0};
    constexpr double kWheelRadiusMeters {0.0508};
    constexpr double kWheelCircumferenceMeters {2.0 * PI * kWheelRadiusMeters};
    constexpr double kDriveGearRatio {6.11};
    constexpr double kAngleGearRatio {13.3714};
    constexpr double kRotationsPerMeter {1.0 / kWheelCircumferenceMeters};
    constexpr int kFalconEncoderTicksPerRevolution {2048};
    constexpr int kCANcoderTicksPerRevolution {4096};
    constexpr double kWheelCOF {1.200}; // estimated
}
