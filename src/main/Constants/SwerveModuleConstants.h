#pragma once

#include <ctre/phoenix6/signals/SpnEnums.hpp>

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
    // TO BE DETERMINED
    constexpr double kFrontLeftOffset {0.921143};
    constexpr double kFrontRightOffset {-0.351562};
    constexpr double kBackLeftOffset {0.115479}; 
    constexpr double kBackRightOffset {0.332031};

     // Motor + sensor inversions
    constexpr bool kDriveMotorInverted = false;
    constexpr bool kAngleMotorInverted = false;
    constexpr bool kCANcoderInverted = false;

    // Encoder sensor range
    constexpr auto kCANcoderSensorRange = ctre::phoenix6::signals::AbsoluteSensorRangeValue::Signed_PlusMinusHalf;

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

    constexpr double kMaxVoltage {10.0};
    // Current limits -- allows continuous operation at certain amps, or a peak of greater amps for <threshold time
    constexpr int kAngleContinuousCurrentLimit = 25;
    constexpr int kAnglePeakCurrentLimit = 40;
    constexpr double kAnglePeakCurrentDuration = 0.1; // seconds
    constexpr bool kAngleEnableCurrentLimit = true;
    
    constexpr int kDriveContinuousCurrentLimit = 35;
    constexpr int kDrivePeakCurrentLimit = 60;
    constexpr double kDrivePeakCurrentDuration = 0.1;
    constexpr bool kDriveEnableCurrentLimit = true;

    // PID, sensor IDs passed in via structs in namespace
    constexpr auto kMaxSpeed {4.0_mps};
    constexpr double kMPSToRPM {600.0};
    constexpr double kDEGToRAD {57.2957795131};
    constexpr double kWheelRadiusInches {2.0};
    constexpr double kWheelRadiusMeters {0.0508};
    constexpr double kWheelCircumferenceMeters {2.0 * Pi * kWheelRadiusMeters};
    constexpr double kDriveGearRatio {8.1};
    constexpr double kAngleGearRatio {15.43};
    constexpr double kRotationsPerMeter {1.0 / kWheelCircumferenceMeters};
    constexpr int kFalconEncoderTicksPerRevolution {2048};
    constexpr int kCANcoderTicksPerRevolution {4096};

    
}
