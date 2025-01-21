#pragma once
#include <iostream>
#include <units/time.h>
#include <units/frequency.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/current.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <ctre/phoenix6/TalonFX.hpp>

namespace AlgaeIntakeConstants{
    constexpr int kRotationMotorID {14};
    constexpr int kRollerMotorID {17};

    // TODO
    constexpr double kSRotation {0.0}; // Static voltage applied to overcome friction
    constexpr double kGRotation {0.0}; // Variable gravity voltage
    constexpr double kVRotation {0.0}; // Volts applied per rotation per second
    constexpr double kARotation {0.0}; // Volts applied per rotation per second squared
    constexpr double kPRotation {0.0}; // Volts applied per rotation error
    constexpr double kIRotation {0.0}; // Volts applied per rotation integral error
    constexpr double kDRotation {0.0}; // Volts applied per rotation velocity error

    // TODO In rotations
    constexpr auto kMotionMagicMaxVelocity {2_tr / 1.0_s}; // rotations per second
    constexpr auto kMotionMagicMaxAcceleration {10_tr / 1.0_s / 1.0_s}; // rotations per second squared
    constexpr auto kMotionMagicMaxJerk {25_tr / 1.0_s / 1.0_s / 1.0_s}; // rotations per second cubed

    // TODO
    // Current limits -- allows continuous operation at certain amps, or a peak of greater amps for <threshold time
    constexpr auto kRotationContinuousCurrentLimit = units::ampere_t{25.0};
    constexpr auto kRotationPeakCurrentLimit = units::ampere_t{40.0};
    constexpr auto kRotationPeakCurrentDuration = 0.1_s; // seconds
    constexpr bool kRotationEnableCurrentLimit = true;

    constexpr auto kRollerContinuousCurrentLimit = units::ampere_t{25.0};
    constexpr auto kRollerPeakCurrentLimit = units::ampere_t{40.0};
    constexpr auto kRollerPeakCurrentDuration = 0.1_s; // seconds
    constexpr bool kRollerEnableCurrentLimit = true;

    // Other rotation configs     
    constexpr double kRotationGearRatio {45.0}; // ratio between rotation shaft cancoder and rotation motor
    constexpr units::degree_t kRotationZeroAngle {149.59_deg}; // This varies based on other subsystems; angle that intake should be set to on startup
    constexpr auto kDefaultRetractAngle {90.0_deg}; // TODO
    constexpr auto kIntakeAlgaeAngle {45.0_deg}; // TODO
    constexpr auto kScoreProcessorAngle {80.0_deg}; // TODO
    constexpr bool kRotationMotorInverted {false};
    constexpr auto kRotationNeutralMode {ctre::phoenix6::signals::NeutralModeValue::Brake};
    constexpr auto kRotationAngleTolerance {5.0_deg}; // TODO: go lower if possible (test various numbers and make preference for live-setting)
    constexpr auto kRotationIdleTolerance {1.5_deg}; // TODO: same as above

    constexpr auto kRollerScorePower {0.5};
    constexpr auto kRollerIntakePower {-0.5};
    
    // Other CANcoder configs
    // constexpr auto kCANcoderOffset {0.0_tr}; // TODO
    // constexpr bool kCANcoderInverted {false};
    // constexpr auto kCANcoderDiscontinuityPoint {1_tr}; // rotation will be unsigned [0,1)

    // Other roller configs
    constexpr bool kRollerMotorInverted {false};
    constexpr unsigned int kRollerMotorCurrentLimit {45};

}
