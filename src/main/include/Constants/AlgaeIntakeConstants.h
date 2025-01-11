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
    constexpr int kRotationMotorID {17};
    constexpr int kRollerMotorID {18};

    // TODO
    constexpr unsigned int kRotationCurrentLimit {40};
    constexpr unsigned int kRollerCurrentLimit {45};

    // constexpr double kPRotation {0.005};
    // constexpr double kIRotation {0.00000};
    // constexpr double kDRotation {0.01};
    // constexpr auto kSRotation {0.84149_V};
    // constexpr auto kGRotation {0.52939_V};
    // constexpr auto kVRotation {0.015044_V * 1.0_s / 1.0_rad};
    // constexpr auto kARotation {0.0006516_V * 1.0_s * 1.0_s / 1.0_rad};

    // TODO

    constexpr double kPRotation {0.05};
    constexpr double kIRotation {0.0};
    constexpr double kDRotation {0.0};
    constexpr auto kSRotation {1.0_V};
    constexpr auto kGRotation {0.5_V};
    constexpr auto kVRotation {1.0_V * 1.0_s / 1.0_rad};
    // constexpr auto kVRotation {0.65_V * 1.0_s / 1.0_rad};
    constexpr auto kARotation {0.0_V * 1.0_s * 1.0_s / 1.0_rad};

    // TODO
    constexpr auto kMaxVoltage {10.0_V}; // volts
    // Current limits -- allows continuous operation at certain amps, or a peak of greater amps for <threshold time
    constexpr auto kRotationContinuousCurrentLimit = units::ampere_t{25.0};
    constexpr auto kRotationPeakCurrentLimit = units::ampere_t{40.0};
    constexpr auto kRotationPeakCurrentDuration = 0.1_s; // seconds
    constexpr bool kRotationEnableCurrentLimit = true;
    
    constexpr double kFeedforward {1.0};

    // In degrees
    constexpr auto kMaxRotationVelocity {180.0_deg / 1.0_s};
    constexpr auto kMaxRotationAcceleration {480.0_deg / 1.0_s / 1.0_s};

    constexpr bool kRollerInverted {true};
    constexpr auto kDefaultRetractAngle {90.0_deg}; // TODO
    constexpr auto kIntakeAlgaeAngle {45.0_deg}; // TODO
    constexpr auto kScoreProcessorAngle {80.0_deg}; // TODO
    // constexpr auto kExtendTarget {0.0_deg};
    constexpr double kRotationOffset {263.0 / 360.0};
    constexpr double kRotationConversion {360.0}; 
    constexpr bool kRotationInverted {false};
    constexpr bool kRotationMotorInverted {true};
    constexpr auto kRotationAngleTolerance {5.0_deg}; // TODO: go lower if possible (test various numbers and make preference for live-setting)
    constexpr auto kRotationIdleTolerance {1.5_deg}; // TODO: same as above
    constexpr double kRotationGearRatio {15.43};

    constexpr auto kRotationNeutralMode {ctre::phoenix6::signals::NeutralModeValue::Brake};
}
