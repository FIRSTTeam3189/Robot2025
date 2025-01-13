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

#define PI 3.14159265358979323846

namespace CoralElevatorConstants{
    constexpr int kExtensionMotorID {17};

    // TODO
    constexpr double kSExtension {0.5}; // Static voltage applied to overcome friction
    constexpr double kGExtension {0.0}; // Variable gravity voltage
    constexpr double kVExtension {0.0}; // Volts applied per meter per second
    constexpr double kAExtension {0.0}; // Volts applied per meter per second squared
    constexpr double kPExtension {0.0}; // Volts applied per meter error
    constexpr double kIExtension {0.0}; // Volts applied per meter integral error
    constexpr double kDExtension {0.0}; // Volts applied per meter velocity error

    // TODO
    // Current limits -- allows continuous operation at certain amps, or a peak of greater amps for <threshold time
    constexpr auto kExtensionContinuousCurrentLimit = units::ampere_t{25.0};
    constexpr auto kExtensionPeakCurrentLimit = units::ampere_t{40.0};
    constexpr auto kExtensionPeakCurrentDuration = 0.1_s; // seconds
    constexpr bool kExtensionEnableCurrentLimit = true;

    // TODO In rotations
    constexpr auto kMotionMagicMaxVelocity {0_tr / 1.0_s}; // rotations per second
    constexpr auto kMotionMagicMaxAcceleration {0_tr / 1.0_s / 1.0_s}; // rotations per second squared
    constexpr auto kMotionMagicMaxJerk {0_tr / 1.0_s / 1.0_s / 1.0_s}; // rotations per second cubed

    // TODO: config/figure out for all of these
    constexpr auto kDefaultRetractHeight {0.0_m}; // TODO
    constexpr auto kExtensionHeightL1 {0.0_m};
    constexpr auto kExtensionHeightL2 {0.0_m};
    constexpr auto kExtensionHeightL3 {0.0_m};
    constexpr auto kExtensionHeightL4 {0.0_m};
    constexpr auto kExtensionHeightIntake {0.0_m};
    constexpr auto kExtensionOffset {0.176758_tr}; // TODO
    constexpr bool kExtensionMotorInverted {false};
    constexpr auto kExtensionHeightTolerance {0.3_m}; // TODO: go lower if possible (test various numbers and make preference for live-setting)
    // This is the diameter of the wheel that the telescoping arm's cable wraps around
    // Used to figure out the conversion from rotational to linear motion
    constexpr auto kPulleyDiameterMeters {0.04851146_m}; 
    constexpr auto kPulleyCircumferenceMeters {kPulleyDiameterMeters * PI}; // TODO see below
    constexpr double kExtensionGearRatio {1.0}; // TODO adjust if using telescoping arm or elevator
    constexpr double kExtensionConversionRotationsToMeters {kPulleyCircumferenceMeters / kExtensionGearRatio}; 
    constexpr auto kExtensionNeutralMode {ctre::phoenix6::signals::NeutralModeValue::Brake};
}

namespace CoralManipulatorConstants{
    constexpr int kRotationMotorID {18};  // TODO

    constexpr unsigned int kRotationCurrentLimit {40};
    constexpr bool kRotationInverted {true};
    constexpr double kRotationConversion {360.0}; 

    // TODO
    constexpr double kPRotation {0.0};
    constexpr double kIRotation {0.0};
    constexpr double kDRotation {0.0};
    constexpr auto kSRotation {0.0_V};
    constexpr auto kGRotation {0.0_V};
    constexpr auto kVRotation {0.0_V * 1.0_s / 1.0_rad};
    constexpr auto kARotation {0.0_V * 1.0_s * 1.0_s / 1.0_rad};

    constexpr auto kRotationAngleTolerance {5.0_deg}; // TODO: go lower if possible (test various numbers and make preference for live-setting)

    // TODO
    constexpr auto kDefaultAngle {90.0_deg};
    constexpr auto kIntakeAngle {35.0_deg};
    constexpr auto kScoreL123Angle {-37.0_deg};
    constexpr auto kScoreL4Angle {27.0_deg};

    constexpr double kFeedforward {1.0};
}
