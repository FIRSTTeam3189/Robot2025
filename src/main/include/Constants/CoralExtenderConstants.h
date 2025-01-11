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

namespace CoralExtenderConstants{
    constexpr int kExtensionMotorID {17};

    // TODO
    constexpr double kSExtension {0.0}; // Static voltage applied to overcome friction
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
    constexpr double kExtensionOffset {0.0 / 360.0}; // TODO
    constexpr bool kExtensionMotorInverted {true};
    constexpr auto kExtensionHeightTolerance {0.3_m}; // TODO: go lower if possible (test various numbers and make preference for live-setting)
    // This is the diameter of the wheel that the telescoping arm's cable wraps around
    // Used to figure out the conversion from rotational to linear motion
    constexpr auto kPulleyDiameterMeters {0.04851146_m}; 
    constexpr auto kPulleyCircumferenceMeters {kPulleyDiameterMeters * PI}; // TODO see below
    constexpr double kExtensionGearRatio {1.0}; // TODO adjust if using telescoping arm or elevator
    constexpr double kExtensionConversionRotationsToMeters {kPulleyCircumferenceMeters / kExtensionGearRatio}; 
    constexpr auto kExtensionNeutralMode {ctre::phoenix6::signals::NeutralModeValue::Brake};
}
