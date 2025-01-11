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

namespace CoralManipulatorConstants{
    constexpr int kRotationMotorID {17};  // TODO

    constexpr unsigned int kRotationCurrentLimit {40};
    constexpr bool kRotationInverted {true};
    constexpr double kRotationConversion {360.0}; 

    constexpr double kPRotation {0.0};
    constexpr double kIRotation {0.0};
    constexpr double kDRotation {0.0};
    constexpr auto kSRotation {0.0_V};
    constexpr auto kGRotation {0.0_V};
    constexpr auto kVRotation {1.0_V * 1.0_s / 1.0_rad};
    // constexpr auto kVRotation {0.65_V * 1.0_s / 1.0_rad};
    constexpr auto kARotation {0.0_V * 1.0_s * 1.0_s / 1.0_rad};

    constexpr auto kRotationAngleTolerance {5.0_deg}; // TODO: go lower if possible (test various numbers and make preference for live-setting)

    constexpr auto kDefaultAngle {90.0_deg};
    constexpr auto kScoreL123Angle {-37.0_deg};
    constexpr auto kScoreL4Angle {27.0_deg};

    constexpr double kFeedforward {1.0};

}
