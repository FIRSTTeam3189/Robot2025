#pragma once

#include <pathplanner/lib/config/PIDConstants.h>
#include <pathplanner/lib/config/ModuleConfig.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <frc/system/plant/DCMotor.h>

#define PI 3.14159265358979323846

namespace AutoConstants {
    // Distance from robot center to furthest module
    constexpr auto kDriveBaseRadius {0.2921_m};
    constexpr auto kMaxAutoModuleSpeed{4.0_mps}; // 5.0_mps

    // Translation PID
    // constexpr double kPTranslationAuto {4.5};
    constexpr double kPTranslationAuto {0.0};
    constexpr double kITranslationAuto {0.0};
    constexpr double kDTranslationAuto {0.0};

    // Rotation PID
    // constexpr double kPRotationAuto {5.0};
    constexpr double kPRotationAuto {0.0};
    constexpr double kIRotationAuto {0.0};
    constexpr double kDRotationAuto {0.0};

    // TODO
    constexpr auto kAlgaeIntakeMaxExtendTime {1.0_s};
    constexpr auto kAlgaeIntakeMaxRetractTime {1.0_s};
    constexpr auto kCoralElevatorMaxIntakeExtendTime {1.0_s};
    constexpr auto kCoralManipulatorMaxIntakeExtendTime {1.0_s};
    constexpr auto kCoralIntakeWaitingTime {2.5_s};
    constexpr auto kCoralElevatorMaxRetractTime {1.0_s};
    constexpr auto kCoralManipulatorMaxRetractTime {1.0_s};
    constexpr auto kCoralIntakeScoreTime {1.5_s};

    constexpr auto kNominalVoltage {12.0_V};
    constexpr auto kStallTorque = units::newton_meter_t{9.37};
    constexpr auto kStallCurrent = units::ampere_t{483};
    constexpr auto kFreeCurrent = units::ampere_t{2};
    constexpr auto kFreeSpeed = units::radians_per_second_t{607.374579};

    // using namespace std::literals;
    // constexpr std::array kAutonomousPaths {
    //     "Test - Line"sv
    // };
}