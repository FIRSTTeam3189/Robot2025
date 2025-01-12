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
    // constexpr double kPTranslationAuto {1.5};
    constexpr double kPTranslationAuto {4.5};
    constexpr double kITranslationAuto {0.0};
    constexpr double kDTranslationAuto {0.0};

    // Rotation PID
    // constexpr double kPRotationAuto {1.5};
    constexpr double kPRotationAuto {5.0};
    constexpr double kIRotationAuto {0.0};
    constexpr double kDRotationAuto {0.0};

    // TODO: redo for krakens
    // nominalVoltage, stallTorque, stallCurrent, freeCurrent, freeSpeed, numMotors
    constexpr frc::DCMotor kDriveMotorConfig {
        12.0_V, units::newton_meter_t{5.84}, units::ampere_t{304.0}, units::ampere_t{1.5}, units::radians_per_second_t{636.6961104}
    };
}