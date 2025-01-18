#pragma once

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include <iostream>

namespace VisionConstants {
    constexpr wpi::array<double, 3> kEncoderTrustCoefficients {0.1, 0.1, 0.1};
    constexpr wpi::array<double, 3> kVisionTrustCoefficients {0.5, 0.5, 0.5};
    constexpr double kVisionStdDevPerMeter {0.1};
    constexpr units::meter_t kCameraXOffset {0.308_m}; // TODO
    constexpr units::meter_t kCameraYOffset {-0.1778_m}; // TODO
    constexpr units::meter_t kCameraZOffset {0.0_m}; // TODO
    constexpr auto kCameraPitchOffset{35.0_deg}; // TODO
    constexpr auto kCameraYawOffset {180.0_deg}; // TODO
    constexpr bool kShouldUseVision {true};

    constexpr int kBaudRate {115200};
    constexpr int kBufferSize {1024};
    
    // Tag poses in order from 1 to 22
    const std::vector<frc::Pose3d> kTagPoses {
        //* 1
        frc::Pose3d{16.697_m, 0.655_m, 1.486_m, frc::Rotation3d{0.0_deg, 0.0_deg, 126.0_deg}},
        //* 2
        frc::Pose3d{16.697_m, 7.396_m, 1.486_m, frc::Rotation3d{0.0_deg, 0.0_deg, 234.0_deg}},
        //* 3
        frc::Pose3d{11.561_m, 8.056_m, 1.302_m, frc::Rotation3d{0.0_deg, 0.0_deg, 270.0_deg}},
        //* 4
        frc::Pose3d{9.276_m, 6.138_m, 1.868_m, frc::Rotation3d{30.0_deg, 0.0_deg, 0.0_deg}},
        //* 5
        frc::Pose3d{9.276_m, 1.915_m, 1.868_m, frc::Rotation3d{30.0_deg, 0.0_deg, 0.0_deg}},
        //* 6
        frc::Pose3d{13.474_m, 3.306_m, 0.308_m, frc::Rotation3d{0.0_deg, 0.0_deg, 300.0_deg}},
        //* 7
        frc::Pose3d{13.891_m, 4.026_m, 0.308_m, frc::Rotation3d{0.0_deg, 0.0_deg, 0.0_deg}},
        //* 8
        frc::Pose3d{13.474_m, 4.745_m, 0.308_m, frc::Rotation3d{0.0_deg, 0.0_deg, 60.0_deg}},
        //* 9
        frc::Pose3d{12.643_m, 4.745_m, 0.308_m, frc::Rotation3d{0.0_deg, 0.0_deg, 120.0_deg}},
        //* 10
        frc::Pose3d{12.227_m, 4.026_m, 0.308_m, frc::Rotation3d{0.0_deg, 0.0_deg, 180.0_deg}},
        //* 11
        frc::Pose3d{12.643_m, 3.306_m, 0.308_m, frc::Rotation3d{0.0_deg, 0.0_deg, 240.0_deg}},
        //* 12
        frc::Pose3d{0.851_m, 0.655_m, 1.486_m, frc::Rotation3d{0.0_deg, 0.0_deg, 54.0_deg}},
        //* 13
        frc::Pose3d{0.851_m, 7.396_m, 1.486_m, frc::Rotation3d{0.0_deg, 0.0_deg, 306.0_deg}},
        //* 14
        frc::Pose3d{8.272_m, 6.138_m, 1.868_m, frc::Rotation3d{30.0_deg, 0.0_deg, 180.0_deg}},
        //* 15
        frc::Pose3d{8.272_m, 1.915_m, 1.868_m, frc::Rotation3d{30.0_deg, 0.0_deg, 180.0_deg}},
        //* 16
        frc::Pose3d{5.988_m, -0.004_m, 1.302_m, frc::Rotation3d{0.0_deg, 0.0_deg, 90.0_deg}},
        //* 17
        frc::Pose3d{4.074_m, 3.306_m, 0.308_m, frc::Rotation3d{0.0_deg, 0.0_deg, 240.0_deg}},
        //* 18
        frc::Pose3d{3.658_m, 4.026_m, 0.308_m, frc::Rotation3d{0.0_deg, 0.0_deg, 180.0_deg}},
        //* 19
        frc::Pose3d{4.074_m, 4.745_m, 0.308_m, frc::Rotation3d{0.0_deg, 0.0_deg, 120.0_deg}},
        //* 20
        frc::Pose3d{4.905_m, 4.745_m, 0.308_m, frc::Rotation3d{0.0_deg, 0.0_deg, 60.0_deg}},
        //* 21
        frc::Pose3d{5.321_m, 4.026_m, 0.308_m, frc::Rotation3d{0.0_deg, 0.0_deg, 0.0_deg}},
        //* 22
        frc::Pose3d{4.905_m, 3.306_m, 0.308_m, frc::Rotation3d{0.0_deg, 0.0_deg, 300.0_deg}}
    };

    // Vision Sync bytes, in char format
    constexpr std::array kSyncBytes {'\x1a', '\xcf', '\xfc', '\x1d'};
}