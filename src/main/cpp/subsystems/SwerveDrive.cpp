// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// if this code doesnt work ethan might eat me in a bucket

#include "subsystems/SwerveDrive.h"

SwerveDrive::SwerveDrive(PoseEstimatorHelper *helper) : 
m_modules{
  {+SwerveDriveConstants::kXDistanceFromCenter, +SwerveDriveConstants::kYDistanceFromCenter},
  {+SwerveDriveConstants::kXDistanceFromCenter, -SwerveDriveConstants::kYDistanceFromCenter},
  {-SwerveDriveConstants::kXDistanceFromCenter, +SwerveDriveConstants::kYDistanceFromCenter},
  {-SwerveDriveConstants::kXDistanceFromCenter, -SwerveDriveConstants::kYDistanceFromCenter},
  {0, SwerveModuleConstants::kFrontLeftDriveID, SwerveModuleConstants::kFrontLeftAngleID,
   SwerveModuleConstants::kFrontLeftCANcoderID, SwerveModuleConstants::kFrontLeftOffset},
  {1, SwerveModuleConstants::kFrontRightDriveID, SwerveModuleConstants::kFrontRightAngleID,
   SwerveModuleConstants::kFrontRightCANcoderID, SwerveModuleConstants::kFrontRightOffset},
  {2, SwerveModuleConstants::kBackLeftDriveID, SwerveModuleConstants::kBackLeftAngleID,
   SwerveModuleConstants::kBackLeftCANcoderID, SwerveModuleConstants::kBackLeftOffset},
  {3, SwerveModuleConstants::kBackRightDriveID, SwerveModuleConstants::kBackRightAngleID,
   SwerveModuleConstants::kBackRightCANcoderID, SwerveModuleConstants::kBackRightOffset}
}, 

m_moduleArray(
    &m_modules.m_frontLeft,
    &m_modules.m_frontRight,
    &m_modules.m_backLeft,
    &m_modules.m_backRight
),
    //addresses to access of each swerve module motor
m_pigeon(SwerveDriveConstants::kGyroID, "Swerve"),
m_poseHelper(helper),
m_modulePositions(
    m_modules.m_frontLeft.GetPosition(true),
    m_modules.m_frontRight.GetPosition(true),
    m_modules.m_backLeft.GetPosition(true),
    m_modules.m_backRight.GetPosition(true)
    //initializes the gyroscope and pose helpers for pose estimator
)
{
    ConfigGyro();
    ConfigSignals();
    auto poseEstimator = new frc::SwerveDrivePoseEstimator<4> (SwerveDriveConstants::kKinematics, m_pigeon.GetRotation2d(), 
    m_modulePositions, frc::Pose2d{}, VisionConstants::kEncoderTrustCoefficients, VisionConstants::kVisionTrustCoefficients);
    m_poseHelper->SetPoseEstimator(poseEstimator);
    
    // / Setup autobuilder for pathplannerlib
    pathplanner::AutoBuilder::configure(
        [this](){ return GetEstimatedAutoPose(); }, // Robot pose supplier
        [this](frc::Pose2d pose){ SetPose(pose, false); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this](){ return GetRobotRelativeSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](frc::ChassisSpeeds speeds){ DriveRobotRelative(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        std::make_shared<pathplanner::PPHolonomicDriveController>( // PPHolonomicController is the built in path following controller for holonomic drive trains
            pathplanner::PIDConstants(AutoConstants::kPTranslationAuto, AutoConstants::kITranslationAuto, AutoConstants::kDTranslationAuto), // Translation PID constants
            pathplanner::PIDConstants(AutoConstants::kPRotationAuto, AutoConstants::kIRotationAuto, AutoConstants::kDRotationAuto) // Rotation PID constants
        ),
        m_autoRobotConfig,
        []() {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        this // Reference to this subsystem to set requirements
    );

    frc::Preferences::SetBoolean(m_tuningModeKey, false);
    frc::Preferences::SetBoolean(m_diagnosticsKey, true);
    
    m_drivePKey = "DriveP";
    m_driveIKey = "DriveI";
    m_driveDKey = "DriveD";
    m_anglePKey = "AngleP";
    m_angleIKey = "AngleI";
    m_angleDKey = "AngleD";
    m_rotationSKey = "RotS";

    frc::Preferences::SetDouble(m_drivePKey, SwerveModuleConstants::kPDrive);
    frc::Preferences::SetDouble(m_driveIKey, SwerveModuleConstants::kIDrive);
    frc::Preferences::SetDouble(m_driveDKey, SwerveModuleConstants::kDDrive);
    frc::Preferences::SetDouble(m_anglePKey, SwerveModuleConstants::kPAngle);
    frc::Preferences::SetDouble(m_angleIKey, SwerveModuleConstants::kIAngle);
    frc::Preferences::SetDouble(m_angleDKey, SwerveModuleConstants::kDAngle);
    frc::Preferences::SetDouble(m_rotationSKey, SwerveDriveConstants::kSRot);
}

frc::Pose2d SwerveDrive::GetEstimatedPose() {
    UpdateEstimator();
    return m_poseHelper->GetEstimatedPose();
}

frc::Pose2d SwerveDrive::GetEstimatedAutoPose() {
    UpdateEstimator();
    auto pose = m_poseHelper->GetEstimatedPose();
    frc::SmartDashboard::PutNumber("Auto pose x", pose.X().value());
    frc::SmartDashboard::PutNumber("Auto pose y", pose.Y().value());
    frc::SmartDashboard::PutNumber("Auto pose rot", pose.Rotation().Degrees().value());
    return pose;
}

void SwerveDrive::SetPose(frc::Pose2d pose, bool justRotation) {
    UpdateEstimator();
    if (justRotation) {
        auto currentPose = GetEstimatedPose();
        m_poseHelper->ResetPose(GetNormalizedYaw(), m_modulePositions, frc::Pose2d{currentPose.X(), currentPose.Y(), pose.Rotation()});
        // m_poseHelper->ResetPose(pose.Rotation(), m_modulePositions, frc::Pose2d{currentPose.X(), currentPose.Y(), pose.Rotation()});
    }
    else {
        m_poseHelper->ResetPose(GetNormalizedYaw(), m_modulePositions, pose);
        // m_poseHelper->ResetPose(pose.Rotation(), m_modulePositions, pose);
    }
    //sets pose to current pose
    frc::SmartDashboard::PutNumber("Auto starting pose x", pose.X().value());
    frc::SmartDashboard::PutNumber("Auto starting pose y", pose.Y().value());
    frc::SmartDashboard::PutNumber("Auto starting pose rot", pose.Rotation().Degrees().value());
    for (int i = 0; i < 10; i++) {
        m_pigeon.SetYaw(pose.Rotation().Degrees());
    }
}

void SwerveDrive::DriveRobotRelative(frc::ChassisSpeeds speeds) {
    if (speeds.omega > units::radians_per_second_t(0.0)){
        speeds.omega += units::radians_per_second_t(m_rotationS);
    } else if (speeds.omega < units::radians_per_second_t(0.0)) {
        speeds.omega -= units::radians_per_second_t(m_rotationS);
    }

    auto states = SwerveDriveConstants::kKinematics.ToSwerveModuleStates(speeds);
    auto [fl, fr, bl, br] = states;
    SwerveDriveConstants::kKinematics.DesaturateWheelSpeeds(&states, AutoConstants::kMaxAutoModuleSpeed);

    double AutoDesiredStates[] = 
    {(double)fl.angle.Degrees(), (double)fl.speed,
     (double)fr.angle.Degrees(), (double)fr.speed,
     (double)bl.angle.Degrees(), (double)bl.speed,
     (double)br.angle.Degrees(), (double)br.speed};

    frc::SmartDashboard::PutNumberArray("Auto Desired States", AutoDesiredStates);
    frc::SmartDashboard::PutNumber("Robot relative desired x speed", speeds.vx.value());
    frc::SmartDashboard::PutNumber("Robot relative desired y speed", speeds.vy.value());
    frc::SmartDashboard::PutNumber("Robot relative desired omega speed", speeds.omega.value());

    SetModuleStates(states);
}

frc::ChassisSpeeds SwerveDrive::GetRobotRelativeSpeeds() {
    frc::SwerveModuleState frontLeftModuleState = m_modules.m_frontLeft.GetState(true);
    frc::SwerveModuleState frontRightModuleState = m_modules.m_frontRight.GetState(true);
    frc::SwerveModuleState backLeftModuleState = m_modules.m_backLeft.GetState(true);
    frc::SwerveModuleState backRightModuleState = m_modules.m_backRight.GetState(true);

    auto speeds = SwerveDriveConstants::kKinematics.ToChassisSpeeds(
        frontLeftModuleState, frontRightModuleState, backLeftModuleState, backRightModuleState);

    frc::SmartDashboard::PutNumber("Robot relative x speed", speeds.vx.value());
    frc::SmartDashboard::PutNumber("Robot relative y speed", speeds.vy.value());
    frc::SmartDashboard::PutNumber("Robot relative omega speed", speeds.omega.value());
    return speeds;
}

void SwerveDrive::SetModuleStates(std::array<frc::SwerveModuleState, 4> desiredStates) {
    m_modules.m_frontLeft.SetDesiredState(desiredStates[0]);
    m_modules.m_frontRight.SetDesiredState(desiredStates[1]);
    m_modules.m_backLeft.SetDesiredState(desiredStates[2]);
    m_modules.m_backRight.SetDesiredState(desiredStates[3]);
}

// This method will be called once per scheduler run
void SwerveDrive::Periodic() {
    if (frc::Preferences::GetBoolean(m_tuningModeKey, false)) {
        m_modules.m_frontLeft.UpdatePreferences();
        m_modules.m_frontRight.UpdatePreferences();
        m_modules.m_backLeft.UpdatePreferences();
        m_modules.m_backRight.UpdatePreferences();

        // Update rotation S value
        m_rotationS = frc::Preferences::GetDouble(m_rotationSKey, SwerveDriveConstants::kSRot);
    }
    UpdateEstimator();
    RefreshAllSignals();
}

void SwerveDrive::RefreshAllSignals() {
    frc::SmartDashboard::PutString("Swerve signal status", ctre::phoenix6::BaseStatusSignal::WaitForAll(0.02_s, m_allSignals).GetName());
}

void SwerveDrive::ConfigSignals() {
    // Takes in robot frequency Talon FX using 
    for(int i=0; i < 4; i++) {
        auto signals = m_moduleArray[i]->GetSignals();
        for(int j=0; j < 4; j++) {
            m_allSignals.emplace_back(signals[j]);
        }
    }

    ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(SwerveDriveConstants::kRefreshRate, m_allSignals);
}

units::degree_t SwerveDrive::GetNormalizedYaw() {
    // Normalizes yaw to (-180, 180)
    double yaw =  m_pigeon.GetYaw().GetValue().value(); 
    // (-360, 360)
    double normalizedYaw = fmod(yaw, 360.0);
    // (-180, 180)
    if (normalizedYaw > 180)
        normalizedYaw -= 360;
    else if (normalizedYaw < -180)
        normalizedYaw += 360;

    frc::SmartDashboard::PutNumber("Normalized Yaw", normalizedYaw);
    return units::degree_t{normalizedYaw};
}

void SwerveDrive::ConfigGyro() {
    m_pigeon.GetConfigurator().Apply(ctre::phoenix6::configs::Pigeon2Configuration{});
    m_pigeonConfigs.MountPose.MountPoseYaw = SwerveDriveConstants::kGyroMountPoseYaw;
    m_pigeon.GetConfigurator().Apply(m_pigeonConfigs);
}

void SwerveDrive::Drive(units::meters_per_second_t xSpeed,
                        units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
                        bool fieldRelative,
                        frc::Translation2d centerOfRotation) {

                        auto states = SwerveDriveConstants::kKinematics.ToSwerveModuleStates(
                        (fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, m_pigeon.GetRotation2d())
                            : frc::ChassisSpeeds{xSpeed, ySpeed, rot}),
                            centerOfRotation);
                        }

void SwerveDrive::UpdateEstimator() {
    // Get current positions and update
    m_modulePositions[0] = m_modules.m_frontLeft.GetPosition(true);
    m_modulePositions[1] = m_modules.m_frontRight.GetPosition(true);
    m_modulePositions[2] = m_modules.m_backLeft.GetPosition(true);
    m_modulePositions[3] = m_modules.m_backRight.GetPosition(true);
}

void SwerveDrive::Stop() {
    m_modules.m_frontLeft.Stop();
    m_modules.m_frontRight.Stop();
    m_modules.m_backLeft.Stop();
    m_modules.m_backRight.Stop();
}

void SwerveDrive::SetPose(frc::Pose2d pose, bool justRotation) {
    UpdateEstimator();
    if (justRotation) {
        auto currentPose = GetEstimatedPose();
        m_poseHelper->ResetPose(GetNormalizedYaw(), m_modulePositions, frc::Pose2d{currentPose.X(), currentPose.Y(), pose.Rotation()});
        // m_poseHelper->ResetPose(pose.Rotation(), m_modulePositions, frc::Pose2d{currentPose.X(), currentPose.Y(), pose.Rotation()});
    }
    else {
        m_poseHelper->ResetPose(GetNormalizedYaw(), m_modulePositions, pose);
        // m_poseHelper->ResetPose(pose.Rotation(), m_modulePositions, pose);
    }
    //sets pose to current pose
    frc::SmartDashboard::PutNumber("Auto starting pose x", pose.X().value());
    frc::SmartDashboard::PutNumber("Auto starting pose y", pose.Y().value());
    frc::SmartDashboard::PutNumber("Auto starting pose rot", pose.Rotation().Degrees().value());
    for (int i = 0; i < 10; i++) {
        m_pigeon.SetYaw(pose.Rotation().Degrees());
    }
}

void SwerveDrive::SetBrakeMode(BrakeMode mode) {
    for (auto mod: m_moduleArray) {
        mod->SetBrakeMode(mode);
    }
}