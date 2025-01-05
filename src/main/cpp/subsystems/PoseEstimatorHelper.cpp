// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/PoseEstimatorHelper.h"

PoseEstimatorHelper::PoseEstimatorHelper() = default;

// This method will be called once per scheduler run
void PoseEstimatorHelper::Periodic() {
    m_estimatedPose.SetRobotPose(GetEstimatedPose());
    frc::SmartDashboard::PutData("Estimated pose", &m_estimatedPose);
    frc::SmartDashboard::PutData("Auto field", &m_autoField);
}

void PoseEstimatorHelper::SetPoseEstimator(frc::SwerveDrivePoseEstimator<4> *poseEstimator) {
    m_poseEstimator = poseEstimator;
    //set the member variable to the pose estimator variable
}

void PoseEstimatorHelper::UpdatePoseEstimator(wpi::array<frc::SwerveModulePosition, 4U> modulePositions, frc::Rotation2d rotation) {
    m_poseEstimator->Update(rotation, modulePositions);
    UpdateRotation(rotation);
    //update the pose estimator with the rotation found by the encoders and the module positions
}

frc::Pose2d PoseEstimatorHelper::GetEstimatedPose() {
    // Ignores rotation component on vision and uses gyroscope instead
    frc::Pose2d pigeonTrustingPose = frc::Pose2d{
        m_poseEstimator->GetEstimatedPosition().Translation(), m_rotation};
    return pigeonTrustingPose;
}

void PoseEstimatorHelper::UpdateRotation(frc::Rotation2d rotation) {
    m_rotation = rotation;
}

void PoseEstimatorHelper::ResetPose(frc::Rotation2d rotation, wpi::array<frc::SwerveModulePosition, 4> modulePositions, frc::Pose2d pose) {
    m_poseEstimator->ResetPosition(rotation, modulePositions, pose);
    //reset the positions based on newest values
}
