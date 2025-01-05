// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/PoseEstimatorHelper.h"

PoseEstimatorHelper::PoseEstimatorHelper() {
}

void PoseEstimatorHelper::SetPoseEstimator(frc::SwerveDrivePoseEstimator<4> *poseEstimator) {
    // set the member variable to the pose estimator variable
    m_poseEstimator = poseEstimator;
} 

void PoseEstimatorHelper::UpdatePoseEstimator(wpi::array<frc::SwerveModulePosition, 4U> modulePositions, frc::Rotation2d rotation) {
    // update the pose estimator with the rotation found by the encoders and the module positions
    m_poseEstimator->Update(rotation, modulePositions);
    UpdateRotation(rotation);
}

void PoseEstimatorHelper::SetActivePath(std::vector<frc::Pose2d> poses) {
    m_autoField.GetObject("path")->SetPoses(poses);
}

void PoseEstimatorHelper::SetCurrentAutoPose(frc::Pose2d pose) {
    m_autoField.SetRobotPose(pose);
}

void PoseEstimatorHelper::SetTargetAutoPose(frc::Pose2d pose) {
    m_autoField.GetObject("target pose")->SetPose(pose);
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
    // reset the positions based on newest values
    m_poseEstimator->ResetPosition(rotation, modulePositions, pose);
}

void PoseEstimatorHelper::AddVisionMeasurement(frc::Pose2d pose, units::second_t timestamp, wpi::array<double, 3> visionMeasurementStdDevs) {
    m_poseEstimator->SetVisionMeasurementStdDevs(visionMeasurementStdDevs);

    // Currently setting pose rotation to estimated rotation rather than vision-deduced one
    // Gyro is already fairly accurate and prevents noisy rotation data for driving/auto
    // Sets pose angle to 0
    pose = pose.TransformBy(frc::Transform2d(0.0_m, 0.0_m, -pose.Rotation()));
    // Then sets pose angle to estimated pose angle
    pose = pose.TransformBy(frc::Transform2d(0.0_m, 0.0_m, m_rotation));
    
    m_visionPose.SetRobotPose(pose);
    frc::SmartDashboard::PutData("Vision pose", &m_visionPose);

    m_poseEstimator->AddVisionMeasurement(pose, frc::Timer::GetFPGATimestamp());
}

void PoseEstimatorHelper::Periodic() {
    m_estimatedPose.SetRobotPose(GetEstimatedPose());
    frc::SmartDashboard::PutData("Estimated pose", &m_estimatedPose);
    frc::SmartDashboard::PutData("Auto field", &m_autoField);
}