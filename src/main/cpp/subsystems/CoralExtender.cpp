// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/CoralExtender.h"

CoralExtender::CoralExtender() :
 m_extensionMotor(CoralExtenderConstants::kExtensionMotorID),
 m_extensionConfig(),
 m_state(CoralExtenderState::DefaultRetract),
 m_targetHeight(CoralExtenderConstants::kDefaultRetractHeight) {
    ConfigExtensionMotor();
    ConfigPID();
}

// This method will be called once per scheduler run
void CoralExtender::Periodic() {
    frc::SmartDashboard::PutNumber("Coral Extender height target", m_targetHeight.value());
    frc::SmartDashboard::PutNumber("Coral Extender height", GetExtension().value());

    if (frc::Preferences::GetBoolean("Tuning Mode", false)) {
        UpdatePreferences();
    }

    SetExtension(m_targetHeight);
}

void CoralExtender::ConfigExtensionMotor() {
    // Set to factory default
    m_extensionMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration({}));

    m_extensionConfig.Slot0.kS = CoralExtenderConstants::kSExtension; 
    m_extensionConfig.Slot0.kV = CoralExtenderConstants::kVExtension;
    m_extensionConfig.Slot0.kG = CoralExtenderConstants::kGExtension;
    m_extensionConfig.Slot0.kA = CoralExtenderConstants::kAExtension;
    m_extensionConfig.Slot0.kP = CoralExtenderConstants::kPExtension;
    m_extensionConfig.Slot0.kI = CoralExtenderConstants::kIExtension;
    m_extensionConfig.Slot0.kD = CoralExtenderConstants::kDExtension;

    // Motion magic configs
    m_extensionConfig.MotionMagic.MotionMagicCruiseVelocity = CoralExtenderConstants::kMotionMagicMaxVelocity;
    m_extensionConfig.MotionMagic.MotionMagicAcceleration = CoralExtenderConstants::kMotionMagicMaxAcceleration;
    m_extensionConfig.MotionMagic.MotionMagicJerk = CoralExtenderConstants::kMotionMagicMaxJerk;

    // m_extensionConfig.Feedback.RotorToSensorRatio = CoralExtenderConstants::kExtensionGearRatio;
    // m_extensionConfig.Feedback.FeedbackRemoteSensorID = CoralExtenderConstants::kRotationCANCoderID; // TODO: check with electrical if using a cancoder for intake/coral wrist rotation
    // m_rotationConfig.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
    m_extensionConfig.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor;

    m_extensionConfig.CurrentLimits.SupplyCurrentLowerLimit = CoralExtenderConstants::kExtensionContinuousCurrentLimit;
    m_extensionConfig.CurrentLimits.SupplyCurrentLimit = CoralExtenderConstants::kExtensionPeakCurrentLimit;
    m_extensionConfig.CurrentLimits.SupplyCurrentLowerTime = CoralExtenderConstants::kExtensionPeakCurrentDuration;
    m_extensionConfig.CurrentLimits.SupplyCurrentLimitEnable = CoralExtenderConstants::kExtensionEnableCurrentLimit;

    m_extensionConfig.MotorOutput.Inverted = CoralExtenderConstants::kExtensionMotorInverted;
    m_extensionConfig.MotorOutput.NeutralMode = CoralExtenderConstants::kExtensionNeutralMode;

    m_extensionMotor.GetConfigurator().Apply(m_extensionConfig);
}

void CoralExtender::ConfigPID() {
    m_extensionPKey = "Coral Extender Rotation P";
    m_extensionIKey = "Coral Extender Rotation I";
    m_extensionDKey = "Coral Extender Rotation D";
    m_extensionGKey = "Coral Extender Rotation G";
    m_extensionSKey = "Coral Extender Rotation S";
    m_extensionVKey = "Coral Extender Rotation V";
    m_extensionAKey = "Coral Extender Rotation A";
    m_extensionTargetKey = "Coral Extender Extension Target";
    //keys for PID and FF 

    frc::Preferences::SetDouble(m_extensionPKey, CoralExtenderConstants::kPExtension);
    frc::Preferences::SetDouble(m_extensionIKey, CoralExtenderConstants::kIExtension);
    frc::Preferences::SetDouble(m_extensionDKey, CoralExtenderConstants::kDExtension);
    frc::Preferences::SetDouble(m_extensionGKey, CoralExtenderConstants::kGExtension);
    frc::Preferences::SetDouble(m_extensionSKey, CoralExtenderConstants::kSExtension);
    frc::Preferences::SetDouble(m_extensionVKey, CoralExtenderConstants::kVExtension);
    frc::Preferences::SetDouble(m_extensionAKey, CoralExtenderConstants::kAExtension);
    frc::Preferences::SetDouble(m_extensionTargetKey, m_targetHeight.value());
}

void CoralExtender::SetState(CoralExtenderState state) {
    // Figure out target angle then set rotation to this angle
    switch (state) {
        case (CoralExtenderState::DefaultRetract):
            m_targetHeight = CoralExtenderConstants::kDefaultRetractHeight;
            break;
        case (CoralExtenderState::L1):
            m_targetHeight = CoralExtenderConstants::kExtensionHeightL1;
            break;
        case (CoralExtenderState::L2):
            m_targetHeight = CoralExtenderConstants::kExtensionHeightL2;
            break;
        case (CoralExtenderState::L3):
            m_targetHeight = CoralExtenderConstants::kExtensionHeightL3;
            break;
        case (CoralExtenderState::L4):
            m_targetHeight = CoralExtenderConstants::kExtensionHeightL4;
            break;
        case (CoralExtenderState::Intake):
            m_targetHeight = CoralExtenderConstants::kExtensionHeightIntake;
            break;
        default:
            m_targetHeight = CoralExtenderConstants::kDefaultRetractHeight;
            break;
    }
    // Extension will be set periodicially
}

void CoralExtender::SetExtension(units::meter_t target) {
    // create a Motion Magic request, voltage output
    ctre::phoenix6::controls::MotionMagicVoltage request{0_tr};

    // set target position to target meters
    // First, convert meters into rotations of the motor
    m_extensionMotor.SetControl(request.WithEnableFOC(true).WithPosition(units::turn_t{target / CoralExtenderConstants::kExtensionConversionRotationsToMeters}));

    frc::SmartDashboard::PutNumber("Coral extender power", m_extensionMotor.Get());
}

units::meter_t CoralExtender::GetExtension() {
    return ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(m_extensionHeight, m_extensionVelocity);
}

units::meter_t CoralExtender::GetCurrentTargetHeight() {
    return m_targetHeight;
}

void CoralExtender::UpdatePreferences() {
    m_extensionConfig.Slot0.kS = frc::Preferences::GetDouble(m_extensionSKey, CoralExtenderConstants::kSExtension);
    m_extensionConfig.Slot0.kV = frc::Preferences::GetDouble(m_extensionVKey, CoralExtenderConstants::kVExtension);
    m_extensionConfig.Slot0.kG = frc::Preferences::GetDouble(m_extensionGKey, CoralExtenderConstants::kGExtension);
    m_extensionConfig.Slot0.kA = frc::Preferences::GetDouble(m_extensionAKey, CoralExtenderConstants::kAExtension);
    m_extensionConfig.Slot0.kP = frc::Preferences::GetDouble(m_extensionPKey, CoralExtenderConstants::kPExtension);
    m_extensionConfig.Slot0.kI = frc::Preferences::GetDouble(m_extensionIKey, CoralExtenderConstants::kIExtension);
    m_extensionConfig.Slot0.kD = frc::Preferences::GetDouble(m_extensionDKey, CoralExtenderConstants::kDExtension);
    m_targetHeight = units::degree_t{frc::Preferences::GetDouble(m_extensionTargetKey, m_targetHeight.value())};

    m_extensionMotor.GetConfigurator().Apply(m_extensionConfig);
}

void CoralExtender::SetExtensionBrakeMode(BrakeMode mode) {
    switch (mode) {
        case(BrakeMode::Brake) :
            m_extensionConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
            break;
        case(BrakeMode::Coast) :
            m_extensionConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
            break;
        case(BrakeMode::Default) :
            m_extensionConfig.MotorOutput.NeutralMode = CoralExtenderConstants::kExtensionNeutralMode;
            break;
    }

    m_extensionMotor.GetConfigurator().Apply(m_extensionConfig);
}