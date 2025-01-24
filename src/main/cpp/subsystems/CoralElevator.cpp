// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/CoralElevator.h"

CoralElevator::CoralElevator() :
 m_extensionMotor(CoralElevatorConstants::kExtensionMotorID, "Swerve"),
 m_extensionConfig(),
 m_state(CoralElevatorState::DefaultRetract),
 m_targetHeight(CoralElevatorConstants::kDefaultRetractHeight) {
    ConfigExtensionMotor();
    ConfigPID();

    m_allSignals.emplace_back(&m_extensionHeight);
    m_allSignals.emplace_back(&m_extensionVelocity);
}

// This method will be called once per scheduler run
void CoralElevator::Periodic() {
    RefreshAllSignals();
    m_currentHeight = GetExtension();

    frc::SmartDashboard::PutNumber("Coral Extender target height", m_targetHeight.value());
    frc::SmartDashboard::PutNumber("Coral Extender current height", GetExtension().value());

    if (frc::Preferences::GetBoolean("Tuning Mode", false)) {
        UpdatePreferences();
    }

    SetExtension(m_targetHeight);
}

void CoralElevator::ConfigExtensionMotor() {
    // Set to factory default
    m_extensionMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration({}));

    m_extensionConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    m_extensionConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    m_extensionConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = CoralElevatorConstants::kTopSoftLimit; // Top
    m_extensionConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = CoralElevatorConstants::kBottomSoftLimit; // Bottom

    m_extensionConfig.MotorOutput.DutyCycleNeutralDeadband = CoralElevatorConstants::kCoralElevatorSpeedDeadband;

    m_extensionConfig.Slot0.kS = CoralElevatorConstants::kSExtension; 
    m_extensionConfig.Slot0.kV = CoralElevatorConstants::kVExtension;
    m_extensionConfig.Slot0.kG = CoralElevatorConstants::kGExtension;
    m_extensionConfig.Slot0.kA = CoralElevatorConstants::kAExtension;
    m_extensionConfig.Slot0.kP = CoralElevatorConstants::kPExtension;
    m_extensionConfig.Slot0.kI = CoralElevatorConstants::kIExtension;
    m_extensionConfig.Slot0.kD = CoralElevatorConstants::kDExtension;
    m_extensionConfig.Slot0.GravityType = ctre::phoenix6::signals::GravityTypeValue::Elevator_Static;

    // Motion magic configs
    m_extensionConfig.MotionMagic.MotionMagicCruiseVelocity = CoralElevatorConstants::kMotionMagicMaxVelocity;
    m_extensionConfig.MotionMagic.MotionMagicAcceleration = CoralElevatorConstants::kMotionMagicMaxAcceleration;
    m_extensionConfig.MotionMagic.MotionMagicJerk = CoralElevatorConstants::kMotionMagicMaxJerk;

    // m_extensionConfig.Feedback.RotorToSensorRatio = CoralElevatorConstants::kExtensionGearRatio;
    // m_extensionConfig.Feedback.FeedbackRemoteSensorID = CoralElevatorConstants::kRotationCANCoderID; // TODO: check with electrical if using a cancoder for intake/coral wrist rotation
    // m_rotationConfig.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
    m_extensionConfig.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor;
    m_extensionConfig.Feedback.FeedbackRotorOffset = CoralElevatorConstants::kExtensionOffset;

    m_extensionConfig.CurrentLimits.SupplyCurrentLowerLimit = CoralElevatorConstants::kExtensionContinuousCurrentLimit;
    m_extensionConfig.CurrentLimits.SupplyCurrentLimit = CoralElevatorConstants::kExtensionPeakCurrentLimit;
    m_extensionConfig.CurrentLimits.SupplyCurrentLowerTime = CoralElevatorConstants::kExtensionPeakCurrentDuration;
    m_extensionConfig.CurrentLimits.SupplyCurrentLimitEnable = CoralElevatorConstants::kExtensionEnableCurrentLimit;

    m_extensionConfig.MotorOutput.Inverted = CoralElevatorConstants::kExtensionMotorInverted;
    m_extensionConfig.MotorOutput.NeutralMode = CoralElevatorConstants::kExtensionNeutralMode;

    m_extensionMotor.GetConfigurator().Apply(m_extensionConfig);
    m_extensionMotor.SetPosition(0.0_tr);
}

void CoralElevator::ConfigPID() {
    m_extensionPKey = "Coral Elevator P";
    m_extensionIKey = "Coral Elevator I";
    m_extensionDKey = "Coral Elevator D";
    m_extensionGKey = "Coral Elevator G";
    m_extensionSKey = "Coral Elevator S";
    m_extensionVKey = "Coral Elevator V";
    m_extensionAKey = "Coral Elevator A";
    m_extensionTargetKey = "Coral Elevator Target Height";
    //keys for PID and FF 

    frc::Preferences::SetDouble(m_extensionPKey, CoralElevatorConstants::kPExtension);
    frc::Preferences::SetDouble(m_extensionIKey, CoralElevatorConstants::kIExtension);
    frc::Preferences::SetDouble(m_extensionDKey, CoralElevatorConstants::kDExtension);
    frc::Preferences::SetDouble(m_extensionGKey, CoralElevatorConstants::kGExtension);
    frc::Preferences::SetDouble(m_extensionSKey, CoralElevatorConstants::kSExtension);
    frc::Preferences::SetDouble(m_extensionVKey, CoralElevatorConstants::kVExtension);
    frc::Preferences::SetDouble(m_extensionAKey, CoralElevatorConstants::kAExtension);
    frc::Preferences::SetDouble(m_extensionTargetKey, m_targetHeight.value());
}

void CoralElevator::SetState(CoralElevatorState state) {
    // Figure out target angle then set rotation to this angle
    switch (state) {
        case (CoralElevatorState::DefaultRetract):
            m_targetHeight = CoralElevatorConstants::kDefaultRetractHeight;
            break;
        case (CoralElevatorState::L1):
            m_targetHeight = CoralElevatorConstants::kExtensionHeightL1;
            break;
        case (CoralElevatorState::L2):
            m_targetHeight = CoralElevatorConstants::kExtensionHeightL2;
            break;
        case (CoralElevatorState::L3):
            m_targetHeight = CoralElevatorConstants::kExtensionHeightL3;
            break;
        case (CoralElevatorState::L4):
            m_targetHeight = CoralElevatorConstants::kExtensionHeightL4;
            break;
        case (CoralElevatorState::Intake):
            m_targetHeight = CoralElevatorConstants::kExtensionHeightIntake;
            break;
        default:
            m_targetHeight = CoralElevatorConstants::kDefaultRetractHeight;
            break;
    }
    // Extension will be set periodicially
}

void CoralElevator::SetExtension(units::meter_t target) {
    // create a Motion Magic request, voltage output
    ctre::phoenix6::controls::MotionMagicVoltage request{0_tr};

    // set target position to target meters
    // First, convert meters into rotations of the motor
    m_extensionMotor.SetControl(request.WithEnableFOC(true).WithPosition(units::turn_t{target.value() / CoralElevatorConstants::kExtensionConversionRotationsToMeters}));

    frc::SmartDashboard::PutBoolean("Manual soft limit enabled", false);
    double speed = m_extensionMotor.Get();
    // if ((m_currentHeight.value() < CoralElevatorConstants::kCoralElevatorBottomTolerance.value() && speed < 0) ||
    //      (m_currentHeight.value() > CoralElevatorConstants::kCoralElevatorTopTolerance.value() && speed > 0)) {
    //     frc::SmartDashboard::PutBoolean("Manual soft limit enabled", true);
    //     speed = std::clamp(speed, -0.1, 0.1); //TODO -- change limits later
    //     m_extensionMotor.Set(speed);
    // }

    // ! AAAAA
    // TODO get rid of this if not using linear actuator
    // if (abs(speed) < CoralElevatorConstants::kCoralElevatorSpeedDeadband) {
    //     m_extensionMotor.Set(0.0);
    // }
    
    frc::SmartDashboard::PutNumber("Coral extender power", speed);
}

units::meter_t CoralElevator::GetExtension() {
    return units::meter_t{CoralElevatorConstants::kExtensionConversionRotationsToMeters * ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(m_extensionHeight, m_extensionVelocity).value()};
}

void CoralElevator::RefreshAllSignals() {
    frc::SmartDashboard::PutString("Coral elevator signal status", ctre::phoenix6::BaseStatusSignal::WaitForAll(0.02_s, m_allSignals).GetName());
}

units::meter_t CoralElevator::GetCurrentTargetHeight() {
    return m_targetHeight;
}

void CoralElevator::UpdatePreferences() {
    m_extensionConfig.Slot0.kS = frc::Preferences::GetDouble(m_extensionSKey, CoralElevatorConstants::kSExtension);
    m_extensionConfig.Slot0.kV = frc::Preferences::GetDouble(m_extensionVKey, CoralElevatorConstants::kVExtension);
    m_extensionConfig.Slot0.kG = frc::Preferences::GetDouble(m_extensionGKey, CoralElevatorConstants::kGExtension);
    m_extensionConfig.Slot0.kA = frc::Preferences::GetDouble(m_extensionAKey, CoralElevatorConstants::kAExtension);
    m_extensionConfig.Slot0.kP = frc::Preferences::GetDouble(m_extensionPKey, CoralElevatorConstants::kPExtension);
    m_extensionConfig.Slot0.kI = frc::Preferences::GetDouble(m_extensionIKey, CoralElevatorConstants::kIExtension);
    m_extensionConfig.Slot0.kD = frc::Preferences::GetDouble(m_extensionDKey, CoralElevatorConstants::kDExtension);
    m_targetHeight = units::meter_t{frc::Preferences::GetDouble(m_extensionTargetKey, m_targetHeight.value())};

    m_extensionMotor.GetConfigurator().Apply(m_extensionConfig);
}

void CoralElevator::SetExtensionBrakeMode(BrakeMode mode) {
    switch (mode) {
        case(BrakeMode::Brake) :
            m_extensionConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
            break;
        case(BrakeMode::Coast) :
            m_extensionConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
            break;
        case(BrakeMode::Default) :
            m_extensionConfig.MotorOutput.NeutralMode = CoralElevatorConstants::kExtensionNeutralMode;
            break;
    }

    m_extensionMotor.GetConfigurator().Apply(m_extensionConfig);
}