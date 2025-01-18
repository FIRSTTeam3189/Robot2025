// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Climber.h"

Climber::Climber() :
 m_climberMotor(ClimberConstants::kClimberMotorID),
 m_climberConfig(),
 m_targetHeightClimber(ClimberConstants::kDefaultRetractHeight) {
    ConfigClimberMotor();
    ConfigPID();

    m_allSignals.emplace_back(&m_climberHeight);
    m_allSignals.emplace_back(&m_climberVelocity);
}

// This method will be called once per scheduler run
void Climber::Periodic() {
    RefreshAllSignals();

    frc::SmartDashboard::PutNumber("Coral Extender target height", m_targetHeightClimber.value());
    frc::SmartDashboard::PutNumber("Coral Extender current height", GetExtension().value());

    if (frc::Preferences::GetBoolean("Tuning Mode", false)) {
        UpdatePreferences();
    }

    SetExtension(m_targetHeightClimber);
}

void Climber::ConfigPID() {
    m_climberPKey = "Climber  Rotation P";
    m_climberIKey = "Climber  Rotation I";
    m_climberDKey = "Climber  Rotation D";
    m_climberGKey = "Climber  Rotation G";
    m_climberSKey = "Climber  Rotation S";
    m_climberVKey = "Climber  Rotation V";
    m_climberAKey = "Climber  Rotation A";
    //keys for PID and FF 

    frc::Preferences::SetDouble(m_climberPKey, ClimberConstants::kPClimber);
    frc::Preferences::SetDouble(m_climberIKey, ClimberConstants::kIClimber);
    frc::Preferences::SetDouble(m_climberDKey, ClimberConstants::kDClimber);
    frc::Preferences::SetDouble(m_climberGKey, ClimberConstants::kGClimber);
    frc::Preferences::SetDouble(m_climberSKey, ClimberConstants::kSClimber);
    frc::Preferences::SetDouble(m_climberVKey, ClimberConstants::kVClimber);
    frc::Preferences::SetDouble(m_climberAKey, ClimberConstants::kAClimber);
}

units::meter_t Climber::GetExtension() {
    return units::meter_t{CoralElevatorConstants::kExtensionConversionRotationsToMeters * ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(m_climberHeight, m_climberVelocity).value()};
}

void Climber::RefreshAllSignals() {
    frc::SmartDashboard::PutString("Climber signal status", ctre::phoenix6::BaseStatusSignal::WaitForAll(0.02_s, m_allSignals).GetName());
}

void Climber::SetExtension(units::meter_t target) {
    // create a Motion Magic request, voltage output
    ctre::phoenix6::controls::MotionMagicVoltage request{0_tr};

    // set target position to target meters
    // First, convert meters into rotations of the motor
    m_climberMotor.SetControl(request.WithEnableFOC(true).WithPosition(units::turn_t{target.value() / CoralElevatorConstants::kExtensionConversionRotationsToMeters}));

    frc::SmartDashboard::PutNumber("Climber power", m_climberMotor.Get());
}

void Climber::UpdatePreferences() {
    m_climberConfig.Slot0.kS = frc::Preferences::GetDouble(m_climberSKey, ClimberConstants::kSClimber);
    m_climberConfig.Slot0.kV = frc::Preferences::GetDouble(m_climberVKey, ClimberConstants::kVClimber);
    m_climberConfig.Slot0.kG = frc::Preferences::GetDouble(m_climberGKey, ClimberConstants::kGClimber);
    m_climberConfig.Slot0.kA = frc::Preferences::GetDouble(m_climberAKey, ClimberConstants::kAClimber);
    m_climberConfig.Slot0.kP = frc::Preferences::GetDouble(m_climberPKey, ClimberConstants::kPClimber);
    m_climberConfig.Slot0.kI = frc::Preferences::GetDouble(m_climberIKey, ClimberConstants::kIClimber);
    m_climberConfig.Slot0.kD = frc::Preferences::GetDouble(m_climberDKey, ClimberConstants::kDClimber);
    m_targetHeightClimber = units::meter_t{frc::Preferences::GetDouble(m_climberTargetKey, m_targetHeightClimber.value())};

    m_climberMotor.GetConfigurator().Apply(m_climberConfig);
}

void Climber::ConfigClimberMotor() {
    m_climberMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration({}));

    m_climberConfig.Slot0.kS = ClimberConstants::kSClimber; 
    m_climberConfig.Slot0.kV = ClimberConstants::kVClimber;
    m_climberConfig.Slot0.kG = ClimberConstants::kGClimber;
    m_climberConfig.Slot0.kA = ClimberConstants::kAClimber;
    m_climberConfig.Slot0.kP = ClimberConstants::kPClimber;
    m_climberConfig.Slot0.kI = ClimberConstants::kIClimber;
    m_climberConfig.Slot0.kD = ClimberConstants::kDClimber;
    m_climberConfig.Slot0.GravityType = ctre::phoenix6::signals::GravityTypeValue::Elevator_Static;

    //Motion magic configs
    m_climberConfig.MotionMagic.MotionMagicCruiseVelocity = ClimberConstants::kMotionMagicMaxVelocity;
    m_climberConfig.MotionMagic.MotionMagicAcceleration = ClimberConstants::kMotionMagicMaxAcceleration;
    m_climberConfig.MotionMagic.MotionMagicJerk = ClimberConstants::kMotionMagicMaxJerk;

    //m_climberConfig.Feedback.RotorToSensorRatio = ClimberConstants::kClimberGearRatio;
    m_climberConfig.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor;
    m_climberConfig.CurrentLimits.SupplyCurrentLowerLimit = CoralElevatorConstants::kExtensionContinuousCurrentLimit;
}

void Climber::SetPower(double voltage) {
    m_climberMotor.Set(voltage);
}

void Climber::SetExtensionBrakeMode(BrakeMode mode) {
    switch (mode) {
        case(BrakeMode::Brake) :
            m_climberConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
            break;
        case(BrakeMode::Coast) :
            m_climberConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
            break;
        case(BrakeMode::Default) :
            m_climberConfig.MotorOutput.NeutralMode = CoralElevatorConstants::kExtensionNeutralMode;
            break;
    }

    m_climberMotor.GetConfigurator().Apply(m_climberConfig);
}
