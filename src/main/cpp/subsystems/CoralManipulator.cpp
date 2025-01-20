// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/CoralManipulator.h"

CoralManipulator::CoralManipulator() :
    m_rotationMotor(CoralManipulatorConstants::kRotationMotorID, rev::spark::SparkMax::MotorType::kBrushless),
    m_rollerMotor(CoralManipulatorConstants::kRollerMotorID, rev::spark::SparkMax::MotorType::kBrushless),
    m_rotationEncoder(m_rotationMotor.GetAbsoluteEncoder()),
    m_rotationConfig(),
    m_rollerConfig(),
    m_targetAngle(CoralManipulatorConstants::kDefaultAngle),
    m_state(),
    m_profiledPIDController(CoralManipulatorConstants::kPRotation, CoralManipulatorConstants::kIRotation, CoralManipulatorConstants::kDRotation, m_constraints)
{
    ConfigRotationMotor();
    ConfigRollerMotor();
    ConfigPID();
}
// This method will be called once per scheduler run
void CoralManipulator::Periodic() {
    frc::SmartDashboard::PutNumber("Coral Manipulator PID target", m_targetAngle.value());
    frc::SmartDashboard::PutNumber("Coral Manipulator rotation", GetRotation().value());

    if (frc::Preferences::GetBoolean("Full Diagnostics", false)) {
        frc::SmartDashboard::PutNumber("Coral Manipulator desired rotational velocity", m_profiledPIDController.GetSetpoint().velocity.value());
        frc::SmartDashboard::PutNumber("Coral Manipulator rotational acceleration", m_acceleration.value());
        frc::SmartDashboard::PutNumber("Coral Manipulator desired rotational acceleration", m_targetAcceleration.value());
    }

    if (frc::Preferences::GetBoolean("Tuning Mode", false)) {
        UpdatePreferences();
    }

    SetRotation(m_targetAngle);
}

void CoralManipulator::ConfigRotationMotor(){
    m_rotationConfig
        .Inverted(CoralManipulatorConstants::kRotationInverted)
        .SmartCurrentLimit(CoralManipulatorConstants::kRotationCurrentLimit)
        .SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);
    m_rotationConfig.absoluteEncoder
        .PositionConversionFactor(CoralManipulatorConstants::kRotationConversion)
        .VelocityConversionFactor(CoralManipulatorConstants::kRotationConversion);

    m_rotationMotor.Configure(m_rotationConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
}

void CoralManipulator::ConfigRollerMotor(){
    m_rollerConfig
        .Inverted(CoralManipulatorConstants::kRollerInverted)
        .SmartCurrentLimit(CoralManipulatorConstants::kRollerCurrentLimit)
        .SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);

    m_rollerMotor.Configure(m_rollerConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
}

void CoralManipulator::ConfigPID(){
    m_ff = new frc::ArmFeedforward(
        CoralManipulatorConstants::kSRotation,
        CoralManipulatorConstants::kGRotation,
        CoralManipulatorConstants::kVRotation,
        CoralManipulatorConstants::kARotation
    );

    //takes speed, gravity, velocity and acceleration to construct feed forward object

    m_rotationPKey = "Coral Manipulator Rotation P";
    m_rotationIKey = "Coral Manipulator Rotation I";
    m_rotationDKey = "Coral Manipulator Rotation D";
    m_rotationGKey = "Coral Manipulator Rotation G";
    m_rotationSKey = "Coral Manipulator Rotation S";
    m_rotationVKey = "Coral Manipulator Rotation V";
    m_rotationAKey = "Coral Manipulator Rotation A";
    m_rotationTargetKey = "Coral Manipulator Rotation Target";
    //keys for PID and FF 

    frc::Preferences::SetDouble(m_rotationPKey, CoralManipulatorConstants::kPRotation);
    frc::Preferences::SetDouble(m_rotationIKey, CoralManipulatorConstants::kIRotation);
    frc::Preferences::SetDouble(m_rotationDKey, CoralManipulatorConstants::kDRotation);
    frc::Preferences::SetDouble(m_rotationGKey, CoralManipulatorConstants::kGRotation.value());
    frc::Preferences::SetDouble(m_rotationSKey, CoralManipulatorConstants::kSRotation.value());
    frc::Preferences::SetDouble(m_rotationVKey, CoralManipulatorConstants::kVRotation.value());
    frc::Preferences::SetDouble(m_rotationAKey, CoralManipulatorConstants::kARotation.value());
    frc::Preferences::SetDouble(m_rotationTargetKey, m_targetAngle.value());
}

void CoralManipulator::SetRotation(units::degree_t targetAngle){
    units::volt_t PIDValue = units::volt_t{(targetAngle - GetRotation()).value() * m_profiledPIDController.GetP()};  
    units::volt_t ffValue = 0.0_V;

    // Only use feedforward/motion profile if actively trying to move
    if (m_state == CoralManipulatorState::GoTarget) {
        ffValue = GetMotionProfileFeedForwardValue();
    }

    m_rotationMotor.SetVoltage(std::clamp((PIDValue + ffValue), -12.0_V, 12.0_V));

    frc::SmartDashboard::PutNumber("Coral Manipulator power", m_rotationMotor.Get());
    frc::SmartDashboard::PutNumber("Coral Manipulator rotation volts", PIDValue.value() + ffValue.value());
    frc::SmartDashboard::PutNumber("Coral Manipulator rotation PID", PIDValue.value());
    frc::SmartDashboard::PutNumber("Coral Manipulator rotation FF", ffValue.value());

    m_lastTargetSpeed = m_profiledPIDController.GetSetpoint().velocity;
    m_lastSpeed = units::degrees_per_second_t{m_rotationEncoder.GetVelocity()};
    m_lastTime = frc::Timer::GetFPGATimestamp();
}

units::volt_t CoralManipulator::GetMotionProfileFeedForwardValue() {
    // Generate motion profile
    m_profiledPIDController.Calculate(GetRotation(), m_targetAngle);

    // Calculates the change in velocity (acceleration) since last control loop
    // Uses the acceleration value and desired velocity to calculate feedforward gains
    // Feedforward gains are approximated based on the current state of the system and a known physics model
    // Gains calculated with SysID   
    m_acceleration = (units::degrees_per_second_t{m_rotationEncoder.GetVelocity()} - m_lastSpeed) /
      (frc::Timer::GetFPGATimestamp() - m_lastTime);

    m_targetAcceleration = (m_profiledPIDController.GetSetpoint().velocity - m_lastTargetSpeed) /
      (frc::Timer::GetFPGATimestamp() - m_lastTime);
    units::volt_t ffValue = m_ff->Calculate(units::radian_t{m_targetAngle}, units::radians_per_second_t{m_profiledPIDController.GetSetpoint().velocity},
                                           units::radians_per_second_squared_t{m_targetAcceleration});

    return ffValue;
}

units::degree_t CoralManipulator::GetRotation() {
    auto rawRotation = m_rotationEncoder.GetPosition();
    if (rawRotation > 180.0)
        rawRotation -= 360.0;

    return units::degree_t{rawRotation};
}

void CoralManipulator::SetState(CoralManipulatorState state, CoralManipulatorTarget target) {
    switch(state){
        case (CoralManipulatorState::HoldPosition) :
            m_state = CoralManipulatorState::HoldPosition;
            break;
        case (CoralManipulatorState::GoTarget) :
            m_state = CoralManipulatorState::GoTarget;
            m_targetAngle = GetTargetAngleFromTarget(target);
            break;
    }
    // SetRotation is already going to be called in Periodic.
}

void CoralManipulator::SetRollerPower(double power) {
    m_rollerMotor.Set(power);
}

units::degree_t CoralManipulator::GetCurrentTargetAngle() {
    return m_targetAngle;
}

units::degree_t CoralManipulator::GetTargetAngleFromTarget(CoralManipulatorTarget target) {
    auto targetAngle = 0.0_deg;
    switch (target) {
        case (CoralManipulatorTarget::ScoreCoralL123):
            targetAngle = CoralManipulatorConstants::kScoreL123Angle;
            break;
        case (CoralManipulatorTarget::ScoreCoralL4):
            targetAngle = CoralManipulatorConstants::kScoreL4Angle;
            break;
        case (CoralManipulatorTarget::Intake):
            targetAngle = CoralManipulatorConstants::kIntakeAngle;
            break;
        case (CoralManipulatorTarget::AlgaeRemovalPosition):
            targetAngle = CoralManipulatorConstants::AlgaeRemovalAngle;
            break;
        case (CoralManipulatorTarget::DefaultPosition):
            targetAngle = CoralManipulatorConstants::kDefaultAngle;
            break;
        default:
            targetAngle = CoralManipulatorConstants::kDefaultAngle;
            break;
    }

    return targetAngle;
}

void CoralManipulator::UpdatePreferences(){
    m_profiledPIDController.SetP(frc::Preferences::GetDouble(m_rotationPKey, CoralManipulatorConstants::kPRotation));
    m_profiledPIDController.SetI(frc::Preferences::GetDouble(m_rotationIKey, CoralManipulatorConstants::kIRotation));
    m_profiledPIDController.SetD(frc::Preferences::GetDouble(m_rotationDKey, CoralManipulatorConstants::kDRotation));
    // set the PID values based on the inputted key
    double s = frc::Preferences::GetDouble(m_rotationSKey, CoralManipulatorConstants::kSRotation.value());
    double g = frc::Preferences::GetDouble(m_rotationGKey, CoralManipulatorConstants::kGRotation.value());
    double v = frc::Preferences::GetDouble(m_rotationVKey, CoralManipulatorConstants::kVRotation.value());
    double a = frc::Preferences::GetDouble(m_rotationAKey, CoralManipulatorConstants::kARotation.value());
    //get the speed, gravity, velocity and acceleration values
    m_targetAngle = units::degree_t{frc::Preferences::GetDouble(m_rotationTargetKey, m_targetAngle.value())};
    delete m_ff;
    m_ff = new frc::ArmFeedforward(
        units::volt_t{s},
        units::volt_t{g},
        units::unit_t<frc::ArmFeedforward::kv_unit>{v},
        units::unit_t<frc::ArmFeedforward::ka_unit>{a}
    );
}
