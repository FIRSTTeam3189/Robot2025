#include "subsystems/AlgaeIntake.h"

AlgaeIntake::AlgaeIntake() : 
 m_rotationMotor(AlgaeIntakeConstants::kRotationMotorID, rev::spark::SparkMax::MotorType::kBrushless),
 m_rollerMotor(AlgaeIntakeConstants::kRollerMotorID, rev::spark::SparkMax::MotorType::kBrushless),
 m_rotationConfig(),
 m_rollerConfig(),
 m_constraints(AlgaeIntakeConstants::kMaxRotationVelocity, AlgaeIntakeConstants::kMaxRotationAcceleration),
 m_profiledPIDController(AlgaeIntakeConstants::kPRotation, AlgaeIntakeConstants::kIRotation, AlgaeIntakeConstants::kDRotation, m_constraints),
 m_targetAngle(AlgaeIntakeConstants::kRetractTarget),
 m_isActive(false)
{
    ConfigRotationMotor();
    ConfigRollerMotor();
    ConfigPID();

    //configure motors and PID

    // std::cout << "Intake constructed\n";
}

void Intake::ConfigRotationMotor() {
    m_rotationConfig
        .Inverted(IntakeConstants::kRotationMotorInverted)
        .SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake)
        .SmartCurrentLimit(IntakeConstants::kRotationCurrentLimit);
    m_rotationConfig.absoluteEncoder
        .PositionConversionFactor(IntakeConstants::kRotationConversion)
        .VelocityConversionFactor(IntakeConstants::kRotationConversion)
        .ZeroOffset(IntakeConstants::kRotationOffset);
    m_rotationConfig.signals
        .AbsoluteEncoderPositionPeriodMs(20)
        .AbsoluteEncoderVelocityPeriodMs(20);
    m_rotationMotor.Configure(m_rotationConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);

    // RevLib 2024 old code
    // m_rotationMotor.RestoreFactoryDefaults();
    // m_rotationMotor.SetIdleMode(IntakeConstants::kIdleMode);
    // m_rotationMotor.SetSmartCurrentLimit(IntakeConstants::kRotationCurrentLimit);
    // m_rotationMotor.SetInverted(IntakeConstants::kRotationMotorInverted);
    // m_rotationMotor.SetPeriodicFramePeriod(rev::spark::SparkMax::PeriodicFrame::kStatus5, 20);
    // m_rotationMotor.SetPeriodicFramePeriod(rev::spark::SparkMax::PeriodicFrame::kStatus6, 20);
    // m_rotationEncoder.SetInverted(IntakeConstants::kRotationInverted);
    // m_rotationEncoder.SetPositionConversionFactor(IntakeConstants::kRotationConversion);
    // m_rotationEncoder.SetVelocityConversionFactor(IntakeConstants::kRotationConversion);
    // m_rotationEncoder.SetZeroOffset(IntakeConstants::kRotationOffset);
}

void Intake::ConfigRollerMotor() {
    m_rollerConfig
        .Inverted(IntakeConstants::kRollerInverted)
        .SmartCurrentLimit(IntakeConstants::kRollerCurrentLimit);
    m_rollerMotor.Configure(m_rollerConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
}

void Intake::ConfigPID() {
    m_ff = new frc::ArmFeedforward(
        IntakeConstants::kSRotation,
        IntakeConstants::kGRotation,
        IntakeConstants::kVRotation,
        IntakeConstants::kARotation
    );

    //takes speed, gravity, velocity and acceleration to construct feed forward object

    m_rotationPKey = "Intake Rotation P";
    m_rotationIKey = "Intake Rotation I";
    m_rotationDKey = "IntakeSetState Rotation D";
    m_rotationGKey = "Intake Rotation G";
    m_rotationSKey = "Intake Rotation S";
    m_rotationVKey = "Intake Rotation V";
    m_rotationAKey = "Intake Rotation A";
    m_rotationTargetKey = "Intake Rotation Target";
    //keys for PID and FF 

    frc::Preferences::SetDouble(m_rotationPKey, IntakeConstants::kPRotation);
    frc::Preferences::SetDouble(m_rotationIKey, IntakeConstants::kIRotation);
    frc::Preferences::SetDouble(m_rotationDKey, IntakeConstants::kDRotation);
    frc::Preferences::SetDouble(m_rotationGKey, IntakeConstants::kGRotation.value());
    frc::Preferences::SetDouble(m_rotationSKey, IntakeConstants::kSRotation.value());
    frc::Preferences::SetDouble(m_rotationVKey, IntakeConstants::kVRotation.value());
    frc::Preferences::SetDouble(m_rotationAKey, IntakeConstants::kARotation.value());
    frc::Preferences::SetDouble(m_rotationTargetKey, m_targetAngle.value());
}

//initializing PID and SGVA values

units::volt_t Intake::GetMotionProfileFeedForwardValue() {
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
    units::volt_t ffValue = m_ff->Calculate(units::radian_t{target}, units::radians_per_second_t{m_profiledPIDController.GetSetpoint().velocity},
                                           units::radians_per_second_squared_t{m_targetAcceleration});

    return ffValue;
}

void Intake::SetRotation(units::degree_t target) {
    // Calculates PID value in volts based on position and target
    units::volt_t PIDValue = units::volt_t{(target - GetRotation()).value() * m_profiledPIDController.GetP()};  
    
    // Only use feedforward/motion profile if actively trying to move
    if
    m_rotationMotor.SetVoltage(std::clamp((PIDValue + ffValue), -12.0_V, 12.0_V));

    frc::SmartDashboard::PutNumber("Intake power", m_rotationMotor.Get());
    frc::SmartDashboard::PutNumber("Intake rotation volts", PIDValue.value() + ffValue.value());
    frc::SmartDashboard::PutNumber("Intake rotation PID", PIDValue.value());
    frc::SmartDashboard::PutNumber("Intake rotation FF", ffValue.value());

    m_lastTargetSpeed = m_profiledPIDController.GetSetpoint().velocity;
    m_lastSpeed = units::degrees_per_second_t{m_rotationEncoder.GetVelocity()};
    m_lastTime = frc::Timer::GetFPGATimestamp();
}

// This method will be called once per scheduler run
void Intake::Periodic() {
    frc::SmartDashboard::PutNumber("Intake PID target", m_targetAngle.value());
    frc::SmartDashboard::PutNumber("Intake rotation", GetRotation().value());
    frc::SmartDashboard::PutNumber("Intake power", m_rollerMotor.Get());

    if (frc::Preferences::GetBoolean("Full Diagnostics", false)) {
        frc::SmartDashboard::PutNumber("Intake rotational velocity", m_rotationEncoder.GetVelocity());
        frc::SmartDashboard::PutNumber("Intake desired rotational velocity", m_profiledPIDController.GetSetpoint().velocity.value());
        frc::SmartDashboard::PutNumber("Intake rotational acceleration", m_acceleration.value());
        frc::SmartDashboard::PutNumber("Intake desired rotational acceleration", m_targetAcceleration.value());
    }

    if (frc::Preferences::GetBoolean("Tuning Mode", false)) {
        UpdatePreferences();
    }

    SetRotation(m_targetAngle);
}

void AlgaeIntake::SetState(AlgaeIntakeState state, AlgaeIntakeTarget target) {
    auto targetAngle = 0.0_deg;

    switch(state){
        case (AlgaeIntakeState::HoldCurrentPosition) :
            m_state = AlgaeIntakeState::HoldCurrentPosition;
            break;
        case (AlgaeIntakeState::GoTarget) :
            m_state = AlgaeIntakeState::GoTarget;
            targetAngle = IntakeConstants::kExtendTarget;
            break;
    }

    m_targetAngle = targetAngle;
    //update the m_target variable with the target value changed from the state
}

units::degree_t AlgaeIntake::GetTargetAngleFromTarget(AlgaeIntakeTarget target) {
    auto targetAngle = 0.0_deg;
    switch (target) {
        case (AlgaeIntakeTarget::IntakeAlgae):
            targetAngle = AlgaeIntakeConstants::kIntakeAlgaeAngle;
            break;
        case (AlgaeIntakeTarget::ScoreProcessor):
            targetAngle = AlgaeIntakeConstants::kScoreProcessorAngle;
            break;
        case (AlgaeIntakeTarget::DefaultRetract):
            targetAngle = AlgaeIntakeConstants::kDefaultRetractAngle;
            break;
        default:
            targetAngle = AlgaeIntakeConstants::kDefaultRetractAngle;
            break;
    }

    return targetAngle;
}