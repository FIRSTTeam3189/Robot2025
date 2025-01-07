#include "subsystems/AlgaeIntake.h"

AlgaeIntake::AlgaeIntake() : 
 m_rotationMotor(AlgaeIntakeConstants::kRotationMotorID, rev::spark::SparkMax::MotorType::kBrushless),
 m_rollerMotor(AlgaeIntakeConstants::kRollerMotorID, rev::spark::SparkMax::MotorType::kBrushless),
 m_rotationConfig(),
 m_rollerConfig(),
 m_constraints(AlgaeIntakeConstants::kMaxRotationVelocity, AlgaeIntakeConstants::kMaxRotationAcceleration),
 m_profiledPIDController(AlgaeIntakeConstants::kPRotation, AlgaeIntakeConstants::kIRotation, AlgaeIntakeConstants::kDRotation, m_constraints),
 m_target(AlgaeIntakeConstants::kRetractTarget),
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
    frc::Preferences::SetDouble(m_rotationTargetKey, m_target.value());
}

//initializing PID and SGVA values

// This method will be called once per scheduler run
void Intake::Periodic() {
    frc::SmartDashboard::PutNumber("Intake PID target", m_target.value());
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

    if (m_isActive) {
        SetRotation(m_target);
    } else {
        HoldPosition();
    }
}