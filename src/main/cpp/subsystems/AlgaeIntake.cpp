#include "subsystems/AlgaeIntake.h"

AlgaeIntake::AlgaeIntake(int CANcoderID) : 

 // TODO
 m_rotationMotor(AlgaeIntakeConstants::kRotationMotorID, rev::spark::SparkMax::MotorType::kBrushless),
 m_rollerMotor(AlgaeIntakeConstants::kRollerMotorID, rev::spark::SparkMax::MotorType::kBrushless),
 m_rollerConfig(),
 m_constraints(AlgaeIntakeConstants::kMaxRotationVelocity, AlgaeIntakeConstants::kMaxRotationAcceleration),
 m_profiledPIDController(AlgaeIntakeConstants::kPRotation, AlgaeIntakeConstants::kIRotation, AlgaeIntakeConstants::kDRotation, m_constraints),
 m_targetAngle(AlgaeIntakeConstants::kRetractTarget),
{ rev::spark::SparkMax::MotorType::kBrushless
    ConfigRotationMotor();
    ConfigRollerMotor();
    ConfigPID();

    //configure motors and PID

    // std::cout << "Intake constructed\n";
}

void AlgaeIntake::ConfigRotationMotor(int CANcoderID) {
    // Set to factory default
    m_rotationMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration({}));

    m_rotationConfig.Slot0.kP = AlgaeIntakeConstants::kPRotation;
    m_rotationConfig.Slot0.kI = AlgaeIntakeConstants::kIRotation;
    m_rotationConfig.Slot0.kD = AlgaeIntakeConstants::kDRotation;
    m_rotationConfig.Slot0.kV = AlgaeIntakeConstants::kVRotation;
    m_rotationConfig.Slot0.kS = AlgaeIntakeConstants::kSRotation;

    m_rotationConfig.ClosedLoopGeneral.ContinuousWrap = true;

    // TODO
    m_rotationConfig.Feedback.RotorToSensorRatio = AlgaeIntakeConstants::kAngleGearRatio;
    m_rotationConfig.Feedback.FeedbackRemoteSensorID = CANcoderID; // TODO: check with electrical if using a cancoder for intake/coral wrist rotation
    m_rotationConfig.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;

    m_rotationConfig.CurrentLimits.SupplyCurrentLowerLimit = AlgaeIntakeConstants::kAngleContinuousCurrentLimit;
    m_rotationConfig.CurrentLimits.SupplyCurrentLimit = AlgaeIntakeConstants::kAnglePeakCurrentLimit;
    m_rotationConfig.CurrentLimits.SupplyCurrentLowerTime = AlgaeIntakeConstants::kAnglePeakCurrentDuration;
    m_rotationConfig.CurrentLimits.SupplyCurrentLimitEnable = AlgaeIntakeConstants::kAngleEnableCurrentLimit;

    m_rotationConfig.Voltage.PeakForwardVoltage = AlgaeIntakeConstants::kMaxVoltage;
    m_rotationConfig.Voltage.PeakReverseVoltage = AlgaeIntakeConstants::kMaxVoltage;

    m_rotationConfig.MotorOutput.Inverted = AlgaeIntakeConstants::kAngleMotorInverted;
    m_rotationConfig.MotorOutput.NeutralMode = AlgaeIntakeConstants::kAngleNeutralMode;

    m_rotationMotor.GetConfigurator().Apply(m_rotationConfig);
}


void AlgaeIntake::ConfigRollerMotor() {
    m_rollerConfig
        .Inverted(AlgaeIntakeConstants::kRollerInverted)
        .SmartCurrentLimit(AlgaeIntakeConstants::kRollerCurrentLimit);
    m_rollerMotor.Configure(m_rollerConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
}

void AlgaeIntake::ConfigPID() {
    m_ff = new frc::ArmFeedforward(
        AlgaeIntakeConstants::kSRotation,
        AlgaeIntakeConstants::kGRotation,
        AlgaeIntakeConstants::kVRotation,
        AlgaeIntakeConstants::kARotation
    );

    //takes speed, gravity, velocity and acceleration to construct feed forward object

    m_rotationPKey = "Algae Intake Rotation P";
    m_rotationIKey = "Algae Intake Rotation I";
    m_rotationDKey = "Algae IntakeSetState Rotation D";
    m_rotationGKey = "Algae Intake Rotation G";
    m_rotationSKey = "Algae Intake Rotation S";
    m_rotationVKey = "Algae Intake Rotation V";
    m_rotationAKey = "Algae Intake Rotation A";
    m_rotationTargetKey = "Algae Intake Rotation Target";
    //keys for PID and FF 

    frc::Preferences::SetDouble(m_rotationPKey, AlgaeIntakeConstants::kPRotation);
    frc::Preferences::SetDouble(m_rotationIKey, AlgaeIntakeConstants::kIRotation);
    frc::Preferences::SetDouble(m_rotationDKey, AlgaeIntakeConstants::kDRotation);
    frc::Preferences::SetDouble(m_rotationGKey, AlgaeIntakeConstants::kGRotation.value());
    frc::Preferences::SetDouble(m_rotationSKey, AlgaeIntakeConstants::kSRotation.value());
    frc::Preferences::SetDouble(m_rotationVKey, AlgaeIntakeConstants::kVRotation.value());
    frc::Preferences::SetDouble(m_rotationAKey, AlgaeIntakeConstants::kARotation.value());
    frc::Preferences::SetDouble(m_rotationTargetKey, m_targetAngle.value());
}

//initializing PID and SGVA values

units::volt_t AlgaeIntake::GetMotionProfileFeedForwardValue() {
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

void AlgaeIntake::SetRotation(units::degree_t target) {
    // Calculates PID value in volts based on position and target
    units::volt_t PIDValue = units::volt_t{(target - GetRotation()).value() * m_profiledPIDController.GetP()};  
    units::volt_t ffValue = 0.0_V;

    // Only use feedforward/motion profile if actively trying to move
    if (m_state = AlgaeIntakeState::GoTarget) {
        ffValue = GetMotionProfileFeedForwardValue();
    }

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
void AlgaeIntake::Periodic() {
    frc::SmartDashboard::PutNumber("Algae Intake PID target", m_targetAngle.value());
    frc::SmartDashboard::PutNumber("Algae Intake rotation", GetRotation().value());
    frc::SmartDashboard::PutNumber("Algae Intake power", m_rollerMotor.Get());

    if (frc::Preferences::GetBoolean("Full Diagnostics", false)) {
        frc::SmartDashboard::PutNumber("Algae Intake desired rotational velocity", m_profiledPIDController.GetSetpoint().velocity.value());
        frc::SmartDashboard::PutNumber("Algae Intake rotational acceleration", m_acceleration.value());
        frc::SmartDashboard::PutNumber("Algae Intake desired rotational acceleration", m_targetAcceleration.value());
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