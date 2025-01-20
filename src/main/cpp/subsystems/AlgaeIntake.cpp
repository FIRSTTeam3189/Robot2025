#include "subsystems/AlgaeIntake.h"

AlgaeIntake::AlgaeIntake() : 
 m_rotationMotor(AlgaeIntakeConstants::kRotationMotorID),
 m_CANcoder(AlgaeIntakeConstants::kRotationCANCoderID),
 m_rollerMotor(AlgaeIntakeConstants::kRollerMotorID, rev::spark::SparkMax::MotorType::kBrushless),
 m_rotationConfig(),
 m_encoderConfig(),
 m_rollerConfig(),
 m_target(AlgaeIntakeState::DefaultRetract),
 m_targetAngle(AlgaeIntakeConstants::kDefaultRetractAngle)
{
    ConfigRotationMotor();
    ConfigRotationCANcoder();
    ConfigRollerMotor();
    ConfigPID();

    //configure motors and PID

    // std::cout << "Intake constructed\n";
}

void AlgaeIntake::ConfigRotationMotor() {
    // Set to factory default
    m_rotationMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration({}));

    m_rotationConfig.Slot0.kS = AlgaeIntakeConstants::kSRotation; 
    m_rotationConfig.Slot0.kV = AlgaeIntakeConstants::kVRotation;
    m_rotationConfig.Slot0.kG = AlgaeIntakeConstants::kGRotation;
    m_rotationConfig.Slot0.kA = AlgaeIntakeConstants::kARotation;
    m_rotationConfig.Slot0.kP = AlgaeIntakeConstants::kPRotation;
    m_rotationConfig.Slot0.kI = AlgaeIntakeConstants::kIRotation;
    m_rotationConfig.Slot0.kD = AlgaeIntakeConstants::kDRotation;
    m_rotationConfig.Slot0.GravityType = ctre::phoenix6::signals::GravityTypeValue::Arm_Cosine;

    // Motion magic configs
    m_rotationConfig.MotionMagic.MotionMagicCruiseVelocity = AlgaeIntakeConstants::kMotionMagicMaxVelocity;
    m_rotationConfig.MotionMagic.MotionMagicAcceleration = AlgaeIntakeConstants::kMotionMagicMaxAcceleration;
    m_rotationConfig.MotionMagic.MotionMagicJerk = AlgaeIntakeConstants::kMotionMagicMaxJerk;

    // m_rotationConfig.Feedback.FeedbackRemoteSensorID = AlgaeIntakeConstants::kRotationCANCoderID; // TODO: check with electrical if using a cancoder for intake/coral wrist rotation
    // m_rotationConfig.Feedback.RotorToSensorRatio = AlgaeIntakeConstants::kRotationGearRatio;
    // m_rotationConfig.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
    m_rotationConfig.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor;
    m_rotationConfig.Feedback.SensorToMechanismRatio = AlgaeIntakeConstants::kRotationGearRatio;

    m_rotationConfig.CurrentLimits.SupplyCurrentLowerLimit = AlgaeIntakeConstants::kRotationContinuousCurrentLimit;
    m_rotationConfig.CurrentLimits.SupplyCurrentLimit = AlgaeIntakeConstants::kRotationPeakCurrentLimit;
    m_rotationConfig.CurrentLimits.SupplyCurrentLowerTime = AlgaeIntakeConstants::kRotationPeakCurrentDuration;
    m_rotationConfig.CurrentLimits.SupplyCurrentLimitEnable = AlgaeIntakeConstants::kRotationEnableCurrentLimit;

    m_rotationConfig.MotorOutput.Inverted = AlgaeIntakeConstants::kRotationMotorInverted;
    m_rotationConfig.MotorOutput.NeutralMode = AlgaeIntakeConstants::kRotationNeutralMode;

    m_rotationMotor.GetConfigurator().Apply(m_rotationConfig);
}

void AlgaeIntake::ConfigRotationCANcoder() {
    m_CANcoder.GetConfigurator().Apply(ctre::phoenix6::configs::CANcoderConfiguration{});

    m_encoderConfig.MagnetSensor.MagnetOffset = AlgaeIntakeConstants::kCANcoderOffset;
    m_encoderConfig.MagnetSensor.SensorDirection = AlgaeIntakeConstants::kCANcoderInverted;
    m_encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = AlgaeIntakeConstants::kCANcoderDiscontinuityPoint; // 0-1

    m_CANcoder.GetConfigurator().Apply(m_encoderConfig);
}

void AlgaeIntake::ConfigRollerMotor() {
    m_rollerConfig
        .Inverted(AlgaeIntakeConstants::kRollerMotorInverted)
        .SmartCurrentLimit(AlgaeIntakeConstants::kRollerMotorCurrentLimit);
    m_rollerMotor.Configure(m_rollerConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
}

void AlgaeIntake::ConfigPID() {
    m_rotationPKey = "Algae Intake Rotation P";
    m_rotationIKey = "Algae Intake Rotation I";
    m_rotationDKey = "Algae Intake Rotation D";
    m_rotationGKey = "Algae Intake Rotation G";
    m_rotationSKey = "Algae Intake Rotation S";
    m_rotationVKey = "Algae Intake Rotation V";
    m_rotationAKey = "Algae Intake Rotation A";
    m_rotationTargetKey = "Algae Intake Rotation Target";
    //keys for PID and FF 

    frc::Preferences::SetDouble(m_rotationPKey, AlgaeIntakeConstants::kPRotation);
    frc::Preferences::SetDouble(m_rotationIKey, AlgaeIntakeConstants::kIRotation);
    frc::Preferences::SetDouble(m_rotationDKey, AlgaeIntakeConstants::kDRotation);
    frc::Preferences::SetDouble(m_rotationGKey, AlgaeIntakeConstants::kGRotation);
    frc::Preferences::SetDouble(m_rotationSKey, AlgaeIntakeConstants::kSRotation);
    frc::Preferences::SetDouble(m_rotationVKey, AlgaeIntakeConstants::kVRotation);
    frc::Preferences::SetDouble(m_rotationAKey, AlgaeIntakeConstants::kARotation);
    frc::Preferences::SetDouble(m_rotationTargetKey, m_targetAngle.value());
}

//initializing PID and SGVA values

// This method will be called once per scheduler run
void AlgaeIntake::Periodic() {
    frc::SmartDashboard::PutNumber("Algae Intake PID target", m_targetAngle.value());
    frc::SmartDashboard::PutNumber("Algae Intake rotation", GetRotation().value());
    frc::SmartDashboard::PutNumber("Algae Intake roller power", m_rollerMotor.Get());
    
    // if (frc::Preferences::GetBoolean("Full Diagnostics", false)) {
    //     frc::SmartDashboard::PutNumber("Algae Intake desired rotational velocity", m_rotationMotor.get);
    //     frc::SmartDashboard::PutNumber("Algae Intake rotational acceleration", m_acceleration.value());
    //     frc::SmartDashboard::PutNumber("Algae Intake desired rotational acceleration", m_targetAcceleration.value());
    // }

    if (frc::Preferences::GetBoolean("Tuning Mode", false)) {
        UpdatePreferences();
    }

    SetRotation(m_targetAngle);
}

void AlgaeIntake::UpdatePreferences() {
    m_rotationConfig.Slot0.kS = frc::Preferences::GetDouble(m_rotationSKey, AlgaeIntakeConstants::kSRotation);
    m_rotationConfig.Slot0.kV = frc::Preferences::GetDouble(m_rotationVKey, AlgaeIntakeConstants::kVRotation);
    m_rotationConfig.Slot0.kG = frc::Preferences::GetDouble(m_rotationGKey, AlgaeIntakeConstants::kGRotation);
    m_rotationConfig.Slot0.kA = frc::Preferences::GetDouble(m_rotationAKey, AlgaeIntakeConstants::kARotation);
    m_rotationConfig.Slot0.kP = frc::Preferences::GetDouble(m_rotationPKey, AlgaeIntakeConstants::kPRotation);
    m_rotationConfig.Slot0.kI = frc::Preferences::GetDouble(m_rotationIKey, AlgaeIntakeConstants::kIRotation);
    m_rotationConfig.Slot0.kD = frc::Preferences::GetDouble(m_rotationDKey, AlgaeIntakeConstants::kDRotation);
    m_targetAngle = units::degree_t{frc::Preferences::GetDouble(m_rotationTargetKey, m_targetAngle.value())};

    m_rotationMotor.GetConfigurator().Apply(m_rotationConfig);
}

void AlgaeIntake::SetState(AlgaeIntakeState state) {
    // Figure out target angle then set rotation to this angle
    auto targetAngle = 0.0_deg;
    switch (state) {
        case (AlgaeIntakeState::IntakeAlgae):
            targetAngle = AlgaeIntakeConstants::kIntakeAlgaeAngle;
            break;
        case (AlgaeIntakeState::ScoreProcessor):
            targetAngle = AlgaeIntakeConstants::kScoreProcessorAngle;
            break;
        case (AlgaeIntakeState::DefaultRetract):
            targetAngle = AlgaeIntakeConstants::kDefaultRetractAngle;
            break;
        default:
            targetAngle = AlgaeIntakeConstants::kDefaultRetractAngle;
            break;
    }

    SetRotation(targetAngle);
}

void AlgaeIntake::SetRotation(units::degree_t target) {
    // create a Motion Magic request, voltage output
    ctre::phoenix6::controls::MotionMagicVoltage request{0_tr};

    // set target position to target degrees
    m_rotationMotor.SetControl(request.WithEnableFOC(true).WithPosition(target));

    frc::SmartDashboard::PutNumber("ALgae Intake power", m_rotationMotor.Get());
}

units::degree_t AlgaeIntake::GetCurrentTargetAngle() {
    return m_targetAngle;
}

units::degree_t AlgaeIntake::GetRotation() {
    return ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(m_rotationAngle, m_rotationVelocity);
}

void AlgaeIntake::SetRollerPower(double power) {
    m_rollerMotor.Set(power);
}

void AlgaeIntake::SetRotationBrakeMode(BrakeMode mode) {
    switch (mode) {
        case(BrakeMode::Brake) :
            m_rotationConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
            break;
        case(BrakeMode::Coast) :
            m_rotationConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
            break;
        case(BrakeMode::Default) :
            m_rotationConfig.MotorOutput.NeutralMode = AlgaeIntakeConstants::kRotationNeutralMode;
            break;
    }

    m_rotationMotor.GetConfigurator().Apply(m_rotationConfig);
}