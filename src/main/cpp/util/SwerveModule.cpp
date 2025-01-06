#include "util/SwerveModule.h"

SwerveModule::SwerveModule(int moduleNumber, int driveMotorID, int angleMotorID, 
                           int CANcoderID, units::turn_t CANcoderOffset):

m_driveMotor(driveMotorID, "Swerve"),
m_angleMotor(angleMotorID, "Swerve"),
m_CANcoder(CANcoderID, "Swerve"),
m_PIDValues{SwerveModuleConstants::kPDrive, SwerveModuleConstants::kIDrive, SwerveModuleConstants::kDDrive,
            SwerveModuleConstants::kPAngle, SwerveModuleConstants::kIAngle, SwerveModuleConstants::kDAngle},
m_moduleNumber(moduleNumber),
m_CANcoderOffset(CANcoderOffset)

{
    ConfigDriveMotor();
    ConfigAngleMotor(CANcoderID);
    ConfigCANcoder();

    m_signals.emplace_back(&m_drivePosition);
    m_signals.emplace_back(&m_anglePosition);
    m_signals.emplace_back(&m_driveVelocity);
    m_signals.emplace_back(&m_angleVelocity);

}

void SwerveModule::ConfigDriveMotor() {
    // Set to factory default
    m_driveMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration{});

    m_driveConfigs.Slot0.kP = m_PIDValues.driveP;
    m_driveConfigs.Slot0.kI = m_PIDValues.driveI;
    m_driveConfigs.Slot0.kD = m_PIDValues.driveD;
    m_driveConfigs.Slot0.kV = SwerveModuleConstants::kVDrive;
    m_driveConfigs.Slot0.kS = SwerveModuleConstants::kSDrive;

    m_driveConfigs.CurrentLimits.SupplyCurrentLowerLimit = SwerveModuleConstants::kDriveContinuousCurrentLimit;
    m_driveConfigs.CurrentLimits.SupplyCurrentLimit = SwerveModuleConstants::kDrivePeakCurrentLimit;
    m_driveConfigs.CurrentLimits.SupplyCurrentLowerTime = SwerveModuleConstants::kDrivePeakCurrentDuration;
    m_driveConfigs.CurrentLimits.SupplyCurrentLimitEnable = SwerveModuleConstants::kDriveEnableCurrentLimit;

    m_driveConfigs.MotorOutput.Inverted = SwerveModuleConstants::kDriveMotorInverted;

    m_driveConfigs.MotorOutput.NeutralMode = SwerveModuleConstants::kDriveNeutralMode;
    m_driveConfigs.MotorOutput.DutyCycleNeutralDeadband = SwerveModuleConstants::kDriveNeutralDeadband;
        
    m_driveConfigs.Feedback.SensorToMechanismRatio = SwerveModuleConstants::kDriveGearRatio;

    m_driveMotor.GetConfigurator().Apply(m_driveConfigs);

    m_driveMotor.SetPosition(0.0_rad);
}

void SwerveModule::UpdatePreferences() {

    // Will need for PID tuning. Will most likely not add FF or Integral
    m_driveConfigs.Slot0.kP = frc::Preferences::GetDouble("DriveP", SwerveModuleConstants::kPDrive);
    m_driveConfigs.Slot0.kI = frc::Preferences::GetDouble("DriveI", SwerveModuleConstants::kIDrive);
    m_driveConfigs.Slot0.kD = frc::Preferences::GetDouble("DriveD", SwerveModuleConstants::kDDrive);
    m_angleConfigs.Slot0.kP = frc::Preferences::GetDouble("AngleP", SwerveModuleConstants::kPAngle);
    m_angleConfigs.Slot0.kI = frc::Preferences::GetDouble("AngleI", SwerveModuleConstants::kIAngle);
    m_angleConfigs.Slot0.kD = frc::Preferences::GetDouble("AngleD", SwerveModuleConstants::kDAngle);

    m_driveMotor.GetConfigurator().Apply(m_driveConfigs);
    m_angleMotor.GetConfigurator().Apply(m_angleConfigs);
    // m_CANcoder.GetConfigurator().Apply(m_encoderConfigs);
}

void SwerveModule::ConfigAngleMotor(int CANcoderID) {
    // Set to factory default
    m_angleMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration({}));

    m_angleConfigs.Slot0.kP = m_PIDValues.angleP;
    m_angleConfigs.Slot0.kI = m_PIDValues.angleI;
    m_angleConfigs.Slot0.kD = m_PIDValues.angleD;

    m_angleConfigs.ClosedLoopGeneral.ContinuousWrap = true;

    // m_angleConfigs.Feedback.SensorToMechanismRatio = SwerveModuleConstants::kAngleGearRatio;
    // TODO: Not sure if this number is correct/if it actually works this way on our modules
    m_angleConfigs.Feedback.RotorToSensorRatio = SwerveModuleConstants::kAngleGearRatio;
    m_angleConfigs.Feedback.FeedbackRemoteSensorID = CANcoderID;
    m_angleConfigs.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
    // m_angleConfigs.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;

    m_angleConfigs.CurrentLimits.SupplyCurrentLowerLimit = SwerveModuleConstants::kAngleContinuousCurrentLimit;
    m_angleConfigs.CurrentLimits.SupplyCurrentLimit = SwerveModuleConstants::kAnglePeakCurrentLimit;
    m_angleConfigs.CurrentLimits.SupplyCurrentLowerTime = SwerveModuleConstants::kAnglePeakCurrentDuration;
    m_angleConfigs.CurrentLimits.SupplyCurrentLimitEnable = SwerveModuleConstants::kAngleEnableCurrentLimit;

    m_angleConfigs.Voltage.PeakForwardVoltage = SwerveModuleConstants::kMaxVoltage;
    m_angleConfigs.Voltage.PeakReverseVoltage = -SwerveModuleConstants::kMaxVoltage;

    m_angleConfigs.MotorOutput.Inverted = SwerveModuleConstants::kAngleMotorInverted;
    m_angleConfigs.MotorOutput.NeutralMode = SwerveModuleConstants::kAngleNeutralMode;

    m_angleMotor.GetConfigurator().Apply(m_angleConfigs);
}

void SwerveModule::ConfigCANcoder() {
    m_CANcoder.GetConfigurator().Apply(ctre::phoenix6::configs::CANcoderConfiguration{});

    m_encoderConfigs.MagnetSensor.MagnetOffset = m_CANcoderOffset;
    m_encoderConfigs.MagnetSensor.SensorDirection = SwerveModuleConstants::kCANcoderInverted;
    // m_encoderConfigs.MagnetSensor.AbsoluteSensorRange = SwerveModuleConstants::kCANcoderSensorRange;
    m_encoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = SwerveModuleConstants::kCANcoderDiscontinuityPoint; // +- half

    m_CANcoder.GetConfigurator().Apply(m_encoderConfigs);
}

void SwerveModule::SetBrakeMode(BrakeMode mode) {
    switch (mode) {
        case(BrakeMode::Brake) :
            m_driveConfigs.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
            m_angleConfigs.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
            break;
        case(BrakeMode::Coast) :
            m_driveConfigs.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
            m_angleConfigs.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
            break;
        case(BrakeMode::Default) :
            m_driveConfigs.MotorOutput.NeutralMode = SwerveModuleConstants::kDriveNeutralMode;
            m_angleConfigs.MotorOutput.NeutralMode = SwerveModuleConstants::kAngleNeutralMode;
            break;
        default :
            break;
    }

    m_driveMotor.GetConfigurator().Apply(m_driveConfigs);
    m_angleMotor.GetConfigurator().Apply(m_angleConfigs);
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState &state) {
    // In built function to optimize angle turning (taking shortest path)
    const auto optimizedState = frc::SwerveModuleState::Optimize(state, m_position.angle);
    double targetSpeed = optimizedState.speed.value() * SwerveModuleConstants::kRotationsPerMeter;
    auto targetAngle = optimizedState.angle.Degrees();

    std::string speedKey = std::to_string(m_moduleNumber) + " target speed";
    std::string angleKey = std::to_string(m_moduleNumber) + " target angle";
    frc::SmartDashboard::PutNumber(speedKey, targetSpeed);
    frc::SmartDashboard::PutNumber(angleKey, targetAngle.value());

    frc::SmartDashboard::PutNumber("Target Speed ", targetSpeed);

    // Output target speed to see if multiplier is necessary
    m_driveMotor.SetControl(m_driveSetter.WithEnableFOC(true).WithVelocity(units::turns_per_second_t{targetSpeed})); // TODO *1.311
    if (fabs(targetSpeed) < .05 && fabs(m_lastAngle - targetAngle.value()) < 5.0) {
        m_driveMotor.SetControl(m_driveSetter.WithEnableFOC(true).WithVelocity(units::turns_per_second_t{0.0}));
        targetAngle = units::degree_t{m_lastAngle};
    } else {
        m_angleMotor.SetControl(m_angleSetter.WithEnableFOC(true).WithPosition(targetAngle));
    }
    
    m_lastAngle = targetAngle.value();
}

std::vector<ctre::phoenix6::BaseStatusSignal*> SwerveModule::GetSignals() {
    return m_signals;
}

void SwerveModule::Stop() {
    m_driveMotor.SetControl(m_driveSetter.WithVelocity(units::turns_per_second_t{0.0}));
    m_angleMotor.StopMotor();
}

void SwerveModule::UpdatePosition() {
    // Should already refresh, otherwise need to explicitly refresh as to get the compenstated value, they must be refreshed
    auto driveRotations = ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(m_drivePosition, m_driveVelocity);
    auto angleRotations = ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(m_anglePosition, m_angleVelocity);

    double distance = driveRotations.value() / SwerveModuleConstants::kRotationsPerMeter;
    m_position.distance = units::meter_t{distance};
    
    frc::Rotation2d angle{units::degree_t{angleRotations}};
    m_position.angle = angle;
}

frc::SwerveModulePosition SwerveModule::GetPosition(bool refresh) {
    if (refresh)
        UpdatePosition();

    return m_position;
}

frc::SwerveModuleState SwerveModule::GetState(bool refresh) {
    if (refresh)
        UpdatePosition();

    // Uses initializer list syntax and lets compiler make swerve module state since we can't construct directly
    // Note the 360 converting rotations to degrees, and the turns per second to meters per second
    return {units::meters_per_second_t{m_driveVelocity.GetValue().value() / SwerveModuleConstants::kRotationsPerMeter}, 
            frc::Rotation2d(units::degree_t{360 * m_anglePosition.GetValue().value()})};
}

units::meters_per_second_t SwerveModule::GetDriveSpeed() {
    return units::meters_per_second_t{m_driveMotor.GetVelocity().Refresh().GetValue().value() / SwerveModuleConstants::kRotationsPerMeter};
}
