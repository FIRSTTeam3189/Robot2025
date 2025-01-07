#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/AnalogPotentiometer.h>
#include <frc/Timer.h>
#include <frc/DigitalInput.h>

#include <frc/Preferences.h>
#include <frc/RobotController.h>

#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include <rev/SparkMax.h>

#include "Constants/GlobalConstants.h"
#include "Constants/AlgaeIntakeConstants.h"

enum class AlgaeIntakeState { HoldCurrentPosition, GoTarget };

// IntakeAlgae extends it to the target to get the algae and spin rollers, ScoreProcessor goes to vertical position and DefaultRetract is its state when doing nothing (retracted in the robot)
enum class AlgaeIntakeTarget { IntakeAlgae, ScoreProcessor, DefaultRetract };

class AlgaeIntake : public frc2::SubsystemBase {
 public:
  AlgaeIntake();
  void SetRollerPower(double power);
  void SetRotationPower(double power);
  void SetRotation(units::degree_t target);
  units::volt_t GetMotionProfileFeedForwardValue();
  units::degree_t GetRotation();
  units::degree_t GetCurrentTargetAngle();
  units::degree_t GetTargetAngleFromTarget(AlgaeIntakeTarget target);
  void SetState(AlgaeIntakeState state);
  void SetBrakeMode(BrakeMode mode);
  void UpdatePreferences();
  void ConfigRollerMotor();
  void ConfigRotationMotor();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 protected:
  void ConfigPID();

 private:
  ctre::phoenix6::hardware::TalonFX m_rotationMotor;
  rev::spark::SparkMax m_rollerMotor;

  frc::TrapezoidProfile<units::degrees>::Constraints m_constraints;
  frc::ProfiledPIDController<units::degrees> m_profiledPIDController;
  frc::ArmFeedforward *m_ff;
  units::degree_t m_targetAngle;
  AlgaeIntakeState m_state;
  units::degrees_per_second_t m_lastSpeed;
  units::degrees_per_second_t m_lastTargetSpeed;
  units::degrees_per_second_squared_t m_acceleration;
  units::degrees_per_second_squared_t m_targetAcceleration;
  units::second_t m_lastTime;
  
  // String keys for PID preferences
  std::string m_rotationPKey;
  std::string m_rotationIKey;
  std::string m_rotationDKey;
  std::string m_rotationGKey;
  std::string m_rotationSKey;
  std::string m_rotationVKey;
  std::string m_rotationAKey;
  std::string m_rotationTargetKey;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};


