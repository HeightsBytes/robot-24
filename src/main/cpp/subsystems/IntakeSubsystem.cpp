// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem.h"

#include "Constants.h"

IntakeSubsystem::IntakeSubsystem()
    : m_pivot(IntakeConstants::kPivotMotorID,
              rev::CANSparkMax::MotorType::kBrushless),
      m_intake(IntakeConstants::kIntakeMotorID,
               rev::CANSparkMax::MotorType::kBrushless),
      m_pivotEncoder(m_pivot.GetAbsoluteEncoder(
          rev::SparkAbsoluteEncoder::Type::kDutyCycle)),
      m_pivotController(m_pivot.GetPIDController()),
      m_limitSwitch(IntakeConstants::kLimitID),
      m_target(PivotTarget::kNone),
      m_state(IntakeState::kNone) {
  m_pivot.RestoreFactoryDefaults();
  m_intake.RestoreFactoryDefaults();

  m_pivot.SetSmartCurrentLimit(IntakeConstants::kPivotLimit.value());
  m_intake.SetSmartCurrentLimit(IntakeConstants::kIntakeLimit.value());

  m_pivot.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_intake.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

  m_pivotEncoder.SetPositionConversionFactor(
      IntakeConstants::kPositionConversion);

  m_pivotController.SetFeedbackDevice(m_pivotEncoder);

  m_pivotController.SetP(IntakeConstants::kP);
  m_pivotController.SetI(IntakeConstants::kI);
  m_pivotController.SetD(IntakeConstants::kD);
}

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {
  m_pivotController.SetReference(TargetToSetpoint(m_target).value(),
                                 rev::CANSparkMax::ControlType::kPosition);
  m_intake.Set(StateToSetpoint(m_state));
}

bool IntakeSubsystem::HasNote() const {
  return m_limitSwitch.Get();
}

void IntakeSubsystem::SetPivotTarget(PivotTarget target) {
  m_target = target;
}

void IntakeSubsystem::SetIntakeState(IntakeState state) {
  m_state = state;
}

IntakeSubsystem::PivotTarget IntakeSubsystem::GetCurrentState() const {
  return PivotTarget::kNone;
}

frc2::CommandPtr IntakeSubsystem::SetPivotTargetCMD(PivotTarget target) {
  return this->RunOnce([this, target] { SetPivotTarget(target); });
}

frc2::CommandPtr IntakeSubsystem::SetIntakeStateCMD(IntakeState state) {
  return this->RunOnce([this, state] { SetIntakeState(state); });
}

frc2::Trigger IntakeSubsystem::HasNoteTrigger() {
  return frc2::Trigger([this] { return HasNote(); });
}

void IntakeSubsystem::InitSendable(wpi::SendableBuilder& builder) {
  builder.SetSmartDashboardType("Intake Subsystem");

#define LAMBDA(x) [this] { return x; }

  builder.AddStringProperty("State", LAMBDA(ToStr(m_state)), nullptr);
  builder.AddStringProperty("Pivot Target", LAMBDA(ToStr(m_target)), nullptr);

  builder.AddDoubleProperty("Angle", LAMBDA(m_pivotEncoder.GetPosition()),
                            nullptr);
  builder.AddDoubleProperty("Speed Percent", LAMBDA(m_intake.Get()), nullptr);

#undef LAMBDA
}

std::string IntakeSubsystem::ToStr(PivotTarget target) const {
  switch (target) {
    default:
    case PivotTarget::kNone:
      return "None";
      break;

    case PivotTarget::kGround:
      return "Ground";
      break;

    case PivotTarget::kStow:
      return "Stow";
      break;
  }
}

std::string IntakeSubsystem::ToStr(IntakeState state) const {
  switch (state) {
    default:
    case IntakeState::kNone:
      return "None";
      break;

    case IntakeState::kIntake:
      return "Intaking";
      break;

    case IntakeState::kEject:
      return "Ejecting";
      break;

    case IntakeState::kFeedShooter:
      return "Feeding Shooter";
      break;
  }
}

units::degree_t IntakeSubsystem::TargetToSetpoint(PivotTarget target) const {
  switch (target) {
    default:
    case PivotTarget::kNone:
      // no target means no motion
      return units::degree_t(m_pivotEncoder.GetPosition());
      break;

    case PivotTarget::kGround:
      return IntakeConstants::Positions::kGround;
      break;

    case PivotTarget::kStow:
      return IntakeConstants::Positions::kStow;
      break;
  }
}

double IntakeSubsystem::StateToSetpoint(IntakeState state) const {
  switch (state) {
    default:
    case IntakeState::kNone:
      return 0;
      break;

    case IntakeState::kEject:
      return IntakeConstants::Speeds::kEject;
      break;

    case IntakeState::kFeedShooter:
      return IntakeConstants::Speeds::kFeedShooter;
      break;

    case IntakeState::kIntake:
      return IntakeConstants::Speeds::kIntake;
      break;
  }
}
