// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem.h"

#include "Constants.h"
#include "utils/Util.h"

IntakeSubsystem::IntakeSubsystem()
    : m_pivotMotor(IntakeConstants::kPivotMotorID,
                   rev::CANSparkMax::MotorType::kBrushless),
      m_pivotController(m_pivotMotor.GetPIDController()),
      m_pivotEncoder(m_pivotMotor.GetAbsoluteEncoder(
          rev::SparkAbsoluteEncoder::Type::kDutyCycle)),
      m_intakeMotor(IntakeConstants::kIntakeMotorID,
                    rev::CANSparkMax::MotorType::kBrushless),
      m_noteSwitch(IntakeConstants::kLimitID),
      m_motion(IntakeSubsystem::MotionState::kAutomatic),
      m_pivotCurrent(PivotState::kMoving),
      m_pivotTarget(PivotState::kStow),
      m_intakeCurrent(IntakeState::kStopped),
      m_intakeTarget(IntakeState::kStopped) {
  m_pivotMotor.RestoreFactoryDefaults();
  m_intakeMotor.RestoreFactoryDefaults();

  m_pivotMotor.SetSmartCurrentLimit(IntakeConstants::kPivotLimit.value());
  m_intakeMotor.SetSmartCurrentLimit(IntakeConstants::kIntakeLimit.value());

  m_pivotMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_intakeMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  m_pivotEncoder.SetPositionConversionFactor(
      IntakeConstants::kPositionConversion);
  m_pivotEncoder.SetZeroOffset(IntakeConstants::kZeroOffset);

  m_pivotController.SetFeedbackDevice(m_pivotEncoder);

  m_pivotController.SetP(IntakeConstants::kP);
  m_pivotController.SetI(IntakeConstants::kI);
  m_pivotController.SetD(IntakeConstants::kD);
  m_pivotController.SetOutputRange(-1, 1);

  m_pivotMotor.BurnFlash();
  m_intakeMotor.BurnFlash();
}

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {
  UpdatePivotState();
  UpdateIntakeState();
  if (m_motion != MotionState::kDisabled) {
    ControlLoop();
  } else {
    m_intakeMotor.Set(0.0);
    m_pivotMotor.Set(0.0);
  }
}

void IntakeSubsystem::InitSendable(wpi::SendableBuilder& builder) {
  builder.SetSmartDashboardType("Intake Subsystem");
}

void IntakeSubsystem::ControlLoop() {
  m_intakeMotor.Set(ToOutput(m_intakeTarget));
  m_pivotController.SetReference(ToOutput(m_pivotTarget),
                                 rev::CANSparkMax::ControlType::kPosition);
}

void IntakeSubsystem::UpdatePivotState() {
  double position = GetPosition().value();

  // Check ground
  if (hb::InRange(position, IntakeConstants::Positions::kGround.value(),
                  IntakeConstants::Positions::kTollerance.value())) {
    m_pivotCurrent = PivotState::kGround;
    return;
  }

  // Check stow
  if (hb::InRange(position, IntakeConstants::Positions::kStow.value(),
                  IntakeConstants::Positions::kTollerance.value())) {
    m_pivotCurrent = PivotState::kStow;
    return;
  }

  m_pivotCurrent = PivotState::kMoving;
}

void IntakeSubsystem::UpdateIntakeState() {
  double speed = m_intakeMotor.Get();

  if (speed == 0) {
    m_intakeCurrent = IntakeState::kStopped;
    return;
  }

  if (speed == IntakeConstants::Speeds::kEject) {
    m_intakeCurrent = IntakeState::kEject;
    return;
  }

  if (speed == IntakeConstants::Speeds::kFeedShooter) {
    m_intakeCurrent = IntakeState::kFeed;
    return;
  }

  if (speed == IntakeConstants::Speeds::kIntake) {
    m_intakeCurrent = IntakeState::kIntaking;
    return;
  }
}

double IntakeSubsystem::ToOutput(IntakeState state) const {
  switch (state) {
    case IntakeState::kEject:
      return IntakeConstants::Speeds::kEject;
      break;

    case IntakeState::kFeed:
      return IntakeConstants::Speeds::kFeedShooter;
      break;

    case IntakeState::kIntaking:
      return IntakeConstants::Speeds::kIntake;
      break;

    default:
    case IntakeState::kStopped:
      return 0.0;
      break;
  }
}

double IntakeSubsystem::ToOutput(PivotState state) const {
  switch (state) {
    default:
    case PivotState::kStow:
      return IntakeConstants::Positions::kStow.value();
      break;

    case PivotState::kGround:
      return IntakeConstants::Positions::kGround.value();
      break;
  }
}

std::string IntakeSubsystem::ToStr(MotionState state) const {
  switch (state) {
    case MotionState::kAutomatic:
      return "Automatic";
      break;

    default:
    case MotionState::kDisabled:
      return "Disabled";
      break;
  }
}

std::string IntakeSubsystem::ToStr(PivotState state) const {
  switch (state) {
    case PivotState::kGround:
      return "Ground";
      break;

    case PivotState::kStow:
      return "Stow";
      break;

    default:
    case PivotState::kMoving:
      return "Moving";
      break;
  }
}

std::string IntakeSubsystem::ToStr(IntakeState state) const {
  switch (state) {
    case IntakeState::kIntaking:
      return "Intaking";
      break;

    case IntakeState::kEject:
      return "Ejecting";
      break;

    case IntakeState::kFeed:
      return "Feeding";
      break;

    default:
    case IntakeState::kStopped:
      return "Stopped";
      break;
  }
}
