// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem.h"

#include <frc/MathUtil.h>

#include <chrono>
#include <thread>

#include "Constants.h"

IntakeSubsystem::IntakeSubsystem()
    : m_intake(IntakeConstants::kIntakeMotorID,
               rev::CANSparkFlex::MotorType::kBrushless),
      m_pivot(IntakeConstants::kPivotMotorID,
              rev::CANSparkMax::MotorType::kBrushless),
      m_pivotEncoder(m_pivot.GetAbsoluteEncoder(
          rev::SparkAbsoluteEncoder::Type::kDutyCycle)),
      m_pivotController(m_pivot.GetPIDController()),
      m_pivotActual(PivotState::kSwitching),
      m_pivotTarget(PivotState::kStow),
      m_intakeTarget(IntakeState::kStopped) {
  m_pivot.RestoreFactoryDefaults();
  m_intake.RestoreFactoryDefaults();

  m_pivot.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_intake.SetIdleMode(rev::CANSparkFlex::IdleMode::kCoast);
  m_pivot.SetInverted(true);
  m_pivot.SetSmartCurrentLimit(30);

  m_pivotEncoder.SetPositionConversionFactor(360);

  m_pivotController.SetFeedbackDevice(m_pivotEncoder);
  m_pivotController.SetP(IntakeConstants::kP);
  m_pivotController.SetI(IntakeConstants::kI);
  m_pivotController.SetD(IntakeConstants::kD);
  m_pivotController.SetPositionPIDWrappingEnabled(false);

  // m_pivotEncoder.SetPosition(GetAngle().value());

  m_pivotController.SetOutputRange(-1, 1);

  m_pivot.BurnFlash();
  m_intake.BurnFlash();
}

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {
  if (tuning) {
    m_pivotController.SetP(kP);
    m_pivotController.SetI(kI);
    m_pivotController.SetD(kD);
  }

  CheckState();

  m_pivotController.SetReference(StateToOutput(m_pivotTarget).value() + 90.5,
                                 rev::CANSparkMax::ControlType::kPosition);

  // m_pivot.Set(target);

  m_intake.SetVoltage(units::volt_t(StateToOutput(m_intakeTarget)));
}

void IntakeSubsystem::InitSendable(wpi::SendableBuilder& builder) {
  if constexpr (!Telemetry::kIntake) {
    return;
  }

  builder.SetSmartDashboardType("Intake");

#define LAMBDA(x) [this] { return x; }

  // builder.AddStringProperty("Pivot Actual", LAMBDA(ToStr(m_pivotActual)),
  //                           nullptr);
  // builder.AddStringProperty("Pivot Target", LAMBDA(ToStr(m_pivotTarget)),
  //                           nullptr);
  // builder.AddStringProperty("Intake Target", LAMBDA(ToStr(m_intakeTarget)),
  //                           nullptr);

  builder.AddDoubleProperty("Pivot Angle", LAMBDA(GetAngle().value()), nullptr);

  builder.AddDoubleProperty("Angle", LAMBDA(m_pivotEncoder.GetPosition()),
                            nullptr);

  builder.AddDoubleProperty("P", LAMBDA(kP),
                            [this](double newval) { kP = newval; });
  builder.AddDoubleProperty("I", LAMBDA(kI),
                            [this](double newval) { kI = newval; });
  builder.AddDoubleProperty("D", LAMBDA(kD),
                            [this](double newval) { kD = newval; });
  builder.AddDoubleProperty("Target", LAMBDA(target),
                            [this](double newval) { target = newval; });

#undef LAMBDA
}

double IntakeSubsystem::StateToOutput(IntakeState state) const {
  using enum IntakeState;
  namespace IS = IntakeConstants::Speeds;

  switch (state) {
    case kStopped:
      return IS::kStopped;
      break;
    case kIntaking:
      return IS::kIntake;
      break;
    case kHandoff:
      return IS::kHandoff;
      break;
    default:
      return 0;
      break;
  }
}

units::degree_t IntakeSubsystem::StateToOutput(PivotState state) const {
  using enum PivotState;
  namespace IP = IntakeConstants::Positions;

  switch (state) {
    case kDeployed:
      return IP::kDeployed;
      break;
    case kHandoff:
      return IP::kHandoff;
      break;
    case kStow:
      return IP::kStow;
      break;
    default:
    case kSwitching:
      fmt::println("[INTAKE FAIL], at {}", __LINE__);
      return 0_deg;
      break;
  }
}

void IntakeSubsystem::CheckState() {
  using enum PivotState;
  namespace IP = IntakeConstants::Positions;

  auto angle = GetAngle();

  if (frc::IsNear(IP::kDeployed, angle, IP::kTollerance)) {
    m_pivotActual = kDeployed;
    return;
  }

  if (frc::IsNear(IP::kHandoff, angle, IP::kTollerance)) {
    m_pivotActual = kHandoff;
    return;
  }

  if (frc::IsNear(IP::kStow, angle, IP::kTollerance)) {
    m_pivotActual = kStow;
    return;
  }

  m_pivotActual = kSwitching;
}

std::string IntakeSubsystem::ToStr(PivotState state) const {
  using enum PivotState;
  switch (state) {
    case kDeployed:
      return "Deployed";
      break;
    case kHandoff:
      return "Handoff";
      break;
    case kStow:
      return "Stow";
      break;
    case kSwitching:
      return "Switching";
      break;
    default:
      return "Sad :(";
      break;
  }
}

std::string IntakeSubsystem::ToStr(IntakeState state) const {
  using enum IntakeState;

  switch (state) {
    case kIntaking:
      return "Intaking";
      break;
    case kStopped:
      return "Stopped";
      break;
    case kHandoff:
      return "Handoff";
      break;
    default:
      return "Tiempo por Miller";
      break;
  }
}
