// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ArmSubsystem.h"

#include <frc/DriverStation.h>
#include <frc/MathUtil.h>
#include <units/math.h>

#include <utility>

#include "Constants.h"
#include "utils/Util.h"
#include "utils/cams/Limelight.h"

ArmSubsystem::ArmSubsystem(std::function<frc::Pose2d()> poseFunction)
    : m_motor(ArmConstants::kMotorID, rev::CANSparkMax::MotorType::kBrushless),
      m_encoder(m_motor.GetAbsoluteEncoder(
          rev::SparkAbsoluteEncoder::Type::kDutyCycle)),
      m_controller(m_motor.GetPIDController()),
      m_target(State::kStow),
      m_actual(State::kSwitching),
      m_atTarget(false),
      m_targetVal(0) {
  // m_motor.RestoreFactoryDefaults();

  m_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  // m_motor.SetSmartCurrentLimit(ArmConstants::kCurrentLimit.value());

  // m_encoder.SetPositionConversionFactor(ArmConstants::kConversionFactor);
  // m_encoder.SetInverted(ArmConstants::kEncoderInverted);
  // m_encoder.SetZeroOffset(ArmConstants::kOffset.value());

  m_motor.SetInverted(true);
  m_encoder.SetPositionConversionFactor(360);

  m_controller.SetFeedbackDevice(m_encoder);
  m_controller.SetP(ArmConstants::kP);
  m_controller.SetI(ArmConstants::kI);
  m_controller.SetD(ArmConstants::kD);
  m_controller.SetIZone(1.0);

  m_controller.SetPositionPIDWrappingEnabled(true);
  m_controller.SetPositionPIDWrappingMinInput(0);
  m_controller.SetPositionPIDWrappingMaxInput(360);

  m_controller.SetOutputRange(-0.5, 0.5);

  m_motor.BurnFlash();
}

// This method will be called once per scheduler run
void ArmSubsystem::Periodic() {
  CheckState();
  ControlLoop();
}

units::degree_t ArmSubsystem::GetAngle() const {
  return units::degree_t(m_encoder.GetPosition());
}

units::degrees_per_second_t ArmSubsystem::GetVelocity() const {
  return units::degrees_per_second_t(m_encoder.GetVelocity());
}

frc2::CommandPtr ArmSubsystem::SetTargetStateCMD(State state) {
  return this->RunOnce([this, state] { SetTargetState(state); });
}

void ArmSubsystem::InitSendable(wpi::SendableBuilder& builder) {
  // if constexpr (!Telemetry::kArm) {
  //   return;
  // }

  builder.SetSmartDashboardType("Arm");

#define LAMBDA(x) [this] { return x; }

  builder.AddDoubleProperty("Angle", LAMBDA(GetAngle().value()), nullptr);
  builder.AddDoubleProperty("Velocity", LAMBDA(GetVelocity().value()), nullptr);
  builder.AddStringProperty("Actual", LAMBDA(ToStr(m_actual)), nullptr);
  builder.AddStringProperty("Target", LAMBDA(ToStr(m_target)), nullptr);
  builder.AddBooleanProperty("At Target", LAMBDA(AtTarget()), nullptr);
  builder.AddDoubleProperty("Angle Target",
                            LAMBDA(ToSetpoint(m_target).value()), nullptr);

  builder.AddDoubleProperty("Tune Target", LAMBDA(m_targetVal), [this](double set) {m_targetVal = set;});

#undef LAMBDA
}

void ArmSubsystem::ControlLoop() {
  m_controller.SetReference(ToSetpoint(m_target).value(),
                            rev::CANSparkMax::ControlType::kPosition);
}

units::degree_t ArmSubsystem::TargettingAngle() const {
  return units::degree_t(std::clamp(m_regLin.Calculate(hb::LimeLight::GetY()), 0.0, 57.5));
}

void ArmSubsystem::CheckState() {
  using namespace ArmConstants;
  using enum State;
  auto angle = GetAngle();

  // Check Handoff
  if (frc::IsNear(Setpoint::kHandoff, angle, Setpoint::kTollerance)) {
    m_actual = kHandoff;
    return;
  }

  // Check Targetting
  if (frc::IsNear(TargettingAngle(), angle, Setpoint::kTollerance)) {
    m_actual = kTargetting;
    return;
  }

  if (frc::IsNear(Setpoint::kInFrame, angle, Setpoint::kTollerance)) {
    m_actual = kInFrame;
    return;
  }

  if (frc::IsNear(Setpoint::kStow, angle, Setpoint::kTollerance)) {
    m_actual = kStow;
    return;
  }

  m_actual = kSwitching;
}

std::string ArmSubsystem::ToStr(State state) const {
  using enum State;
  switch (state) {
    case kSwitching:
      return "Switching";
      break;
    case kHandoff:
      return "Handoff";
      break;
    case kTargetting:
      return "Targetting";
      break;
    case kInFrame:
      return "In Frame";
      break;
    case kStow:
      return "Stow";
      break;
  }
  return "FAIL";
}

units::degree_t ArmSubsystem::ToSetpoint(State state) const {
  using enum State;
  switch (state) {
    case kSwitching:
      return 0_deg;
      break;
    case kHandoff:
      return ArmConstants::Setpoint::kHandoff;
      break;
    case kTargetting:
      return TargettingAngle();
      break;
    case kInFrame:
      return ArmConstants::Setpoint::kInFrame;
      break;
    case kStow:
      return ArmConstants::Setpoint::kStow;
      break;
  }
  std::printf("[WARNING] ArmSubsystem::ToSetpoint Failed!");
  return 0_deg;
}
