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

ArmSubsystem::ArmSubsystem(/**std::function<frc::Pose2d()> poseFunction**/)
    : m_motor(ArmConstants::kMotorID, rev::CANSparkMax::MotorType::kBrushless),
      m_encoder(m_motor.GetAbsoluteEncoder(
          rev::SparkAbsoluteEncoder::Type::kDutyCycle)),
      m_controller(m_motor.GetPIDController()),
      m_target(ArmConstants::Setpoint::kStow),
      m_atTarget(false) {
  // m_motor.RestoreFactoryDefaults();

  m_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  // m_motor.SetSmartCurrentLimit(ArmConstants::kCurrentLimit.value());

  // m_encoder.SetPositionConversionFactor(ArmConstants::kConversionFactor);
  // m_encoder.SetInverted(ArmConstants::kEncoderInverted);
  // m_encoder.SetZeroOffset(ArmConstants::kOffset.value());

  m_encoder.SetPositionConversionFactor(360);
  m_encoder.SetVelocityConversionFactor((1.0 / 60.0) * 360);
  m_encoder.SetInverted(true);

  m_controller.SetFeedbackDevice(m_encoder);
  m_controller.SetP(ArmConstants::kP);
  m_controller.SetI(ArmConstants::kI);
  m_controller.SetD(ArmConstants::kD);
  m_controller.SetIZone(1.0);

  m_controller.SetPositionPIDWrappingEnabled(true);
  m_controller.SetPositionPIDWrappingMaxInput(0);
  m_controller.SetPositionPIDWrappingMaxInput(360);

  m_controller.SetOutputRange(-1, 1);

  // m_motor.BurnFlash();
}

// This method will be called once per scheduler run
void ArmSubsystem::Periodic() {
  CheckState();
  ControlLoop();
  if (m_tuning) {
    m_controller.SetP(kP);
    m_controller.SetI(kI);
    m_controller.SetD(kD);
  } else {
    kP = m_controller.GetP();
    kI = m_controller.GetI();
    kD = m_controller.GetD();
  }
}

units::degree_t ArmSubsystem::GetAngle() const {
  return units::degree_t(m_encoder.GetPosition());
}

units::degrees_per_second_t ArmSubsystem::GetVelocity() const {
  return units::degrees_per_second_t(m_encoder.GetVelocity());
}

bool ArmSubsystem::AtTarget() const {
  return m_atTarget;
}

bool ArmSubsystem::IsAt(units::degree_t val) const {
  return hb::InRange(GetAngle().value(), val.value(),
                     ArmConstants::Setpoint::kTollerance.value());
}

void ArmSubsystem::SetTarget(units::degree_t target) {
  m_target = target;
}

frc2::Trigger ArmSubsystem::AtTargetTrigger() {
  return frc2::Trigger([this] { return AtTarget(); });
}

frc2::CommandPtr ArmSubsystem::SetTargetCMD(units::degree_t target) {
  return this->RunOnce([this, target] { SetTarget(target); });
}

void ArmSubsystem::InitSendable(wpi::SendableBuilder& builder) {
  builder.SetSmartDashboardType("Arm");

#define LAMBDA(x) [this] { return x; }

  builder.AddDoubleProperty("Angle", LAMBDA(GetAngle().value()), nullptr);
  builder.AddDoubleProperty("Velocity", LAMBDA(GetVelocity().value()), nullptr);
  builder.AddDoubleProperty("Setpoint", LAMBDA(m_target.value()), nullptr);

  builder.AddBooleanProperty("At Target", LAMBDA(AtTarget()), nullptr);

  builder.AddDoubleProperty("Tuning Setpoint", LAMBDA(Setpoint),
                            [this](double newval) { Setpoint = newval; });
  builder.AddDoubleProperty("kP", LAMBDA(kP),
                            [this](double value) { kP = value; });
  builder.AddDoubleProperty("kI", LAMBDA(kI),
                            [this](double value) { kI = value; });
  builder.AddDoubleProperty("kD", LAMBDA(kD),
                            [this](double value) { kD = value; });
  builder.AddBooleanProperty("Tuning", LAMBDA(m_tuning),
                             [this](bool tune) { m_tuning = tune; });

#undef LAMBDA
}

void ArmSubsystem::ControlLoop() {
  units::degree_t target = !m_tuning ? m_target : units::degree_t(Setpoint);

  m_controller.SetReference(target.value(),
                            rev::CANSparkMax::ControlType::kPosition);
}

void ArmSubsystem::CheckState() {
  m_atTarget = hb::InRange(m_target.value(), GetAngle().value(),
                           ArmConstants::Setpoint::kTollerance.value());
}
