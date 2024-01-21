// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterSubsystem.h"

#include <frc/MathUtil.h>

#include "Constants.h"

ShooterSubsystem::ShooterSubsystem()
    : m_motor0(ShooterConstants::kMotor0ID,
               rev::CANSparkFlex::MotorType::kBrushless),
      m_motor1(ShooterConstants::kMotor1ID,
               rev::CANSparkFlex::MotorType::kBrushless),
      m_encoder0(
          m_motor0.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)),
      m_encoder1(
          m_motor1.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)),
      m_controller0(m_motor0.GetPIDController()),
      m_controller1(m_motor1.GetPIDController()),
      m_actual(State::kStopped),
      m_target(State::kIdle) {
  m_motor0.RestoreFactoryDefaults();
  m_motor1.RestoreFactoryDefaults();

  m_motor0.SetSmartCurrentLimit(ShooterConstants::kCurrentLimit.value());
  m_motor1.SetSmartCurrentLimit(ShooterConstants::kCurrentLimit.value());

  m_motor0.SetIdleMode(rev::CANSparkFlex::IdleMode::kCoast);
  m_motor1.SetIdleMode(rev::CANSparkFlex::IdleMode::kCoast);

  m_controller0.SetP(ShooterConstants::kP);
  m_controller0.SetI(ShooterConstants::kI);
  m_controller0.SetD(ShooterConstants::kD);
  m_controller0.SetFF(ShooterConstants::kFF);

  m_controller1.SetP(ShooterConstants::kP);
  m_controller1.SetI(ShooterConstants::kI);
  m_controller1.SetD(ShooterConstants::kD);
  m_controller1.SetFF(ShooterConstants::kFF);

  m_motor0.BurnFlash();
  m_motor1.BurnFlash();
}

// This method will be called once per scheduler run
void ShooterSubsystem::Periodic() {
  m_controller0.SetReference(ToRPM(m_target).value(),
                             rev::CANSparkFlex::ControlType::kVelocity);
  m_controller1.SetReference(ToRPM(m_target).value(),
                             rev::CANSparkFlex::ControlType::kVelocity);
}

units::revolutions_per_minute_t ShooterSubsystem::GetSpeed0() const {
  return units::revolutions_per_minute_t(m_encoder0.GetVelocity());
}

units::revolutions_per_minute_t ShooterSubsystem::GetSpeed1() const {
  return units::revolutions_per_minute_t(m_encoder0.GetVelocity());
}

ShooterSubsystem::State ShooterSubsystem::GetTargetState() const {
  return m_target;
}

ShooterSubsystem::State ShooterSubsystem::GetCurrentState() const {
  return m_actual;
}

void ShooterSubsystem::SetTargetState(State target) {
  m_target = target;
}

frc2::CommandPtr ShooterSubsystem::SetTargetStateCMD(State target) {
  return RunOnce([this, target] { SetTargetState(target); });
}

void ShooterSubsystem::InitSendable(wpi::SendableBuilder& builder) {
  builder.SetSmartDashboardType("Shooter");

#define LAMBDA(x) [this] { return x; }

  builder.AddStringProperty("Actual State", LAMBDA(ToStr(m_actual)), nullptr);
  builder.AddStringProperty("Target State", LAMBDA(ToStr(m_target)), nullptr);

#undef LAMBDA
}

void ShooterSubsystem::CheckState() {
  auto average = (GetSpeed0() + GetSpeed1()) / 2.0;

  // Check Stopped
  if (frc::IsNear(0_rpm, average, ShooterConstants::kTollerance)) {
    m_actual = State::kStopped;
    return;
  }

  // Check Idle
  if (frc::IsNear(ShooterConstants::Setpoint::kIdle, average,
                  ShooterConstants::kTollerance)) {
    m_actual = State::kIdle;
    return;
  }

  // Check Trap Amp
  if (frc::IsNear(ShooterConstants::Setpoint::kTrapAmp, average,
                  ShooterConstants::kTollerance)) {
    m_actual = State::kIdle;
    return;
  }

  // Check Speaker
  if (frc::IsNear(ShooterConstants::Setpoint::kShooting, average,
                  ShooterConstants::kTollerance)) {
    m_actual = State::kSpeaker;
    return;
  }

  m_actual = State::kSwitching;
}

std::string ShooterSubsystem::ToStr(State state) const {
  switch (state) {
    default:
    case State::kIdle:
      return "Idle";
      break;

    case State::kSpeaker:
      return "Speaker";
      break;

    case State::kStopped:
      return "Stopped";
      break;

    case State::kSwitching:
      return "Switching";
      break;

    case State::kTrapAmp:
      return "Trap or Amp";
      break;
  }
}

units::revolutions_per_minute_t ShooterSubsystem::ToRPM(State state) const {
  switch (state) {
    default:
    case State::kStopped:
    case State::kSwitching:
      return 0_rpm;
      break;

    case State::kIdle:
      return ShooterConstants::Setpoint::kIdle;
      break;

    case State::kSpeaker:
      return ShooterConstants::Setpoint::kShooting;
      break;

    case State::kTrapAmp:
      return ShooterConstants::Setpoint::kTrapAmp;
      break;
  }
}
