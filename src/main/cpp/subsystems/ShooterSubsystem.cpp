// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterSubsystem.h"

#include <frc/MathUtil.h>

#include "Constants.h"

ShooterSubsystem::ShooterSubsystem()
    : m_leftFlywheel(ShooterConstants::kLeftFlywheelID,
                     rev::CANSparkFlex::MotorType::kBrushless),
      m_rightFlywheel(ShooterConstants::kRightFlywheelID,
                      rev::CANSparkFlex::MotorType::kBrushless),
      m_encoder0(m_leftFlywheel.GetEncoder(
          rev::SparkRelativeEncoder::Type::kHallSensor)),
      m_encoder1(m_rightFlywheel.GetEncoder(
          rev::SparkRelativeEncoder::Type::kHallSensor)),
      m_controller0(m_leftFlywheel.GetPIDController()),
      m_controller1(m_rightFlywheel.GetPIDController()),
      m_leftFeeder(ShooterConstants::kLeftFeederID,
                   rev::CANSparkMax::MotorType::kBrushless),
      m_rightFeeder(ShooterConstants::kRightFeederID,
                    rev::CANSparkMax::MotorType::kBrushless),
      m_beamBreak(ShooterConstants::kBeamBreakPort),
      m_actual(State::kStopped),
      m_target(State::kIdle) {
  m_leftFlywheel.RestoreFactoryDefaults();
  m_rightFlywheel.RestoreFactoryDefaults();

  m_leftFlywheel.SetSmartCurrentLimit(ShooterConstants::kCurrentLimit.value());
  m_rightFlywheel.SetSmartCurrentLimit(ShooterConstants::kCurrentLimit.value());

  m_leftFlywheel.SetIdleMode(rev::CANSparkFlex::IdleMode::kCoast);
  m_rightFlywheel.SetIdleMode(rev::CANSparkFlex::IdleMode::kCoast);

  m_controller0.SetP(ShooterConstants::kP);
  m_controller0.SetI(ShooterConstants::kI);
  m_controller0.SetD(ShooterConstants::kD);
  m_controller0.SetFF(ShooterConstants::kFF);

  m_controller1.SetP(ShooterConstants::kP);
  m_controller1.SetI(ShooterConstants::kI);
  m_controller1.SetD(ShooterConstants::kD);
  m_controller1.SetFF(ShooterConstants::kFF);

  m_leftFeeder.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_rightFeeder.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  m_leftFeeder.BurnFlash();
  m_rightFeeder.BurnFlash();
  m_leftFlywheel.BurnFlash();
  m_rightFlywheel.BurnFlash();
}

// This method will be called once per scheduler run
void ShooterSubsystem::Periodic() {
  m_controller0.SetReference(ToRPM(m_target).value(),
                             rev::CANSparkFlex::ControlType::kVelocity);
  m_controller1.SetReference(ToRPM(m_target).value(),
                             rev::CANSparkFlex::ControlType::kVelocity);
}

bool ShooterSubsystem::HasNote() const {
  return m_beamBreak.Get();
}

bool ShooterSubsystem::AtRPM() const {
  return GetTargetState() == GetCurrentState();
}

bool ShooterSubsystem::ShooterReady() const {
  return HasNote() && AtRPM();
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

void ShooterSubsystem::SetFeeder(double setpoint) {
  m_leftFeeder.Set(setpoint);
  m_rightFeeder.Set(-setpoint);
}

frc2::CommandPtr ShooterSubsystem::SetTargetStateCMD(State target) {
  return RunOnce([this, target] { SetTargetState(target); });
}

frc2::Trigger ShooterSubsystem::HasNoteTrigger() {
  return frc2::Trigger([this] { return HasNote(); });
}

frc2::Trigger ShooterSubsystem::AtRPMTrigger() {
  return frc2::Trigger([this] { return AtRPM(); });
}

frc2::Trigger ShooterSubsystem::ShooterReadyTrigger() {
  return frc2::Trigger([this] { return ShooterReady(); });
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
