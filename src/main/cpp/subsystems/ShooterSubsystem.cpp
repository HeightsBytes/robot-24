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
                   rev::CANSparkMax::MotorType::kBrushed),
      m_rightFeeder(ShooterConstants::kRightFeederID,
                    rev::CANSparkMax::MotorType::kBrushed),
      m_actual0(State::kStopped),
      m_actual1(State::kStopped),
      m_target(State::kStopped) {
  m_rightFlywheel.SetInverted(true);

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

  m_leftFlywheel.SetClosedLoopRampRate(1.25);
  m_rightFlywheel.SetClosedLoopRampRate(1.25);
}

// This method will be called once per scheduler run
void ShooterSubsystem::Periodic() {
  if (m_target == State::kStopped) {
    m_leftFlywheel.Set(0);
    m_rightFlywheel.Set(0);
  } else {
    m_controller0.SetReference(ToRPM0(m_target).value(),
                               rev::CANSparkFlex::ControlType::kVelocity);
    m_controller1.SetReference(ToRPM1(m_target).value(),
                               rev::CANSparkFlex::ControlType::kVelocity);
  }
}

void ShooterSubsystem::SetFeeder(double setpoint) {
  m_leftFeeder.Set(setpoint);
  m_rightFeeder.Set(-setpoint);
}

frc2::CommandPtr ShooterSubsystem::SetTargetStateCMD(State target) {
  return RunOnce([this, target] { SetTargetState(target); });
}

void ShooterSubsystem::InitSendable(wpi::SendableBuilder& builder) {
  builder.SetSmartDashboardType("Shooter");

#define LAMBDA(x) [this] { return x; }

  builder.AddStringProperty("Actual State 0", LAMBDA(ToStr(m_actual0)),
                            nullptr);
  builder.AddStringProperty("Actual State 1", LAMBDA(ToStr(m_actual1)),
                            nullptr);
  builder.AddStringProperty("Target State", LAMBDA(ToStr(m_target)), nullptr);

  builder.AddDoubleProperty("Velocity 0", LAMBDA(GetSpeed0().value()), nullptr);
  builder.AddDoubleProperty("Velocity 1", LAMBDA(GetSpeed1().value()), nullptr);

#undef LAMBDA
}

void ShooterSubsystem::CheckState0() {
  auto average = GetSpeed0();

  // Check Stopped
  if (frc::IsNear(0_rpm, average, ShooterConstants::kTollerance)) {
    m_actual0 = State::kStopped;
    return;
  }

  // Check Idle
  if (frc::IsNear(ShooterConstants::Setpoint::kIdle, average,
                  ShooterConstants::kTollerance)) {
    m_actual0 = State::kIdle;
    return;
  }

  // Check Trap Amp
  if (frc::IsNear(ShooterConstants::Setpoint::kTrapAmp, average,
                  ShooterConstants::kTollerance)) {
    m_actual0 = State::kIdle;
    return;
  }

  // Check Speaker
  if (frc::IsNear(ShooterConstants::Setpoint::kShooting0, average,
                  ShooterConstants::kTollerance)) {
    m_actual0 = State::kSpeaker;
    return;
  }

  m_actual0 = State::kSwitching;
}

void ShooterSubsystem::CheckState1() {
  auto average = GetSpeed1();

  // Check Stopped
  if (frc::IsNear(0_rpm, average, ShooterConstants::kTollerance)) {
    m_actual1 = State::kStopped;
    return;
  }

  // Check Idle
  if (frc::IsNear(ShooterConstants::Setpoint::kIdle, average,
                  ShooterConstants::kTollerance)) {
    m_actual1 = State::kIdle;
    return;
  }

  // Check Trap Amp
  if (frc::IsNear(ShooterConstants::Setpoint::kTrapAmp, average,
                  ShooterConstants::kTollerance)) {
    m_actual1 = State::kIdle;
    return;
  }

  // Check Speaker
  if (frc::IsNear(ShooterConstants::Setpoint::kShooting1, average,
                  ShooterConstants::kTollerance)) {
    m_actual1 = State::kSpeaker;
    return;
  }

  m_actual1 = State::kSwitching;
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

units::revolutions_per_minute_t ShooterSubsystem::ToRPM0(State state) const {
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
      return ShooterConstants::Setpoint::kShooting0;
      break;

    case State::kTrapAmp:
      return ShooterConstants::Setpoint::kTrapAmp;
      break;
  }
}

units::revolutions_per_minute_t ShooterSubsystem::ToRPM1(State state) const {
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
      return ShooterConstants::Setpoint::kShooting1;
      break;

    case State::kTrapAmp:
      return ShooterConstants::Setpoint::kTrapAmp;
      break;
  }
}
