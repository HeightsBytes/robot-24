// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/TrapSubsystem.h"

#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"

TrapSubsystem::TrapSubsystem() :
  m_motor(TrapConstants::kTrapMotorID, rev::CANSparkMax::MotorType::kBrushless),
  m_encoder(m_motor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42)),
  m_controller(m_motor.GetPIDController()),
  m_actual(State::kStow),
  m_target(State::kStow) {
    m_motor.RestoreFactoryDefaults();

    m_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    m_motor.SetSmartCurrentLimit(TrapConstants::kCurrentLimit.value());

    m_encoder.SetPosition(0);

    m_controller.SetP(TrapConstants::kP);
    m_controller.SetI(TrapConstants::kI);
    m_controller.SetD(TrapConstants::kD);
    m_controller.SetOutputRange(-0.75, 0.75);
    
    m_motor.BurnFlash();

  }

// This method will be called once per scheduler run
void TrapSubsystem::Periodic() {
  CheckState();
  ControlLoop();
  frc::SmartDashboard::PutNumber("Trapper Position", GetPosition());
}

void TrapSubsystem::CheckState() {
  auto position = GetPosition();

  namespace TP = TrapConstants::Positions;

  // Check Stow
  if (frc::IsNear(TP::kStow, position, TrapConstants::kTollerance)) {
    m_actual = State::kStow;
    return;
  }

  // Check Deployed
  if (frc::IsNear(TP::kDeployed, position, TrapConstants::kTollerance)) {
    m_actual = State::kDeployed;
    return;
  }

  m_actual = State::kSwitching;
  
}

void TrapSubsystem::ControlLoop() {
  if (m_actual != m_target) {
    m_controller.SetReference(ToOutput(m_target), rev::CANSparkMax::ControlType::kPosition);
    return;
  }
  m_motor.Set(0);
}

std::string TrapSubsystem::ToStr(State state) const {
  switch(state) {
    case State::kDeployed:
      return "Deployed";
      break;
    case State::kStow:
      return "Stowed";
      break;
    case State::kSwitching:
      return "Switching";
      break;
  }
  return "";
}

double TrapSubsystem::ToOutput(State state) const {
  switch(state) {
    case State::kStow:
    case State::kSwitching:
      return TrapConstants::Positions::kStow;
      break;
    case State::kDeployed:
      return TrapConstants::Positions::kDeployed;
      break;
  }
  return 0;
}

void TrapSubsystem::InitSendable(wpi::SendableBuilder& builder) {
  builder.SetSmartDashboardType("Trap subsystem");

  #define LAMBDA(x) [this] {return x;}

  builder.AddDoubleProperty("Position", LAMBDA(GetPosition()), nullptr);

  builder.AddStringProperty("Target State", LAMBDA(ToStr(m_target)), nullptr);
  builder.AddStringProperty("Actual State", LAMBDA(ToStr(m_actual)), nullptr);

  #undef LAMBDA

}