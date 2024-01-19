// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ClimbSubsystem.h"

#include <frc/MathUtil.h>
#include <frc2/command/Commands.h>

#include "Constants.h"

ClimbSubsystem::ClimbSubsystem()
    : m_motorLeft(ClimbConstants::kMotorLeftID,
                  rev::CANSparkMax::MotorType::kBrushless),
      m_motorRight(ClimbConstants::kMotorRightID,
                   rev::CANSparkMax::MotorType::kBrushless),
      m_encoderLeft(
          m_motorLeft.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)),
      m_encoderRight(m_motorRight.GetEncoder(
          rev::SparkRelativeEncoder::Type::kHallSensor)),
      m_controllerLeft(m_motorLeft.GetPIDController()),
      m_controllerRight(m_motorRight.GetPIDController()),
      m_behavior(Behavior::kSync),
      m_syncTarget(ClimbConstants::Positions::kStow),
      m_leftTarget(ClimbConstants::Positions::kStow),
      m_rightTarget(ClimbConstants::Positions::kStow),
      m_zeroed(false),
      m_manual(false) {
  m_motorLeft.RestoreFactoryDefaults();
  m_motorRight.RestoreFactoryDefaults();

  m_encoderLeft.SetPositionConversionFactor(
      ClimbConstants::kPositionConversion);
  m_encoderRight.SetPositionConversionFactor(
      ClimbConstants::kPositionConversion);

  m_controllerLeft.SetP(ClimbConstants::kP);
  m_controllerLeft.SetI(ClimbConstants::kI);
  m_controllerLeft.SetD(ClimbConstants::kD);
  m_controllerLeft.SetFF(ClimbConstants::kFF);

  m_controllerRight.SetP(ClimbConstants::kP);
  m_controllerRight.SetI(ClimbConstants::kI);
  m_controllerRight.SetD(ClimbConstants::kD);
  m_controllerRight.SetFF(ClimbConstants::kFF);

  m_controllerLeft.SetOutputRange(ClimbConstants::kMinPower,
                                  ClimbConstants::kMaxPower);
  m_controllerRight.SetOutputRange(ClimbConstants::kMinPower,
                                   ClimbConstants::kMaxPower);

  m_motorLeft.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_motorRight.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  m_motorLeft.BurnFlash();
  m_motorRight.BurnFlash();
}

// This method will be called once per scheduler run
void ClimbSubsystem::Periodic() {
  ArmControl();
}

units::meter_t ClimbSubsystem::GetLeftHeight() const {
  return units::meter_t(m_encoderLeft.GetPosition());
}

units::meter_t ClimbSubsystem::GetRightHeight() const {
  return units::meter_t(m_encoderRight.GetPosition());
}

void ClimbSubsystem::SetSyncBehavior(ClimbSubsystem::Behavior behavior) {
  m_behavior = behavior;
}

void ClimbSubsystem::SetSyncTarget(units::meter_t target) {
  m_syncTarget = target;
  m_behavior = ClimbSubsystem::Behavior::kSync;
}

void ClimbSubsystem::SetLeftTarget(units::meter_t target) {
  m_leftTarget = target;
  m_behavior = ClimbSubsystem::Behavior::kAsync;
}

void ClimbSubsystem::SetRightTarget(units::meter_t target) {
  m_rightTarget = target;
  m_behavior = ClimbSubsystem::Behavior::kAsync;
}

bool ClimbSubsystem::IsZeroed() const {
  return m_zeroed;
}

bool ClimbSubsystem::AtLeftTarget() const {
  return frc::IsNear(m_leftTarget, GetLeftHeight(),
                     ClimbConstants::kPositionTollerance);
}

bool ClimbSubsystem::AtRightTarget() const {
  return frc::IsNear(m_rightTarget, GetRightHeight(),
                     ClimbConstants::kPositionTollerance);
}

bool ClimbSubsystem::AtTargets() const {
  return AtLeftTarget() && AtRightTarget();
}

frc2::CommandPtr ClimbSubsystem::SetSyncBehaviorCMD(
    ClimbSubsystem::Behavior behavior) {
  return this->RunOnce([this, behavior] { SetSyncBehavior(behavior); });
}

frc2::CommandPtr ClimbSubsystem::SetSyncTargetCMD(units::meter_t target) {
  return this->RunOnce([this, target] { SetSyncTarget(target); });
}

frc2::CommandPtr ClimbSubsystem::SetLeftTargetCMD(units::meter_t target) {
  return this->RunOnce([this, target] { SetLeftTarget(target); });
}

frc2::CommandPtr ClimbSubsystem::SetRightTargetCMD(units::meter_t target) {
  return this->RunOnce([this, target] { SetRightTarget(target); });
}

void ClimbSubsystem::InitSendable(wpi::SendableBuilder& builder) {
  builder.SetSmartDashboardType("Climber Subsystem");

#define LAMBDA(x) [this] { return x; }

  builder.AddDoubleProperty("Left Height", LAMBDA(GetLeftHeight().value()),
                            nullptr);
  builder.AddDoubleProperty("Right Height", LAMBDA(GetRightHeight().value()),
                            nullptr);

  builder.AddDoubleProperty("Left Target", LAMBDA(m_leftTarget.value()),
                            nullptr);
  builder.AddDoubleProperty("Right Target", LAMBDA(m_rightTarget.value()),
                            nullptr);

  builder.AddBooleanProperty("At Left Target", LAMBDA(AtLeftTarget()), nullptr);

  builder.AddBooleanProperty("At Right Target", LAMBDA(AtRightTarget()),
                             nullptr);

  builder.AddBooleanProperty("At Targets", LAMBDA(AtTargets()), nullptr);

#undef LAMBDA
}

void ClimbSubsystem::ClimbSync() {
  m_leftTarget = m_syncTarget;
  m_rightTarget = m_syncTarget;
  ClimbLeft();
  ClimbRight();
}

void ClimbSubsystem::ClimbLeft() {
  m_controllerLeft.SetReference(m_leftTarget.value(),
                                rev::CANSparkMax::ControlType::kPosition);
}

void ClimbSubsystem::ClimbRight() {
  m_controllerRight.SetReference(m_rightTarget.value(),
                                 rev::CANSparkMax::ControlType::kPosition);
}

void ClimbSubsystem::ArmControl() {
  if (m_manual) {
    m_leftTarget = GetLeftHeight();
    m_rightTarget = GetRightHeight();
    m_syncTarget = (m_leftTarget + m_rightTarget) / 2;
    return;
  } else if (m_behavior == Behavior::kSync) {
    ClimbSync();
    return;
  } else {
    ClimbLeft();
    ClimbRight();
    return;
  }
}
