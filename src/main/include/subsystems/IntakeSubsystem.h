// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/Trigger.h>
#include <frc/DigitalInput.h>
#include <rev/CANSparkMax.h>
#include <rev/AbsoluteEncoder.h>
#include <rev/SparkPIDController.h>
#include <wpi/sendable/SendableBuilder.h>
#include <string>
#include <units/angle.h>

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem();

  enum class MotionState { kDisabled, kAutomatic };
  enum class PivotState { kStow, kGround, kMoving };
  enum class IntakeState { kStopped, kIntaking, kFeed, kEject };

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void InitSendable(wpi::SendableBuilder& builder) override;

  inline bool HasNote() const { return m_noteSwitch.Get(); }

  inline void SetMotionState(MotionState state) { m_motion = state; }
  inline MotionState GetMotionState() const { return m_motion; }

  inline void SetPivotState(PivotState target) { m_pivotTarget = target; }
  inline PivotState GetTargetPivotState() const { return m_pivotTarget; }
  inline PivotState GetCurrentPivotState() const { return m_pivotCurrent; }

  inline void SetIntakeState(IntakeState target) { m_intakeTarget = target; }
  inline IntakeState GetTargetIntakeState() const { return m_intakeTarget; }
  inline IntakeState GetCurrentIntakeState() const { return m_intakeCurrent; }

  inline units::degree_t GetPosition() const { 
    return units::degree_t(m_pivotEncoder.GetPosition()); }

  frc2::Trigger HasNoteTrigger() const { return frc2::Trigger{[this] {return HasNote();}}; }

 private:

  void ControlLoop();
  void UpdatePivotState();
  void UpdateIntakeState();

  double ToOutput(IntakeState state) const;
  double ToOutput(PivotState state) const;

  std::string ToStr(MotionState state) const;
  std::string ToStr(PivotState state) const;
  std::string ToStr(IntakeState state) const;

  rev::CANSparkMax m_pivotMotor;
  rev::SparkPIDController m_pivotController;
  rev::SparkAbsoluteEncoder m_pivotEncoder;

  rev::CANSparkMax m_intakeMotor;

  frc::DigitalInput m_noteSwitch;
  
  MotionState m_motion;

  PivotState m_pivotCurrent;
  PivotState m_pivotTarget;

  IntakeState m_intakeCurrent;
  IntakeState m_intakeTarget;

};
