// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/DigitalInput.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/Trigger.h>
#include <rev/CANSparkMax.h>
#include <units/angle.h>
#include <wpi/sendable/SendableBuilder.h>

#include <string>

// 2 NEO motor
// 1 intake motor
// 1 pivot motor
// 1 through bore encoder
class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  enum class PivotTarget { kNone, kGround, kStow };

  enum class IntakeState { kNone, kIntake, kEject, kFeedShooter };

  IntakeSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  bool HasNote() const;

  void SetPivotTarget(PivotTarget target);
  void SetIntakeState(IntakeState state);

  PivotTarget GetCurrentState() const;

  [[nodiscard]]
  frc2::CommandPtr SetPivotTargetCMD(PivotTarget target);
  [[nodiscard]]
  frc2::CommandPtr SetIntakeStateCMD(IntakeState state);

  frc2::Trigger HasNoteTrigger();

  void InitSendable(wpi::SendableBuilder& builder) override;

 private:
  std::string ToStr(PivotTarget target) const;
  std::string ToStr(IntakeState state) const;

  units::degree_t TargetToSetpoint(PivotTarget target) const;
  double StateToSetpoint(IntakeState state) const;

  rev::CANSparkMax m_pivot;
  rev::CANSparkMax m_intake;

  rev::SparkAbsoluteEncoder m_pivotEncoder;
  rev::SparkPIDController m_pivotController;

  frc::DigitalInput m_limitSwitch;

  PivotTarget m_target;
  IntakeState m_state;
};
