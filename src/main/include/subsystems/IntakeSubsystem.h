// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkFlex.h>
#include <rev/CANSparkMax.h>
#include <units/angle.h>
#include <wpi/sendable/SendableBuilder.h>

#include <string>

class IntakeSubsystem : public frc2::SubsystemBase {
 public:

  enum class PivotState { kDeployed, kHandoff, kStow, kSwitching };
  enum class IntakeState { kIntaking, kHandoff, kStopped };

  IntakeSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  units::degree_t GetAngle() const { return units::degree_t(m_pivotEncoder.GetPosition()); }

  bool AtPivotTarget() const { return m_pivotTarget == m_pivotActual; }
  
  PivotState GetTargetState() const { return m_pivotTarget; }
  PivotState GetActualState() const { return m_pivotActual; }

  void SetPivotTarget(PivotState state) { m_pivotTarget = state; }
  void SetIntakeTarget(IntakeState state) { m_intakeTarget = state; }

  frc2::CommandPtr DeployIntakeCMD();
  frc2::CommandPtr SetPivotTargetCMD();
  frc2::CommandPtr StowIntakeCMD();

  void InitSendable(wpi::SendableBuilder& builder);

 private:

  double StateToOutput(IntakeState state) const;
  units::degree_t StateToOutput(PivotState state) const;

  void CheckState();

  std::string ToStr(PivotState state) const;
  std::string ToStr(IntakeState state) const;

  rev::CANSparkFlex m_intake;
  rev::CANSparkMax m_pivot;

  rev::SparkAbsoluteEncoder m_pivotEncoder;
  rev::SparkPIDController m_pivotController;

  PivotState m_pivotActual;
  PivotState m_pivotTarget;

  IntakeState m_intakeTarget;

};
