// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkPIDController.h>
#include <rev/SparkRelativeEncoder.h>
#include <units/length.h>
#include <wpi/sendable/SendableBuilder.h>

#include <iostream>

// 2 NEO Motor
// No absolute encoder
// Independent control
// Assume zero upon startup
class ClimbSubsystem : public frc2::SubsystemBase {

  friend class ZeroClimber;

 public:
  enum class Behavior { kSync, kAsync };

  ClimbSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  units::meter_t GetLeftHeight() const;
  units::meter_t GetRightHeight() const;

  void SetSyncBehavior(Behavior behavior);
  void SetSyncTarget(units::meter_t target);
  void SetLeftTarget(units::meter_t target);
  void SetRightTarget(units::meter_t target);

  // Commands -- [[nodiscard]] added because they need to be scheduled to work
  [[nodiscard]]
  frc2::CommandPtr SetSyncBehaviorCMD(Behavior behavior);
  [[nodiscard]]
  frc2::CommandPtr SetSyncTargetCMD(units::meter_t target);
  [[nodiscard]]
  frc2::CommandPtr SetLeftTargetCMD(units::meter_t target);
  [[nodiscard]]
  frc2::CommandPtr SetRightTargetCMD(units::meter_t target);

  void InitSendable(wpi::SendableBuilder& builder) override;

 private:
  void ClimbSync();

  void ClimbLeft();

  void ClimbRight();

  // left and right relative to the back of the robot
  rev::CANSparkMax m_motorLeft;
  rev::CANSparkMax m_motorRight;

  rev::SparkRelativeEncoder m_encoderLeft;
  rev::SparkRelativeEncoder m_encoderRight;

  rev::SparkPIDController m_controllerLeft;
  rev::SparkPIDController m_controllerRight;

  Behavior m_behavior;

  units::meter_t m_syncTarget;
  units::meter_t m_leftTarget;
  units::meter_t m_rightTarget;

  bool m_zeroed;
};
