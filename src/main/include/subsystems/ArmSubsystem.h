// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/Trigger.h>
#include <rev/CANSparkMax.h>
#include <units/angle.h>
#include <units/voltage.h>
#include <wpi/sendable/SendableBuilder.h>

#include <functional>
#include <string>

#include "Constants.h"
#include "VisionSubsystem.h"
#include "utils/regressions/Linear.h"

// 1 NEO (Vortex?) Motor
class ArmSubsystem : public frc2::SubsystemBase {
 public:
  ArmSubsystem();

  void Periodic() override;

  units::degree_t GetAngle() const;
  units::degrees_per_second_t GetVelocity() const;

  bool AtTarget() const;
  bool IsAt(units::degree_t value) const;

  void SetTarget(units::degree_t target);

  frc2::Trigger AtTargetTrigger();

  [[nodiscard]]
  frc2::CommandPtr SetTargetCMD(units::degree_t target);

  void InitSendable(wpi::SendableBuilder& builder) override;

 private:
  void ControlLoop();
  void CheckState();

  rev::CANSparkMax m_motor;

  rev::SparkAbsoluteEncoder m_encoder;

  rev::SparkPIDController m_controller;

  units::degree_t m_target;

  bool m_atTarget;

  bool m_tuning = false;

  double kP = ArmConstants::kP;
  double kI = ArmConstants::kI;
  double kD = ArmConstants::kD;
  double Setpoint = 0;  // degrees
};
