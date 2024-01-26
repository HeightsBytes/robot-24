// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <units/angle.h>
#include <units/voltage.h>
#include <wpi/sendable/SendableBuilder.h>

#include <functional>
#include <string>

#include "VisionSubsystem.h"
#include "utils/regressions/Linear.h"

// 1 NEO (Vortex?) Motor
class ArmSubsystem : public frc2::SubsystemBase {
 public:
  enum class State { kStow, kAmp, kTargeting, kSwitching };

  explicit ArmSubsystem(std::function<frc::Pose2d()> poseFunction);

  void Periodic() override;

  units::degree_t GetAngle() const;

  bool AtTarget() const;

  State GetCurrentState() const;
  State GetTargetState() const;

  void SetState(State state);

  [[nodiscard]]
  frc2::CommandPtr SetStateCMD(State state);

  void InitSendable(wpi::SendableBuilder& builder) override;

 private:
  void ControlLoop();
  void CheckState();

  units::degree_t ToAngle(State state) const;
  units::degree_t StageAngle() const;

  units::volt_t CalculateKg() const;

  std::string ToStr(State state) const;

  rev::CANSparkMax m_motor;

  rev::SparkAbsoluteEncoder m_encoder;

  rev::SparkPIDController m_controller;

  VisionSubsystem& m_vision;

  State m_actual;
  State m_target;

  std::function<frc::Pose2d()> m_pose;

  // placeholder regression
  hb::Linear m_angleFunction;
};
