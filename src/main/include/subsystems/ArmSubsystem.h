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
#include "utils/regressions/Exponential.h"
#include "utils/regressions/Linear.h"
#include "utils/regressions/Logarithmic.h"
#include "utils/regressions/SquareRoot.h"

class ArmSubsystem : public frc2::SubsystemBase {
 public:
  enum class State {
    kInFrame,
    kStow,
    kHandoff,
    kTargetting,
    kSwitching,
    kTrap
  };

  explicit ArmSubsystem(std::function<frc::Pose2d()> poseFunc = nullptr);

  void Periodic() override;

  units::degree_t GetAngle() const;
  units::degrees_per_second_t GetVelocity() const;

  inline bool AtTarget() const { return m_target == m_actual; }

  inline State GetTargetState() const { return m_target; }
  inline State GetActualState() const { return m_actual; }

  inline void SetTargetState(State state) { m_target = state; }

  [[nodiscard]]
  frc2::CommandPtr SetTargetStateCMD(State state);

  frc2::Trigger AtSpeakerTargetTrigger() {
    return frc2::Trigger{
        [this] { return GetActualState() == State::kTargetting; }};
  }

  void InitSendable(wpi::SendableBuilder& builder) override;

 private:
  void ControlLoop();
  void CheckState();

  std::string ToStr(State state) const;
  units::degree_t ToSetpoint(State state) const;
  units::degree_t TargettingAngle() const;

  rev::CANSparkMax m_motor;

  rev::SparkAbsoluteEncoder m_encoder;

  rev::SparkPIDController m_controller;

  State m_target;
  State m_actual;

  bool m_atTarget;

  // hb::Logarithmic m_reg{117.035, -16.9294};
  // hb::SquareRoot m_regLin{-11.115, -1.7112, -88.4067, 141.609};
  hb::Exponential m_regLin{31.6811, 1.03527};

  double m_targetVal;

  mutable double m_lastLLAngle = 0;
};
