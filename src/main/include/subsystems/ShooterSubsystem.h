// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkFlex.h>
#include <rev/SparkPIDController.h>
#include <units/angular_velocity.h>

#include <string>

// 2 NEO Vortex running flywheel
// Seperate PID controllers in order to ensure both wheels are as accurate as
// can-be
class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  enum class State { kStopped, kIdle, kSpinningUp, kReady };

  ShooterSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  State GetState() const;

 private:
  // std::string StateToString(State state) const;

  // rev::CANSparkFlex m_motor0;
  // rev::CANSparkFlex m_motor1;

  // rev::SparkRelativeEncoder m_encoder0;
  // rev::SparkRelativeEncoder m_encoder1;

  // rev::SparkPIDController m_controller0;
  // rev::SparkPIDController m_controller1;

  // units::revolutions_per_minute_t m_target;
  // State m_state;
};
