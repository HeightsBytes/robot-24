// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix/sensors/PigeonIMU.h>
#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

namespace hb {
  class PigeonGyro {
   public:
    explicit PigeonGyro(int ID);

    units::degree_t GetAngle() const;

    units::degrees_per_second_t GetRate() const;

    void Reset();

    units::degree_t GetPitch() const;

    units::degree_t GetRoll() const;

    frc::Rotation2d GetRot2d() const;

    units::radian_t GetRad() const;

    void SetPosition(units::degree_t);

    units::degree_t GetCompassHeading() const;

    void Set(units::degree_t heading);

   private:
    ctre::phoenix::sensors::PigeonIMU* pigeon;
    mutable double m_angle;
    mutable double m_rate;
    double m_offset;
  };
}  // namespace hb
