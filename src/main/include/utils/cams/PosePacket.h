// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Pose2d.h>
#include <units/time.h>

struct PosePacket {
  PosePacket() = default;

  PosePacket(frc::Pose2d pose, units::second_t timestamp) {
    this->pose = pose;
    this->timestamp = timestamp;
  }

  frc::Pose2d pose;
  units::second_t timestamp;
};
