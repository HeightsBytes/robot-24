// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/MathUtil.h>

#include <cmath>

namespace hb {

  inline int sgn(double x) {
    return x >= 0 ? 1 : -1;
  }

  inline bool InRange(double val, double target, double epsilon) {
    // Check to see if the val is within the bounded range created by [target -
    // epsilon, target + epsilon]
    return (val > (target - epsilon) && val < (target + epsilon));
  }

  inline double hypot(double a, double b) {
    return std::sqrt(std::pow(a, 2) + std::pow(b, 2));
  }

}  // namespace hb
