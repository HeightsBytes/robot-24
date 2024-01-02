// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

namespace hb {

/**
 * @brief the regresion base is an abstract class meant to be inherited so a
 * regression may be passed generally
 *
 */
class RegressionBase {
 public:
  /**
   * @brief calculates the value of the function at x
   *
   * @param x
   * @return double
   */
  virtual double Calculate(double x) const = 0;

 protected:
  RegressionBase() = default;
};
}  // namespace hb
