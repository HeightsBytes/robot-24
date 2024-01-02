// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "RegressionBase.h"

namespace hb {

/**
 * The values entered in the constructor shoudld be the same ones coming from
 * this desmos: https://www.desmos.com/calculator/jjwe4f3fl0
 *
 * The numbers should be entered into the contructor in the order found in
 * desmos but for reference they will be this: a * std::sin(b(x - c)) + d
 *
 * @warning This uses radians, not degrees
 */
class Sinusoidal : public RegressionBase {
 public:
  /**
   * @brief Construct a new Sinusoidal regression object
   * the coefficents should be entered according to the equation
   * a*std::sin(b(x-c))+d
   *
   * @param a
   * @param b
   * @param c
   * @param d
   */
  explicit Sinusoidal(double a, double b, double c, double d);

  double Calculate(double x) const override;

 private:
  double m_a, m_b, m_c, m_d;
};
}  // namespace hb
