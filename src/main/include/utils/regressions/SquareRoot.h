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
 * desmos but for reference they will be this: a * std::sqrt(b * x - h) + k
 *
 */
class SquareRoot : public RegressionBase {
 public:
  /**
   * @brief Construct a new Square Root regression object
   * the coefficients should be entered according to the equation
   * a*std::sqrt(b*x-h) + k
   *
   * @param a
   * @param b
   * @param h
   * @param k
   */
  SquareRoot(double a, double b, double h, double k);

  double Calculate(double x) const override;

 private:
  double m_a, m_b, m_h, m_k;
};
}  // namespace hb
