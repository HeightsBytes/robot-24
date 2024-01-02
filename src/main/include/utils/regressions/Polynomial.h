// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/array.h>

#include <cmath>

#include "RegressionBase.h"

namespace hb {
template <int terms>
  requires(terms > 0)
class Polynomial : public RegressionBase {
 public:
  /** from least to greatest **/
  explicit Polynomial(wpi::array<double, terms> constants) {
    m_constants = constants;
  }

  double Calculate(double input) const override {
    double output = 0;
    for (int i = 0; i < degree; i++) {
      output += std::pow(input, i) * m_constants[i];
    }
  }

 private:
  wpi::array<double, terms> m_constants;
};
}  // namespace hb
