#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace rocket {
constexpr double kTwoPi = 6.283185307179586476925286766559;
struct SignalComponent {
  double rate = 0;  // Hz for body torque, harmonic number for local IMU.
  std::array<double, 3> amplitude{};
  std::array<double, 3> phase{};
};
inline std::array<double, 3> Signal(const std::vector<SignalComponent> &components,
                                    double phase, double envelope) {
  std::array<double, 3> result{};
  for (const auto &c : components)
    for (unsigned int i = 0; i < 3; ++i)
      result[i] += envelope * c.amplitude[i] * std::sin(c.rate * phase + c.phase[i]);
  return result;
}
inline double Ramp(double previous, bool active, double dt, double rampTime) {
  const double step = std::max(0.0, dt) / rampTime;
  return active ? std::min(1.0, previous + step) : std::max(0.0, previous - step);
}
}  // namespace rocket
