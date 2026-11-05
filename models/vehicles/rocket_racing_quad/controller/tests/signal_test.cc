#include <cmath>
#include <iostream>
#include "vibration_signal.hh"

int main() {
  using namespace rocket;
  const std::vector<SignalComponent> harmonics = {
      {1.0, {2.0, 0.0, 0.0}, {0.3, 0.0, 0.0}},
      {2.0, {3.0, 0.0, 0.0}, {1.2, 0.0, 0.0}}};
  double sum = 0, squares = 0;
  for (int i = 0; i < 10000; ++i) {
    const auto a = Signal(harmonics, kTwoPi * i / 10000, 1.0);
    if (Signal(harmonics, kTwoPi * i / 10000, 0.0)[0] != 0 || a[1] != 0 || a[2] != 0) return 1;
    sum += a[0]; squares += a[0] * a[0];
  }
  if (std::abs(sum / 10000) > 1e-12 || std::abs(squares / 10000 - 6.5) > 1e-10) return 2;
  double ramp = 0;
  for (int i = 0; i < 2000; ++i) ramp = Ramp(ramp, true, .001, 2.0);
  if (std::abs(ramp - 1.0) > 1e-10 || Ramp(ramp, false, 3.0, 2.0) != 0 || Ramp(0, true, -1, 2) != 0) return 3;
  std::cout << "harmonic RMS, axis isolation, zero gate and ramp verified\n";
  return 0;
}
