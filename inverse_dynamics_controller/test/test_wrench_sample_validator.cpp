#include "WrenchSampleValidator.h"

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

using inverse_dynamics_controller::WrenchSampleValidator;

namespace {
void require(bool condition, const char* description) {
  if (!condition) {
    std::cerr << "FAIL: " << description << std::endl;
    std::exit(EXIT_FAILURE);
  }
}
bool allInvalid(const std::array<double, 6>& wrench) {
  for (double value : wrench) if (!std::isnan(value)) return false;
  return true;
}
}  // namespace

int main() {
  WrenchSampleValidator validator;
  const std::array<double, 6> original{{1.0, -2.0, 3.0, 0.4, -0.5, 0.6}};
  auto wrench = original;
  require(validator.validate("sensor", "sensor", 10, 1, wrench), "first sensor sample accepted");
  require(wrench == original, "all force and moment components preserved");

  require(!validator.validate("sensor", "sensor", 10, 1, wrench), "replayed sample rejected");
  require(allInvalid(wrench), "replay clears every component rather than retaining old force");

  wrench = original;
  require(!validator.validate("wrong", "sensor", 10, 2, wrench), "wrong sensor frame rejected");
  require(allInvalid(wrench), "wrong frame is transmitted as invalid");
  wrench = original;
  require(!validator.validate("sensor", "sensor", 10, 2, wrench), "previously rejected stamp cannot be replayed");

  wrench = original;
  wrench[4] = std::numeric_limits<double>::infinity();
  require(!validator.validate("sensor", "sensor", 10, 3, wrench), "nonfinite moment rejected");
  require(allInvalid(wrench), "one invalid component invalidates complete wrench");
  wrench = original;
  require(validator.validate("sensor", "sensor", 10, 4, wrench), "fresh valid sample recovers");
  require(wrench == original, "recovery preserves raw wrench");

  wrench = original;
  require(!validator.validate("sensor", "sensor", 9, 999999999, wrench), "backward source time rejected");
  require(allInvalid(wrench), "backward time invalidates output");

  WrenchSampleValidator unstamped;
  wrench = original;
  require(!unstamped.validate("sensor", "sensor", 0, 0, wrench), "zero acquisition stamp rejected");
  wrench = original;
  require(!unstamped.validate("sensor", "", 1, 0, wrench), "empty expected frame rejected");
  wrench = original;
  require(!unstamped.validate("sensor", "sensor", 2, 1000000000, wrench), "invalid nanoseconds rejected");
  wrench = original;
  require(unstamped.validate("sensor", "sensor", 2, 0, wrench), "unrepresentable stamp does not poison subsequent valid input");
  wrench = original;
  require(unstamped.validate("sensor", "sensor", 0x80000000u, 0, wrench), "full unsigned ROS and RTC seconds range is preserved");
  std::cout << "Wrench sample validator checks passed" << std::endl;
  return EXIT_SUCCESS;
}
