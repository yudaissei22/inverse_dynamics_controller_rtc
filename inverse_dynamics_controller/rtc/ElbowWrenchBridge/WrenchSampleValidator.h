#ifndef INVERSE_DYNAMICS_CONTROLLER_WRENCH_SAMPLE_VALIDATOR_H
#define INVERSE_DYNAMICS_CONTROLLER_WRENCH_SAMPLE_VALIDATOR_H

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

namespace inverse_dynamics_controller {

// Does not invent acquisition timestamps or infer freshness from force changes.
// The consuming controller must also check age against its own clock.
class WrenchSampleValidator {
public:
  bool validate(const std::string& frame, const std::string& expected_frame,
                std::uint32_t sec, std::uint32_t nsec,
                std::array<double, 6>& wrench) {
    const bool representable = nsec < 1000000000u;
    const std::uint64_t stamp = static_cast<std::uint64_t>(sec) * 1000000000ull + nsec;
    bool valid = representable && stamp > 0 && stamp > last_stamp_ &&
        !expected_frame.empty() && frame == expected_frame;
    for (double value : wrench) valid = valid && std::isfinite(value);
    // Remember every representable, positive source timestamp, including that
    // of an invalid wrench; resending an old sample cannot make it fresh again.
    if (representable && stamp > last_stamp_) last_stamp_ = stamp;
    if (!valid) wrench.fill(std::numeric_limits<double>::quiet_NaN());
    return valid;
  }

private:
  std::uint64_t last_stamp_ = 0;
};

}  // namespace inverse_dynamics_controller

#endif
