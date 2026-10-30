#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

// Acquisition stamps use the same wall clock as RobotHardware / WrenchStamped.
// Monotonic receive times independently detect stopped publishers and clock jumps.
// No state is extrapolated and no repeated sample refreshes its lifetime.
class TransparencyInputs {
public:
  struct Config {
    size_t joints;
    double timeout, max_skew, velocity_cutoff_hz, acceleration_cutoff_hz;
    // Empty preserves legacy profiles without an explicit velocity threshold.
    std::vector<double> max_velocity;
    std::vector<double> max_acceleration;
    std::array<double, 6> wrench_limits;
    // Gravity uses q only; dq monitoring remains mandatory, wrench is optional.
    // Full measured-motion profiles retain the sensor requirement by default.
    bool require_wrench = true;
  };
  explicit TransparencyInputs(const Config& config) : config_(config) {
    if (!config.joints || !positive(config.timeout) ||
        !positive(config.max_skew) || config.max_skew > config.timeout ||
        !positive(config.velocity_cutoff_hz) || !positive(config.acceleration_cutoff_hz) ||
        (!config.max_velocity.empty() && config.max_velocity.size() != config.joints) ||
        config.max_acceleration.size() != config.joints)
      throw std::invalid_argument("invalid transparency input configuration");
    for (double x : config.max_velocity)
      if (!positive(x)) throw std::invalid_argument("invalid velocity limit");
    for (double x : config.max_acceleration)
      if (!positive(x)) throw std::invalid_argument("invalid acceleration limit");
    for (double x : config.wrench_limits)
      if (!positive(x)) throw std::invalid_argument("invalid wrench limit");
  }

  void reset() {
    position_ = Stream(); velocity_ = Stream(); force_ = Stream();
    acceleration_ready_ = false;
    q_.clear(); dq_.clear(); ddq_.clear();
    wrench_.fill(0.0);
  }
  bool position(const std::vector<double>& value, double stamp, double now, double wall) {
    if (!accept(position_, value, config_.joints, stamp, now, wall)) return false;
    q_ = value;
    return true;
  }
  bool velocity(const std::vector<double>& value, double stamp, double now, double wall) {
    const bool initialized = velocity_.valid;
    const double previous_stamp = velocity_.stamp;
    const double previous_receive = velocity_.received;
    if (!accept(velocity_, value, config_.joints, stamp, now, wall)) {
      acceleration_ready_ = false;
      return false;
    }
    // Check the unfiltered measurement, including the first sample after a gap.
    // Otherwise startup seeding or a low cutoff could hide an excessive speed.
    for (size_t i = 0; i < config_.max_velocity.size(); ++i) {
      if (std::abs(value[i]) > config_.max_velocity[i]) {
        return rejectVelocity(limitReason("velocity", i, value[i], config_.max_velocity[i], "rad/s"));
      }
    }
    const double interval = stamp - previous_stamp;
    if (!initialized || interval >= config_.timeout || now - previous_receive >= config_.timeout) {
      dq_ = value;
      ddq_.assign(value.size(), 0.0);
      acceleration_ready_ = false; // Need two new measurements after a gap.
      return true;
    }
    const double av = -std::expm1(-2.0 * pi() * config_.velocity_cutoff_hz * interval);
    const double aa = -std::expm1(-2.0 * pi() * config_.acceleration_cutoff_hz * interval);
    for (size_t i = 0; i < value.size(); ++i) {
      const double next_velocity = dq_[i] + av * (value[i] - dq_[i]);
      const double derivative = (next_velocity - dq_[i]) / interval;
      ddq_[i] += aa * (derivative - ddq_[i]);
      dq_[i] = next_velocity;
      if (!std::isfinite(ddq_[i]) || !std::isfinite(dq_[i])) {
        std::ostringstream error;
        error << "nonfinite velocity/acceleration estimate (joint=" << i
              << ", dq=" << dq_[i] << " rad/s, ddq=" << ddq_[i] << " rad/s^2)";
        return rejectVelocity(error.str());
      }
      if (std::abs(ddq_[i]) > config_.max_acceleration[i]) {
        return rejectVelocity(limitReason("acceleration", i, ddq_[i], config_.max_acceleration[i], "rad/s^2"));
      }
    }
    acceleration_ready_ = true;
    return true;
  }
  bool wrench(const std::vector<double>& value, double stamp, double now, double wall) {
    if (!accept(force_, value, 6, stamp, now, wall)) return false;
    for (size_t i = 0; i < 6; ++i) {
      if (std::abs(value[i]) > config_.wrench_limits[i]) {
        force_.valid = false;
        force_.reason = "wrench limit exceeded";
        return false;
      }
      wrench_[i] = value[i];
    }
    return true;
  }
  std::string reason(double now, double wall) const {
    std::string error = fresh(position_, now, wall);
    if (!error.empty()) return "qAct: " + error;
    error = fresh(velocity_, now, wall);
    if (!error.empty()) return "dqAct: " + error;
    if (!acceleration_ready_) return "dqAct: waiting for acceleration samples";
    if (config_.require_wrench) {
      error = fresh(force_, now, wall);
      if (!error.empty()) return "elbowWrench: " + error;
    }
    double newest = std::max(position_.stamp, velocity_.stamp);
    double oldest = std::min(position_.stamp, velocity_.stamp);
    if (config_.require_wrench) {
      newest = std::max(newest, force_.stamp);
      oldest = std::min(oldest, force_.stamp);
    }
    if (newest - oldest > config_.max_skew) return "input sample skew exceeded";
    return "";
  }
  // Optional diagnostics still need a real, fresh sensor sample aligned to q/dq.
  bool wrenchReady(double now, double wall) const {
    if (!fresh(position_, now, wall).empty() || !fresh(velocity_, now, wall).empty() ||
        !fresh(force_, now, wall).empty()) return false;
    const double newest = std::max({position_.stamp, velocity_.stamp, force_.stamp});
    const double oldest = std::min({position_.stamp, velocity_.stamp, force_.stamp});
    return newest - oldest <= config_.max_skew;
  }
  const std::vector<double>& q() const { return q_; }
  const std::vector<double>& dq() const { return dq_; }
  const std::vector<double>& ddq() const { return ddq_; }
  const std::array<double, 6>& wrench() const { return wrench_; }

private:
  struct Stream {
    double stamp = 0, received = 0;
    bool valid = false;
    std::string reason = "no sample";
  };
  static bool positive(double x) { return std::isfinite(x) && x > 0; }
  static double pi() { return 3.14159265358979323846; }
  static std::string limitReason(const char* quantity, size_t joint, double value,
                                 double limit, const char* units) {
    std::ostringstream error;
    error << std::setprecision(9) << quantity << " limit exceeded (joint=" << joint
          << ", value=" << value << " " << units << ", limit=" << limit << " " << units << ")";
    return error.str();
  }
  bool rejectVelocity(const std::string& reason) {
    velocity_.valid = false;
    velocity_.reason = reason;
    acceleration_ready_ = false;
    return false;
  }
  bool accept(Stream& stream, const std::vector<double>& value, size_t size,
              double stamp, double now, double wall) {
    std::string error;
    if (value.size() != size ||
        !std::all_of(value.begin(), value.end(), [](double x) { return std::isfinite(x); }))
      error = "invalid values";
    else if (!positive(stamp) || !std::isfinite(now) || !std::isfinite(wall))
      error = "invalid timestamp";
    else if (stamp <= stream.stamp)
      error = "non-increasing timestamp";
    else if (stream.stamp > 0 && now < stream.received)
      error = "receive clock moved backwards";
    else if (wall - stamp >= config_.timeout || stamp - wall > config_.max_skew)
      error = "stale or future timestamp";
    if (!error.empty()) {
      stream.valid = false;
      stream.reason = error;
      return false;
    }
    stream.stamp = stamp;
    stream.received = now;
    stream.valid = true;
    stream.reason.clear();
    return true;
  }
  std::string fresh(const Stream& stream, double now, double wall) const {
    if (!stream.valid) return stream.reason;
    if (!std::isfinite(now) || !std::isfinite(wall) || now < stream.received ||
        now - stream.received >= config_.timeout || wall - stream.stamp >= config_.timeout ||
        stream.stamp - wall > config_.max_skew) return "timeout or clock discontinuity";
    return "";
  }
  Config config_;
  Stream position_, velocity_, force_;
  bool acceleration_ready_ = false;
  std::vector<double> q_, dq_, ddq_;
  std::array<double, 6> wrench_{{0, 0, 0, 0, 0, 0}};
};
