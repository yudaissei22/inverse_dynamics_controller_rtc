#include "InverseDynamicsController.h"
#include <chrono>
#include <iomanip>
#include <sstream>
#include <sys/time.h>

namespace {
double monotonicTime() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
double wallTime() {
  timeval time;
  gettimeofday(&time, nullptr);
  return time.tv_sec + time.tv_usec * 1e-6;
}
double sampleTime(const RTC::Time& time) {
  return time.nsec < 1000000000 ? time.sec + time.nsec * 1e-9 : 0.0;
}
std::vector<double> values(const RTC::TimedDoubleSeq& sample) {
  std::vector<double> data(sample.data.length());
  for (size_t i = 0; i < data.size(); ++i) data[i] = sample.data[i];
  return data;
}
template<class Values>
void publish(const Values& values, const RTC::Time& stamp, RTC::TimedDoubleSeq& sample,
             RTC::OutPort<RTC::TimedDoubleSeq>& port) {
  sample.tm = stamp;
  sample.data.length(values.size());
  for (size_t i = 0; i < values.size(); ++i) sample.data[i] = values[i];
  port.write();
}
std::string jsonString(const std::string& value) {
  std::ostringstream out;
  out << '"';
  for (unsigned char c : value) {
    if (c == '"' || c == '\\') out << '\\' << c;
    else if (c < 0x20) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c);
    else out << c;
  }
  out << '"';
  return out.str();
}
}

bool InverseDynamicsController::initializeTransparency() {
  std::string enabled;
  if (!getProperty("transparency_control", enabled) || enabled == "false") return true;
  try {
    if (enabled != "true" || !direct_gate_)
      throw std::invalid_argument("transparency_control requires direct_torque_control=true");
    auto read = [&](const std::string& key, size_t length) {
      std::string text;
      if (!getProperty(key, text)) throw std::invalid_argument("missing " + key);
      std::istringstream stream(text);
      std::vector<double> result;
      double value;
      while (stream >> value) {
        if (!std::isfinite(value)) throw std::invalid_argument("nonfinite " + key);
        result.push_back(value);
      }
      if (!stream.eof() || result.size() != length)
        throw std::invalid_argument("invalid " + key);
      return result;
    };
    const size_t joints = actRobot->numJoints();
    TransparencyDynamics::Config dynamics;
    std::string gravity_only;
    if (getProperty("transparency_gravity_only", gravity_only)) {
      if (gravity_only != "true" && gravity_only != "false")
        throw std::invalid_argument("invalid transparency_gravity_only");
      dynamics.gravity_only = gravity_only == "true";
    }
    transparency_gravity_only_ = dynamics.gravity_only;
    if (!getProperty("transparency_sensor_link", dynamics.sensor_link))
      throw std::invalid_argument("missing transparency_sensor_link");
    auto xyz = read("transparency_sensor_xyz", 3);
    auto rpy = read("transparency_sensor_rpy", 3);
    auto gravity = read("transparency_gravity", 3);
    for (size_t i = 0; i < 3; ++i) {
      dynamics.sensor_translation[i] = xyz[i];
      dynamics.sensor_rpy[i] = rpy[i];
      dynamics.gravity[i] = gravity[i];
    }
    dynamics.wrench_sign = read("transparency_wrench_sign", 1)[0];
    auto bias = read("transparency_wrench_bias", 6);
    std::copy(bias.begin(), bias.end(), dynamics.wrench_bias.begin());
    dynamics.viscous = read("transparency_viscous", joints);
    dynamics.coulomb = read("transparency_coulomb", joints);
    dynamics.friction_velocity_scale = read("transparency_friction_velocity_scale", 1)[0];
    TransparencyInputs::Config inputs;
    inputs.require_wrench = !transparency_gravity_only_;
    inputs.joints = joints;
    inputs.timeout = read("transparency_feedback_timeout", 1)[0];
    inputs.max_skew = read("transparency_max_sample_skew", 1)[0];
    inputs.velocity_cutoff_hz = read("transparency_velocity_cutoff_hz", 1)[0];
    inputs.acceleration_cutoff_hz = read("transparency_acceleration_cutoff_hz", 1)[0];
    // Older profiles only bounded acceleration; preserve them when this key is absent.
    std::string velocity_limit;
    if (getProperty("transparency_max_velocity", velocity_limit))
      inputs.max_velocity = read("transparency_max_velocity", joints);
    inputs.max_acceleration = read("transparency_max_acceleration", joints);
    auto limits = read("transparency_wrench_limits", 6);
    std::copy(limits.begin(), limits.end(), inputs.wrench_limits.begin());
    transparency_dynamics_.reset(new TransparencyDynamics(actRobot, dynamics));
    transparency_inputs_.reset(new TransparencyInputs(inputs));
    return true;
  } catch (const std::exception& error) {
    std::cerr << "Transparency config: " << error.what() << std::endl;
    transparency_inputs_.reset();
    transparency_dynamics_.reset();
    return false;
  }
}

// Called with mutex_ held, on every direct-control RTC cycle, even in POSITION.
// This allows diagnostics to be inspected before enabling torque generation.
void InverseDynamicsController::executeTransparency(double now) {
  if (!transparency_inputs_) return;
  const double wall = wallTime();
  if (ports_.m_dqActIn_.isNew()) {
    ports_.m_dqActIn_.read();
    transparency_inputs_->velocity(values(ports_.m_dqAct_), sampleTime(ports_.m_dqAct_.tm), now, wall);
  }
  if (ports_.m_elbowWrenchIn_.isNew()) {
    ports_.m_elbowWrenchIn_.read();
    transparency_inputs_->wrench(values(ports_.m_elbowWrench_), sampleTime(ports_.m_elbowWrench_.tm), now, wall);
  }
  transparency_error_ = transparency_inputs_->reason(now, wall);
  if (transparency_error_.empty()) {
    try {
      const bool wrench_ready = transparency_inputs_->wrenchReady(now, wall);
      // In gravity-only mode wrench is unused by the motor compensation. An
      // absent/invalid optional sensor must neither stop g(q) nor become a
      // fabricated zero measurement in diagnostics.
      const auto wrench = wrench_ready ? transparency_inputs_->wrench() : TransparencyDynamics::Wrench{};
      const auto result = transparency_dynamics_->calculate(
          transparency_inputs_->q(), transparency_inputs_->dq(), transparency_inputs_->ddq(),
          wrench);
      publish(result.torque, ports_.m_qAct_.tm, ports_.m_tauModel_, ports_.m_tauModelOut_);
      publish(transparency_inputs_->ddq(), ports_.m_dqAct_.tm, ports_.m_ddqMeasured_, ports_.m_ddqMeasuredOut_);
      if (wrench_ready) {
        publish(result.human_torque, ports_.m_elbowWrench_.tm, ports_.m_tauHuman_, ports_.m_tauHumanOut_);
        publish(result.contact_wrench, ports_.m_elbowWrench_.tm, ports_.m_elbowWrenchOutData_, ports_.m_elbowWrenchOut_);
      }
      if (transparency_active_) {
        // The gate remains the sole writer of tauOut, including rate/limit checks.
        // Wrench is deliberately NOT added to or subtracted from result.torque.
        // A diagnostic subscriber can delay a synchronous OutPort write.
        // Recheck acquisition/receive age after those writes before arming torque.
        const double command_now = monotonicTime();
        transparency_error_ = transparency_inputs_->reason(command_now, wallTime());
        if (transparency_error_.empty() &&
            !direct_gate_->setCommand(result.torque, 0.1, command_now,
                                     transparency_gravity_only_)) transparency_active_ = false;
      }
    } catch (const std::exception& error) {
      transparency_error_ = error.what();
    }
  }
  if (transparency_active_ && !transparency_error_.empty()) {
    direct_gate_->fault("transparency: " + transparency_error_);
    transparency_active_ = false;
  }
}

bool InverseDynamicsController::startTransparency() {
  std::lock_guard<std::mutex> guard(mutex_);
  if (!transparency_inputs_ || transparency_active_ || !direct_gate_->torqueIsZero()) return false;
  const auto mode = direct_gate_->mode();
  if (mode != DirectTorqueGate::TORQUE &&
      !(transparency_gravity_only_ && mode == DirectTorqueGate::POSITION)) return false;
  transparency_error_ = transparency_inputs_->reason(monotonicTime(), wallTime());
  if (!transparency_error_.empty()) return false;
  // Recompute for this sample so an old diagnostic result cannot arm control.
  try {
    const auto wrench = transparency_inputs_->wrenchReady(monotonicTime(), wallTime()) ?
        transparency_inputs_->wrench() : TransparencyDynamics::Wrench{};
    const auto result = transparency_dynamics_->calculate(
        transparency_inputs_->q(), transparency_inputs_->dq(), transparency_inputs_->ddq(),
        wrench);
    // Validate hardware freshness and torque limits before acknowledging start.
    // mutex_ prevents either onExecute or a manual writer racing this handover.
    if (!direct_gate_->setCommand(result.torque, 0.1, monotonicTime(),
                                 transparency_gravity_only_)) return false;
  } catch (const std::exception& error) {
    transparency_error_ = error.what();
    return false;
  }
  transparency_active_ = true;
  return true;
}

bool InverseDynamicsController::stopTransparency() {
  std::lock_guard<std::mutex> guard(mutex_);
  if (!transparency_inputs_) return false;
  if (transparency_active_) {
    transparency_active_ = false;
    direct_gate_->cancelCommand();
  }
  return true;
}

std::string InverseDynamicsController::getTransparencyStatus() {
  std::lock_guard<std::mutex> guard(mutex_);
  const double now = monotonicTime(), wall = wallTime();
  std::string reason = transparency_inputs_ ?
      transparency_inputs_->reason(now, wall) : "transparency is not configured";
  if (reason.empty()) reason = transparency_error_;
  if (direct_gate_ && direct_gate_->mode() == DirectTorqueGate::FAULT)
    reason = direct_gate_->faultReason();
  std::ostringstream out;
  out << "{\"configured\":" << (transparency_inputs_ ? "true" : "false")
      << ",\"gravity_only\":" << (transparency_gravity_only_ ? "true" : "false")
      << ",\"active\":" << (transparency_active_ ? "true" : "false")
      << ",\"ready\":" << (reason.empty() ? "true" : "false")
      << ",\"output_ready\":" << (transparency_gravity_only_ && transparency_active_ &&
          reason.empty() && direct_gate_ && direct_gate_->gravityOutputReady(now) ? "true" : "false")
      << ",\"wrench_ready\":" << (transparency_inputs_ &&
          transparency_inputs_->wrenchReady(now, wall) ? "true" : "false")
      << ",\"reason\":" << jsonString(reason) << "}";
  return out.str();
}
