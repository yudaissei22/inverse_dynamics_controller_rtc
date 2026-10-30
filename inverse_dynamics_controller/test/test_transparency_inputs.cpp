// Timestamp/filter/invalid-input regression tests without ROS or hardware.
#ifdef NDEBUG
#undef NDEBUG
#endif
#include "TransparencyInputs.h"
#include <cassert>
#include <iostream>
#include <limits>

namespace {
using Inputs = TransparencyInputs;
Inputs::Config config() {
  Inputs::Config cfg;
  cfg.joints = 2;
  cfg.timeout = 0.2;
  cfg.max_skew = 0.03;
  cfg.velocity_cutoff_hz = 10;
  cfg.acceleration_cutoff_hz = 5;
  cfg.max_acceleration = {1000, 1000};
  cfg.wrench_limits = {{100,100,100,10,10,10}};
  return cfg;
}
void near(double a, double b) { assert(std::isfinite(a) && std::abs(a-b) < 1e-9); }
void warm(Inputs& inputs) {
  assert(inputs.position({0.1,0.2}, 100, 0, 100));
  assert(inputs.velocity({0,0}, 100, 0, 100));
  assert(inputs.wrench({1,2,3,0.1,0.2,0.3}, 100, 0, 100));
  assert(inputs.reason(0,100) == "dqAct: waiting for acceleration samples");
  assert(inputs.position({0.1,0.2}, 100.01, 0.01, 100.01));
  assert(inputs.velocity({0,0}, 100.01, 0.01, 100.01));
  assert(inputs.wrench({1,2,3,0.1,0.2,0.3}, 100.01, 0.01, 100.01));
  assert(inputs.reason(0.01,100.01).empty());
}
template<class Operation> void rejects(Operation operation) {
  bool rejected = false;
  try { operation(); } catch (const std::invalid_argument&) { rejected = true; }
  assert(rejected);
}
}

int main() {
  auto cfg = config();
  {
    auto gravity_cfg = cfg;
    gravity_cfg.require_wrench = false;
    Inputs gravity(gravity_cfg), full(cfg);
    for (Inputs* inputs : {&gravity, &full}) {
      assert(inputs->position({0,0},100,0,100));
      assert(inputs->velocity({0,0},100,0,100));
      assert(inputs->position({0,0},100.01,0.01,100.01));
      assert(inputs->velocity({0,0},100.01,0.01,100.01));
    }
    assert(gravity.reason(0.01,100.01).empty());
    assert(!gravity.wrenchReady(0.01,100.01));
    assert(full.reason(0.01,100.01) == "elbowWrench: no sample");
    for (Inputs* inputs : {&gravity, &full}) {
      assert(inputs->wrench({0,0,0,0,0,0},100.01,0.01,100.01));
      assert(inputs->wrenchReady(0.01,100.01));
      // A stale optional sensor cannot remove gravity support.
      assert(inputs->position({0,0},100.25,0.25,100.25));
      assert(inputs->velocity({0,0},100.25,0.25,100.25));
      assert(inputs->position({0,0},100.26,0.26,100.26));
      assert(inputs->velocity({0,0},100.26,0.26,100.26));
      assert(!inputs->wrenchReady(0.26,100.26));
    }
    assert(gravity.reason(0.26,100.26).empty());
    assert(full.reason(0.26,100.26) == "elbowWrench: timeout or clock discontinuity");
    for (Inputs* inputs : {&gravity, &full}) {
      assert(!inputs->wrench({0,0,0,0,0,std::numeric_limits<double>::quiet_NaN()},
                             100.26,0.26,100.26));
      assert(!inputs->wrenchReady(0.26,100.26));
    }
    assert(gravity.reason(0.26,100.26).empty());
    assert(full.reason(0.26,100.26) == "elbowWrench: invalid values");
    // Optional wrench does not weaken q/dq acquisition, skew or age checks.
    assert(gravity.position({0,0},100.30,0.30,100.30));
    assert(gravity.reason(0.30,100.30) == "input sample skew exceeded");
    assert(gravity.reason(0.6,100.6) == "qAct: timeout or clock discontinuity");
  }
  {
    Inputs inputs(cfg);
    assert(inputs.reason(0,100) == "qAct: no sample");
    warm(inputs);
    // Reception jitter must not replace acquisition time in differentiation.
    assert(inputs.velocity({1,-2}, 100.03, 0.06, 100.06));
    const double interval = 0.02;
    const double av = -std::expm1(-2*std::acos(-1.0)*cfg.velocity_cutoff_hz*interval);
    const double aa = -std::expm1(-2*std::acos(-1.0)*cfg.acceleration_cutoff_hz*interval);
    near(inputs.dq()[0], av);
    near(inputs.dq()[1], -2*av);
    near(inputs.ddq()[0], aa*av/interval);
    near(inputs.ddq()[1], -2*aa*av/interval);
    // Do not reset a derivative when the input velocity happens to repeat.
    assert(inputs.velocity({1,-2}, 100.05, 0.07, 100.07));
    near(inputs.dq()[0], av+av*(1-av));
    near(inputs.ddq()[0], aa*av/interval + aa*(av*(1-av)/interval-aa*av/interval));
    inputs.reset();
    assert(inputs.q().empty() && inputs.dq().empty() && inputs.ddq().empty());
    assert(inputs.reason(0.07,100.07) == "qAct: no sample");
    for (double x : inputs.wrench()) near(x,0);
    warm(inputs);
  }
  {
    Inputs inputs(cfg); warm(inputs);
    // A duplicate sample invalidates the stream rather than extending age.
    assert(!inputs.position({8,9}, 100.01, 0.1, 100.1));
    assert(inputs.reason(0.1,100.1) == "qAct: non-increasing timestamp");
    near(inputs.q()[0],0.1);
    assert(inputs.position({0.1,0.2}, 100.02, 0.1, 100.1));
    assert(!inputs.velocity({1,1}, 100.01, 0.1, 100.1));
    assert(inputs.reason(0.1,100.1) == "dqAct: non-increasing timestamp");
    assert(inputs.velocity({1,1}, 100.02, 0.1, 100.1));
    assert(inputs.reason(0.1,100.1) == "dqAct: waiting for acceleration samples");
    near(inputs.ddq()[0],0);
    assert(inputs.velocity({1,1}, 100.03, 0.11, 100.11));
    assert(inputs.reason(0.11,100.11).empty());
    assert(!inputs.wrench({1,2,3,4,5,6}, 100.01, 0.12, 100.12));
    assert(inputs.reason(0.12,100.12) == "elbowWrench: non-increasing timestamp");
  }
  {
    Inputs inputs(cfg); warm(inputs);
    assert(inputs.reason(0.22,100.22) == "qAct: timeout or clock discontinuity");
    // A long acquisition gap starts a new acceleration history at zero.
    assert(inputs.position({0,0}, 100.5, 0.5, 100.5));
    assert(inputs.velocity({100,100}, 100.5, 0.5, 100.5));
    assert(inputs.wrench({0,0,0,0,0,0}, 100.5, 0.5, 100.5));
    assert(inputs.reason(0.5,100.5) == "dqAct: waiting for acceleration samples");
    near(inputs.ddq()[0],0);
    assert(inputs.velocity({100,100}, 100.51, 0.51, 100.51));
    near(inputs.ddq()[0],0);
    assert(inputs.reason(0.51,100.51).empty());
  }
  {
    Inputs inputs(cfg); warm(inputs);
    // Even a short acquisition interval cannot bridge a receive-time outage.
    assert(inputs.position({0,0}, 100.02, 0.5, 100.02));
    assert(inputs.velocity({100,100}, 100.02, 0.5, 100.02));
    assert(inputs.reason(0.5,100.02) == "dqAct: waiting for acceleration samples");
    near(inputs.ddq()[0],0);
  }
  {
    Inputs inputs(cfg); warm(inputs);
    assert(inputs.position({0,0}, 100.09, 0.09, 100.09));
    assert(inputs.reason(0.09,100.09) == "input sample skew exceeded");
    assert(inputs.velocity({0,0}, 100.09, 0.09, 100.09));
    assert(inputs.reason(0.09,100.09) == "input sample skew exceeded");
    assert(inputs.wrench({0,0,0,0,0,0}, 100.09, 0.09, 100.09));
    assert(inputs.reason(0.09,100.09).empty());
    assert(inputs.reason(-1,100.09) == "qAct: timeout or clock discontinuity");
    assert(inputs.reason(0.1,99) == "qAct: timeout or clock discontinuity");
    assert(inputs.reason(0.1,101) == "qAct: timeout or clock discontinuity");
  }
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (double stamp : {0.0, -1.0, nan, 99.7, 100.1}) {
    Inputs inputs(cfg);
    assert(!inputs.position({0,0}, stamp, 0, 100));
    assert(!inputs.velocity({0,0}, stamp, 0, 100));
    assert(!inputs.wrench({0,0,0,0,0,0}, stamp, 0, 100));
    assert(!inputs.reason(0,100).empty());
  }
  {
    Inputs inputs(cfg); warm(inputs);
    assert(!inputs.position({0}, 100.02,0.02,100.02));
    assert(inputs.reason(0.02,100.02) == "qAct: invalid values");
    assert(!inputs.position({nan,0},100.02,0.02,100.02));
    assert(!inputs.velocity({0,nan},100.02,0.02,100.02));
    assert(!inputs.velocity({0},100.02,0.02,100.02));
    assert(!inputs.wrench({0,0,0,0,0,nan},100.02,0.02,100.02));
    assert(!inputs.wrench({0},100.02,0.02,100.02));
  }
  {
    Inputs inputs(cfg); warm(inputs);
    assert(!inputs.position({0,0},100.02,nan,100.02));
    assert(!inputs.velocity({0,0},100.02,0.02,nan));
  }
  {
    Inputs inputs(cfg); warm(inputs);
    assert(!inputs.position({0,0},100.02,-0.01,100.02));
    assert(inputs.reason(0.02,100.02) == "qAct: receive clock moved backwards");
  }
  {
    auto low = cfg; low.max_acceleration = {0.1,0.1};
    Inputs inputs(low); warm(inputs);
    assert(!inputs.velocity({0,-1},100.02,0.02,100.02));
    const std::string reason = inputs.reason(0.02,100.02);
    assert(reason.find("dqAct: acceleration limit exceeded (joint=1, value=-") == 0);
    assert(reason.find(" rad/s^2, limit=0.1 rad/s^2)") != std::string::npos);
    const double recorded = std::stod(reason.substr(reason.find("value=") + 6));
    assert(std::abs(recorded - inputs.ddq()[1]) < 1e-6);
    assert(inputs.velocity({1,1},100.03,0.03,100.03));
    assert(inputs.reason(0.03,100.03) == "dqAct: waiting for acceleration samples");
    assert(inputs.velocity({1,1},100.04,0.04,100.04));
    near(inputs.ddq()[0],0);
  }
  {
    auto limited = cfg; limited.max_velocity = {1,2};
    Inputs inputs(limited);
    assert(inputs.position({0,0},100,0,100));
    // Startup must not seed an excessive raw speed into the filter.
    assert(!inputs.velocity({0,2.1},100,0,100));
    assert(inputs.reason(0,100) ==
           "dqAct: velocity limit exceeded (joint=1, value=2.1 rad/s, limit=2 rad/s)");
    assert(inputs.dq().empty() && inputs.ddq().empty());
    // Recovery needs two new valid samples, as for any rejected velocity input.
    assert(inputs.velocity({0,0},100.01,0.01,100.01));
    assert(inputs.reason(0.01,100.01) == "dqAct: waiting for acceleration samples");
    assert(inputs.wrench({0,0,0,0,0,0},100.01,0.01,100.01));
    assert(inputs.velocity({0,0},100.02,0.02,100.02));
    assert(inputs.reason(0.02,100.02).empty());
    // A long acquisition gap cannot bypass the speed threshold either.
    assert(inputs.position({0,0},100.5,0.5,100.5));
    assert(!inputs.velocity({-1.1,0},100.5,0.5,100.5));
    assert(inputs.reason(0.5,100.5) ==
           "dqAct: velocity limit exceeded (joint=0, value=-1.1 rad/s, limit=1 rad/s)");
  }
  for (double sign : {-1.0,1.0}) {
    auto limited = cfg; limited.max_velocity = {1,2};
    limited.velocity_cutoff_hz = 0.001;
    Inputs inputs(limited); warm(inputs);
    // Equality is allowed in both directions and each joint has its own bound.
    assert(inputs.velocity({sign,sign*2},100.02,0.02,100.02));
    assert(std::abs(inputs.dq()[0]) < 0.001);
    assert(!inputs.velocity({sign*1.001,0},100.03,0.03,100.03));
    assert(inputs.reason(0.03,100.03).find("dqAct: velocity limit exceeded (joint=0,") == 0);
    // The tiny low-pass output must not conceal the excessive raw measurement.
    assert(std::abs(inputs.dq()[0]) < 0.001);
  }
  {
    Inputs inputs(cfg);
    const double largest = std::numeric_limits<double>::max();
    assert(inputs.position({0,0},100,0,100));
    assert(inputs.velocity({largest,0},100,0,100));
    assert(!inputs.velocity({-largest,0},100.01,0.01,100.01));
    assert(inputs.reason(0.01,100.01).find("dqAct: nonfinite velocity/acceleration estimate (joint=0,") == 0);
  }
  for (int axis = 0; axis < 6; ++axis) {
    Inputs inputs(cfg); warm(inputs);
    std::vector<double> load(6,0);
    load[axis] = cfg.wrench_limits[axis];
    assert(inputs.wrench(load,100.02,0.02,100.02));
    load[axis] = -cfg.wrench_limits[axis]-0.01;
    assert(!inputs.wrench(load,100.03,0.03,100.03));
    assert(inputs.reason(0.03,100.03) == "elbowWrench: wrench limit exceeded");
  }
  auto bad = cfg; bad.joints = 0;
  rejects([&] { Inputs ignored(bad); });
  bad = cfg; bad.timeout = 0;
  rejects([&] { Inputs ignored(bad); });
  bad = cfg; bad.max_skew = 1;
  rejects([&] { Inputs ignored(bad); });
  bad = cfg; bad.velocity_cutoff_hz = nan;
  rejects([&] { Inputs ignored(bad); });
  bad = cfg; bad.acceleration_cutoff_hz = -1;
  rejects([&] { Inputs ignored(bad); });
  bad = cfg; bad.max_acceleration = {1};
  rejects([&] { Inputs ignored(bad); });
  bad = cfg; bad.max_acceleration[0] = -1;
  rejects([&] { Inputs ignored(bad); });
  bad = cfg; bad.max_velocity = {1};
  rejects([&] { Inputs ignored(bad); });
  for (double invalid : {0.0, -1.0, nan, std::numeric_limits<double>::infinity()}) {
    bad = cfg; bad.max_velocity = {1,invalid};
    rejects([&] { Inputs ignored(bad); });
  }
  bad = cfg; bad.wrench_limits[5] = 0;
  rejects([&] { Inputs ignored(bad); });
  std::cout << "TransparencyInputs: all tests passed\n";
}
