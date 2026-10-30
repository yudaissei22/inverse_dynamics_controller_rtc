// Analytic mechanics tests, with no ROS, CORBA, motor or sensor connection.
#ifdef NDEBUG
#undef NDEBUG
#endif
#include "TransparencyDynamics.h"
#include <cnoid/BodyLoader>
#include <cnoid/ForceSensor>
#include <cassert>
#include <iostream>
#include <limits>

namespace {
const double pi = std::acos(-1.0);
const TransparencyDynamics::Wrench zero_wrench = {{0, 0, 0, 0, 0, 0}};

void near(double actual, double expected, double tolerance = 1e-10) {
  if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance) {
    std::cerr << "actual " << actual << ", expected " << expected << std::endl;
    assert(false);
  }
}

cnoid::BodyPtr baseBody() {
  cnoid::BodyPtr body = new cnoid::Body;
  cnoid::Link* root = new cnoid::Link;
  root->setName("base");
  root->setJointType(cnoid::Link::FIXED_JOINT);
  root->setInertia(cnoid::Matrix3::Zero());
  body->setRootLink(root);
  return body;
}

cnoid::Link* addJoint(cnoid::BodyPtr body, cnoid::Link* parent,
                     const std::string& name, int id, const cnoid::Vector3& axis,
                     const cnoid::Vector3& offset, double mass,
                     const cnoid::Vector3& center, double inertia) {
  cnoid::Link* joint = new cnoid::Link;
  joint->setName(name);
  joint->setJointId(id);
  joint->setJointType(cnoid::Link::REVOLUTE_JOINT);
  joint->setJointAxis(axis);
  joint->setOffsetTranslation(offset);
  joint->setMass(mass);
  joint->setCenterOfMass(center);
  joint->setInertia(inertia * cnoid::Matrix3::Identity());
  parent->appendChild(joint);
  body->updateLinkTree();
  return joint;
}

TransparencyDynamics::Config configuration(cnoid::BodyPtr body, const std::string& sensor) {
  TransparencyDynamics::Config config;
  config.sensor_link = sensor;
  config.viscous.assign(body->numJoints(), 0);
  config.coulomb.assign(body->numJoints(), 0);
  return config;
}

template<class Operation> void rejects(Operation operation) {
  bool rejected = false;
  try { operation(); } catch (const std::exception&) { rejected = true; }
  assert(rejected);
}
}

int main(int argc, char** argv) {
  // A single hanging pendulum: tau = (I+m*l^2+Jm2)*ddq + m*g*l*sin(q).
  auto pendulum = baseBody();
  auto joint = addJoint(pendulum, pendulum->rootLink(), "pendulum", 0,
      cnoid::Vector3::UnitY(), cnoid::Vector3::Zero(), 2.0,
      cnoid::Vector3(0, 0, -0.4), 0.05);
  joint->setEquivalentRotorInertia(0.02);
  auto config = configuration(pendulum, "pendulum");
  config.viscous[0] = 0.2;
  config.coulomb[0] = 0.3;
  config.friction_velocity_scale = 0.1;
  TransparencyDynamics dynamics(pendulum, config);
  const double q = 0.4, dq = 0.7, ddq = 2.3;
  const double expected = (0.05 + 2 * 0.4 * 0.4 + 0.02) * ddq +
      2 * 9.80665 * 0.4 * std::sin(q) + 0.2 * dq + 0.3 * std::tanh(dq / 0.1);
  auto result = dynamics.calculate({q}, {dq}, {ddq}, zero_wrench);
  near(result.torque[0], expected);
  near(result.human_torque[0], 0);
  // Clone isolation: caller state and model are not changed by the calculator.
  near(joint->q(), 0);
  near(joint->dq(), 0);
  near(joint->u(), 0);
  near(dynamics.calculate({0}, {0}, {0}, zero_wrench).torque[0], 0);
  near(dynamics.calculate({pi / 2}, {0}, {0}, zero_wrench).torque[0], 2 * 9.80665 * 0.4);
  config.gravity_only = true;
  TransparencyDynamics gravity_only(pendulum, config);
  // Even with nonzero velocity/acceleration/friction settings, g(q) alone.
  near(gravity_only.calculate({q}, {dq}, {ddq}, zero_wrench).torque[0],
       2 * 9.80665 * 0.4 * std::sin(q));

  // Planar two-link arm. Independent closed-form M and Coriolis equations
  // exercise the off-diagonal inertia and velocity coupling missing from g(q).
  auto arm = baseBody();
  auto a = addJoint(arm, arm->rootLink(), "a", 0, cnoid::Vector3::UnitZ(),
      cnoid::Vector3::Zero(), 1.3, cnoid::Vector3(0.3, 0, 0), 0.08);
  addJoint(arm, a, "b", 1, cnoid::Vector3::UnitZ(),
      cnoid::Vector3(0.7, 0, 0), 0.9, cnoid::Vector3(0.2, 0, 0), 0.04);
  auto cfg = configuration(arm, "b");
  cfg.gravity.setZero();
  TransparencyDynamics two_link(arm, cfg);
  const double q1 = 0.3, q2 = -0.6, v1 = 1.1, v2 = -0.8, a1 = 0.7, a2 = -1.2;
  const double coupling = 0.9 * 0.7 * 0.2;
  const double m11 = 0.08 + 0.04 + 1.3 * 0.3 * 0.3 +
      0.9 * (0.7 * 0.7 + 0.2 * 0.2) + 2 * coupling * std::cos(q2);
  const double m12 = 0.04 + 0.9 * 0.2 * 0.2 + coupling * std::cos(q2);
  const double m22 = 0.04 + 0.9 * 0.2 * 0.2;
  auto two = two_link.calculate({q1, q2}, {v1, v2}, {a1, a2}, zero_wrench);
  near(two.torque[0], m11 * a1 + m12 * a2 - coupling * std::sin(q2) * (2*v1*v2 + v2*v2));
  near(two.torque[1], m12 * a1 + m22 * a2 + coupling * std::sin(q2) * v1*v1);

  // Sensor at x=.2 on b, rotated +90 deg. Sensor Fx becomes world Fy.
  // Raw 11 N minus 1 N bias => 10 N: moments are 9 and 2 Nm plus 3 Nm.
  cfg.sensor_translation = cnoid::Vector3(0.2, 0, 0);
  cfg.sensor_rpy = cnoid::Vector3(0, 0, pi / 2);
  cfg.wrench_bias[0] = 1.0;
  TransparencyDynamics with_sensor(arm, cfg);
  auto contact = with_sensor.calculate({0, 0}, {0, 0}, {0, 0}, {{11, 0, 0, 0, 0, 3}});
  near(contact.contact_wrench[0], 10);
  near(contact.human_torque[0], 12);
  near(contact.human_torque[1], 5);
  near(contact.torque[0], 0); // Contact wrench is NOT subtracted in inverse dynamics.
  near(contact.torque[1], 0);
  contact = with_sensor.calculate({pi/2, -pi/2}, {0,0}, {0,0}, {{11,0,0,0,0,3}});
  near(contact.human_torque[0], 5); // Shoulder rotation moves elbow to y=.7.
  near(contact.human_torque[1], 5);
  // Simultaneous nonzero dynamics and arbitrary wrench: same motor command.
  auto loaded = with_sensor.calculate({q1,q2}, {v1,v2}, {a1,a2}, {{11,2,3,4,5,6}});
  near(loaded.torque[0], two.torque[0]);
  near(loaded.torque[1], two.torque[1]);
  cfg.wrench_sign = -1;
  TransparencyDynamics opposite_sensor(arm, cfg);
  contact = opposite_sensor.calculate({0,0}, {0,0}, {0,0}, {{11,0,0,0,0,3}});
  near(contact.human_torque[0], -12);
  near(contact.human_torque[1], -5);

  // Upstream mounting must leave downstream joint torque zero.
  cfg = configuration(arm, "a");
  cfg.sensor_translation = cnoid::Vector3(0.2, 0, 0);
  TransparencyDynamics upstream(arm, cfg);
  contact = upstream.calculate({0,0}, {0,0}, {0,0}, {{0,10,0,0,0,3}});
  near(contact.human_torque[0], 5);
  near(contact.human_torque[1], 0);

  // Prismatic projection uses axis dot force; moment is irrelevant.
  auto slide = baseBody();
  auto slider = addJoint(slide, slide->rootLink(), "slide", 0,
      cnoid::Vector3::UnitX(), cnoid::Vector3::Zero(), 2,
      cnoid::Vector3::Zero(), 0.1);
  slider->setJointType(cnoid::Link::PRISMATIC_JOINT);
  auto sc = configuration(slide, "slide");
  sc.gravity = cnoid::Vector3(-9.80665, 0, 0);
  TransparencyDynamics sliding(slide, sc);
  auto sr = sliding.calculate({0.3}, {0.5}, {1.2}, {{3,4,5,100,200,300}});
  near(sr.torque[0], 2 * (9.80665 + 1.2));
  near(sr.human_torque[0], 3);

  const double nan = std::numeric_limits<double>::quiet_NaN();
  rejects([&] { dynamics.calculate({}, {0}, {0}, zero_wrench); });
  rejects([&] { dynamics.calculate({nan}, {0}, {0}, zero_wrench); });
  rejects([&] { dynamics.calculate({0}, {nan}, {0}, zero_wrench); });
  rejects([&] { dynamics.calculate({0}, {0}, {nan}, zero_wrench); });
  rejects([&] { dynamics.calculate({0}, {0}, {0}, {{nan,0,0,0,0,0}}); });
  auto bad = config;
  bad.sensor_link = "missing";
  rejects([&] { TransparencyDynamics rejected(pendulum, bad); });
  bad = config; bad.coulomb[0] = -1;
  rejects([&] { TransparencyDynamics rejected(pendulum, bad); });
  bad = config; bad.viscous.clear();
  rejects([&] { TransparencyDynamics rejected(pendulum, bad); });
  bad = config; bad.sensor_rpy[0] = nan;
  rejects([&] { TransparencyDynamics rejected(pendulum, bad); });
  bad = config; bad.friction_velocity_scale = 0;
  rejects([&] { TransparencyDynamics rejected(pendulum, bad); });
  bad = config; bad.wrench_sign = 0;
  rejects([&] { TransparencyDynamics rejected(pendulum, bad); });
  pendulum->rootLink()->setJointType(cnoid::Link::FREE_JOINT);
  rejects([&] { TransparencyDynamics rejected(pendulum, config); });
  // Optional workspace-model regression. Loading does not modify the model.
  // Compare g(q) with the numerical gradient of gravitational potential,
  // independently of Choreonoid's inverse dynamics implementation.
  if (argc > 1) {
    cnoid::BodyLoader loader;
    cnoid::BodyPtr model = loader.load(argv[1]);
    assert(model && model->rootLink()->isFixedJoint() && model->numJoints() == 3);
    assert(cnoid::DeviceList<cnoid::ForceSensor>(model->devices()).empty());
    const std::array<std::string,3> expected_names = {{"SHOULDER_PITCH", "SHOULDER_ROLL_YAW", "ELBOW_PITCH"}};
    for (int i = 0; i < model->numJoints(); ++i) {
      assert(model->joint(i)->name() == expected_names[i]);
      assert(model->joint(i)->m() > 0);
      std::cout << "LowDof joint " << i << ": " << model->joint(i)->name()
                << ", mass=" << model->joint(i)->m() << " kg\n";
    }
    auto real_config = configuration(model, "ELBOW_PITCH");
    TransparencyDynamics real(model, real_config);
    std::vector<double> pose = {-0.3, 0.2, 0.5}, zeros(3, 0.0);
    auto gravity_result = real.calculate(pose, zeros, zeros, zero_wrench);
    auto potential = [&]() {
      model->calcForwardKinematics();
      double energy = 0;
      for (int i = 0; i < model->numLinks(); ++i) {
        auto link = model->link(i);
        energy -= link->m() * real_config.gravity.dot(link->p() + link->R()*link->c());
      }
      return energy;
    };
    for (int i = 0; i < 3; ++i) model->joint(i)->q() = pose[i];
    for (int i = 0; i < 3; ++i) {
      const double epsilon = 1e-6;
      model->joint(i)->q() = pose[i] + epsilon;
      const double up = potential();
      model->joint(i)->q() = pose[i] - epsilon;
      const double down = potential();
      model->joint(i)->q() = pose[i];
      near(gravity_result.torque[i], (up-down)/(2*epsilon), 1e-6);
      std::cout << "LowDof gravity torque " << i << ": " << gravity_result.torque[i] << " Nm\n";
    }
    auto moving = real.calculate(pose, {0.2,-0.1,0.3}, {0.4,0.2,-0.3}, {{1,2,3,0.1,0.2,0.3}});
    for (int i = 0; i < 3; ++i) {
      assert(std::isfinite(moving.torque[i]) && std::isfinite(moving.human_torque[i]));
    }
  }
  std::cout << "TransparencyDynamics: all tests passed\n";
}
