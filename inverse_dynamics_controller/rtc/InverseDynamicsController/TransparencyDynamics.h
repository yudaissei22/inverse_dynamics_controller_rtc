#pragma once // ヘッダを二重に読み込ままいようにするもお
　　　
#include <cnoid/Body>
#include <cnoid/EigenUtil>
#include <cnoid/src/Body/InverseDynamics.h>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

// Fixed-base, measured-state inverse dynamics. Wrench measurements are used
// only for the contact-torque diagnostic and never enter the motor command.
// All joint vectors follow Body::joint(i), i.e. model / RobotHardware joint IDs.
class TransparencyDynamics {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW /* eigen型の行列・ベクトルが必要とするメモリ配置を確保するためのマクロ */
  using Wrench = std::array<double, 6>;  /* wrench用の配列の作成 */

  /* 力センサーの取り付け位置・向き・重力など */
  struct Config {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::string sensor_link; /* センサーがついているリンクの名前 */
    cnoid::Vector3 sensor_translation = cnoid::Vector3::Zero(); /* 取り付けリンク位置から見た、センサ原点の位置 */
    cnoid::Vector3 sensor_rpy = cnoid::Vector3::Zero(); /* センサー座標からみたリンク座標系への回転位置 */
    cnoid::Vector3 gravity = cnoid::Vector3(0.0, 0.0, -9.80665); // World acceleration [m/s^2].
    double wrench_sign = 1.0; // +1 when the sensor reports human-on-robot wrench.
    Wrench wrench_bias = {{0, 0, 0, 0, 0, 0}}; // Raw sensor coordinates, removed before sign.
    std::vector<double> viscous; /* 各関節の粘性摩擦のこと [Nm s/rad]*/
    std::vector<double> coulomb; /* クーロン摩擦で運動方向に対してほぼ一定 [Nm] */
    double friction_velocity_scale = 0.01; // tanh smoothing [rad/s], or [m/s].
    bool gravity_only = false; // Commissioning: g(q), without motion/friction compensation.
  };

  /*
    計算結果の入れ物を用意しエチル
  */
  struct Result {
    std::vector<double> torque; // M(q)ddq + C(q,dq)dq + g(q) + friction.
    std::vector<double> human_torque; // J_sensor(q)^T * human-on-robot wrench.
    Wrench contact_wrench; // Bias-corrected, signed [Fx,Fy,Fz,Mx,My,Mz] in sensor frame.
  };


  /*
   初期化時にロボットモデルを確認し、複製する.

   クラスと同じ名前の関数で、オブジェクトを作るときに実行されるコンストラクタ
   const 型& 変数名 ... 渡された値をコピーせず、読み取り専用の参照として値そのものを受け取る
   : config_(config) ... 本体の処理より前に、メンバ変数config_を初期化
   -> ... ポインタを通して、メンバにアクセス
   */
  TransparencyDynamics(const cnoid::BodyPtr& model, const Config& config)
      : config_(config) {
    if (!model || !model->rootLink() || !model->rootLink()->isFixedJoint() ||
        model->numJoints() == 0)
      throw std::invalid_argument("transparency requires a fixed-base model with joints");
    body_ = model->clone();
    sensor_link_ = body_->link(config.sensor_link);
    if (!sensor_link_) throw std::invalid_argument("unknown transparency sensor link");
    if (!config.sensor_translation.allFinite() || !config.sensor_rpy.allFinite() ||
        !config.gravity.allFinite() ||
        (config.wrench_sign != 1.0 && config.wrench_sign != -1.0) ||
        !std::isfinite(config.friction_velocity_scale) || config.friction_velocity_scale <= 0)
      throw std::invalid_argument("invalid transparency frame or friction scale");
    validateJointVector(config.viscous, "viscous friction");
    validateJointVector(config.coulomb, "Coulomb friction");
    for (int i = 0; i < body_->numJoints(); ++i) {
      const cnoid::Link* joint = body_->joint(i);
      if (joint->jointId() != i || (!joint->isRevoluteJoint() && !joint->isPrismaticJoint()))
        throw std::invalid_argument("transparency requires contiguous revolute/prismatic joint IDs");
      if (config.viscous[i] < 0 || config.coulomb[i] < 0)
        throw std::invalid_argument("negative transparency friction coefficient");
    }
    for (double value : config.wrench_bias)
      if (!std::isfinite(value)) throw std::invalid_argument("nonfinite wrench bias");
    for (int i = 0; i < body_->numLinks(); ++i) {
      const cnoid::Link* link = body_->link(i);
      if (!std::isfinite(link->m()) || link->m() < 0 || !link->c().allFinite() ||
          !link->I().allFinite() || !std::isfinite(link->Jm2()) || link->Jm2() < 0)
        throw std::invalid_argument("invalid transparency model inertial properties");
    }
    sensor_rotation_ = cnoid::rotFromRpy(config.sensor_rpy);
    if (!body_->rootLink()->T().matrix().allFinite())
      throw std::invalid_argument("invalid transparency base pose");
  }

  Result calculate(const std::vector<double>& q,
                   const std::vector<double>& dq,
                   const std::vector<double>& ddq,
                   const Wrench& wrench) {
    validateJointVector(q, "q");
    validateJointVector(dq, "dq");
    validateJointVector(ddq, "ddq");
    Result result;
    for (size_t i = 0; i < wrench.size(); ++i) {
      result.contact_wrench[i] = config_.wrench_sign * (wrench[i] - config_.wrench_bias[i]);
      if (!std::isfinite(result.contact_wrench[i]))
        throw std::invalid_argument("invalid transparency contact wrench");
    }
    // Explicitly overwrite state from a complete actual-state sample. No
    // reference trajectory or previously computed joint acceleration is used.
    for (int i = 0; i < body_->numJoints(); ++i) {
      cnoid::Link* joint = body_->joint(i);
      joint->q() = q[i];
      joint->dq() = config_.gravity_only ? 0.0 : dq[i];
      joint->ddq() = config_.gravity_only ? 0.0 : ddq[i];
    }
    for (int i = 0; i < body_->numLinks(); ++i) body_->link(i)->F_ext().setZero();
    cnoid::Link* base = body_->rootLink();
    base->v().setZero();
    base->w().setZero();
    base->dv() = -config_.gravity; // Fictitious acceleration for gravity compensation.
    base->dw().setZero();
    body_->calcForwardKinematics(true, true);
    cnoid::calcInverseDynamics(base);
    result.torque.resize(body_->numJoints());
    for (int i = 0; i < body_->numJoints(); ++i) {
      result.torque[i] = body_->joint(i)->u();
      if (!config_.gravity_only)
        result.torque[i] += config_.viscous[i] * dq[i] +
            config_.coulomb[i] * std::tanh(dq[i] / config_.friction_velocity_scale);
      if (!std::isfinite(result.torque[i]))
        throw std::runtime_error("nonfinite transparency inverse dynamics torque");
    }

    // J^T W at the sensor origin. Rotating both vectors alone is not enough:
    // each revolute joint also receives the moment arm crossed with force.
    const cnoid::Matrix3 world_rotation = sensor_link_->R() * sensor_rotation_;
    const cnoid::Vector3 sensor_position = sensor_link_->p() +
        sensor_link_->R() * config_.sensor_translation;
    const cnoid::Vector3 force = world_rotation * cnoid::Vector3(
        result.contact_wrench[0], result.contact_wrench[1], result.contact_wrench[2]);
    const cnoid::Vector3 moment = world_rotation * cnoid::Vector3(
        result.contact_wrench[3], result.contact_wrench[4], result.contact_wrench[5]);
    result.human_torque.assign(body_->numJoints(), 0.0);
    for (const cnoid::Link* link = sensor_link_; link; link = link->parent()) {
      const int id = link->jointId();
      if (id < 0) continue;
      const cnoid::Vector3 axis = link->R() * link->jointAxis();
      if (link->isRevoluteJoint())
        result.human_torque[id] = axis.dot(moment + (sensor_position - link->p()).cross(force));
      else if (link->isPrismaticJoint())
        result.human_torque[id] = axis.dot(force);
      if (!std::isfinite(result.human_torque[id]))
        throw std::runtime_error("nonfinite transparency human torque");
    }
    return result;
  }

private:
  void validateJointVector(const std::vector<double>& values, const char* label) const {
    if (values.size() != static_cast<size_t>(body_->numJoints()))
      throw std::invalid_argument(std::string("incorrect transparency vector size: ") + label);
    for (double value : values)
      if (!std::isfinite(value))
        throw std::invalid_argument(std::string("nonfinite transparency vector: ") + label);
  }

  Config config_;
  cnoid::BodyPtr body_;
  cnoid::Link* sensor_link_ = nullptr;
  cnoid::Matrix3 sensor_rotation_;
};
