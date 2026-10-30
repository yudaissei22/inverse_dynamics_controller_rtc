#ifndef InverseDynamicsController_H
#define InverseDynamicsController_H

#include <memory>
#include <map>
#include <time.h>
#include <mutex>

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/CorbaNaming.h>

#include <cnoid/Body>

#include <cpp_filters/TwoPointInterpolator.h>

#include <hrpsys/idl/RobotHardwareService.hh>
#include <hrpsys/idl/HRPDataTypes.hh>

#include "InverseDynamicsControllerService_impl.h"
#include "DirectTorqueGate.h"
#include "DirectServoFeedback.h"
#include "TransparencyDynamics.h"
#include "TransparencyInputs.h"

class InverseDynamicsController : public RTC::DataFlowComponentBase{
public:
  InverseDynamicsController(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onFinalize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  bool startInverseDynamicsController();
  bool stopInverseDynamicsController();
  bool setInverseDynamicsControllerParam(const OpenHRP::InverseDynamicsControllerService::InverseDynamicsControllerParam& i_param);
  bool getInverseDynamicsControllerParam(OpenHRP::InverseDynamicsControllerService::InverseDynamicsControllerParam& i_param);

  bool beginDirectControlTransition(bool torque, double duration);
  bool markDirectGainsStarted();
  bool setDirectTorqueCommand(
			      const OpenHRP::InverseDynamicsControllerService::DirectTorqueSequence& tau,
			      double ttl);
  long getDirectControlMode();
  long getDirectServoStateMask();
  std::string getDirectControlFaultReason();
  bool directTorqueIsZero();
  bool directPositionReferenceAligned();
  void abortDirectControl();
  bool resetDirectControlAfterServoOff();
  bool startTransparency();
  bool stopTransparency();
  std::string getTransparencyStatus();

protected:
  std::mutex mutex_;
  std::unique_ptr<DirectTorqueGate> direct_gate_;
  DirectServoFeedback direct_servo_feedback_;
  bool direct_fault_reported_ = false;
  std::unique_ptr<TransparencyDynamics> transparency_dynamics_;
  std::unique_ptr<TransparencyInputs> transparency_inputs_;
  bool transparency_active_ = false;
  bool transparency_gravity_only_ = false;
  std::string transparency_error_;

  unsigned int debugLevel_;
  unsigned long long loop_;
  double dt_;

  class Ports {
  public:
    Ports();

    // Input ports
    //// Refefence
    RTC::TimedDoubleSeq m_qRef_;
    RTC::InPort<RTC::TimedDoubleSeq> m_qRefIn_;
    RTC::TimedDoubleSeq m_dqRef_;
    RTC::InPort<RTC::TimedDoubleSeq> m_dqRefIn_;
    RTC::TimedDoubleSeq m_tauRef_;
    RTC::InPort<RTC::TimedDoubleSeq> m_tauRefIn_;

    /* 表示用のtau act*/
    // RTC::TimedDoubleSeq m_tauAct_; /*TimedDoubleSeqは構造体*/
    // RTC::InPort<RTC::TimedDoubleSeq> m_tauActIn_;
    /* <> ... テンプレート引数 */
    
    RTC::TimedPoint3D m_basePosRef_; // Reference World frame
    RTC::InPort<RTC::TimedPoint3D> m_basePosRefIn_;
    RTC::TimedOrientation3D m_baseRpyRef_; // Reference World frame
    RTC::InPort<RTC::TimedOrientation3D> m_baseRpyRefIn_;
    // std::vector<RTC::TimedDoubleSeq> m_refEEWrench_; // Reference FootOrigin frame. EndEffector origin. 要素数及び順番はgaitParam_.eeNameと同じ. ロボットが受ける力
    // std::vector<std::unique_ptr<RTC::InPort<RTC::TimedDoubleSeq> > > m_refEEWrenchIn_;
    // std::vector<RTC::TimedPose3D> m_refEEPose_; // Reference World frame. 要素数及び順番はgaitParam_.eeNameと同じ
    // std::vector<std::unique_ptr<RTC::InPort<RTC::TimedPose3D> > > m_refEEPoseIn_;
    // RTC::Time refEEPoseLastUpdateTime_; // m_refEEPoseIn_のどれかに最後にdataが届いたときの、m_qRef_.tmの時刻
    //// Actual
    RTC::TimedDoubleSeq m_qAct_;
    RTC::InPort<RTC::TimedDoubleSeq> m_qActIn_;
    RTC::TimedDoubleSeq m_dqAct_;
    RTC::InPort<RTC::TimedDoubleSeq> m_dqActIn_;
    RTC::TimedOrientation3D m_actImu_; // Actual Imu World Frame. robotのgyrometerという名のRateGyroSensorの傾きを表す
    RTC::InPort<RTC::TimedOrientation3D> m_actImuIn_;
    // std::vector<RTC::TimedDoubleSeq> m_actWrench_; // Actual ForceSensor frame. ForceSensor origin. 要素数及び順番はrobot->forceSensorsと同じ. ロボットが受ける力
    // std::vector<std::unique_ptr<RTC::InPort<RTC::TimedDoubleSeq> > > m_actWrenchIn_;
    // std::vector<RTC::TimedDoubleSeq> m_actEEWrench_; // Generate World frame. EndEffector origin. 要素数及び順番はgaitParam_.eeNameと同じ. ロボットが受ける力
    // std::vector<std::unique_ptr<RTC::OutPort<RTC::TimedDoubleSeq> > > m_actEEWrenchOut_;
    // std::vector<RTC::TimedPose3D> m_actEEPose_; // Generate World frame. 要素数及び順番はgaitParam_.eeNameと同じ
    // std::vector<std::unique_ptr<RTC::OutPort<RTC::TimedPose3D> > > m_actEEPoseOut_;

    // Output ports
    RTC::TimedDoubleSeq m_tau_;
    RTC::OutPort<RTC::TimedDoubleSeq> m_tauOut_;
    // std::vector<RTC::TimedDoubleSeq> m_tgtEEWrench_; // Generate World frame. EndEffector origin. 要素数及び順番はgaitParam_.eeNameと同じ. ロボットが受ける力
    // std::vector<std::unique_ptr<RTC::OutPort<RTC::TimedDoubleSeq> > > m_tgtEEWrenchOut_;

    // Service Ports
    InverseDynamicsControllerService_impl m_service0_;
    RTC::CorbaPort m_InverseDynamicsControllerServicePort_;
    // RTC::CorbaConsumer<OpenHRP::RobotHardwareService> m_robotHardwareService0_;
    // RTC::CorbaPort m_RobotHardwareServicePort_;

    RTC::TimedDoubleSeq m_qServoRef_;
    RTC::InPort<RTC::TimedDoubleSeq> m_qServoRefIn_;
    OpenHRP::TimedLongSeqSeq m_servoState_;
    RTC::InPort<OpenHRP::TimedLongSeqSeq> m_servoStateIn_;
    RTC::TimedDoubleSeq m_elbowWrench_, m_tauModel_, m_tauHuman_, m_ddqMeasured_, m_elbowWrenchOutData_;
    RTC::InPort<RTC::TimedDoubleSeq> m_elbowWrenchIn_;
    RTC::OutPort<RTC::TimedDoubleSeq> m_tauModelOut_, m_tauHumanOut_, m_ddqMeasuredOut_, m_elbowWrenchOut_;
  };
  Ports ports_;

  class ControlMode{
  public:
    /*
      MODE_IDLE -> startInverseDynamicsController() -> MODE_SYNC_TO_IDC -> MODE_IDC -> stopInverseDynamicsController() -> MODE_SYNC_TO_IDLE -> MODE_IDLE
      MODE_SYNC_TO*の時間はtransition_timeの時間をかけて遷移するが、少なくとも1周期はMODE_SYNC_TO*を経由する.
      MODE_SYNC_TO*では、基本的に次のMODEと同じ処理が行われるが、出力時に前回のMODEの出力から補間するような軌道に加工されることで出力の連続性を確保する
      補間している途中で別のmodeに切り替わることは無いので、そこは安心してプログラムを書いてよい(例外はonActivated). 同様に、remainTimeが突然減ったり増えたりすることもない
     */
    enum Mode_enum{ MODE_IDLE, MODE_SYNC_TO_IDC, MODE_IDC, MODE_SYNC_TO_IDLE};
    enum Transition_enum{ START_IDC, STOP_IDC};
    double idc_start_transition_time, idc_stop_transition_time;
  private:
    Mode_enum current, previous, next;
    double remain_time;
  public:
    ControlMode(){ reset(); idc_start_transition_time = 2.0; idc_stop_transition_time = 2.0;}
    void reset(){ current = previous = next = MODE_IDLE; remain_time = 0;}
    bool setNextTransition(const Transition_enum request){
      switch(request){
      case START_IDC:
        if(current == MODE_IDLE){ next = MODE_SYNC_TO_IDC; return true; }else{ return false; }
      case STOP_IDC:
        if(current == MODE_IDC){ next = MODE_SYNC_TO_IDLE; return true; }else{ return false; }
      default:
        return false;
      }
    }
    void update(double dt){
      if(current != next) {
        previous = current; current = next;
        switch(current){
        case MODE_SYNC_TO_IDC:
          remain_time = idc_start_transition_time; break;
        case MODE_SYNC_TO_IDLE:
          remain_time = idc_stop_transition_time; break;
        default:
          break;
        }
      }else{
        previous = current;
        remain_time -= dt;
        if(remain_time <= 0.0){
          remain_time = 0.0;
          switch(current){
          case MODE_SYNC_TO_IDC:
            current = next = MODE_IDC; break;
          case MODE_SYNC_TO_IDLE:
            current = next = MODE_IDLE; break;
          default:
            break;
          }
        }
      }
    }
    double remainTime() const{ return remain_time;}
    Mode_enum now() const{ return current; }
    Mode_enum pre() const{ return previous; }
    bool isIDCRunning() const{ return (current==MODE_SYNC_TO_IDC) || (current==MODE_IDC) || (current==MODE_SYNC_TO_IDLE);}
    bool isSyncToIDC() const{ return current==MODE_SYNC_TO_IDC;}
    bool isSyncToIDCInit() const{ return (current != previous) && isSyncToIDC();}
    bool isSyncToIdle() const{ return current==MODE_SYNC_TO_IDLE;}
    bool isSyncToIdleInit() const{ return (current != previous) && isSyncToIdle();}
  };
  ControlMode mode_;
  cpp_filters::TwoPointInterpolator<double> idleToIdcTransitionInterpolator_ = cpp_filters::TwoPointInterpolator<double>(0.0, 0.0, 0.0, cpp_filters::LINEAR);

  cnoid::BodyPtr actRobot; // actual (sensor frame)
  cnoid::BodyPtr refRobot; // reference (reference frame)

  protected:
  // utility functions
  bool getProperty(const std::string& key, std::string& ret); /*参照、呼び出しもとの変数をcopyせず、そのまま使います*/
  void writeDirectTorqueOutput(const std::vector<double>& torque);
  bool initializeTransparency();
  void executeTransparency(double now);

  static bool readInPortData(const double& dt, const InverseDynamicsController::ControlMode& mode, InverseDynamicsController::Ports& ports, cnoid::BodyPtr refRobot, cnoid::BodyPtr actRobot);
  static bool execInverseDynamicsController(const InverseDynamicsController::ControlMode& mode, const double dt, cnoid::BodyPtr refRobot, cnoid::BodyPtr actRobot);
  static bool writeOutPortData(const double& dt, const InverseDynamicsController::ControlMode& mode, InverseDynamicsController::Ports& ports, cnoid::BodyPtr refRobot, cnoid::BodyPtr actRobot, cpp_filters::TwoPointInterpolator<double>& idleToIdcTransitionInterpolator);
};


extern "C"
{
  void InverseDynamicsControllerInit(RTC::Manager* manager);
};

#endif // InverseDynamicsController_H
