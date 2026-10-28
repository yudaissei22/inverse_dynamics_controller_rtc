// -*-C++-*-
#ifndef InverseDynamicsControllerServiceSVC_IMPL_H
#define InverseDynamicsControllerServiceSVC_IMPL_H

#include "inverse_dynamics_controller/idl/InverseDynamicsControllerService.hh"

class InverseDynamicsController;

class InverseDynamicsControllerService_impl
  : public virtual POA_OpenHRP::InverseDynamicsControllerService,
    public virtual PortableServer::RefCountServantBase
{
public:
  InverseDynamicsControllerService_impl();// 実装は.cppファイルの方に書かないと、registerProvider時にSegmentation Faultになる
  ~InverseDynamicsControllerService_impl();
  CORBA::Boolean startInverseDynamicsController();
  CORBA::Boolean stopInverseDynamicsController();
  CORBA::Boolean beginDirectControlTransition(CORBA::Boolean torque, CORBA::Double duration);
  CORBA::Boolean markDirectGainsStarted();
  CORBA::Boolean setDirectTorqueCommand(
      const OpenHRP::InverseDynamicsControllerService::DirectTorqueSequence& tau,
      CORBA::Double ttl);
  CORBA::Long getDirectControlMode();
  CORBA::Long getDirectServoStateMask();
  char* getDirectControlFaultReason();
  CORBA::Boolean directTorqueIsZero();
  CORBA::Boolean directPositionReferenceAligned();
  void abortDirectControl();
  CORBA::Boolean resetDirectControlAfterServoOff();
  CORBA::Boolean setInverseDynamicsControllerParam(const OpenHRP::InverseDynamicsControllerService::InverseDynamicsControllerParam& i_param);
  CORBA::Boolean getInverseDynamicsControllerParam(OpenHRP::InverseDynamicsControllerService::InverseDynamicsControllerParam_out i_param);
  //
  //
  void setComp(InverseDynamicsController *i_comp);
private:
  InverseDynamicsController *comp_;
};

#endif
