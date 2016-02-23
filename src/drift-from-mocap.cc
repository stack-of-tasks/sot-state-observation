#include <sot-state-observation/drift-from-mocap.hh>

#include <dynamic-graph/command-bind.h>
#include <dynamic-graph/factory.h>
#include <jrl/mal/matrixabstractlayervector3jrlmath.hh>
#include <sot/core/vector-utheta.hh>

namespace sotStateObservation
{
  using dynamicgraph::command::makeCommandVoid0;
  using dynamicgraph::command::docCommandVoid0;

  DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN ( DriftFromMocap, "DriftFromMocap" );

  DriftFromMocap::DriftFromMocap(const std::string& inName):
    Entity(inName),
    limbGlobalSIN(0x0 , "DriftFromMocap("+inName+")::input(MatrixHomogeneous)::limbGlobal"),
    limbLocalSIN(0x0 , "DriftFromMocap("+inName+")::input(MatrixHomogeneous)::limbLocal"),
    driftSOUT( limbGlobalSIN<<limbLocalSIN,
             "DriftFromMocap("+inName+")::output(MatrixHomogeneous)::drift")
  {
    signalRegistration (limbGlobalSIN);
    signalRegistration (limbLocalSIN);
    signalRegistration (driftSOUT);

    driftSOUT.setFunction(boost::bind(&DriftFromMocap::computedrift,
                                    this, _1, _2));

    std::string docstring;

    addCommand ("init",
                makeCommandVoid0 (*this, &DriftFromMocap::init,
                                  docCommandVoid0 ("reset the drift to zero")));
    init_.setIdentity();

  }

  DriftFromMocap::~DriftFromMocap()
  {}

  void DriftFromMocap::init()
  {
    int i= driftSOUT.getTime();
    const ::dynamicgraph::sot::MatrixHomogeneous & limbGlobal = limbGlobalSIN(i+1);
    const ::dynamicgraph::sot::MatrixHomogeneous & limbLocal = limbLocalSIN(i+1);

    init_ = limbLocal * limbGlobal.inverse();


  }

  ::dynamicgraph::sot::MatrixHomogeneous& DriftFromMocap::computedrift
              (::dynamicgraph::sot::MatrixHomogeneous & drift, const int& inTime)
  {
    const ::dynamicgraph::sot::MatrixHomogeneous & limbGlobal = limbGlobalSIN(inTime);
    const ::dynamicgraph::sot::MatrixHomogeneous & limbLocal = limbLocalSIN(inTime);

    drift = init_ * limbGlobal * limbLocal.inverse();

    return drift;
  }


}
