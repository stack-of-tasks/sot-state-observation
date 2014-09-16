#include <sstream>

#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-direct-setter.h>
#include <dynamic-graph/command-direct-getter.h>
#include <dynamic-graph/command-bind.h>



#include <dynamic-graph/factory.h>
#include <jrl/mal/matrixabstractlayervector3jrlmath.hh>
#include <sot/core/vector-utheta.hh>

#include <sot-state-observation/moving-frame-transformation.hh>


namespace sotStateObservation
{
  using dynamicgraph::command::makeCommandVoid0;
  using dynamicgraph::command::docCommandVoid0;
  using dynamicgraph::command::docDirectSetter;
  using dynamicgraph::command::makeDirectSetter;
  using dynamicgraph::command::docDirectGetter;
  using dynamicgraph::command::makeDirectGetter;

  DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN ( MovingFrameTransformation,
                                       "MovingFrameTransformation" );

  MovingFrameTransformation::MovingFrameTransformation
  ( const std::string & inName):
    Entity(inName),
    gMlSIN(0x0 , "MovingFrameTransformation("+inName+")::input(MatrixHomogeneous)::gMl"),
    gVlSIN(0x0 , "MovingFrameTransformation("+inName+")::input(vector)::gVl"),
    lM0SIN(0x0 , "MovingFrameTransformation("+inName+")::input(MatrixHomogeneous)::lM0"),
    lV0SIN(0x0 , "MovingFrameTransformation("+inName+")::input(vector)::lV0"),
    gM0SOUT( gMlSIN<<lM0SIN,
             "MovingFrameTransformation("+inName+")::output(vector)::gM0"),
    gV0SOUT( gMlSIN<<gVlSIN<<lM0SIN<<lV0SIN,
             "MovingFrameTransformation("+inName+")::output(vector)::gV0"),
    yawRemoved_(false)
  {
    signalRegistration (gMlSIN);
    signalRegistration (gVlSIN);
    signalRegistration (lM0SIN);
    signalRegistration (lV0SIN);
    signalRegistration (gM0SOUT);
    signalRegistration (gV0SOUT);

    dynamicgraph::Vector velocity (6);
    dynamicgraph::Matrix homoMatrix(4,4);

    homoMatrix.setIdentity();

    gMlSIN.setConstant(homoMatrix);
    gVlSIN.setConstant(velocity);
    lM0SIN.setConstant(homoMatrix);
    lV0SIN.setConstant (velocity);
    gM0SOUT.setConstant(homoMatrix);
    gV0SOUT.setConstant(velocity);


    gM0SOUT.setFunction(boost::bind(&MovingFrameTransformation::computegM0,
                                    this, _1, _2));

    gV0SOUT.setFunction(boost::bind(&MovingFrameTransformation::computegV0,
                                    this, _1, _2));

    addCommand ("setYawRemoved",
                makeDirectSetter (*this, &yawRemoved_,
                                  docDirectSetter
                                  ("Sets if yes or no, the yaw component gMl anv gVl should be removed",
                                   "bool")));
    addCommand ("getYawRemoved",
                makeDirectGetter (*this, &yawRemoved_,
                                  docDirectGetter
                                  ("Gets if yes or no, the yaw component of gMl anv gVl is be removed",
                                   "bool")));

    std::string docstring;
  }

  void removeYaw(::dynamicgraph::sot::MatrixHomogeneous & M)
  {
    dynamicgraph::sot::VectorUTheta v;

    dynamicgraph::sot::MatrixRotation R;
    dynamicgraph::Vector p(3);
    M.extract(p);
    M.extract(R);
    v.fromMatrix(R);
    v(2)=0;
    v.toMatrix(R);

    M.buildFrom(R,p);

  }

  MovingFrameTransformation::~MovingFrameTransformation()
  {
  }

  dynamicgraph::Vector& MovingFrameTransformation::computegV0
  (dynamicgraph::Vector & velocity, const int& inTime)
  {
    dynamicgraph::sot::MatrixHomogeneous gMl (gMlSIN(inTime));
    dynamicgraph::sot::MatrixHomogeneous lM0 (lM0SIN(inTime));

    dynamicgraph::Vector gVl (gVlSIN(inTime));
    dynamicgraph::Vector lV0 (lV0SIN(inTime));

    if (yawRemoved_)
    {

        gVl(2) =0;
        removeYaw(gMl);
    }

    dynamicgraph::Matrix gRl(3,3);

    dynamicgraph::Vector lT0(3);

    dynamicgraph::Vector gOmegal(3);

    gMl.extract(gRl);

    lM0.extract(lT0);

    gOmegal = getSubvector(gVl,3,3);

    dynamicgraph::Vector omega = gRl * getSubvector(lV0,3,3) + gOmegal;


    dynamicgraph::Vector tdot = crossProduct(gOmegal , gRl * lT0)
                                + gRl * getSubvector(lV0,0,3)
                                + getSubvector(gVl,0,3);

    velocity.resize(6);
    setSubvector(velocity, 0, tdot );
    setSubvector(velocity, 3, omega);

    return velocity;

  }

  ::dynamicgraph::sot::MatrixHomogeneous& MovingFrameTransformation::computegM0
  (::dynamicgraph::sot::MatrixHomogeneous & homo, const int& inTime)
  {
    ::dynamicgraph::sot::MatrixHomogeneous lM0 (lM0SIN(inTime));
    ::dynamicgraph::sot::MatrixHomogeneous gMl (gMlSIN(inTime));


    if (yawRemoved_)
    {
        removeYaw(gMl);
    }

    homo = gMl * lM0;

    //std::cout << "computegM0" << std::endl;

    return homo;
  }



}
