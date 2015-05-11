#include <sstream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-bind.h>

#include <sot-state-observation/calibrate_imu.hh>


namespace sotStateObservation
{
    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN ( CalibrateImu, "CalibrateImu" );

    CalibrateImu::CalibrateImu( const std::string & inName):
        Entity(inName),
        imuSIN(0x0 , "CalibrateImu("+inName+")::input(vector)::imuIn"),
        imuSOUT(0x0 , "CalibrateImu("+inName+")::output(vector)::imuOut"),
        R_(6,6)
    {
        dynamicgraph::Vector imuVector(6);

        signalRegistration (imuSIN);
        imuSIN.setConstant(imuVector);

        signalRegistration (imuSOUT);
        imuSOUT.setConstant(imuVector);

       std::string docstring;

       //settRa
       docstring =
               "\n"
               "    Set the rotational matrix for the accelerometer \n"
               "\n";

       addCommand(std::string("setRa"),
            new
            ::dynamicgraph::command::Setter <CalibrateImu,dynamicgraph::Matrix>
               (*this, &CalibrateImu::setRa, docstring));

       //getRa
       docstring =
               "\n"
               "    Get the rotational matrix for the accelerometer \n"
               "\n";

       addCommand(std::string("getRa"),
            new
            ::dynamicgraph::command::Getter <CalibrateImu,dynamicgraph::Matrix>
               (*this, &CalibrateImu::getRa, docstring));

       //settRg
       docstring =
               "\n"
               "    Set the rotational matrix for the gyrometer \n"
               "\n";

       addCommand(std::string("setRg"),
            new
            ::dynamicgraph::command::Setter <CalibrateImu,dynamicgraph::Matrix>
               (*this, &CalibrateImu::setRg, docstring));

       //getRg
       docstring =
               "\n"
               "    Get the rotational matrix for the gyrometer \n"
               "\n";

       addCommand(std::string("getRg"),
            new
            ::dynamicgraph::command::Getter <CalibrateImu,dynamicgraph::Matrix>
               (*this, &CalibrateImu::getRg, docstring));


        imuSOUT.setFunction(boost::bind(&CalibrateImu::computeImu, this, _1, _2));

        R_.setIdentity();

    }

    CalibrateImu::~CalibrateImu()
    {
    }

    dynamicgraph::Vector& CalibrateImu::computeImu(dynamicgraph::Vector & imuOut, const int& inTime)
    {

        const stateObservation::Vector& imuIn=convertVector<stateObservation::Vector>(imuSIN.access(inTime));


        imuOut=convertVector<dynamicgraph::Vector>(R_*imuIn);
        return imuOut;

    }

}

