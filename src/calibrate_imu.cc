#include <sstream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-bind.h>

#include <math.h>

#include <sot-state-observation/calibrate_imu.hh>


namespace sotStateObservation
{
    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN ( CalibrateImu, "CalibrateImu" );

    CalibrateImu::CalibrateImu( const std::string & inName):
        Entity(inName),
        imuSIN(0x0 , "CalibrateImu("+inName+")::input(vector)::imuIn"),
        imuSOUT(0x0 , "CalibrateImu("+inName+")::output(vector)::imuOut"),
        R_(6,6), calibrate_(false), sumImuIn_(6), nbStep_(0), currentStep_(0)
    {
        dynamicgraph::Vector imuVector(6);
        imuVector.setZero();

        signalRegistration (imuSIN);
        imuSIN.setConstant(imuVector);

        signalRegistration (imuSOUT);
        imuSOUT.setConstant(imuVector);

        sumImuIn_.setZero();
        R_.setIdentity();

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


       docstring  =
               "\n"
               "    Start the calibration \n"
               "\n";

       addCommand(std::string("startCalibration"),
            new
            ::dynamicgraph::command::Setter <CalibrateImu,int>
               (*this, &CalibrateImu::startCalibration, docstring));


        imuSOUT.setFunction(boost::bind(&CalibrateImu::computeImu, this, _1, _2));

    }

    CalibrateImu::~CalibrateImu()
    {
    }

    stateObservation::Matrix3 Rdetermination(stateObservation::Vector3 v)
    {
        stateObservation::Vector3 ez;
        ez <<   0,
                0,
                1;

        stateObservation::Vector3 crossProduct;
        double scalarProduct;
        crossProduct=v.cross(ez);
        scalarProduct=v.dot(ez);

        std::cout << "crossProduct: " << crossProduct.transpose() << " scalarProduct: " << scalarProduct/v.norm() << std::endl;

        double angle=acos(scalarProduct/v.norm());
        stateObservation::Vector3 axis=crossProduct/crossProduct.norm();
        stateObservation::AngleAxis utheta=stateObservation::AngleAxis(angle,axis);

        std::cout << "angle: " << angle << " axis: " << axis << std::endl;

        return utheta.toRotationMatrix();
    }

    void CalibrateImu::calibrate()
    {
        stateObservation::Vector meanImuIn;
        meanImuIn=sumImuIn_/nbStep_;

        std::cout << "sumImu" << sumImuIn_ << std::endl;
        std::cout << "meanImu" << meanImuIn << std::endl;

        // determination of Ra
        R_.block(0,0,3,3)=Rdetermination(meanImuIn.block(0,0,3,1));
        // determination of Rg
        R_.block(3,3,3,3)=Rdetermination(meanImuIn.block(3,0,3,1));

        calibrate_=false;
    }

    dynamicgraph::Vector& CalibrateImu::computeImu(dynamicgraph::Vector & imuOut, const int& inTime)
    {
        const stateObservation::Vector& imuIn=convertVector<stateObservation::Vector>(imuSIN.access(inTime));

        if(calibrate_==true && currentStep_ < nbStep_)
        {
            sumImuIn_+=imuIn;
            currentStep_++;
            std::cout << "calibration time= " << currentStep_ << std::endl;
        }
        else if(calibrate_==true && currentStep_ == nbStep_)
        {
            calibrate();
        }

        stateObservation::Vector prod=R_*imuIn;
        imuOut=convertVector<dynamicgraph::Vector>(prod);
        return imuOut;
    }
}

