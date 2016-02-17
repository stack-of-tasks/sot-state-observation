#include <sstream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-bind.h>

#include <math.h>

#include <sot-state-observation/calibrate.hh>


namespace sotStateObservation
{
    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN ( Calibrate, "Calibrate" );

    Calibrate::Calibrate( const std::string & inName):
        Entity(inName),
        imuSIN(0x0 , "Calibrate("+inName+")::input(vector)::imuIn"),
        imuSOUT(0x0 , "Calibrate("+inName+")::output(vector)::imuOut"),
        contactsPositionSIN(0x0 , "Calibrate("+inName+")::input(vector)::contactsPositionIn"),
        contactsPositionSOUT(0x0 , "Calibrate("+inName+")::output(vector)::contactsPositionOut"),
        comSIN(0x0 , "Calibrate("+inName+")::input(vector)::comIn"),
        contactsNbrSIN(0x0 , "Calibrate("+inName+")::input(unsigned)::contactsNbr"),
        R_(6,6), sumImuIn_(6), sumComIn_(3),
        tc_(6), sumContactsPositionIn_(12),
        calibrate_(false), nbStep_(0), currentStep_(0), inTime_(0), mode_(0)
    {
        dynamicgraph::Vector imuVector(6);
        imuVector.setZero();
        signalRegistration (imuSIN);
        imuSIN.setConstant(imuVector);
        signalRegistration (imuSOUT);
//        imuSOUT.setConstant(imuVector);
        sumImuIn_.setZero();
        R_.setIdentity();

        dynamicgraph::Vector contactsPositionVector(12);
        contactsPositionVector.setZero();
        signalRegistration (contactsPositionSIN);
        contactsPositionSIN.setConstant(contactsPositionVector);
        signalRegistration (contactsPositionSOUT);
//        contactsPositionSOUT.setConstant(contactsPositionVector);
        sumContactsPositionIn_.setZero();
        tc_.setZero();

        dynamicgraph::Vector comVector(3);
        comVector.setZero();
        signalRegistration (comSIN);
        comSIN.setConstant(comVector);
        sumComIn_.setZero();

        unsigned zero;
        zero=0;
        signalRegistration (contactsNbrSIN);
        contactsNbrSIN.setConstant(zero);

       std::string docstring;

       //settRa
       docstring =
               "\n"
               "    Set the rotational matrix for the accelerometer \n"
               "\n";

       addCommand(std::string("setRa"),
            new
            ::dynamicgraph::command::Setter <Calibrate,dynamicgraph::Matrix>
               (*this, &Calibrate::setRa, docstring));

       //getRa
       docstring =
               "\n"
               "    Get the rotational matrix for the accelerometer \n"
               "\n";

       addCommand(std::string("getRa"),
            new
            ::dynamicgraph::command::Getter <Calibrate,dynamicgraph::Matrix>
               (*this, &Calibrate::getRa, docstring));

       //settRg
       docstring =
               "\n"
               "    Set the rotational matrix for the gyrometer \n"
               "\n";

       addCommand(std::string("setRg"),
            new
            ::dynamicgraph::command::Setter <Calibrate,dynamicgraph::Matrix>
               (*this, &Calibrate::setRg, docstring));

       //getRg
       docstring =
               "\n"
               "    Get the rotational matrix for the gyrometer \n"
               "\n";

       addCommand(std::string("getRg"),
            new
            ::dynamicgraph::command::Getter <Calibrate,dynamicgraph::Matrix>
               (*this, &Calibrate::getRg, docstring));

       //sett
       docstring =
               "\n"
               "    Set the translational transformtation for the contacts position \n"
               "\n";

       addCommand(std::string("settc"),
            new
            ::dynamicgraph::command::Setter <Calibrate,dynamicgraph::Vector>
               (*this, &Calibrate::settc, docstring));

       //gett
       docstring =
               "\n"
               "    Get the translational transformtation for the contacts position \n"
               "\n";

       addCommand(std::string("gettc"),
            new
            ::dynamicgraph::command::Getter <Calibrate,dynamicgraph::Vector>
               (*this, &Calibrate::gettc, docstring));

       //settcom
       docstring =
               "\n"
               "    Set the translational transformtation for the com \n"
               "\n";

       docstring  =
               "\n"
               "    Start the calibration \n"
               "\n";


       addCommand(std::string("start"),
                  ::dynamicgraph::command::makeCommandVoid2(*this, & Calibrate::start, docstring));

       docstring  =
               "\n"
               "    Reset the calibration \n"
               "\n";

       addCommand(std::string("reset"),
                   ::dynamicgraph::command::makeCommandVoid1(*this, & Calibrate::reset ,
                                       docstring));


        imuSOUT.setFunction(boost::bind(&Calibrate::computeImu, this, _1, _2));
        contactsPositionSOUT.setFunction(boost::bind(&Calibrate::computeContactsPosition, this, _1, _2));
    }

    Calibrate::~Calibrate()
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

        double angle=acos(scalarProduct/v.norm());
        stateObservation::Vector3 axis=crossProduct/crossProduct.norm();
        stateObservation::AngleAxis utheta=stateObservation::AngleAxis(angle,axis);

        return utheta.toRotationMatrix();
    }

    void Calibrate::calibrate(const int& inTime)
    {
        const stateObservation::Vector& imuIn=convertVector<stateObservation::Vector>(imuSIN.access(inTime));
        const stateObservation::Vector& contactsPositionRotIn=convertVector<stateObservation::Vector>(contactsPositionSIN.access(inTime));
        const stateObservation::Vector& comIn=convertVector<stateObservation::Vector>(comSIN.access(inTime));
        const int& contactsNbr=contactsNbrSIN.access(inTime);

        if(currentStep_ < nbStep_)
        {
            sumImuIn_+=imuIn;
            sumComIn_+=comIn;
            sumContactsPositionIn_+=contactsPositionRotIn;
            currentStep_++;
        }
        else if(currentStep_ == nbStep_)
        {
            // IMU calibration
            if(mode_==0 || mode_==1)
            {
               stateObservation::Vector meanImuIn;
               meanImuIn=sumImuIn_/nbStep_;
               R_.block(0,0,3,3)=Rdetermination(meanImuIn.block(0,0,3,1));
               R_.block(3,3,3,3)=R_.block(0,0,3,3);
            }

            // Feet position calibration
            if(mode_==0 || mode_==2)
            {
                stateObservation::Vector meanComIn;
                meanComIn=sumComIn_/nbStep_;
                stateObservation::Vector meanContactsPositionIn(12);
                meanContactsPositionIn=sumContactsPositionIn_/nbStep_;
                stateObservation::Vector contactsPositionIn(6);
                contactsPositionIn  <<  meanContactsPositionIn.block(0,0,3,1),
                                        meanContactsPositionIn.block(6,0,3,1);
                stateObservation::Vector calibrateContactsPosition(6);
                if(contactsNbr==2){
                    calibrateContactsPosition   <<  meanComIn(0),
                                                    contactsPositionIn(1),
                                                    contactsPositionIn(2),
                                                    meanComIn(0),
                                                    contactsPositionIn(4),
                                                    contactsPositionIn(5);
                }else if (contactsNbr==1){
                    calibrateContactsPosition   <<  meanComIn(0),
                                                    meanComIn(1),
                                                    contactsPositionIn(2),
                                                    contactsPositionIn(3),
                                                    contactsPositionIn(4),
                                                    contactsPositionIn(5);
                }
                tc_=calibrateContactsPosition-contactsPositionIn;
            }
            calibrate_=false;
        }
        inTime_=inTime;
    }

    dynamicgraph::Vector& Calibrate::computeImu(dynamicgraph::Vector & imuOut, const int& inTime)
    {
        const stateObservation::Vector& imuIn=convertVector<stateObservation::Vector>(imuSIN.access(inTime));

        if(calibrate_==true && inTime!=inTime_){
            calibrate(inTime);
        }

        stateObservation::Vector prod=R_*imuIn;
        imuOut=convertVector<dynamicgraph::Vector>(prod);
        return imuOut;
    }

    dynamicgraph::Vector& Calibrate::computeContactsPosition(dynamicgraph::Vector & contactsPositionOut, const int& inTime)
    {
        const stateObservation::Vector& contactsPositionIn=convertVector<stateObservation::Vector>(contactsPositionSIN.access(inTime));

        if(calibrate_==true && inTime!=inTime_){
            calibrate(inTime);
        }

        stateObservation::Vector sum;
        sum=contactsPositionIn;
        sum.block(0,0,3,1)+=tc_.block(0,0,3,1);
        sum.block(6,0,3,1)+=tc_.block(3,0,3,1);
        contactsPositionOut=convertVector<dynamicgraph::Vector>(sum);
        return contactsPositionOut;
    }

}

