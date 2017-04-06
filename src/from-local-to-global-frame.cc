#include <sstream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-bind.h>

#include <math.h>

#include <sot-state-observation/from-local-to-global-frame.hh>
#include <state-observation/flexibility-estimation/imu-elastic-local-frame-dynamical-system.hpp>


namespace sotStateObservation
{
    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN ( FromLocalToGlobalFrame, "FromLocalToGlobalFrame" );

    FromLocalToGlobalFrame::FromLocalToGlobalFrame( const std::string & inName):
        Entity(inName),
        sinPosSIN(0x0 , "FromLocalToGlobalFrame("+inName+")::input(vector)::sinPos"),
        sinVelSIN(0x0 , "FromLocalToGlobalFrame("+inName+")::input(vector)::sinVel"),
        flexStateSIN(0x0 , "FromLocalToGlobalFrame("+inName+")::input(vector)::flexState"),
        soutPosSOUT("FromLocalToGlobalFrame("+inName+")::output(vector)::soutPos"),
        soutVelSOUT("FromLocalToGlobalFrame("+inName+")::output(vector)::soutVel"),
        soutHomoSOUT("FromLocalToGlobalFrame("+inName+")::output(Matrix)::soutHomo"),
        timeGetMandatorySignals_(0), timeComputeSoutPos_(0)
    {

        soutPosSOUT.addDependency(sinPosSIN);
        soutPosSOUT.addDependency(flexStateSIN);
        soutVelSOUT.addDependency(sinPosSIN);
        soutVelSOUT.addDependency(sinVelSIN);
        soutVelSOUT.addDependency(flexStateSIN);

        signalRegistration(sinPosSIN);
        signalRegistration(sinVelSIN);
        signalRegistration(flexStateSIN);
        signalRegistration(soutPosSOUT);
        signalRegistration(soutVelSOUT);
        signalRegistration(soutHomoSOUT);

        soutPosSOUT.setFunction(boost::bind(&FromLocalToGlobalFrame::getSoutPos, this, _1, _2));
        soutVelSOUT.setFunction(boost::bind(&FromLocalToGlobalFrame::getSoutVel, this, _1, _2));
        soutHomoSOUT.setFunction(boost::bind(&FromLocalToGlobalFrame::getSoutHomo, this, _1, _2));

        sinPos_.setZero();
        sinVel_.setZero();
        soutPos_.setZero();
        soutVel_.setZero();
        soutHomo_.setZero();

        dynamicgraph::Vector vect;
        vect.resize(6); vect.setZero();
        sinPosSIN.setConstant(vect);
        sinVelSIN.setConstant(vect);
        flexStateSIN.setConstant(vect);

        omegaflex_.resize(3); omegaflex_.setZero();
        tflex_.resize(3); tflex_.setZero();
        wflex_.resize(3); wflex_.setZero();
        dtflex_.resize(3); dtflex_.setZero();
        Rflex_.resize(3,3); Rflex_.setZero();
    }

    FromLocalToGlobalFrame::~FromLocalToGlobalFrame()
    {
    }

    void FromLocalToGlobalFrame::getMandatorySignals(const int& inTime)
    {
        const stateObservation::Vector& flexState=convertVector<stateObservation::Vector>(flexStateSIN.access(inTime));
        omegaflex_ = flexState.segment(stateObservation::flexibilityEstimation::IMUElasticLocalFrameDynamicalSystem::state::ori,3);
        Rflex_ = stateObservation::kine::rotationVectorToRotationMatrix(omegaflex_);
        tflex_ = flexState.segment(stateObservation::flexibilityEstimation::IMUElasticLocalFrameDynamicalSystem::state::pos,3);
        wflex_ = flexState.segment(stateObservation::flexibilityEstimation::IMUElasticLocalFrameDynamicalSystem::state::angVel,3);
        dtflex_ = flexState.segment(stateObservation::flexibilityEstimation::IMUElasticLocalFrameDynamicalSystem::state::linVel,3);

        sinPos_=convertVector<stateObservation::Vector>(sinPosSIN.access(inTime));

        timeGetMandatorySignals_=inTime;
    }

    void FromLocalToGlobalFrame::computeSoutPos(const int& inTime)
    {
        if(inTime!=timeGetMandatorySignals_) getMandatorySignals(inTime);

        soutPos_ << Rflex_*sinPos_.segment<3>(0)+tflex_,
                    omegaflex_+sinPos_.segment<3>(3);
    }

    void FromLocalToGlobalFrame::computeSoutVel(const int& inTime)
    {
        if(inTime!=timeGetMandatorySignals_) getMandatorySignals(inTime);
        if(inTime!=timeComputeSoutPos_) computeSoutPos(inTime);

        sinVel_ = convertVector<stateObservation::Vector>(sinVelSIN.access(inTime));

        soutVel_ << stateObservation::kine::skewSymmetric(wflex_)*Rflex_*sinPos_.segment<3>(0)+Rflex_*sinVel_.segment<3>(0)+dtflex_,
                    Rflex_*sinVel_.segment<3>(3)+wflex_;
    }

}

