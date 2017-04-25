//
// Copyright (c) 2016,
// Alexis Mifsud
//
// CNRS
//
// This file is part of sot-state-observation.
// sot-dynamic is free software: you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of
// the License, or (at your option) any later version.
// sot-dynamic is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.  You should
// have received a copy of the GNU Lesser General Public License along
// with sot-dynamic.  If not, see <http://www.gnu.org/licenses/>.
//

#include <sstream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-bind.h>

#include <math.h>

#include <state-observation/tools/definitions.hpp>
#include <sot-state-observation/tools/definitions.hh>
#include <state-observation/tools/miscellaneous-algorithms.hpp>

#include <sot-state-observation/estimator-interface.hh>

namespace sotStateObservation
{
    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN ( EstimatorInterface, "EstimatorInterface" );

    EstimatorInterface::EstimatorInterface( const std::string & inName):
        Entity(inName),
        enabledContacts_lf_rf_lh_rhSIN_ (NULL, "EstimatorInterface("+inName+")::input(vector)::enabledContacts_lf_rf_lh_rh"),
        inputSOUT_ (NULL, "EstimatorInterface("+inName+")::output(vector)::input"),
        inputConstSizeSOUT_ (NULL, "EstimatorInterface("+inName+")::output(vector)::inputConstSize"),
        measurementSOUT_ (NULL, "EstimatorInterface("+inName+")::output(vector)::measurement"),
        measurementConstSizeSOUT_ (NULL, "EstimatorInterface("+inName+")::output(vector)::measurementConstSize"),
        contactsModelSOUT_ (NULL, "EstimatorInterface("+inName+")::output(unsigned)::contactsModel"),
        configSOUT_ (NULL, "EstimatorInterface("+inName+")::output(Vector)::config"),
        contactsNbrSOUT_ (NULL, "EstimatorInterface("+inName+")::output(unsigned)::contactsNbr"),
        modeledContactsNbrSOUT_ (NULL, "EstimatorInterface("+inName+")::output(unsigned)::modeledContactsNbr"),
        unmodeledContactsNbrSOUT_ (NULL, "EstimatorInterface("+inName+")::output(unsigned)::unmodeledContactsNbr"),
        supportContactsNbrSOUT_ (NULL, "EstimatorInterface("+inName+")::output(unsigned)::supportContactsNbr"),
        stackOfSupportContactsSOUT_ (NULL, "EstimatorInterface("+inName+")::output(vector)::stackOfSupportContacts"),
        positionSupport1SOUT_ (NULL, "EstimatorInterface("+inName+")::output(HomoMatrix)::positionSupport1"),
        positionSupport2SOUT_ (NULL, "EstimatorInterface("+inName+")::output(HomoMatrix)::positionSupport2"),
        forceSupport1SOUT_ (NULL, "EstimatorInterface("+inName+")::output(Vector)::forceSupport1"),
        forceSupport2SOUT_ (NULL, "EstimatorInterface("+inName+")::output(Vector)::forceSupport2"),
        positionLeftFootSIN_ (NULL, "EstimatorInterface("+inName+")::input(HomoMatrix)::position_lf"),
        velocityLeftFootSIN_ (NULL, "EstimatorInterface("+inName+")::input(vector)::velocity_lf"),
        forceLeftFootSIN_ (NULL, "EstimatorInterface("+inName+")::input(vector)::force_lf"),
        positionRightFootSIN_ (NULL, "EstimatorInterface("+inName+")::input(HomoMatrix)::position_rf"),
        velocityRightFootSIN_ (NULL, "EstimatorInterface("+inName+")::input(vector)::velocity_rf"),
        forceRightFootSIN_ (NULL, "EstimatorInterface("+inName+")::input(vector)::force_rf"),
        positionLeftHandSIN_ (NULL, "EstimatorInterface("+inName+")::input(HomoMatrix)::position_lh"),
        forceLeftHandSIN_ (NULL, "EstimatorInterface("+inName+")::input(vector)::force_lh"),
        positionRightHandSIN_ (NULL, "EstimatorInterface("+inName+")::input(HomoMatrix)::position_rh"),
        forceRightHandSIN_ (NULL, "EstimatorInterface("+inName+")::input(vector)::force_rh"),
        positionLeftRopeSIN_ (NULL, "EstimatorInterface("+inName+")::input(vector)::position_ls"),
        positionRightRopeSIN_ (NULL, "EstimatorInterface("+inName+")::input(vector)::position_rs"),
        comVectorSIN(NULL , "EstimatorInterface("+inName+")::input(vector)::comVector"),
        inertiaSIN(NULL , "EstimatorInterface("+inName+")::input(matrix)::inertia"),
        dinertiaSIN(NULL , "EstimatorInterface("+inName+")::input(vector)::dinertia"),
        positionWaistSIN(NULL , "EstimatorInterface("+inName+")::input(matrix)::positionWaist"),
        angMomentumSIN(NULL , "EstimatorInterface("+inName+")::input(vector)::angMomentum"),
        dangMomentumSIN(NULL , "EstimatorInterface("+inName+")::input(vector)::dangMomentum"),
        imuVectorSIN(NULL , "EstimatorInterface("+inName+")::input(vector)::imuVector"),
        accelerometerSIN(NULL , "EstimatorInterface("+inName+")::input(vector)::accelerometer"),
        gyrometerSIN(NULL , "EstimatorInterface("+inName+")::input(vector)::gyrometer"),
        driftSIN(NULL , "EstimatorInterface("+inName+")::input(vector)::drift"),
        timeStackOfContacts_(-1), timeInput_(-1), timeMeasurement_(-1),
        timeSensorsPositions_(-1), timeForces_(-1),
        timeTransformForcesFrames_(-1), timeDrift_(-1), timeContacts_(-1),
        contactsModel_(1),elastPendulumModel_(1),
        inputForces_(hrp2::contact::nbMax),
        controlFrameForces_(hrp2::contact::nbMax),
        inputPosition_(hrp2::contact::nbMax),
        inputHomoPosition_(hrp2::contact::nbMax),
        inputVelocity_(hrp2::contact::nbMax),
        forceSensorsTransformation_(hrp2::contact::nbMax),
        forceSensorsTransfoMatrix_(hrp2::contact::nbMax),
        bias_(hrp2::contact::nbMax),
        withUnmodeledMeasurements_(false), withModeledForces_(true), withAbsolutePose_(false), externalContactPresence_(true)
    {

        /// Signals

        // Input
        MatrixHomogeneous pos;
        stateObservation::Vector6 force;
        stateObservation::Vector6 velocity;
        dynamicgraph::Vector vpos; vpos.resize(6);

        signalRegistration (enabledContacts_lf_rf_lh_rhSIN_);
        dynamicgraph::Vector stack(4); stack.setZero();
        enabledContacts_lf_rf_lh_rhSIN_.setConstant(stack);

        signalRegistration (positionLeftFootSIN_ << velocityLeftFootSIN_ << forceLeftFootSIN_);
        positionLeftFootSIN_.setConstant(pos);
        positionLeftFootSIN_.setTime (timeStackOfContacts_);
        velocityLeftFootSIN_.setConstant(convertVector<dynamicgraph::Vector>(velocity));
        velocityLeftFootSIN_.setTime (timeStackOfContacts_);
        forceLeftFootSIN_.setConstant(convertVector<dynamicgraph::Vector>(force));
        forceLeftFootSIN_.setTime (timeStackOfContacts_);

        signalRegistration (positionRightFootSIN_ << velocityRightFootSIN_ << forceRightFootSIN_);
        positionRightFootSIN_.setConstant(pos);
        positionRightFootSIN_.setTime (timeStackOfContacts_);
        velocityRightFootSIN_.setConstant(convertVector<dynamicgraph::Vector>(velocity));
        velocityRightFootSIN_.setTime (timeStackOfContacts_);
        forceRightFootSIN_.setConstant(convertVector<dynamicgraph::Vector>(force));
        forceRightFootSIN_.setTime (timeStackOfContacts_);

        signalRegistration (positionLeftHandSIN_ << forceLeftHandSIN_);
        positionLeftHandSIN_.setConstant(pos);
        positionLeftHandSIN_.setTime (timeStackOfContacts_);
        forceLeftHandSIN_.setConstant(convertVector<dynamicgraph::Vector>(force));
        forceLeftHandSIN_.setTime (timeStackOfContacts_);

        signalRegistration (positionRightHandSIN_ << forceRightHandSIN_);
        positionRightHandSIN_.setConstant(pos);
        positionRightHandSIN_.setTime (timeStackOfContacts_);
        forceRightHandSIN_.setConstant(convertVector<dynamicgraph::Vector>(force));
        forceRightHandSIN_.setTime (timeStackOfContacts_);

        signalRegistration (positionLeftRopeSIN_);
        positionLeftRopeSIN_.setConstant(vpos);
        positionLeftRopeSIN_.setTime (timeStackOfContacts_);

        signalRegistration (positionRightRopeSIN_);
        positionRightRopeSIN_.setConstant(vpos);
        positionRightRopeSIN_.setTime (timeStackOfContacts_);

        signalRegistration (comVectorSIN);
        dynamicgraph::Vector comVector(3);
        comVectorSIN.setConstant(comVector);

        signalRegistration (inertiaSIN);
        dynamicgraph::Matrix inertia(6);
        inertiaSIN.setConstant(inertia);

        signalRegistration (dinertiaSIN);
        dynamicgraph::Vector dinertia(6);
        dinertiaSIN.setConstant(dinertia);

        signalRegistration (positionWaistSIN);
        dynamicgraph::Matrix positionWaist;
        positionWaistSIN.setConstant(positionWaist);

        signalRegistration (angMomentumSIN);
        dynamicgraph::Vector angMomentum(6);
        angMomentumSIN.setConstant(angMomentum);

        signalRegistration (dangMomentumSIN);
        dynamicgraph::Vector dangMomentum(6);
        dangMomentumSIN.setConstant(dangMomentum);

        signalRegistration (imuVectorSIN);
        dynamicgraph::Vector imuVector(15);
        imuVectorSIN.setConstant(imuVector);

        signalRegistration (accelerometerSIN);
        dynamicgraph::Vector accelerometer(3);
        accelerometerSIN.setConstant(accelerometer);

        signalRegistration (gyrometerSIN);
        dynamicgraph::Vector gyrometer(3);
        gyrometerSIN.setConstant(gyrometer);

        signalRegistration (driftSIN);
        dynamicgraph::Vector drift(6);
        driftSIN.setConstant(drift);

        // Output
        signalRegistration (inputSOUT_);
        inputSOUT_.setFunction(boost::bind(&EstimatorInterface::getInput, this, _1, _2));

        signalRegistration (inputConstSizeSOUT_);
        inputConstSizeSOUT_.setFunction(boost::bind(&EstimatorInterface::getInputConstSize, this, _1, _2));

        signalRegistration (measurementSOUT_);
        measurementSOUT_.setFunction(boost::bind(&EstimatorInterface::getMeasurement, this, _1, _2));

        signalRegistration (measurementConstSizeSOUT_);
        measurementConstSizeSOUT_.setFunction(boost::bind(&EstimatorInterface::getMeasurementConstSize, this, _1, _2));

        signalRegistration (contactsModelSOUT_);
        contactsModelSOUT_.setFunction(boost::bind(&EstimatorInterface::getContactsModel, this, _1, _2));

        signalRegistration (configSOUT_);
        configSOUT_.setFunction(boost::bind(&EstimatorInterface::getConfig, this, _1, _2));

        signalRegistration (contactsNbrSOUT_);
        contactsNbrSOUT_.setFunction(boost::bind(&EstimatorInterface::getContactsNbr, this, _1, _2));

        signalRegistration (modeledContactsNbrSOUT_);
        modeledContactsNbrSOUT_.setFunction(boost::bind(&EstimatorInterface::getModeledContactsNbr, this, _1, _2));

        signalRegistration (unmodeledContactsNbrSOUT_);
        unmodeledContactsNbrSOUT_.setFunction(boost::bind(&EstimatorInterface::getUnmodeledContactsNbr, this, _1, _2));

        signalRegistration (supportContactsNbrSOUT_);
        supportContactsNbrSOUT_.setFunction(boost::bind(&EstimatorInterface::getSupportContactsNbr, this, _1, _2));

        signalRegistration (stackOfSupportContactsSOUT_);
        stackOfSupportContactsSOUT_.setFunction(boost::bind(&EstimatorInterface::getStackOfSupportContacts, this, _1, _2));

        signalRegistration (positionSupport1SOUT_);
        positionSupport1SOUT_.setFunction(boost::bind(&EstimatorInterface::getPositionSupport1, this, _1, _2));

        signalRegistration (positionSupport2SOUT_);
        positionSupport2SOUT_.setFunction(boost::bind(&EstimatorInterface::getPositionSupport2, this, _1, _2));

        signalRegistration (forceSupport1SOUT_);
        forceSupport1SOUT_.setFunction(boost::bind(&EstimatorInterface::getForceSupport1, this, _1, _2));

        signalRegistration (forceSupport2SOUT_);
        forceSupport2SOUT_.setFunction(boost::bind(&EstimatorInterface::getForceSupport2, this, _1, _2));

        /// Commands

        std::string docstring;

        docstring  =
                "\n"
                "    Set the force thresholds \n"
                "\n";

        addCommand(std::string("setForceThresholds"),
             new
             ::dynamicgraph::command::Setter <EstimatorInterface,dynamicgraph::Vector>
                (*this, &EstimatorInterface::setForceThresholds, docstring));
        docstring =
                "\n"
                "    Get the force thresholds \n"
                "\n";

        addCommand(std::string("getForceThresholds"),
             new
             ::dynamicgraph::command::Getter <EstimatorInterface,dynamicgraph::Vector>
                (*this, &EstimatorInterface::getForceThresholds, docstring));

        docstring =
                "\n"
                "    Set if the derivative of inertia matrix is computed using finite differences"
                "\n";

        addCommand(std::string("setFDInertiaDot"),
             new
             ::dynamicgraph::command::Setter <EstimatorInterface,bool>
                (*this, &EstimatorInterface::setFDInertiaDot, docstring));

        docstring =
                "\n"
                "    Set bias for left foot"
                "\n";

        addCommand(std::string("setLeftFootBias"),
             new
             ::dynamicgraph::command::Setter <EstimatorInterface,dynamicgraph::Vector>
                (*this, &EstimatorInterface::setLeftFootBias, docstring));

        docstring =
                "\n"
                "    Set bias for right foot"
                "\n";

        addCommand(std::string("setRightFootBias"),
             new
             ::dynamicgraph::command::Setter <EstimatorInterface,dynamicgraph::Vector>
                (*this, &EstimatorInterface::setRightFootBias, docstring));

        docstring =
                "\n"
                "    Set lastInertia_\n"
                "\n";

        addCommand(std::string("setLastInertia"),
             new
             ::dynamicgraph::command::Setter <EstimatorInterface,dynamicgraph::Matrix>
                (*this, &EstimatorInterface::setLastInertia, docstring));

        docstring =
                "\n"
                "    Set left hand force sensor transformation\n"
                "\n";

        addCommand(std::string("setLeftHandSensorTransformation"),
             new
             ::dynamicgraph::command::Setter <EstimatorInterface,dynamicgraph::Vector>
                (*this, &EstimatorInterface::setLeftHandSensorTransformation, docstring));

        docstring =
                "\n"
                "    Get left hand force sensor transformation\n"
                "\n";

        addCommand(std::string("getLeftHandSensorTransformation"),
                   new
                   ::dynamicgraph::command::Getter <EstimatorInterface,dynamicgraph::Vector>
                      (*this, &EstimatorInterface::getLeftHandSensorTransformation, docstring));

        docstring =
                "\n"
                "    Set right hand force sensor transformation\n"
                "\n";

        addCommand(std::string("setRightHandSensorTransformation"),
             new
             ::dynamicgraph::command::Setter <EstimatorInterface,dynamicgraph::Vector>
                (*this, &EstimatorInterface::setRightHandSensorTransformation, docstring));

        docstring =
                "\n"
                "    Get right hand force sensor transformation\n"
                "\n";

        addCommand(std::string("getRightHandSensorTransformation"),
                   new
                   ::dynamicgraph::command::Getter <EstimatorInterface,dynamicgraph::Vector>
                      (*this, &EstimatorInterface::getRightHandSensorTransformation, docstring));

        docstring =
                "\n"
                "    Set withUnmodeleedMeausrements \n"
                "\n";

        addCommand(std::string("setWithUnmodeledMeasurements"),
             new
             ::dynamicgraph::command::Setter <EstimatorInterface,bool>
                (*this, &EstimatorInterface::setWithUnmodeledMeasurements, docstring));

        docstring =
                "\n"
                "    Get withUnmodeleedMeausrements\n"
                "\n";

        addCommand(std::string("getWithUnmodeledMeausrements"),
                   new
                   ::dynamicgraph::command::Getter <EstimatorInterface,bool>
                      (*this, &EstimatorInterface::getWithUnmodeledMeasurements, docstring));

        docstring =
                "\n"
                "    Set withModeledForces \n"
                "\n";

        addCommand(std::string("setWithModeledForces"),
             new
             ::dynamicgraph::command::Setter <EstimatorInterface,bool>
                (*this, &EstimatorInterface::setWithModeledForces, docstring));

        docstring =
                "\n"
                "    Get withModeledForces\n"
                "\n";

        addCommand(std::string("getWithModeledForces"),
                   new
                   ::dynamicgraph::command::Getter <EstimatorInterface,bool>
                      (*this, &EstimatorInterface::getWithModeledForces, docstring));

        docstring =
                "\n"
                "    Set withAbsolutePose \n"
                "\n";

        addCommand(std::string("setWithAbsolutePose"),
             new
             ::dynamicgraph::command::Setter <EstimatorInterface,bool>
                (*this, &EstimatorInterface::setWithAbsolutePose, docstring));

        docstring =
                "\n"
                "    Get withAbsolutePose\n"
                "\n";

        addCommand(std::string("getWithAbsolutePose"),
                   new
                   ::dynamicgraph::command::Getter <EstimatorInterface,bool>
                      (*this, &EstimatorInterface::getWithAbsolutePose, docstring));

        docstring =
                "\n"
                "    Set the elast pendulum to use \n"
                "\n";

        addCommand(std::string("setElastPendulumModel"),
             new
             ::dynamicgraph::command::Setter <EstimatorInterface,unsigned>
                (*this, &EstimatorInterface::setElastPendulumModel, docstring));

        docstring =
                "\n"
                "    Get externalContactPresence_\n"
                "\n";

        addCommand(std::string("getExternalContactPresence"),
                   new
                   ::dynamicgraph::command::Getter <EstimatorInterface,bool>
                      (*this, &EstimatorInterface::getExternalContactPresence, docstring));

        docstring =
                "\n"
                "    Set externalContactPresence_ \n"
                "\n";

        addCommand(std::string("setExternalContactPresence"),
             new
             ::dynamicgraph::command::Setter <EstimatorInterface,bool>
                (*this, &EstimatorInterface::setExternalContactPresence, docstring));

        //setSampligPeriod
        docstring =
                "\n"
                "    Sets the sampling period.\n"
                "    takes a floating point number\n"
                "\n";

        addCommand(std::string("setSamplingPeriod"),
             new
             dynamicgraph::command::Setter <EstimatorInterface,double>
            (*this, &EstimatorInterface::setSamplingPeriod, docstring));

        /// Parameters

        config_.resize(3); config_.setZero();

        // ForceThresholds
        forceThresholds_.resize(hrp2::contact::nbMax);
        forceThresholds_.setOnes();
        forceThresholds_*=0.02 * 56.8*stateObservation::cst::gravityConstant; // default value
        forceThresholds_[hrp2::contact::lh]=100;//2.5;
        forceThresholds_[hrp2::contact::rh]=100;//2.5;

        // ForceResidus
        forceResidus_.resize(hrp2::contact::nbMax);
        forceResidus_[hrp2::contact::lf]=7.8;
        forceResidus_[hrp2::contact::lf]=7.8;
        forceResidus_[hrp2::contact::lh]=11.25;
        forceResidus_[hrp2::contact::rh]=11.25;

        // Modeled
        modeled_.resize(hrp2::contact::nbMax);
        modeled_[hrp2::contact::lf]=true;
        modeled_[hrp2::contact::rf]=true;
        modeled_[hrp2::contact::lh]=false;
        modeled_[hrp2::contact::rh]=false;

        // Support
        support_.resize(hrp2::contact::nbMax);
        support_[hrp2::contact::lf]=true;
        support_[hrp2::contact::rf]=true;
        support_[hrp2::contact::lh]=false;
        support_[hrp2::contact::rh]=false;

        // From input reconstructor
        for (int i=0; i<hrp2::contact::nbMax;++i)
        {
            bias_[i].resize(6); bias_[i].setZero();
            forceSensorsTransformation_[i].resize(3);
            forceSensorsTransformation_[i].setZero();
            forceSensorsTransfoMatrix_[i].resize(3,3);
            forceSensorsTransfoMatrix_[i].setIdentity();
        }
        lastInertia_.setZero();
        dt_=5e-3;

        measurement_.resize(12); measurement_.setZero();
    }

    EstimatorInterface::~EstimatorInterface()
    {
    }

    void EstimatorInterface::getDrift(const int& time)
    {
        timeDrift_=time;
        drift_ = convertVector<stateObservation::Vector>(driftSIN.access (time));
    }

    void EstimatorInterface::getSensorsKineInControlFrame(const int& time)
    {
        timeSensorsPositions_=time;

        // Positions
        inputHomoPosition_[hrp2::contact::rf] = convertMatrix<stateObservation::Matrix4>(Matrix(positionRightFootSIN_.access (time)));
        inputHomoPosition_[hrp2::contact::lf] = convertMatrix<stateObservation::Matrix4>(Matrix(positionLeftFootSIN_.access (time)));
        inputHomoPosition_[hrp2::contact::rh] = convertMatrix<stateObservation::Matrix4>(Matrix(positionRightHandSIN_.access (time)));
        inputHomoPosition_[hrp2::contact::lh] = convertMatrix<stateObservation::Matrix4>(Matrix(positionLeftHandSIN_.access (time)));

        for (int i=0; i<hrp2::contact::nbMax;++i)
        {
            inputPosition_[i]=kine::homogeneousMatrixToVector6(inputHomoPosition_[i]);
        }

        // Velocities
        inputVelocity_[hrp2::contact::rf] = convertVector<stateObservation::Vector6>(velocityRightFootSIN_.access (time));
        inputVelocity_[hrp2::contact::lf] = convertVector<stateObservation::Vector6>(velocityLeftFootSIN_.access (time));

    }

    void EstimatorInterface::getForces(const int& time)
    {
        timeForces_=time;

        inputForces_[hrp2::contact::rf] = convertVector<stateObservation::Vector>(forceRightFootSIN_.access (time));
        inputForces_[hrp2::contact::lf] = convertVector<stateObservation::Vector>(forceLeftFootSIN_.access (time));
        inputForces_[hrp2::contact::rh] = convertVector<stateObservation::Vector>(forceRightHandSIN_.access (time));
        inputForces_[hrp2::contact::lh] = convertVector<stateObservation::Vector>(forceLeftHandSIN_.access (time));
    }

    void EstimatorInterface::transformForcesFrames(const int& time)
    {
        timeTransformForcesFrames_=time;

        if(time!=timeForces_) getForces(time);
        if(time!=timeSensorsPositions_) getSensorsKineInControlFrame(time);

        for (int i=0; i<hrp2::contact::nbMax;++i)
        {          
            // force sensor position
            op_.Rc=inputHomoPosition_[i].block(0,0,3,3);
            op_.Rct=op_.Rc.transpose();
            op_.pc=inputHomoPosition_[i].block(0,3,3,1);

            // Reorientation of frames
            controlFrameForces_[i] << forceSensorsTransfoMatrix_[i] * inputForces_[i].segment(0,3),
                                      forceSensorsTransfoMatrix_[i] * inputForces_[i].segment(3,3);

            // For hands
            if(i == hrp2::contact::lh | i==hrp2::contact::rh)
            {
                // Computation in the local frame of the weight action of theend-effector on the sensor
                op_.weight << 0,
                              0,
                              -forceResidus_[i];
                op_.l << 0,
                         0,
                         -0.035;

                op_.forceResidusVector << op_.Rct*op_.weight,
                                          kine::skewSymmetric(op_.l)*op_.Rct*op_.weight;

                // Substract the weight action from input forces
                controlFrameForces_[i]-=op_.forceResidusVector;
            }
        }
    }

    bool EstimatorInterface::getContactPresence(int & i, const int& time)
    {
        if(externalContactPresence_)
        {
            const dynamicgraph::Vector& stackOfContacts=enabledContacts_lf_rf_lh_rhSIN_.access(time);
            return stackOfContacts(i);
        }
        else
        {
            op_.contactForce=inputForces_[i].segment(0,3).norm()-forceResidus_[i];
            return (op_.contactForce>forceThresholds_[i] ||  op_.contactForce<-forceThresholds_[i]);
        }
    }

    void EstimatorInterface::computeStackOfContacts(const int& time)
    {
        timeStackOfContacts_=time;
        if(time!=timeForces_) getForces(time);

        for (int i=0; i<hrp2::contact::nbMax;++i)
        {
            op_.found = (std::find(stackOfContacts_.begin(), stackOfContacts_.end(), i) != stackOfContacts_.end());

            if(getContactPresence(i, time))
            {
                if (!op_.found)
                {
                    stackOfContacts_.push_back(i);
                    if(modeled_[i]) { stackOfModeledContacts_.push_back(i); }
                    if(!modeled_[i]) { stackOfUnmodeledContacts_.push_back(i); }
                    if(support_[i]) { stackOfSupportContacts_.push_back(i); }
                }
            }
            else
            {
                if(op_.found)
                {
                    stackOfContacts_.remove(i);
                    if(modeled_[i]) { stackOfModeledContacts_.remove(i); }
                    if(!modeled_[i]) { stackOfUnmodeledContacts_.remove(i); }
                    if(support_[i]) { stackOfSupportContacts_.remove(i); }
                }
            }
        }
    }

    void EstimatorInterface::computeContacts(const int& time)
    {
        timeContacts_=time;
        if(time!=timeStackOfContacts_) computeStackOfContacts(time);

        // Update all a priori contacts numbers.
        contactsNbr_=stackOfContacts_.size();
        modeledContactsNbr_=stackOfModeledContacts_.size();
        supportContactsNbr_=stackOfSupportContacts_.size();
        unmodeledContactsNbr_=stackOfUnmodeledContacts_.size();

        // Treat the case where the robot is supported by the ropes
        if(supportContactsNbr_<1)
        {
            contactsNbr_+=2;
            modeledContactsNbr_=2;
            contactsModel_=elastPendulumModel_+1;
        }
        else
        {
            contactsModel_=1;
        }
    }

    void EstimatorInterface::computeInert(const stateObservation::Matrix & inertia,
                                          const stateObservation::Matrix & homoWaist,
                                          const stateObservation::Vector& comVector,
                                          stateObservation::Vector& inert)
    {
        double m=inertia(0,0); //<=== donne 56.8;
        //std::cout << "Masse=" << m << std::endl;

        stateObservation::Vector waist=homoWaist.block(0,3,3,1);
        stateObservation::Vector com=comVector.segment(0,3);

        // Inertia expressed at waist
        inert(0)=inertia(3,3);
        inert(1)=inertia(4,4);
        inert(2)=inertia(5,5);
        inert(3)=inertia(3,4);
        inert(4)=inertia(3,5);
        inert(5)=inertia(4,5);

        // From waist to com
        inert(0) += -m*((com(1)-waist(1))*(com(1)-waist(1))+(com(2)-waist(2))*(com(2)-waist(2)));
        inert(1) += -m*((com(0)-waist(0))*(com(0)-waist(0))+(com(2)-waist(2))*(com(2)-waist(2)));
        inert(2) += -m*((com(0)-waist(0))*(com(0)-waist(0))+(com(1)-waist(1))*(com(1)-waist(1)));
        inert(3) += m*(com(0)-waist(0))*(com(1)-waist(1));
        inert(4) += m*(com(0)-waist(0))*(com(2)-waist(2));
        inert(5) += m*(com(1)-waist(1))*(com(2)-waist(2));

        // From com to local frame
        inert(0) -= -m*((com(1))*(com(1))+(com(2))*(com(2)));
        inert(1) -= -m*((com(0))*(com(0))+(com(2))*(com(2)));
        inert(2) -= -m*((com(0))*(com(0))+(com(1))*(com(1)));
        inert(3) -= m*(com(0))*(com(1));
        inert(4) -= m*(com(0))*(com(2));
        inert(5) -= m*(com(1))*(com(2));
    }

   void EstimatorInterface::computeInput(const int& time)
   {
       timeInput_=time;
       if(time!=timeSensorsPositions_) getSensorsKineInControlFrame(time);
       if(time!=timeContacts_) computeContacts(time);

       const stateObservation::Matrix& inertia=convertMatrix<stateObservation::Matrix>(inertiaSIN.access(time));
       const stateObservation::Matrix& homoWaist=convertMatrix<stateObservation::Matrix>(positionWaistSIN.access(time));
       const stateObservation::Vector& comVector=convertVector<stateObservation::Vector>(comVectorSIN.access(time));
       const stateObservation::Vector& dinertia=convertVector<stateObservation::Vector>(dinertiaSIN.access(time));
       const stateObservation::Vector& angMomentum=convertVector<stateObservation::Vector>(angMomentumSIN.access(time));
       const stateObservation::Vector& dangMomentum=convertVector<stateObservation::Vector>(dangMomentumSIN.access(time));
       const stateObservation::Vector& imuVector=convertVector<stateObservation::Vector>(imuVectorSIN.access(time));

       const stateObservation::Vector& rightRopePosition=convertVector<stateObservation::Vector>(positionRightRopeSIN_.access(time));
       const stateObservation::Vector& leftRopePosition=convertVector<stateObservation::Vector>(positionLeftRopeSIN_.access(time));


       if(contactsModel_==1)
       {
           op_.contactKine.resize(modeledContactsNbr_*12);
           op_.bias.resize(modeledContactsNbr_*12); op_.bias.setZero();
           op_.i=0;

           for (iterator = stackOfModeledContacts_.begin(); iterator != stackOfModeledContacts_.end(); ++iterator)
           {
               op_.contactKine.segment(op_.i*12,6)=inputPosition_[*iterator];
               op_.contactKine.segment(op_.i*12+6,6)=inputVelocity_[*iterator];
               op_.bias.segment(op_.i*12,6)=bias_[*iterator];
               ++op_.i;
           }
       }
       else
       {
           op_.contactKine.resize(modeledContactsNbr_*12);
           op_.bias.resize(modeledContactsNbr_*12); op_.bias.setZero();

           op_.contactKine.segment(0*12,6)=leftRopePosition;
           op_.contactKine.segment(0*12+6,6).setZero();
           op_.bias.segment(0*12,6).setZero();

           op_.contactKine.segment(1*12,6)=rightRopePosition;
           op_.contactKine.segment(1*12+6,6).setZero();
           op_.bias.segment(1*12,6).setZero();
       }

       // Inertia and derivative
       op_.inert.resize(6);
       computeInert(inertia,homoWaist,comVector,op_.inert);
       if (derivateInertiaFD_)
       {
         if (lastInertia_.size()>0 && lastInertia_.norm()!=0)
           op_.dinert = (1/dt_)*(op_.inert - lastInertia_);
         else
         {
           op_.dinert.resize(6);
           op_.dinert.setZero();
         }
       }
       else
         op_.dinert=dinertia;
       lastInertia_ = op_.inert;

       op_.m=inertia(0,0);

       // Com
       op_.com=comVector.segment(0,3);
       op_.comddot=comVector.segment(6,3);

       // Angular momentum and derivative
       op_.dangMomentumOut = op_.m*kine::skewSymmetric(op_.com)*op_.comddot;

       // Concatenate input
       input_.resize(42+op_.contactKine.size()); input_.setZero();
       input_.segment(0,9)=comVector;
       input_.segment(9,6)=op_.inert;
       input_.segment(15,6)=op_.dinert;
       input_.segment(21,3)=angMomentum;
       input_.segment(24,3)=op_.dangMomentumOut;
       input_.segment(27,15)=imuVector;
       input_.segment(42,op_.contactKine.size())=op_.contactKine+op_.bias;

   }

   void EstimatorInterface::computeMeasurement(const int& time)
   {
       timeMeasurement_=time;
       if(time!=timeTransformForcesFrames_) transformForcesFrames(time);
       if(time!=timeSensorsPositions_) getSensorsKineInControlFrame(time);
       if(time!=timeContacts_) computeContacts(time);
       if(time!=timeDrift_) getDrift(time);

       const stateObservation::Vector& accelerometer=convertVector<stateObservation::Vector>(accelerometerSIN.access(time));
       const stateObservation::Vector& gyrometer=convertVector<stateObservation::Vector>(gyrometerSIN.access(time));

       measurement_.resize(6+withUnmodeledMeasurements_*6+withModeledForces_*modeledContactsNbr_*6+withAbsolutePose_*6); measurement_.setZero();
       measurement_.segment(0,3)=accelerometer;
       measurement_.segment(3,3)=gyrometer;

       op_.i=6;

       if(withUnmodeledMeasurements_)
       {
           for (iterator = stackOfUnmodeledContacts_.begin(); iterator != stackOfUnmodeledContacts_.end(); ++iterator)
           {
               measurement_.segment(op_.i,6)+=controlFrameForces_[*iterator];
           }
           op_.i+=6;
       }

       if(withModeledForces_)
       {
           for (iterator = stackOfModeledContacts_.begin(); iterator != stackOfModeledContacts_.end(); ++iterator)
           {
               measurement_.segment(op_.i,6)=controlFrameForces_[*iterator];
               op_.i+=6;
           }
       }

       if(withAbsolutePose_)
       {
           measurement_.segment(op_.i,6) = drift_;
       }
   }
}

