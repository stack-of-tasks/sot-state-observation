#include <sstream>

#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-bind.h>

#include <sot-state-observation/dg-imu-model-base-flex-estimation.hh>

#include <state-observation/flexibility-estimation/imu-elastic-local-frame-dynamical-system.hpp>

#include <sot-state-observation/tools/stop-watch.hh>

#define PROFILE_READ_INPUT_SIGNALS "DGIMUModelBaseFlexEstimation: read input signals: "
#define PROFILE_READ_ESTIMATOR_CONFIG "DGIMUModelBaseFlexEstimation: estimator configuration: "
#define PROFILE_READ_ESTIMATOR_ALONE "DGIMUModelBaseFlexEstimation: estimator without input signals computation: "

namespace sotStateObservation
{
    using namespace stateObservation;
    using namespace flexibilityEstimation;

    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN ( DGIMUModelBaseFlexEstimation, "DGIMUModelBaseFlexEstimation" );

    DGIMUModelBaseFlexEstimation::DGIMUModelBaseFlexEstimation(const std::string & inName):
        Entity(inName),
        measurementSIN(0x0 , "DGIMUModelBaseFlexEstimation("+inName+")::input(vector)::measurement"),
        inputSIN(0x0 , "DGIMUModelBaseFlexEstimation("+inName+")::input(vector)::input"),
        contactsNbrSIN(0x0 , "DGIMUModelBaseFlexEstimation("+inName+")::input(unsigned)::contactNbr"),
        contactsModelSIN(0x0 , "DGIMUModelBaseFlexEstimation("+inName+")::input(unsigned)::contactsModel"),
        configSIN(0x0 , "DGIMUModelBaseFlexEstimation("+inName+")::input(Vector)::config"),
        observationMatrixSOUT("DGIMUModelBaseFlexEstimation("+inName+")::output(matrix)::observationMatrix"),
        AMatrixSOUT("DGIMUModelBaseFlexEstimation("+inName+")::output(matrix)::AMatrix"),
        CMatrixSOUT("DGIMUModelBaseFlexEstimation("+inName+")::output(matrix)::CMatrix"),
        stateSOUT("DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::state"),
        flexibilitySOUT("DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexibility"),
        momentaDotFromForcesSOUT("DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::momentaDotFromForces"),
        momentaDotFromKinematicsSOUT("DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::momentaDotFromKinematics"),
        accelerationsSOUT("DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::accelerations"),
        flexPositionSOUT(flexibilitySOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexPosition"),
        flexVelocitySOUT(flexibilitySOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexVelocity"),
        flexPoseThetaUSOUT(flexibilitySOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexPoseThetaU"),
        comBiasSOUT(flexibilitySOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::comBias"),
        flexOmegaSOUT(flexibilitySOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexOmega"),
        flexOrientationSOUT(flexibilitySOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(matrix)::flexOrientation"),
        flexTransformationMatrixSOUT(flexibilitySOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(homogeneousMatrix)::flexTransformationMatrix"),
        flexThetaUSOUT(flexibilitySOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexThetaU"),
        flexVelocityVectorSOUT(flexibilitySOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexVelocityVector"),
        flexInverseSOUT (flexibilitySOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexInverse"),
        flexMatrixInverseSOUT(flexInverseSOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(homogeneousMatrix)::flexMatrixInverse"),
        flexInversePoseThetaUSOUT(flexInverseSOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexInversePoseThetaU"),
        flexInverseVelocityVectorSOUT(flexInverseSOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexInverseVelocityVector"),
        flexInverseVelocitySOUT(flexInverseSOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexInverseVelocity"),
        flexInverseOmegaSOUT(flexInverseSOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexInverseOmega"),
        simulatedSensorsSOUT(flexibilitySOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::simulatedSensors"),
        predictedSensorsSOUT(flexibilitySOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::predictedSensors"),
        forcesAndMomentsSOUT(flexibilitySOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::forcesAndMoments"),
        forcesSupport1SOUT(flexibilitySOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::forcesSupport1"),
        forcesSupport2SOUT(flexibilitySOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::forcesSupport2"),
        flexibilityComputationTimeSOUT(flexibilitySOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(double)::flexibilityComputationTime"),
        inovationSOUT(flexibilitySOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::inovation"),
        predictionSOUT(flexibilitySOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::prediction"),
        stateCovarianceSOUT(flexibilitySOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::stateCovariance")
    {
        signalRegistration (measurementSIN);
        signalRegistration (inputSIN);
        signalRegistration (contactsNbrSIN);
        signalRegistration (contactsModelSIN);
        signalRegistration (configSIN);

        signalRegistration (stateSOUT);
        signalRegistration (observationMatrixSOUT);
        signalRegistration (AMatrixSOUT);
        signalRegistration (CMatrixSOUT);
        signalRegistration (flexibilitySOUT);
        signalRegistration (momentaDotFromForcesSOUT);
        signalRegistration (momentaDotFromKinematicsSOUT);
        signalRegistration (accelerationsSOUT);
        signalRegistration (flexPositionSOUT);
        signalRegistration (flexVelocitySOUT);
        signalRegistration (flexThetaUSOUT);
        signalRegistration (flexOmegaSOUT);
        signalRegistration (flexOrientationSOUT);
        signalRegistration (flexTransformationMatrixSOUT);
        signalRegistration (flexPoseThetaUSOUT);
        signalRegistration (flexVelocityVectorSOUT);

        signalRegistration (comBiasSOUT);

        signalRegistration (flexInverseSOUT);
        signalRegistration (flexMatrixInverseSOUT);
        signalRegistration (flexInversePoseThetaUSOUT);
        signalRegistration (flexInverseVelocityVectorSOUT);
        signalRegistration (flexInverseVelocitySOUT);
        signalRegistration (flexInverseOmegaSOUT);

        signalRegistration (simulatedSensorsSOUT);
        signalRegistration (predictedSensorsSOUT);
        signalRegistration (flexibilityComputationTimeSOUT);
        signalRegistration (forcesAndMomentsSOUT);
        signalRegistration (forcesSupport1SOUT);
        signalRegistration (forcesSupport2SOUT);
        signalRegistration (inovationSOUT);
        signalRegistration (predictionSOUT);
        signalRegistration (stateCovarianceSOUT);

        stateSOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::computeState,
                                     this, _1, _2));

        observationMatrixSOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::getObservationMatrix,
                                     this, _1, _2));
        AMatrixSOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::getAMatrix,
                                     this, _1, _2));
        CMatrixSOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::getCMatrix,
                                     this, _1, _2));

        flexibilitySOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::computeFlexibility,
				    this, _1, _2));
        momentaDotFromForcesSOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::computeMomentaDotFromForces,
                                    this, _1, _2));

        momentaDotFromKinematicsSOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::computeMomentaDotFromKinematics,
                                    this, _1, _2));

        accelerationsSOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::computeAccelerations,
                                    this, _1, _2));

        flexPositionSOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::computeFlexPosition,
				    this, _1, _2));

        flexVelocitySOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::computeFlexVelocity,
				    this, _1, _2));

        flexThetaUSOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::computeFlexThetaU,
				    this, _1, _2));

        flexOmegaSOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::computeFlexOmega,
				    this, _1, _2));

        flexOrientationSOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::computeFlexOrientation,
                                    this, _1, _2));

        flexTransformationMatrixSOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::computeFlexTransformationMatrix,
				    this, _1, _2));

        flexPoseThetaUSOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::computeFlexPoseThetaU,
				    this, _1, _2));

        flexVelocityVectorSOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::computeFlexVelocityVector,
				    this, _1, _2));

        comBiasSOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::computeComBias,
                                    this, _1, _2));


        flexInverseSOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::computeFlexInverse,
				    this, _1, _2));

        flexMatrixInverseSOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::computeFlexMatrixInverse,
				    this, _1, _2));

        flexInversePoseThetaUSOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::computeFlexInversePoseThetaU,
				    this, _1, _2));

        flexInverseVelocityVectorSOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::computeFlexInverseVelocityVector,
				    this, _1, _2));

        flexInverseVelocitySOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::computeFlexInverseVelocity,
                    this, _1, _2));

        flexInverseOmegaSOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::computeFlexInverseOmega,
                    this, _1, _2));

        simulatedSensorsSOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::computeSimulatedSensors,
                    this, _1, _2));

        predictedSensorsSOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::computePredictedSensors,
                    this, _1, _2));

        flexibilityComputationTimeSOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::computeFlexibilityComputationTime,
                    this, _1, _2));

        forcesAndMomentsSOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::getForcesAndMoments,
                    this, _1, _2));
        forcesSupport1SOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::getForcesSupport1,
                    this, _1, _2));
        forcesSupport2SOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::getForcesSupport2,
                    this, _1, _2));

        inovationSOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::computeInovation,
                    this, _1, _2));

        predictionSOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::computePrediction,
                    this, _1, _2));

        stateCovarianceSOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::getStateCovariance, this, _1, _2));

        std::ostringstream stateSizeString;
        stateSizeString << stateSize;

        std::ostringstream measurementSizeString;
        stateSizeString << measurementSizeBase;

        std::ostringstream inputSizeString;
        inputSizeString << inputSizeBase;

        std::string docstring;

        //on
        docstring =
                "\n"
                "    Enable (true) or disable (false) the estimation. \n"
                "    Default is false"
                "\n";

        addCommand(std::string("setOn"),
	     new
	     ::dynamicgraph::command::Setter <DGIMUModelBaseFlexEstimation,bool>
                (*this, &DGIMUModelBaseFlexEstimation::setOn, docstring));

        //setComBias
        docstring =
                "\n"
                "    Set a guess for the com bias \n"
                "    takes a tuple of 2 \n"
                "    floating point mumbers as input \n"
                "\n";

        addCommand(std::string("setComBiasGuess"),
             new
             ::dynamicgraph::command::Setter <DGIMUModelBaseFlexEstimation,dynamicgraph::Vector>
                (*this, &DGIMUModelBaseFlexEstimation::setComBiasGuess, docstring));

        //setStateGuess
        docstring =
                "\n"
                "    Set a guess of the flexibility state  \n"
                "    takes a tuple of " + stateSizeString.str() + "\n"
                "    floating point mumbers as input \n"
                "\n";

        addCommand(std::string("setFlexibilityGuess"),
	     new
	     ::dynamicgraph::command::Setter <DGIMUModelBaseFlexEstimation,dynamicgraph::Vector>
                (*this, &DGIMUModelBaseFlexEstimation::setFlexibilityGuess, docstring));

        //setStateGuess
        docstring =
                "\n"
                "    Set the anchorage position for the elastic ropes model \n"
                "\n";

        addCommand(std::string("setPe"),
             new
             ::dynamicgraph::command::Setter <DGIMUModelBaseFlexEstimation,dynamicgraph::Vector>
                (*this, &DGIMUModelBaseFlexEstimation::setPe, docstring));

         //setStateGuessCovariance
        docstring =
                "\n"
                "    Set the covariance matrix of the current flexibility estimation \n"
                "    takes " + stateSizeString.str() + " tuples of" + stateSizeString.str() + "\n"
                "    floating point mumbers as input \n"
                "\n";

        addCommand(std::string("setFlexibilityCovariance"),
	     new
	     ::dynamicgraph::command::Setter <DGIMUModelBaseFlexEstimation,dynamicgraph::Matrix>
                (*this, &DGIMUModelBaseFlexEstimation::setFlexibilityCovariance, docstring));

        //getStateCovariance
        docstring =
                "\n"
                "    Get the covariance matrix of the current flexibility estimation \n"
                "    provides " + stateSizeString.str() + " tuples of" + stateSizeString.str() + "\n"
                "    floating point mumbers as input \n"
                "\n";

        addCommand(std::string("getFlexibilityCovariance"),
	     new
	     ::dynamicgraph::command::Getter <DGIMUModelBaseFlexEstimation,dynamicgraph::Matrix>
                (*this, &DGIMUModelBaseFlexEstimation::getFlexibilityCovariance, docstring));

         //setProcessNoiseCovariance
        docstring =
                "\n"
                "    Set the covariance matrix of the process noise \n"
                "    takes " + stateSizeString.str() + " tuples of" + stateSizeString.str() + "\n"
                "    floating point mumbers as input \n"
                "\n";

        addCommand(std::string("setProcessNoiseCovariance"),
	     new
	     ::dynamicgraph::command::Setter <DGIMUModelBaseFlexEstimation,dynamicgraph::Matrix>
                (*this, &DGIMUModelBaseFlexEstimation::setProcessNoiseCovariance, docstring));


         //getProcessNoiseCovariance
        docstring =
                "\n"
                "    Get the covariance matrix of the process noise \n"
                "    provides" + stateSizeString.str() + " tuples of" + stateSizeString.str() + "\n"
                "    floating point mumbers as input \n"
                "\n";

        addCommand(std::string("getProcessNoiseCovariance"),
	     new
	     ::dynamicgraph::command::Getter <DGIMUModelBaseFlexEstimation,dynamicgraph::Matrix>
                (*this, &DGIMUModelBaseFlexEstimation::getProcessNoiseCovariance, docstring));


                 //setMeasurementNoiseCovariance
        docstring =
                "\n"
                "    Set the covariance matrix of the measuement noise \n"
                "    takes " + measurementSizeString.str() + " tuples of" +  measurementSizeString.str()+ "\n"
                "    floating point mumbers as input \n"
                "\n";

        addCommand(std::string("setMeasurementNoiseCovariance"),
	     new
	     ::dynamicgraph::command::Setter <DGIMUModelBaseFlexEstimation,dynamicgraph::Matrix>
                (*this, &DGIMUModelBaseFlexEstimation::setMeasurementNoiseCovariance, docstring));

        //getMeasurementNoiseCovariance
        docstring =
                "\n"
                "    Get the covariance matrix of the measuement noise \n"
                "    provides " + measurementSizeString.str() + " tuples of" +  measurementSizeString.str()+ "\n"
                "    floating point mumbers as input \n"
                "\n";

        addCommand(std::string("getMeasurementNoiseCovariance"),
	     new
	     ::dynamicgraph::command::Getter <DGIMUModelBaseFlexEstimation,dynamicgraph::Matrix>
                (*this, &DGIMUModelBaseFlexEstimation::getMeasurementNoiseCovariance, docstring));


         //setSamplingPeriod
        docstring =
                "\n"
                "    Set the sampling period of the system \n"
                "    takes a floating point mumber as input \n"
                "\n";

        addCommand(std::string("setSamplingPeriod"),
	     new
	     ::dynamicgraph::command::Setter <DGIMUModelBaseFlexEstimation,double>
                (*this, &DGIMUModelBaseFlexEstimation::setSamplingPeriod, docstring));

        //increment
        docstring  =
                "\n"
                "    Increments the time index of the output signal \n"
                "    takes no argument \n"
                "\n";

        addCommand(std::string("increment"),
                    ::dynamicgraph::command::makeCommandVoid0(*this, & DGIMUModelBaseFlexEstimation::increment ,
    					docstring));

        //increment
        docstring  =
                "\n"
                "    Gets the time index of the flexibility estimation \n"
                "\n";

        addCommand(std::string("getFlexTime"),
                    new ::dynamicgraph::command::Getter <DGIMUModelBaseFlexEstimation,int>
                    (*this, & DGIMUModelBaseFlexEstimation::getFlexTime ,docstring));

        //set the linear and angular stifness et damping of the flexibility
        docstring  =
                "\n"
                "    Sets the linear stifness of the flexibility \n"
                "\n";

        addCommand(std::string("setKfe"),
                   new ::dynamicgraph::command::Setter <DGIMUModelBaseFlexEstimation,dynamicgraph::Matrix>
                    (*this, & DGIMUModelBaseFlexEstimation::setKfe ,docstring));

        docstring  =
                "\n"
                "    Sets the linear damping of the flexibility \n"
                "\n";

        addCommand(std::string("setKfv"),
                   new ::dynamicgraph::command::Setter <DGIMUModelBaseFlexEstimation,dynamicgraph::Matrix>
                    (*this, & DGIMUModelBaseFlexEstimation::setKfv ,docstring));

        docstring  =
                "\n"
                "    Sets the angular stifness of the flexibility \n"
                "\n";

        addCommand(std::string("setKte"),
                   new ::dynamicgraph::command::Setter <DGIMUModelBaseFlexEstimation,dynamicgraph::Matrix>
                    (*this, & DGIMUModelBaseFlexEstimation::setKte ,docstring));

        docstring  =
                "\n"
                "    Sets the angular damping of the flexibility \n"
                "\n";

        addCommand(std::string("setKtv"),
                   new ::dynamicgraph::command::Setter <DGIMUModelBaseFlexEstimation,dynamicgraph::Matrix>
                    (*this, & DGIMUModelBaseFlexEstimation::setKtv ,docstring));


        //set the linear and angular stifness et damping of the flexibility
        docstring  =
                "\n"
                "    Sets the linear ropes stifness of the flexibility \n"
                "\n";

        addCommand(std::string("setKfeRopes"),
                   new ::dynamicgraph::command::Setter <DGIMUModelBaseFlexEstimation,dynamicgraph::Matrix>
                    (*this, & DGIMUModelBaseFlexEstimation::setKfeRopes ,docstring));

        docstring  =
                "\n"
                "    Sets the linear ropes damping of the flexibility \n"
                "\n";

        addCommand(std::string("setKfvRopes"),
                   new ::dynamicgraph::command::Setter <DGIMUModelBaseFlexEstimation,dynamicgraph::Matrix>
                    (*this, & DGIMUModelBaseFlexEstimation::setKfvRopes ,docstring));

        docstring  =
                "\n"
                "    Sets the angular ropes stifness of the flexibility \n"
                "\n";

        addCommand(std::string("setKteRopes"),
                   new ::dynamicgraph::command::Setter <DGIMUModelBaseFlexEstimation,dynamicgraph::Matrix>
                    (*this, & DGIMUModelBaseFlexEstimation::setKteRopes ,docstring));

        docstring  =
                "\n"
                "    Sets the angular ropes damping of the flexibility \n"
                "\n";

        addCommand(std::string("setKtvRopes"),
                   new ::dynamicgraph::command::Setter <DGIMUModelBaseFlexEstimation,dynamicgraph::Matrix>
                    (*this, & DGIMUModelBaseFlexEstimation::setKtvRopes ,docstring));



        docstring  =
                "\n"
                "    Sets the contact model number \n"
                "\n";

        addCommand(std::string("setContactModel"),
                   new ::dynamicgraph::command::Setter <DGIMUModelBaseFlexEstimation,unsigned>
                    (*this, & DGIMUModelBaseFlexEstimation::setContactModel ,docstring));

        docstring  =
                "\n"
                "    Sets if the force sensors are used or not. "
                " make sure the measurement vectors already contain the force signals. \n"
                "\n";

        addCommand(std::string("setWithForceSensors"),
                   new ::dynamicgraph::command::Setter <DGIMUModelBaseFlexEstimation,bool >
                    (*this, & DGIMUModelBaseFlexEstimation::setWithForce,docstring));

        docstring  =
                "\n"
                "    Sets if the flexibility state contains comBias or not "
                "\n";

        addCommand(std::string("setWithComBias"),
                   new ::dynamicgraph::command::Setter <DGIMUModelBaseFlexEstimation,bool >
                    (*this, & DGIMUModelBaseFlexEstimation::setWithComBias,docstring));

        docstring  =
                "\n"
                "    Sets if an absolute position sensor is used or not. "
                " make sure the measurement vectors already contain the measurement signals. \n"
                "\n";

        addCommand(std::string("setAbsolutePosition"),
                   new ::dynamicgraph::command::Setter <DGIMUModelBaseFlexEstimation,bool >
                    (*this, & DGIMUModelBaseFlexEstimation::setWithAbsolutePosition,docstring));

        docstring  =
                "\n"
                "    Sets if an unmodeled forces are used or not. "
                "\n";

        addCommand(std::string("setWithUnmodeledMeasurements"),
                   new ::dynamicgraph::command::Setter <DGIMUModelBaseFlexEstimation,bool >
                    (*this, & DGIMUModelBaseFlexEstimation::setWithUnmodeledMeasurements,docstring));

        docstring  =
                "\n"
                "    Sets if the config signal is used or not. "
                "\n";

        addCommand(std::string("setWithConfigSignal"),
                   new ::dynamicgraph::command::Setter <DGIMUModelBaseFlexEstimation,bool >
                    (*this, & DGIMUModelBaseFlexEstimation::setWithConfigSignal,docstring));

        docstring  =
                "\n"
                "    Sets unmodeled force variance"
                "\n";

        addCommand(std::string("setUnmodeledForceVariance"),
                   new ::dynamicgraph::command::Setter <DGIMUModelBaseFlexEstimation,double >
                    (*this, & DGIMUModelBaseFlexEstimation::setUnmodeledForceVariance,docstring));

        docstring  =
                "\n"
                "    Sets the variance of the noise of force/torque sensors. "
                "\n";

        addCommand(std::string("setForceVariance"),
                   new ::dynamicgraph::command::Setter <DGIMUModelBaseFlexEstimation,double >
                    (*this, & DGIMUModelBaseFlexEstimation::setForceVariance,docstring));

        docstring  =
                "\n"
                "    Sets the variance of the noise of absolute position sensors. "
                "\n";

        addCommand(std::string("setAbsolutePosVariance"),
                   new ::dynamicgraph::command::Setter <DGIMUModelBaseFlexEstimation,double >
                    (*this, & DGIMUModelBaseFlexEstimation::setAbsolutePosVariance,docstring));

        addCommand(std::string("setVirtualBiasCom"),
                   new ::dynamicgraph::command::Setter <DGIMUModelBaseFlexEstimation,dynamicgraph::Vector>
                    (*this, & DGIMUModelBaseFlexEstimation::setBias ,docstring));

        addCommand(std::string("setRobotMass"),
                   new ::dynamicgraph::command::Setter <DGIMUModelBaseFlexEstimation,double>
                    (*this, & DGIMUModelBaseFlexEstimation::setRobotMass ,docstring));

        addCommand(std::string("getRobotMass"),
                   new ::dynamicgraph::command::Getter <DGIMUModelBaseFlexEstimation,double>
                    (*this, & DGIMUModelBaseFlexEstimation::getRobotMass ,docstring));

        addCommand(std::string("setForcesLimit"),
                   new ::dynamicgraph::command::Setter <DGIMUModelBaseFlexEstimation,dynamicgraph::Vector>
                    (*this, & DGIMUModelBaseFlexEstimation::setForcesLimit ,docstring));

        addCommand(std::string("getForcesLimit"),
                   new ::dynamicgraph::command::Getter <DGIMUModelBaseFlexEstimation,dynamicgraph::Vector>
                    (*this, & DGIMUModelBaseFlexEstimation::getForcesLimit ,docstring));

        addCommand(std::string("setTorquesLimit"),
                   new ::dynamicgraph::command::Setter <DGIMUModelBaseFlexEstimation,dynamicgraph::Vector>
                    (*this, & DGIMUModelBaseFlexEstimation::setTorquesLimit ,docstring));

        addCommand(std::string("getTorquesLimit"),
                   new ::dynamicgraph::command::Getter <DGIMUModelBaseFlexEstimation,dynamicgraph::Vector>
                    (*this, & DGIMUModelBaseFlexEstimation::getTorquesLimit ,docstring));

        docstring  =
                "\n"
                "    Sets if there is a saturation or not in forces and torques. "
                "\n";

        addCommand(std::string("setLimitOn"),
                   new ::dynamicgraph::command::Setter <DGIMUModelBaseFlexEstimation,bool >
                    (*this, & DGIMUModelBaseFlexEstimation::setLimitOn,docstring));

        docstring  =
                "\n"
                "    Gets if there is a saturation or not in forces and torques. "
                "\n";

        addCommand(std::string("getLimitOn"),
                   new ::dynamicgraph::command::Getter <DGIMUModelBaseFlexEstimation,bool>
                    (*this, & DGIMUModelBaseFlexEstimation::getLimitOn ,docstring));


        stateObservation::ObserverBase::InputVector input(inputSizeBase); input.setZero();
        inputSIN.setConstant(convertVector<dynamicgraph::Vector>(input));

        stateObservation::Vector measure(measurementSizeBase); measure.setZero();
        measurementSIN.setConstant(convertVector<dynamicgraph::Vector>(measure));

        contactsNbrSIN.setConstant(0);

        withComBias_=false;
        estimator_.setWithComBias(withComBias_);

        bias_.resize(2); bias_.setZero();

        // Contacts model
        contactsModelSIN.setConstant(1);
        contactsModel_=1;
        estimator_.setContactModel(1);

        // Config
        withConfigSignal_ = false;
        withForce_=true;
        withUnmodeledForces_ = false;
        withAbsolutePose_ = false;
        config_.resize(3); config_.setZero();
        configSIN.setConstant(config_);

        currentTime_=0;
        Q_=estimator_.getProcessNoiseCovariance();
        recomputeQ_=false;

    }

    DGIMUModelBaseFlexEstimation::~DGIMUModelBaseFlexEstimation()
    {
    }


    dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeState
                  (dynamicgraph::Vector & state, const int& inTime)
    {

        std::cout << "\n" << inTime << std::endl;

#ifdef SOT_STATE_OBSERVATION_CHECK_UNIQUENESS_IN_TIME
        if (inTime!=currentTime_)
        {
            currentTime_=inTime;
#endif

        getProfiler().start(PROFILE_READ_INPUT_SIGNALS);
        const dynamicgraph::Vector & measurement = measurementSIN.access(inTime);
        const dynamicgraph::Vector & input = inputSIN.access(inTime);
        const unsigned & contactNb = contactsNbrSIN.access(inTime);
        const unsigned & contactsModel = contactsModelSIN.access(inTime);
        getProfiler().stop(PROFILE_READ_INPUT_SIGNALS);

        getProfiler().start(PROFILE_READ_ESTIMATOR_CONFIG);
        // Update of the state size
        if(estimator_.getWithComBias()!=withComBias_) estimator_.setWithComBias(withComBias_);

        if (withConfigSignal_)
        {
            setConfig(inTime);
        }
        else
        {
            if(withForce_!=estimator_.getWithForcesMeasurements()) estimator_.setWithForcesMeasurements(withForce_);
            if(withUnmodeledForces_!=estimator_.getWithUnmodeledMeasurements()) estimator_.setWithUnmodeledMeasurements(withUnmodeledForces_);
            if(withAbsolutePose_!=estimator_.getWithAbsolutePos()) estimator_.setWithAbsolutePos(withAbsolutePose_);
        }

        if(contactsModel_!=contactsModel)
        {
            contactsModel_=contactsModel;
            estimator_.setContactModel(contactsModel);
        }

        // Update of inputSize_ considering contactsNb
        if (contactNb!=estimator_.getContactsNumber())
        {
            contactNumber_ = contactNb;
            estimator_.setContactsNumber(contactNb);
        }

        // Update the process noise covariance
        if(recomputeQ_)
        {
            estimator_.setProcessNoiseCovariance(Q_);
            recomputeQ_=false;
        }

        estimator_.setMeasurement((convertVector<stateObservation::Vector>(measurement)).head(estimator_.getMeasurementSize()));

        stateObservation::Vector inputWBias = convertVector<stateObservation::Vector>(input);
        inputWBias.block(0,0,2,1)=inputWBias.block(0,0,2,1)+bias_;//for test purpose only

        estimator_.setMeasurementInput(inputWBias);
        getProfiler().stop(PROFILE_READ_ESTIMATOR_CONFIG);

#ifdef SOT_STATE_OBSERVATION_CHECK_UNIQUENESS_IN_TIME
        }
#endif

        getProfiler().start(PROFILE_READ_ESTIMATOR_ALONE);
        state = convertVector<dynamicgraph::Vector>(estimator_.getFlexibilityVector());
        getProfiler().stop(PROFILE_READ_ESTIMATOR_ALONE);

        return state;
    }

    ::dynamicgraph::Matrix& DGIMUModelBaseFlexEstimation::getObservationMatrix
            (::dynamicgraph::Matrix & observationMatrix, const int& inTime)
    {
        stateSOUT(inTime);
        observationMatrix = convertMatrix<dynamicgraph::Matrix>(estimator_.computeLocalObservationMatrix());
        return observationMatrix;
    }


    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeMomentaDotFromForces
                  (dynamicgraph::Vector & momentaDot, const int& inTime)
    {
        stateSOUT(inTime);

        momentaDot = convertVector<dynamicgraph::Vector>(estimator_.getMomentaDotFromForces());

        return momentaDot;
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeMomentaDotFromKinematics
                  (dynamicgraph::Vector & momentaDot, const int& inTime)
    {
        stateSOUT(inTime);

        momentaDot = convertVector<dynamicgraph::Vector>(estimator_.getMomentaDotFromKinematics());

        return momentaDot;
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeAccelerations
                  (dynamicgraph::Vector & accelerations, const int& inTime)
    {
        stateSOUT(inTime);

        accelerations = convertVector<dynamicgraph::Vector>(estimator_.computeAccelerations());

        return accelerations;
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeFlexibility
                  (dynamicgraph::Vector & flexibility, const int& inTime)
    {
        stateSOUT(inTime);

        flexibility = convertVector<dynamicgraph::Vector>
                      (estimator_.getFlexibilityVector().segment(0,18));

        return flexibility;
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeFlexPosition
                        (::dynamicgraph::Vector & flexibilityPosition, const int& inTime)
    {

        stateSOUT(inTime);

        flexibilityPosition = convertVector<dynamicgraph::Vector>
                              (estimator_.getFlexibilityVector().segment(IMUElasticLocalFrameDynamicalSystem::state::pos,3));

        return flexibilityPosition;
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeFlexVelocity
                        (::dynamicgraph::Vector & flexibilityVelocity, const int& inTime)
    {
        stateSOUT(inTime);

        flexibilityVelocity = convertVector<dynamicgraph::Vector>
            (estimator_.getFlexibilityVector().segment(IMUElasticLocalFrameDynamicalSystem::state::linVel,3));

        return flexibilityVelocity;
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeFlexThetaU
                        (::dynamicgraph::Vector & flexibilityThetaU, const int& inTime)
    {
        stateSOUT(inTime);

        flexibilityThetaU = convertVector<dynamicgraph::Vector>
                (estimator_.getFlexibilityVector().segment(IMUElasticLocalFrameDynamicalSystem::state::ori,3));

        return flexibilityThetaU;
    }


    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeFlexOmega
                        (::dynamicgraph::Vector & flexibilityOmega, const int& inTime)
    {
        stateSOUT(inTime);

        flexibilityOmega = convertVector<dynamicgraph::Vector>
                (estimator_.getFlexibilityVector().segment(IMUElasticLocalFrameDynamicalSystem::state::angVel,3));

        return flexibilityOmega;
    }

    ::dynamicgraph::Matrix& DGIMUModelBaseFlexEstimation::computeFlexOrientation
                        (::dynamicgraph::Matrix & flexibilityOrientation, const int& inTime)
    {
        stateSOUT(inTime);

        flexibilityOrientation = convertMatrix<dynamicgraph::Matrix>
                                 (kine::rotationVectorToRotationMatrix(estimator_.getFlexibilityVector().segment(IMUElasticLocalFrameDynamicalSystem::state::ori,3)));

        return flexibilityOrientation;
    }

    ::dynamicgraph::sot::MatrixHomogeneous& DGIMUModelBaseFlexEstimation::computeFlexTransformationMatrix
                        (::dynamicgraph::sot::MatrixHomogeneous & flexibilityTransformationMatrix, const int& inTime)
    {
        stateSOUT(inTime);

        flexibilityTransformationMatrix = convertMatrix<dynamicgraph::Matrix>(estimator_.getFlexibility());

        return flexibilityTransformationMatrix;
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeFlexPoseThetaU
                        (::dynamicgraph::Vector & flexibilityPoseThetaU, const int& inTime)
    {
        //std::cout << "computeFlexPoseThetaU " << inTime << std::endl;

        stateSOUT(inTime);

        stateObservation::Vector v = stateObservation::Vector::Zero(6,1);
        v.head(3) = estimator_.getFlexibilityVector().segment(IMUElasticLocalFrameDynamicalSystem::state::pos,3);
        v.tail(3) = estimator_.getFlexibilityVector().segment(IMUElasticLocalFrameDynamicalSystem::state::ori,3);

        flexibilityPoseThetaU = convertVector<dynamicgraph::Vector>(v);

        return flexibilityPoseThetaU;
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeComBias
                        (::dynamicgraph::Vector & comBias, const int& inTime)
    {
        stateSOUT(inTime);

        stateObservation::Vector3 bias; bias.setZero();
        bias.segment(0,2) = estimator_.getFlexibilityVector().segment(IMUElasticLocalFrameDynamicalSystem::state::comBias,2);

        comBias= convertVector<dynamicgraph::Vector>(bias);
        return comBias;
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeFlexVelocityVector
                        (::dynamicgraph::Vector & flexibilityVelocityVector, const int& inTime)
    {
        //std::cout << "computeFlexPoseThetaU " << inTime << std::endl;

        stateSOUT(inTime);

        stateObservation::Vector v = stateObservation::Vector::Zero(6,1);
        v.head(3) = estimator_.getFlexibilityVector().segment(IMUElasticLocalFrameDynamicalSystem::state::linVel,3);
        v.tail(3) = estimator_.getFlexibilityVector().segment(IMUElasticLocalFrameDynamicalSystem::state::angVel,3);

        flexibilityVelocityVector = convertVector<dynamicgraph::Vector>(v);

        return flexibilityVelocityVector;
    }


    dynamicgraph::Vector & DGIMUModelBaseFlexEstimation::getStateCovariance(::dynamicgraph::Vector & stateCovariance, const int& inTime)
    {
        stateCovariance=convertVector<dynamicgraph::Vector>(estimator_.getStateCovariance());
        return stateCovariance;
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::getForcesAndMoments(::dynamicgraph::Vector & forcesAndMoments, const int& inTime)
    {
        stateSOUT(inTime);
        forcesAndMoments=convertVector<dynamicgraph::Vector>(estimator_.getForcesAndMoments());
        return forcesAndMoments;
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::getForcesSupport1(::dynamicgraph::Vector & forcesSupport1, const int& inTime)
    {
        stateSOUT(inTime);

        stateObservation::Vector forcesAndMoments=estimator_.getForcesAndMoments();
        forcesSupport1.resize(6);
        if(forcesAndMoments.size() >= 6)
        {
            forcesSupport1=convertVector<dynamicgraph::Vector>((forcesAndMoments).segment(0,6));
        }
        else
        {
            forcesSupport1.setZero();
        }

        return forcesSupport1;
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::getForcesSupport2(::dynamicgraph::Vector & forcesSupport2, const int& inTime)
    {
        stateSOUT(inTime);

        stateObservation::Vector forcesAndMoments=estimator_.getForcesAndMoments();
        forcesSupport2.resize(6);
        if(forcesAndMoments.size()>=12)
        {
            forcesSupport2=convertVector<dynamicgraph::Vector>((forcesAndMoments).segment(6,6));
        }
        else
        {
            forcesSupport2.setZero();
        }

        return forcesSupport2;
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeFlexInverse
                        (::dynamicgraph::Vector & flexInverse, const int& inTime)
    {
        stateSOUT(inTime);

        flexInverse = convertVector<dynamicgraph::Vector>
            (stateObservation::kine::invertState(estimator_.getFlexibilityVector()));

        return flexInverse;
    }

    ::dynamicgraph::sot::MatrixHomogeneous& DGIMUModelBaseFlexEstimation::computeFlexMatrixInverse
                        (::dynamicgraph::sot::MatrixHomogeneous & flexMatrixInverse, const int& inTime)
    {

        stateSOUT(inTime);

        flexMatrixInverse = convertMatrix<dynamicgraph::Matrix>
            (stateObservation::kine::invertHomoMatrix(estimator_.getFlexibility()));

        return flexMatrixInverse;
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeFlexInversePoseThetaU
                        (::dynamicgraph::Vector & flexInversePoseThetaU, const int& inTime)
    {
        const ::dynamicgraph::Vector& fi=flexInverseSOUT(inTime);

        flexInversePoseThetaU.resize(6);

        setSubvector(flexInversePoseThetaU, 0,
                            getSubvector(fi,IMUElasticLocalFrameDynamicalSystem::state::pos,3));
        setSubvector(flexInversePoseThetaU, 3,
                            getSubvector(fi,IMUElasticLocalFrameDynamicalSystem::state::ori,3));

        return flexInversePoseThetaU;
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeFlexInverseVelocityVector
                        (::dynamicgraph::Vector & flexInverseVelocityVector, const int& inTime)
    {
        const ::dynamicgraph::Vector& fi=flexInverseSOUT(inTime);

        flexInverseVelocityVector.resize(6);

        setSubvector(flexInverseVelocityVector, 0,
                            getSubvector(fi,IMUElasticLocalFrameDynamicalSystem::state::linVel,3));
        setSubvector(flexInverseVelocityVector, 3,
                            getSubvector(fi,IMUElasticLocalFrameDynamicalSystem::state::angVel,3));

        return flexInverseVelocityVector;
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeFlexInverseVelocity
                        (::dynamicgraph::Vector & flexInverseVelocity, const int& inTime)
    {
        const ::dynamicgraph::Vector& fi=flexInverseSOUT(inTime);

        flexInverseVelocity = getSubvector(fi,IMUElasticLocalFrameDynamicalSystem::state::linVel,3);

        return flexInverseVelocity;
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeFlexInverseOmega
                        (::dynamicgraph::Vector & flexInverseOmega, const int& inTime)
    {
        const ::dynamicgraph::Vector& fi=flexInverseSOUT(inTime);

        flexInverseOmega = getSubvector(fi,IMUElasticLocalFrameDynamicalSystem::state::angVel,3);

        return flexInverseOmega;
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeSimulatedSensors
                        (::dynamicgraph::Vector & sensorSignal, const int& inTime)
    {
        stateSOUT(inTime);

        return sensorSignal = convertVector <dynamicgraph::Vector>
                                        (estimator_.getSimulatedMeasurement());
    }

        ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computePredictedSensors
                        (::dynamicgraph::Vector & sensorSignal, const int& inTime)
    {
        stateSOUT(inTime);

        return sensorSignal = convertVector <dynamicgraph::Vector>
                                        (estimator_.getLastPredictedMeasurement());
    }

        double& DGIMUModelBaseFlexEstimation::computeFlexibilityComputationTime
                    (double& flexibilityComputationTime, const int &inTime)
    {
        return flexibilityComputationTime=estimator_.getComputeFlexibilityTime();
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeInovation
                        (::dynamicgraph::Vector & inovation, const int& inTime)
    {
        stateSOUT(inTime);

        return inovation = convertVector <dynamicgraph::Vector>
                                        (estimator_.getInovation());
    }


    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computePrediction
                        (::dynamicgraph::Vector & prediction, const int& inTime)
    {
        stateSOUT(inTime);

        return prediction = convertVector <dynamicgraph::Vector>
                                        (estimator_.getLastPrediction());
    }


    void DGIMUModelBaseFlexEstimation::display(std::ostream& os) const
    {
        os << "DGIMUModelBaseFlexEstimation "<<getName();
        try
        {
            getProfiler().report_all(3, os);
        }
        catch (int e) {}
    }
}
