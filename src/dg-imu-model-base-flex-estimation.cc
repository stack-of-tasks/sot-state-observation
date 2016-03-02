#include <sstream>

#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-bind.h>

#include <sot-state-observation/dg-imu-model-base-flex-estimation.hh>

namespace sotStateObservation
{
    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN ( DGIMUModelBaseFlexEstimation, "DGIMUModelBaseFlexEstimation" );

    DGIMUModelBaseFlexEstimation::DGIMUModelBaseFlexEstimation(const std::string & inName):
        Entity(inName),
        measurementSIN(0x0 , "DGIMUModelBaseFlexEstimation("+inName+")::input(vector)::measurement"),
        inputSIN(0x0 , "DGIMUModelBaseFlexEstimation("+inName+")::input(vector)::input"),
        contactsNbrSIN(0x0 , "DGIMUModelBaseFlexEstimation("+inName+")::input(unsigned)::contactNbr"),
        stateSOUT("DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::state"),
        flexibilitySOUT("DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexibility"),
        flexPositionSOUT(flexibilitySOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexPosition"),
        flexVelocitySOUT(flexibilitySOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexVelocity"),
        flexAccelerationSOUT(flexibilitySOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexAcceleration"),
        flexPoseThetaUSOUT(flexibilitySOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexPoseThetaU"),
        comBiasSOUT(flexibilitySOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::comBias"),
        flexOmegaSOUT(flexibilitySOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexOmega"),
        flexOmegaDotSOUT(flexibilitySOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexOmegaDot"),
        flexTransformationMatrixSOUT(flexibilitySOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(homogeneousMatrix)::flexTransformationMatrix"),
        flexThetaUSOUT(flexibilitySOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexThetaU"),
        flexVelocityVectorSOUT(flexibilitySOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexVelocityVector"),
        flexInverseSOUT (flexibilitySOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexInverse"),
        flexMatrixInverseSOUT(flexInverseSOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(homogeneousMatrix)::flexMatrixInverse"),
        flexInversePoseThetaUSOUT(flexInverseSOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexInversePoseThetaU"),
        flexInverseVelocityVectorSOUT(flexInverseSOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexInverseVelocityVector"),
        flexAccelerationVectorSOUT(flexibilitySOUT,    "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexAccelerationVector"),
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

        signalRegistration (stateSOUT);
        signalRegistration (flexibilitySOUT);
        signalRegistration (flexPositionSOUT);
        signalRegistration (flexVelocitySOUT);
        signalRegistration (flexAccelerationSOUT);
        signalRegistration (flexThetaUSOUT);
        signalRegistration (flexOmegaSOUT);
        signalRegistration (flexOmegaDotSOUT);
        signalRegistration (flexTransformationMatrixSOUT);
        signalRegistration (flexPoseThetaUSOUT);
        signalRegistration (flexVelocityVectorSOUT);
        signalRegistration (flexAccelerationVectorSOUT);

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

        flexibilitySOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::computeFlexibility,
				    this, _1, _2));

        flexPositionSOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::computeFlexPosition,
				    this, _1, _2));

        flexVelocitySOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::computeFlexVelocity,
				    this, _1, _2));

        flexAccelerationSOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::computeFlexAcceleration,
				    this, _1, _2));

        flexThetaUSOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::computeFlexThetaU,
				    this, _1, _2));

        flexOmegaSOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::computeFlexOmega,
				    this, _1, _2));

        flexOmegaDotSOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::computeFlexOmegaDot,
                    this, _1, _2));


        flexTransformationMatrixSOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::computeFlexTransformationMatrix,
				    this, _1, _2));

        flexPoseThetaUSOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::computeFlexPoseThetaU,
				    this, _1, _2));

        flexVelocityVectorSOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::computeFlexVelocityVector,
				    this, _1, _2));

        flexAccelerationVectorSOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::computeFlexAccelerationVector,
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
        stateSizeString << measurementSize;

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

        withComBias_=false;
        estimator_.setWithComBias(withComBias_);

        bias_.resize(2); bias_.setZero();

        stateObservation::ObserverBase::InputVector input;
        input.resize(inputSizeBase);
        input <<    0.0135672,
                    0.001536,
                    0.80771,
                    -2.50425e-06,
                    -1.03787e-08,
                    5.4317e-08,
                    -2.50434e-06,
                    -1.03944e-08,
                    5.45321e-08,
                    48.1348,
                    46.9498,
                    1.76068,
                    -0.0863332,
                    -0.59487,
                    -0.0402246,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    -0.098,
                    -1.21619e-10,
                    1.1174,
                    3.06752e-22,
                    -1.06094e-20,
                    7.75345e-22,
                    -2.84609e-06,
                    -1.18496e-08,
                    -4.52691e-18,
                    2.95535e-20,
                    -1.0346e-18,
                    7.58731e-20,
                    -0.000284609,
                    -1.18496e-06,
                    -4.52691e-16;
//                    0.00949046,
//                    0.095,
//                    1.19005e-06,
//                    0,
//                    0,
//                    0,
//                    0.00949605,
//                    -0.095,
//                    1.19343e-06,
//                    0,
//                    0,
//                    0;
        inputSIN.setConstant(convertVector<dynamicgraph::Vector>(input));

        stateObservation::Vector measure(measurementSize);
        measure << -0.0330502,
                   -0.169031,
                   9.91812,
                   0.0137655,
                   0.0797922,
                   0.000778988;
//                   6.15302,
//                   -8.44315,
//                   245.826,
//                   0.749795,
//                   2.59329,
//                   0.140388,
//                   5.59534,
//                   7.49882,
//                   227.461,
//                   0.24084,
//                   2.74922,
//                   -0.120347;
        measurementSIN.setConstant(convertVector<dynamicgraph::Vector>(measure));

        contactsNbrSIN.setConstant(0);

        currentTime_=-1;

        dynamicgraph::Vector state(estimator_.getStateSize()+estimator_.getWithComBias()*2);
        state=computeState(state,0);

        Q_=estimator_.getProcessNoiseCovariance();
        recomputeQ_=false;
    }

    DGIMUModelBaseFlexEstimation::~DGIMUModelBaseFlexEstimation()
    {
    }



    dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeState
                  (dynamicgraph::Vector & state, const int& inTime)
    {


#ifdef SOT_STATE_OBSERVATION_CHECK_UNIQUENESS_IN_TIME
        if (inTime!=currentTime_)
        {
            currentTime_=inTime;
#endif
        const dynamicgraph::Vector & measurement = measurementSIN.access(inTime);
        const dynamicgraph::Vector & input = inputSIN.access(inTime);
        const unsigned & contactNb = contactsNbrSIN.access(inTime);

        // Update of the state size
        if(estimator_.getWithComBias()!=withComBias_) estimator_.setWithComBias(withComBias_);

        // Update of inputSize_ considering contactsNb
        if (contactNumber_!= contactNb)
        {
            contactNumber_ = contactNb;
            estimator_.setContactsNumber(contactNb);
        }

        // Update the process noise covariance
        if(recomputeQ_) {
            estimator_.setProcessNoiseCovariance(Q_);
            recomputeQ_=false;
        }

        estimator_.setMeasurement((convertVector<stateObservation::Vector>(measurement)).head(estimator_.getMeasurementSize()));

        stateObservation::Vector inputWBias = convertVector<stateObservation::Vector>(input);
        inputWBias.block(0,0,2,1)=inputWBias.block(0,0,2,1)+bias_;//for test purpose only

        estimator_.setMeasurementInput(inputWBias);

#ifdef SOT_STATE_OBSERVATION_CHECK_UNIQUENESS_IN_TIME
        }
#endif
        state = convertVector<dynamicgraph::Vector>(estimator_.getFlexibilityVector());

        return state;
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
                    (estimator_.getFlexibilityVector().segment(stateObservation::kine::pos,3));

        return flexibilityPosition;
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeFlexVelocity
                        (::dynamicgraph::Vector & flexibilityVelocity, const int& inTime)
    {
        stateSOUT(inTime);

        flexibilityVelocity = convertVector<dynamicgraph::Vector>
            (estimator_.getFlexibilityVector().segment(stateObservation::kine::linVel,3));

        return flexibilityVelocity;
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeFlexAcceleration
                        (::dynamicgraph::Vector & flexibilityAcceleration, const int& inTime)
    {
        stateSOUT(inTime);

        flexibilityAcceleration = convertVector<dynamicgraph::Vector>
                (estimator_.getFlexibilityVector().segment(stateObservation::kine::linAcc,3));

        return flexibilityAcceleration;
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeFlexThetaU
                        (::dynamicgraph::Vector & flexibilityThetaU, const int& inTime)
    {
        stateSOUT(inTime);

        flexibilityThetaU = convertVector<dynamicgraph::Vector>
                (estimator_.getFlexibilityVector().segment(stateObservation::kine::ori,3));

        return flexibilityThetaU;
    }


    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeFlexOmega
                        (::dynamicgraph::Vector & flexibilityOmega, const int& inTime)
    {
        stateSOUT(inTime);

        flexibilityOmega = convertVector<dynamicgraph::Vector>
                (estimator_.getFlexibilityVector().segment(stateObservation::kine::angVel,3));

        return flexibilityOmega;
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeFlexOmegaDot
                        (::dynamicgraph::Vector & flexibilityOmegaDot, const int& inTime)
    {
        stateSOUT(inTime);

        flexibilityOmegaDot = convertVector<dynamicgraph::Vector>
                            (estimator_.getFlexibilityVector().segment(stateObservation::kine::angAcc,3));

        return flexibilityOmegaDot;
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
        v.head(3) = estimator_.getFlexibilityVector().segment(stateObservation::kine::pos,3);
        v.tail(3) = estimator_.getFlexibilityVector().segment(stateObservation::kine::ori,3);

        flexibilityPoseThetaU = convertVector<dynamicgraph::Vector>(v);

        return flexibilityPoseThetaU;
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeComBias
                        (::dynamicgraph::Vector & comBias, const int& inTime)
    {
        stateSOUT(inTime);

        stateObservation::Vector3 bias; bias.setZero();
        bias.segment(0,2) = estimator_.getFlexibilityVector().segment(stateObservation::kine::comBias,2);

        comBias= convertVector<dynamicgraph::Vector>(bias);
        return comBias;
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeFlexVelocityVector
                        (::dynamicgraph::Vector & flexibilityVelocityVector, const int& inTime)
    {
        //std::cout << "computeFlexPoseThetaU " << inTime << std::endl;

        stateSOUT(inTime);

        stateObservation::Vector v = stateObservation::Vector::Zero(6,1);
        v.head(3) = estimator_.getFlexibilityVector().segment(stateObservation::kine::linVel,3);
        v.tail(3) = estimator_.getFlexibilityVector().segment(stateObservation::kine::angVel,3);

        flexibilityVelocityVector = convertVector<dynamicgraph::Vector>(v);

        return flexibilityVelocityVector;
    }


    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeFlexAccelerationVector
                        (::dynamicgraph::Vector & flexibilityAccelerationVector, const int& inTime)
    {
        //std::cout << "computeFlexPoseThetaU " << inTime << std::endl;

        stateSOUT(inTime);

        stateObservation::Vector v = stateObservation::Vector::Zero(6,1);
        v.head(3) = estimator_.getFlexibilityVector().segment(stateObservation::kine::linAcc,3);
        v.tail(3) = estimator_.getFlexibilityVector().segment(stateObservation::kine::angAcc,3);

        flexibilityAccelerationVector = convertVector<dynamicgraph::Vector>(v);

        return flexibilityAccelerationVector;
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::getForcesAndMoments(::dynamicgraph::Vector & forcesAndMoments, const int& inTime)
    {
        stateSOUT(inTime);

        forcesAndMoments=convertVector<dynamicgraph::Vector>(estimator_.getForcesAndMoments());

        return forcesAndMoments;
    }

    dynamicgraph::Vector & DGIMUModelBaseFlexEstimation::getStateCovariance(::dynamicgraph::Vector & stateCovariance, const int& inTime)
    {
        stateCovariance=convertVector<dynamicgraph::Vector>(estimator_.getStateCovariance());
        return stateCovariance;
    }




    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::getForcesSupport1(::dynamicgraph::Vector & forcesSupport1, const int& inTime)
    {
        stateSOUT(inTime);

        stateObservation::Vector forcesAndMoments=estimator_.getForcesAndMoments();
        //std::cout << "forcesAndMoments" << forcesAndMoments.transpose() << std::endl;
        forcesSupport1.resize(6);
        if(forcesAndMoments.size() >= 6){
            forcesSupport1=convertVector<dynamicgraph::Vector>((forcesAndMoments).block(0,0,6,1));
        }else{
            forcesSupport1.setZero();
        }

        return forcesSupport1;
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::getForcesSupport2(::dynamicgraph::Vector & forcesSupport2, const int& inTime)
    {
        stateSOUT(inTime);

        stateObservation::Vector forcesAndMoments=estimator_.getForcesAndMoments();
        forcesSupport2.resize(6);
        if(forcesAndMoments.size()==12){
            forcesSupport2=convertVector<dynamicgraph::Vector>((forcesAndMoments).block(6,0,6,1));
        }else{
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
                            getSubvector(fi,stateObservation::kine::pos,3));
        setSubvector(flexInversePoseThetaU, 3,
                            getSubvector(fi,stateObservation::kine::ori,3));

        return flexInversePoseThetaU;
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeFlexInverseVelocityVector
                        (::dynamicgraph::Vector & flexInverseVelocityVector, const int& inTime)
    {
        const ::dynamicgraph::Vector& fi=flexInverseSOUT(inTime);

        flexInverseVelocityVector.resize(6);

        setSubvector(flexInverseVelocityVector, 0,
                            getSubvector(fi,stateObservation::kine::linVel,3));
        setSubvector(flexInverseVelocityVector, 3,
                            getSubvector(fi,stateObservation::kine::angVel,3));

        return flexInverseVelocityVector;
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeFlexInverseVelocity
                        (::dynamicgraph::Vector & flexInverseVelocity, const int& inTime)
    {
        const ::dynamicgraph::Vector& fi=flexInverseSOUT(inTime);

        flexInverseVelocity = getSubvector(fi,stateObservation::kine::linVel,3);

        return flexInverseVelocity;
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeFlexInverseOmega
                        (::dynamicgraph::Vector & flexInverseOmega, const int& inTime)
    {
        const ::dynamicgraph::Vector& fi=flexInverseSOUT(inTime);

        flexInverseOmega = getSubvector(fi,stateObservation::kine::angVel,3);

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
}
