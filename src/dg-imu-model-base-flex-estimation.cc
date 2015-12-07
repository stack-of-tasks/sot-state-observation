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
        flexibilitySOUT("DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexibility"),
        flexPositionSOUT(flexibilitySOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexPosition"),
        flexVelocitySOUT(flexibilitySOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexVelocity"),
        flexAccelerationSOUT(flexibilitySOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexAcceleration"),
        flexPoseThetaUSOUT(flexibilitySOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexPoseThetaU"),
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
        predictionSOUT(flexibilitySOUT, "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::prediction")
    {
#ifdef SOT_STATE_OBSERVATION_CHECK_UNIQUENESS_IN_TIME
        currentTime_=0;
#endif

        signalRegistration (measurementSIN);
        signalRegistration (inputSIN);
        signalRegistration (contactsNbrSIN);

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

        dynamicgraph::Vector measure(measurementSize);
        //dynamicgraph::Vector input(inputSizeBase);

        stateObservation::ObserverBase::InputVector input; // for init
        input.resize(42);
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


        dynamicgraph::Vector flexibility(stateSize);

        dynamicgraph::Vector simulatedMeasurement(measurementSize);
        dynamicgraph::Vector inovation(stateSize);

        dynamicgraph::Vector flexPosition(3);
        dynamicgraph::Vector flexVelocity(3);
        dynamicgraph::Vector flexAcceleration(3);
        dynamicgraph::Vector flexThetaU(3);
        dynamicgraph::Vector flexOmega(3);
        dynamicgraph::Vector flexOmegaDot(3);
        dynamicgraph::Vector contactPosition(3);

        dynamicgraph::Vector flexPoseThetaU(6);
        dynamicgraph::Matrix flexTransformationMatrix(4,4);
        dynamicgraph::Vector flexVelocityVector(6);

        dynamicgraph::Vector flexInverseState(stateSize);
        dynamicgraph::Matrix flexInverseTransformationMatrix(4,4);
        dynamicgraph::Vector flexInverseThetaU(6);
        dynamicgraph::Vector flexInverseVelocityVector(6);
        dynamicgraph::Vector flexInverseVelocity(3);
        dynamicgraph::Vector flexInverseOmega(3);

        flexTransformationMatrix.setIdentity();
        flexInverseTransformationMatrix.setIdentity();


        measurementSIN.setConstant(measure);

        contactsNbrSIN.setConstant(0);


        flexibilitySOUT.setConstant(flexibility);

        flexPositionSOUT.setConstant(flexPosition);
        flexVelocitySOUT.setConstant(flexVelocity);
        flexAccelerationSOUT.setConstant(flexAcceleration);
        flexThetaUSOUT.setConstant(flexThetaU);
        flexOmegaSOUT.setConstant(flexOmega);
        flexOmegaDotSOUT.setConstant(flexOmegaDot);

        flexTransformationMatrixSOUT.setConstant(flexTransformationMatrix);
        flexPoseThetaUSOUT.setConstant(flexPoseThetaU);
        flexVelocityVectorSOUT.setConstant(flexVelocityVector);
        flexAccelerationVectorSOUT.setConstant(flexVelocityVector);

        flexInverseSOUT.setConstant(flexInverseState);
        flexMatrixInverseSOUT.setConstant(flexMatrixInverseSOUT);
        flexInversePoseThetaUSOUT.setConstant(flexInversePoseThetaUSOUT);
        flexInverseVelocityVectorSOUT.setConstant(flexInverseVelocityVector);
        flexInverseVelocitySOUT.setConstant(flexInverseVelocity);
        flexInverseOmegaSOUT.setConstant(flexInverseOmega);

        simulatedSensorsSOUT.setConstant(simulatedMeasurement);
        predictedSensorsSOUT.setConstant(simulatedMeasurement);
        inovationSOUT.setConstant(inovation);
        predictionSOUT.setConstant(inovation);

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



        std::ostringstream stateSizeString;
        stateSizeString << stateSize;

        std::ostringstream measurementSizeString;
        stateSizeString << measurementSize;

        std::ostringstream inputSizeString;
        inputSizeString << inputSizeBase;

        std::string docstring;

        contactNumber_=0;


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
                "    Sets the variance of the noise of force/toraue sensors. "
                "\n";

        addCommand(std::string("setForceVariance"),
                   new ::dynamicgraph::command::Setter <DGIMUModelBaseFlexEstimation,double >
                    (*this, & DGIMUModelBaseFlexEstimation::setForceVariance,docstring));


        estimator_.setInput(input);
        estimator_.setMeasurementInput(input);
        inputSIN.setConstant(convertVector<dynamicgraph::Vector>(input));

    }

    DGIMUModelBaseFlexEstimation::~DGIMUModelBaseFlexEstimation()
    {
    }



    dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeFlexibility
                  (dynamicgraph::Vector & flexibility, const int& inTime)
    {


#ifdef SOT_STATE_OBSERVATION_CHECK_UNIQUENESS_IN_TIME
        if (inTime!=currentTime_)
        {
            currentTime_=inTime;
#endif
        const dynamicgraph::Vector & measurement = measurementSIN(inTime);
        const dynamicgraph::Vector & input = inputSIN(inTime);
        const unsigned & contactNb = contactsNbrSIN(inTime);

        // Update of inputSize_ considering contactsNb

        if (contactNumber_!= contactNb)
        {
            contactNumber_ = contactNb;
            estimator_.setContactsNumber(contactNb);
        }

        estimator_.setMeasurement((convertVector<stateObservation::Vector>(measurement)).head(estimator_.getMeasurementSize()));
        estimator_.setMeasurementInput(convertVector<stateObservation::Vector>(input));

#ifdef SOT_STATE_OBSERVATION_CHECK_UNIQUENESS_IN_TIME
        }
#endif
        flexibility = convertVector<dynamicgraph::Vector>(estimator_.getFlexibilityVector());
        return flexibility;
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeFlexPosition
                        (::dynamicgraph::Vector & flexibilityPosition, const int& inTime)
    {
        flexibilitySOUT(inTime);

        flexibilityPosition = convertVector<dynamicgraph::Vector>
                    (estimator_.getFlexibilityVector().segment(stateObservation::kine::pos,3));

        return flexibilityPosition;
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeFlexVelocity
                        (::dynamicgraph::Vector & flexibilityVelocity, const int& inTime)
    {
        flexibilitySOUT(inTime);

        flexibilityVelocity = convertVector<dynamicgraph::Vector>
            (estimator_.getFlexibilityVector().segment(stateObservation::kine::linVel,3));

        return flexibilityVelocity;
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeFlexAcceleration
                        (::dynamicgraph::Vector & flexibilityAcceleration, const int& inTime)
    {
        flexibilitySOUT(inTime);

        flexibilityAcceleration = convertVector<dynamicgraph::Vector>
                (estimator_.getFlexibilityVector().segment(stateObservation::kine::linAcc,3));

        return flexibilityAcceleration;
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeFlexThetaU
                        (::dynamicgraph::Vector & flexibilityThetaU, const int& inTime)
    {
        flexibilitySOUT(inTime);

        flexibilityThetaU = convertVector<dynamicgraph::Vector>
                (estimator_.getFlexibilityVector().segment(stateObservation::kine::ori,3));

        return flexibilityThetaU;
    }


    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeFlexOmega
                        (::dynamicgraph::Vector & flexibilityOmega, const int& inTime)
    {
        flexibilitySOUT(inTime);

        flexibilityOmega = convertVector<dynamicgraph::Vector>
                (estimator_.getFlexibilityVector().segment(stateObservation::kine::angVel,3));

        return flexibilityOmega;
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeFlexOmegaDot
                        (::dynamicgraph::Vector & flexibilityOmegaDot, const int& inTime)
    {
        flexibilitySOUT(inTime);

        flexibilityOmegaDot = convertVector<dynamicgraph::Vector>
                            (estimator_.getFlexibilityVector().segment(stateObservation::kine::angAcc,3));

        return flexibilityOmegaDot;
    }


    ::dynamicgraph::sot::MatrixHomogeneous& DGIMUModelBaseFlexEstimation::computeFlexTransformationMatrix
                        (::dynamicgraph::sot::MatrixHomogeneous & flexibilityTransformationMatrix, const int& inTime)
    {
        flexibilitySOUT(inTime);

        flexibilityTransformationMatrix = convertMatrix<dynamicgraph::Matrix>(estimator_.getFlexibility());

        return flexibilityTransformationMatrix;
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeFlexPoseThetaU
                        (::dynamicgraph::Vector & flexibilityPoseThetaU, const int& inTime)
    {
        //std::cout << "computeFlexPoseThetaU " << inTime << std::endl;

        flexibilitySOUT(inTime);

        stateObservation::Vector v = stateObservation::Vector::Zero(6,1);
        v.head(3) = estimator_.getFlexibilityVector().segment(stateObservation::kine::pos,3);
        v.tail(3) = estimator_.getFlexibilityVector().segment(stateObservation::kine::ori,3);

        flexibilityPoseThetaU = convertVector<dynamicgraph::Vector>(v);

        return flexibilityPoseThetaU;
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeFlexVelocityVector
                        (::dynamicgraph::Vector & flexibilityVelocityVector, const int& inTime)
    {
        //std::cout << "computeFlexPoseThetaU " << inTime << std::endl;

        flexibilitySOUT(inTime);

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

        flexibilitySOUT(inTime);

        stateObservation::Vector v = stateObservation::Vector::Zero(6,1);
        v.head(3) = estimator_.getFlexibilityVector().segment(stateObservation::kine::linAcc,3);
        v.tail(3) = estimator_.getFlexibilityVector().segment(stateObservation::kine::angAcc,3);

        flexibilityAccelerationVector = convertVector<dynamicgraph::Vector>(v);

        return flexibilityAccelerationVector;
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::getForcesAndMoments(::dynamicgraph::Vector & forcesAndMoments, const int& inTime)
    {
        flexibilitySOUT(inTime);

        forcesAndMoments=convertVector<dynamicgraph::Vector>(estimator_.getForcesAndMoments());

        return forcesAndMoments;
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::getForcesSupport1(::dynamicgraph::Vector & forcesSupport1, const int& inTime)
    {
        flexibilitySOUT(inTime);

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
        flexibilitySOUT(inTime);

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
        flexibilitySOUT(inTime);

        flexInverse = convertVector<dynamicgraph::Vector>
            (stateObservation::kine::invertState(estimator_.getFlexibilityVector()));

        return flexInverse;
    }

    ::dynamicgraph::sot::MatrixHomogeneous& DGIMUModelBaseFlexEstimation::computeFlexMatrixInverse
                        (::dynamicgraph::sot::MatrixHomogeneous & flexMatrixInverse, const int& inTime)
    {

        flexibilitySOUT(inTime);

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
        flexibilitySOUT(inTime);

        return sensorSignal = convertVector <dynamicgraph::Vector>
                                        (estimator_.getSimulatedMeasurement());
    }

        ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computePredictedSensors
                        (::dynamicgraph::Vector & sensorSignal, const int& inTime)
    {
        flexibilitySOUT(inTime);

        return sensorSignal = convertVector <dynamicgraph::Vector>
                                        (estimator_.getPredictedMeasurement());
    }

        double& DGIMUModelBaseFlexEstimation::computeFlexibilityComputationTime
                    (double& flexibilityComputationTime, const int &inTime)
    {
        return flexibilityComputationTime=estimator_.getComputeFlexibilityTime();
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeInovation
                        (::dynamicgraph::Vector & inovation, const int& inTime)
    {
        flexibilitySOUT(inTime);

        return inovation = convertVector <dynamicgraph::Vector>
                                        (estimator_.getInovation());
    }


    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computePrediction
                        (::dynamicgraph::Vector & prediction, const int& inTime)
    {
        flexibilitySOUT(inTime);

        return prediction = convertVector <dynamicgraph::Vector>
                                        (estimator_.getPrediction());
    }
}
