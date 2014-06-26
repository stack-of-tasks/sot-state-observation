#include <sstream>

#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-bind.h>

#include <sot-state-observation/dg-imu-model-base-flex-estimation.hh>




namespace sotStateObservation
{
    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN ( DGIMUModelBaseFlexEstimation,
                                          "DGIMUModelBaseFlexEstimation" );

    DGIMUModelBaseFlexEstimation::DGIMUModelBaseFlexEstimation
                ( const std::string & inName):
        Entity(inName),
        measurementSIN(0x0 , "DGIMUModelBaseFlexEstimation("+inName+")::input(vector)::measurement"),
        inputSIN(0x0 , "DGIMUModelBaseFlexEstimation("+inName+")::input(vector)::input"),
        contactsNbrSIN(0x0 , "DGIMUModelBaseFlexEstimation("+inName+")::input(unsigned)::contactNbr"),
        flexibilitySOUT("DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexibility"),

        flexPositionSOUT(flexibilitySOUT,
                        "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexPosition"),
        flexVelocitySOUT(flexibilitySOUT,
                        "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexVelocity"),
        flexAccelerationSOUT(flexibilitySOUT,
                        "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexAcceleration"),
        flexPoseThetaUSOUT(flexibilitySOUT,
                        "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexPoseThetaU"),
        flexOmegaSOUT(flexibilitySOUT,
                        "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexOmega"),
        flexOmegaDotSOUT(flexibilitySOUT,
                        "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexOmegaDot"),

        flexTransformationMatrixSOUT(flexibilitySOUT,
                        "DGIMUModelBaseFlexEstimation("+inName+")::output(homogeneousMatrix)::flexTransformationMatrix"),
        flexThetaUSOUT(flexibilitySOUT,
                        "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexThetaU"),
        flexVelocityVectorSOUT(flexibilitySOUT,
                        "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexVelocityVector"),


        flexInverseSOUT (flexibilitySOUT,
                        "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexInverse"),
        flexMatrixInverseSOUT(flexInverseSOUT,
                        "DGIMUModelBaseFlexEstimation("+inName+")::output(homogeneousMatrix)::flexMatrixInverse"),
        flexInversePoseThetaUSOUT(flexInverseSOUT,
                        "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexInversePoseThetaU"),
        flexInverseVelocityVectorSOUT(flexInverseSOUT,
                        "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexInverseVelocityVector"),
        flexInverseVelocitySOUT(flexInverseSOUT,
                        "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexInverseVelocity"),
        flexInverseOmegaSOUT(flexInverseSOUT,
                        "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::flexInverseOmega"),

        simulatedSensorsSOUT(flexibilitySOUT,
                        "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::simulatedSensors"),
        predictedSensorsSOUT(flexibilitySOUT,
                        "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::predictedSensors"),

        inovationSOUT(flexibilitySOUT,
                        "DGIMUModelBaseFlexEstimation("+inName+")::output(vector)::inovation")

    {
#ifdef SOT_STATE_OBSERVATION_CHECK_UNIQUENESS_IN_TIME
        currentTime_=0;
#endif

        signalRegistration (measurementSIN);
        signalRegistration (inputSIN);

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

        signalRegistration (flexInverseSOUT);
        signalRegistration (flexMatrixInverseSOUT);
        signalRegistration (flexInversePoseThetaUSOUT);
        signalRegistration (flexInverseVelocityVectorSOUT);
        signalRegistration (flexInverseVelocitySOUT);
        signalRegistration (flexInverseOmegaSOUT);

        signalRegistration (simulatedSensorsSOUT);
        signalRegistration (predictedSensorsSOUT);
        signalRegistration (inovationSOUT);



        signalRegistration (contactsNbrSIN);


        dynamicgraph::Vector measure(measurementSize);

        dynamicgraph::Vector input(inputSize);

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
        inputSIN.setConstant(input);

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

        flexInverseSOUT.setConstant(flexInverseState);
        flexMatrixInverseSOUT.setConstant(flexMatrixInverseSOUT);
        flexInversePoseThetaUSOUT.setConstant(flexInversePoseThetaUSOUT);
        flexInverseVelocityVectorSOUT.setConstant(flexInverseVelocityVector);
        flexInverseVelocitySOUT.setConstant(flexInverseVelocity);
        flexInverseOmegaSOUT.setConstant(flexInverseOmega);

        simulatedSensorsSOUT.setConstant(simulatedMeasurement);
        predictedSensorsSOUT.setConstant(simulatedMeasurement);
        inovationSOUT.setConstant(inovation);

        contactsNbrSIN.setConstant(0);

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

        predictedSensorsSOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::computeSimulatedSensors,
                    this, _1, _2));

        inovationSOUT.setFunction(boost::bind(&DGIMUModelBaseFlexEstimation::computeInovation,
                    this, _1, _2));



        std::ostringstream stateSizeString;
        stateSizeString << stateSize;

        std::ostringstream measurementSizeString;
        stateSizeString << measurementSize;

        std::ostringstream inputSizeString;
        inputSizeString << inputSize;

        std::string docstring;

        contactNumber_=0;

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


        //setVirtualMeasurementNoiseCovariance
        docstring =
                "\n"
                "    Set the covariance matrix of the measuement noise \n"
                "    takes a floating point mumber as input \n"
                "\n";

        addCommand(std::string("setVirtualMeasurementsCovariance"),
	     new
	     ::dynamicgraph::command::Setter <DGIMUModelBaseFlexEstimation,double>
                (*this, &DGIMUModelBaseFlexEstimation::setVirtualMeasurementsCovariance, docstring));

        //getVirtualMeasurementNoiseCovariance
        docstring =
                "\n"
                "    Get the covariance matrix of the measuement noise \n"
                "    gets a floating point mumbers as input \n"
                "\n";

        addCommand(std::string("getVirtualMeasurementCovariance"),
	     new
	     ::dynamicgraph::command::Getter <DGIMUModelBaseFlexEstimation,double>
                (*this, &DGIMUModelBaseFlexEstimation::getVirtualMeasurementsCovariance, docstring));


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


    }

    DGIMUModelBaseFlexEstimation::~DGIMUModelBaseFlexEstimation()
    {
    }

    dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeFlexibility
                  (dynamicgraph::Vector & flexibility, const int& inTime)
    {
        //std::cout << "computeFlexibility " << inTime << std::endl;
#ifdef SOT_STATE_OBSERVATION_CHECK_UNIQUENESS_IN_TIME
        if (inTime!=currentTime_)
        {
            currentTime_=inTime;
#endif

        const dynamicgraph::Vector & measurement = measurementSIN(inTime);
        const dynamicgraph::Vector & input = inputSIN(inTime);
        const unsigned & contactNb = contactsNbrSIN(inTime);

        if (contactNumber_!= contactNb)
        {
            contactNumber_ = contactNb;

            estimator_.setContactsNumber(contactNb);
        }


//        if (contactNb>0)
//        {
//            estimator_.setContactPosition(0,convertVector<stateObservation::Vector>(contact1SIN(inTime)));
//
//            if (contactNb>1)
//            {
//                estimator_.setContactPosition(1,convertVector<stateObservation::Vector>(contact2SIN(inTime)));
//
//                if (contactNb>2)
//                {
//                    estimator_.setContactPosition(2,convertVector<stateObservation::Vector>(contact3SIN(inTime)));
//
//                    if (contactNb==4)
//                    {
//                        estimator_.setContactPosition(3,convertVector<stateObservation::Vector>(contact4SIN(inTime)));
//                    }
//                }
//            }
//        }


        estimator_.setMeasurement(convertVector<stateObservation::Vector>(measurement));
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
        v.tail(3) = estimator_.getFlexibilityVector().segment(stateObservation::kine::angAcc,3);

        flexibilityVelocityVector = convertVector<dynamicgraph::Vector>(v);

        return flexibilityVelocityVector;
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
                                        (estimator_.getPredictedMeaurement());
    }

    ::dynamicgraph::Vector& DGIMUModelBaseFlexEstimation::computeInovation
                        (::dynamicgraph::Vector & inovation, const int& inTime)
    {
        flexibilitySOUT(inTime);

        return inovation = convertVector <dynamicgraph::Vector>
                                        (estimator_.getInovation());
    }
}
