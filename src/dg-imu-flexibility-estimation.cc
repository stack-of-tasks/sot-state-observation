#include <sstream>

#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-bind.h>

#include <sot-state-observation/dg-imu-flexibility-estimation.hh>


namespace sotStateObservation
{
    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN ( DGIMUFlexibilityEstimation,
                                          "DGIMUFlexibilityEstimation" );

    DGIMUFlexibilityEstimation::DGIMUFlexibilityEstimation
                ( const std::string & inName):
        Entity(inName),
        measurementSIN(0x0 , "DGIMUFlexibilityEstimation("+inName+")::input(vector)::measurement"),
        inputSIN(0x0 , "DGIMUFlexibilityEstimation("+inName+")::input(vector)::input"),
        contactsNbrSIN(0x0 , "DGIMUFlexibilityEstimation("+inName+")::input(unsigned)::contactNbr"),
        contact1SIN(0x0, "DGIMUFlexibilityEstimation("+inName+")::input(vector)::contact1"),
        contact2SIN(0x0, "DGIMUFlexibilityEstimation("+inName+")::input(vector)::contact2"),
        contact3SIN(0x0, "DGIMUFlexibilityEstimation("+inName+")::input(vector)::contact3"),
        contact4SIN(0x0, "DGIMUFlexibilityEstimation("+inName+")::input(vector)::contact4"),
        flexibilitySOUT(measurementSIN << inputSIN,
                        "DGIMUFlexibilityEstimation("+inName+")::output(vector)::flexibility"),
        flexPositionSOUT(flexibilitySOUT,
                        "DGIMUFlexibilityEstimation("+inName+")::output(vector)::flexPosition"),
        flexVelocitySOUT(flexibilitySOUT,
                        "DGIMUFlexibilityEstimation("+inName+")::output(vector)::flexVelocity"),
        flexAccelerationSOUT(flexibilitySOUT,
                        "DGIMUFlexibilityEstimation("+inName+")::output(vector)::flexAcceleration"),
        flexThetaUSOUT(flexibilitySOUT,
                        "DGIMUFlexibilityEstimation("+inName+")::output(vector)::flexThetaU"),
        flexRotationMatrixSOUT(flexibilitySOUT,
                        "DGIMUFlexibilityEstimation("+inName+")::output(Matrix)::flexRotationMatrix"),
        flexOmegaSOUT(flexibilitySOUT,
                        "DGIMUFlexibilityEstimation("+inName+")::output(vector)::flexOmega"),
        flexOmegaDotSOUT(flexibilitySOUT,
                        "DGIMUFlexibilityEstimation("+inName+")::output(vector)::flexOmegaDot")
    {

        signalRegistration (measurementSIN);
        signalRegistration (inputSIN);
        signalRegistration (flexibilitySOUT);

        signalRegistration (flexPositionSOUT);
        signalRegistration (flexVelocitySOUT);
        signalRegistration (flexAccelerationSOUT);
        signalRegistration (flexThetaUSOUT);
        signalRegistration (flexRotationMatrixSOUT);
        signalRegistration (flexOmegaSOUT);
        signalRegistration (flexOmegaDotSOUT);

        signalRegistration (contact1SIN);
        signalRegistration (contact2SIN);
        signalRegistration (contact3SIN);
        signalRegistration (contact4SIN);
        signalRegistration (contactsNbrSIN);



        dynamicgraph::Vector measure(measurementSize);
        dynamicgraph::Vector input(inputSize);
        dynamicgraph::Vector flexibility(stateSize);

        measurementSIN.setConstant(measure);
        inputSIN.setConstant(input);
        flexibilitySOUT.setConstant(flexibility);

        flexibilitySOUT.setFunction(boost::bind(&DGIMUFlexibilityEstimation::computeFlexibility,
				    this, _1, _2));

        flexPositionSOUT.setFunction(boost::bind(&DGIMUFlexibilityEstimation::computeFlexPosition,
				    this, _1, _2));

        flexVelocitySOUT.setFunction(boost::bind(&DGIMUFlexibilityEstimation::computeFlexVelocity,
				    this, _1, _2));

        flexAccelerationSOUT.setFunction(boost::bind(&DGIMUFlexibilityEstimation::computeFlexAcceleration,
				    this, _1, _2));

        flexThetaUSOUT.setFunction(boost::bind(&DGIMUFlexibilityEstimation::computeFlexThetaU,
				    this, _1, _2));

        flexRotationMatrixSOUT.setFunction(boost::bind(&DGIMUFlexibilityEstimation::computeFlexRotationMatrix,
				    this, _1, _2));

        flexOmegaSOUT.setFunction(boost::bind(&DGIMUFlexibilityEstimation::computeFlexOmega,
				    this, _1, _2));

        flexOmegaDotSOUT.setFunction(boost::bind(&DGIMUFlexibilityEstimation::computeFlexOmegaDot,
				    this, _1, _2));

        std::ostringstream stateSizeString;
        stateSizeString << stateSize;

        std::ostringstream measurementSizeString;
        stateSizeString << measurementSize;

        std::ostringstream inputSizeString;
        inputSizeString << inputSize;

        std::string docstring;

        //setStateGuess
        docstring =
                "\n"
                "    Set a guess of the flexibility state  \n"
                "    takes a tuple of " + stateSizeString.str() + "\n"
                "    floating point mumbers as input \n"
                "\n";

        addCommand(std::string("setFlexibilityGuess"),
	     new
	     ::dynamicgraph::command::Setter <DGIMUFlexibilityEstimation,dynamicgraph::Vector>
                (*this, &DGIMUFlexibilityEstimation::setFlexibilityGuess, docstring));

         //setStateGuessCovariance
        docstring =
                "\n"
                "    Set the covariance matrix of the current flexibility estimation \n"
                "    takes " + stateSizeString.str() + " tuples of" + stateSizeString.str() + "\n"
                "    floating point mumbers as input \n"
                "\n";

        addCommand(std::string("setFlexibilityGuessCovariance"),
	     new
	     ::dynamicgraph::command::Setter <DGIMUFlexibilityEstimation,dynamicgraph::Matrix>
                (*this, &DGIMUFlexibilityEstimation::setFlexibilityGuessCovariance, docstring));
    }

    DGIMUFlexibilityEstimation::~DGIMUFlexibilityEstimation()
    {
    }

    dynamicgraph::Vector& DGIMUFlexibilityEstimation::computeFlexibility
                  (dynamicgraph::Vector & flexibility, const int& inTime)
    {
        const dynamicgraph::Vector & measurement = measurementSIN(inTime);
        const dynamicgraph::Vector & input = inputSIN(inTime);
        const unsigned & contactNb = contactsNbrSIN(inTime);

        estimator_.setContactsNumber(contactNb);

        if (contactNb>=1)
        {
            estimator_.setContactPosition(1,convertVector<stateObservation::Vector>(contact1SIN(inTime)));

            if (contactNb>=2)
            {
                estimator_.setContactPosition(2,convertVector<stateObservation::Vector>(contact4SIN(inTime)));

                if (contactNb>=3)
                {
                    estimator_.setContactPosition(3,convertVector<stateObservation::Vector>(contact3SIN(inTime)));

                    if (contactNb==4)
                    {
                        estimator_.setContactPosition(4,convertVector<stateObservation::Vector>(contact4SIN(inTime)));
                    }
                }
            }
        }

        estimator_.setMeasurement(convertVector<stateObservation::Vector>(measurement));
        estimator_.setMeasurementInput(convertVector<stateObservation::Vector>(input));

        flexibility = convertVector<dynamicgraph::Vector>(estimator_.getFlexibilityVector());

        return flexibility;
    }

    ::dynamicgraph::Vector& DGIMUFlexibilityEstimation::computeFlexPosition
                        (::dynamicgraph::Vector & flexibilityPosition, const int& inTime)
    {
        flexibilitySOUT(inTime);

        flexibilityPosition = convertVector<dynamicgraph::Vector>(estimator_.getFlexibilityVector().head(3));

        return flexibilityPosition;
    }


    ::dynamicgraph::Vector& DGIMUFlexibilityEstimation::computeFlexVelocity
                        (::dynamicgraph::Vector & flexibilityVelocity, const int& inTime)
    {
        flexibilitySOUT(inTime);

        flexibilityVelocity = convertVector<dynamicgraph::Vector>(estimator_.getFlexibilityVector().segment(3,3));

        return flexibilityVelocity;
    }

    ::dynamicgraph::Vector& DGIMUFlexibilityEstimation::computeFlexAcceleration
                        (::dynamicgraph::Vector & flexibilityAcceleration, const int& inTime)
    {
        flexibilitySOUT(inTime);

        flexibilityAcceleration = convertVector<dynamicgraph::Vector>(estimator_.getFlexibilityVector().segment(6,3));

        return flexibilityAcceleration;
    }

    ::dynamicgraph::Vector& DGIMUFlexibilityEstimation::computeFlexThetaU
                        (::dynamicgraph::Vector & flexibilityThetaU, const int& inTime)
    {
        flexibilitySOUT(inTime);

        flexibilityThetaU = convertVector<dynamicgraph::Vector>(estimator_.getFlexibilityVector().segment(9,3));

        return flexibilityThetaU;
    }

    ::dynamicgraph::Matrix& DGIMUFlexibilityEstimation::computeFlexRotationMatrix
                        (::dynamicgraph::Matrix & flexibilityRotationMatrix, const int& inTime)
    {
        flexibilitySOUT(inTime);

        flexibilityRotationMatrix = convertMatrix<dynamicgraph::Matrix>(estimator_.getFlexibility());

        return flexibilityRotationMatrix;
    }

    ::dynamicgraph::Vector& DGIMUFlexibilityEstimation::computeFlexOmega
                        (::dynamicgraph::Vector & flexibilityOmega, const int& inTime)
    {
        flexibilitySOUT(inTime);

        flexibilityOmega = convertVector<dynamicgraph::Vector>(estimator_.getFlexibilityVector().segment(12,3));

        return flexibilityOmega;
    }

    ::dynamicgraph::Vector& DGIMUFlexibilityEstimation::computeFlexOmegaDot
                        (::dynamicgraph::Vector & flexibilityOmegaDot, const int& inTime)
    {
        flexibilitySOUT(inTime);

        flexibilityOmegaDot = convertVector<dynamicgraph::Vector>(estimator_.getFlexibilityVector().segment(15,3));

        return flexibilityOmegaDot;
    }
}

