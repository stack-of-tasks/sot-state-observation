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
        measurementSIN(0x0 , "DynamicGraphIMUAttitudeEstimation("+inName+")::input(vector)::measurement"),
        inputSIN(0x0 , "DynamicGraphIMUAttitudeEstimation("+inName+")::input(vector)::input"),
        contactsNbrSIN(0x0 , "DynamicGraphIMUAttitudeEstimation("+inName+")::input(unsigned)::input"),
        contact1SIN(0x0, "DynamicGraphIMUAttitudeEstimation("+inName+")::input(vector)::input"),
        contact2SIN(0x0, "DynamicGraphIMUAttitudeEstimation("+inName+")::input(vector)::input"),
        contact3SIN(0x0, "DynamicGraphIMUAttitudeEstimation("+inName+")::input(vector)::input"),
        contact4SIN(0x0, "DynamicGraphIMUAttitudeEstimation("+inName+")::input(vector)::input"),
        flexibilitySOUT(measurementSIN << inputSIN,
                        "DynamicGraphIMUAttitudeEstimation("+inName+")::input(vector)::attitude")
    {
        signalRegistration (contactsNbrSIN);
        signalRegistration (contact1SIN);
        signalRegistration (contact2SIN);
        signalRegistration (contact3SIN);
        signalRegistration (contact4SIN);

        signalRegistration (measurementSIN);
        signalRegistration (inputSIN);
        signalRegistration (flexibilitySOUT);

        dynamicgraph::Vector measure(measurementSize);
        dynamicgraph::Vector input(inputSize);
        dynamicgraph::Vector flexibility(stateSize);

        measurementSIN.setConstant(measure);
        inputSIN.setConstant(input);
        flexibilitySOUT.setConstant(flexibility);

        flexibilitySOUT.setFunction(boost::bind(&DGIMUFlexibilityEstimation::computeFlexibility,
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
}

