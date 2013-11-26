#include <sstream>

#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-bind.h>

#include <sot-state-observation/dynamic-graph-imu-attitude-estimation.hh>



namespace sotStateObservation
{
    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN ( DynamicGraphIMUAttitudeEstimation,
                                          "DynamicGraphIMUAttitudeEstimation" );

    DynamicGraphIMUAttitudeEstimation::DynamicGraphIMUAttitudeEstimation
                ( const std::string & inName):
        Entity(inName),
        measurementSIN(0x0 , "DynamicGraphIMUAttitudeEstimation("+inName+")::input(vector)::measurement"),
        inputSIN(0x0 , "DynamicGraphIMUAttitudeEstimation("+inName+")::input(vector)::input"),
        attitudeSOUT(measurementSIN,
                        "DynamicGraphIMUAttitudeEstimation("+inName+")::input(vector)::attitude"),
        filter_(stateSize, measurementSize, inputSize, false),
        currentTime_(0)
    {
        signalRegistration (measurementSIN);
        signalRegistration (inputSIN);
        signalRegistration (attitudeSOUT);

        dynamicgraph::Vector measure(measurementSize);
        dynamicgraph::Vector input(inputSize);
        dynamicgraph::Vector attitude(stateSize);

        measurementSIN.setConstant(measure);
        inputSIN.setConstant(input);
        attitudeSOUT.setConstant(attitude);

        attitudeSOUT.setFunction(boost::bind(&DynamicGraphIMUAttitudeEstimation::computeAttitude,
				    this, _1, _2));

        std::ostringstream stateSizeString;
        stateSizeString << stateSize;

        std::ostringstream measurementSizeString;
        stateSizeString << measurementSize;

        std::ostringstream inputSizeString;
        inputSizeString << inputSize;

        std::string docstring;

        //setCurrentTime
        docstring =
                "\n"
                "    Set the current time, should be used for initialisation only \n"
                "    takes an unsigned number as input \n"
                "\n";

        addCommand(std::string("setStateGuess"),
	     new
	     ::dynamicgraph::command::Setter <DynamicGraphIMUAttitudeEstimation,dynamicgraph::Vector>
	     (*this, &DynamicGraphIMUAttitudeEstimation::setStateGuess, docstring));

        //setStateGuess
        docstring =
                "\n"
                "    Set a guess of the state at a given instant  \n"
                "    takes a tuple of \n" + stateSizeString.str() +
                "    floating point mumbers as input \n"
                "\n";

        addCommand(std::string("setStateGuess"),
	     new
	     ::dynamicgraph::command::Setter <DynamicGraphIMUAttitudeEstimation,dynamicgraph::Vector>
	     (*this, &DynamicGraphIMUAttitudeEstimation::setStateGuess, docstring));


         //setStateGuessCovariance
        docstring =
                "\n"
                "    Set the covariance matrix of the current state estimation \n"
                "    takes a tuple of \n" + stateSizeString.str() + "x" + stateSizeString.str() +
                "    floating point mumbers as input \n"
                "\n";

        addCommand(std::string("setStateGuessCovariance"),
	     new
	     ::dynamicgraph::command::Setter <DynamicGraphIMUAttitudeEstimation,dynamicgraph::Matrix>
	     (*this, &DynamicGraphIMUAttitudeEstimation::setStateGuessCovariance, docstring));


        //setSensorsNoiseCovariance
        docstring =
                "\n"
                "    Set the covariance matrix of the sensor noise \n"
                "    takes a tuple of \n" + measurementSizeString.str() + "x" + measurementSizeString.str() +
                "    floating point mumbers as input \n"
                "\n";

        addCommand(std::string("setSensorsNoiseCovariance"),
	     new
	     ::dynamicgraph::command::Setter <DynamicGraphIMUAttitudeEstimation,dynamicgraph::Matrix>
	     (*this, &DynamicGraphIMUAttitudeEstimation::setSensorsNoiseCovariance, docstring));


        //setProcessNoiseCovariance
        docstring =
                "\n"
                "    Set the covariance matrix of the sensor noise \n"
                "    takes a tuple of \n" + stateSizeString.str() + "x" + stateSizeString.str() +
                "    floating point mumbers as input \n"
                "\n";

        addCommand(std::string("setProcessNoiseCovariance"),
	     new
	     ::dynamicgraph::command::Setter <DynamicGraphIMUAttitudeEstimation,dynamicgraph::Matrix>
	     (*this, &DynamicGraphIMUAttitudeEstimation::setProcessNoiseCovariance, docstring));






    }



}
