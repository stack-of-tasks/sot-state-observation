#include <sstream>

#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-bind.h>

#include <sot-state-observation/dg-imu-attitude-estimation.hh>



namespace sotStateObservation
{
    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN ( DGIMUAttitudeEstimation,
                                          "DGIMUAttitudeEstimation" );

    DGIMUAttitudeEstimation::DGIMUAttitudeEstimation
                ( const std::string & inName):
        Entity(inName),
        measurementSIN(0x0 , "DGIMUAttitudeEstimation("+inName+")::input(vector)::measurement"),
        inputSIN(0x0 , "DGIMUAttitudeEstimation("+inName+")::input(vector)::input"),
        attitudeSOUT(measurementSIN,
                        "DGIMUAttitudeEstimation("+inName+")::input(vector)::attitude"),
        filter_(stateSize, measurementSize, inputSize, false)
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

        attitudeSOUT.setFunction(boost::bind(&DGIMUAttitudeEstimation::computeAttitude,
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
                "    Set a guess of the state  \n"
                "    takes a tuple of " + stateSizeString.str() + "\n"
                "    floating point mumbers as input \n"
                "\n";

        addCommand(std::string("setStateGuess"),
	     new
	     ::dynamicgraph::command::Setter <DGIMUAttitudeEstimation,dynamicgraph::Vector>
	     (*this, &DGIMUAttitudeEstimation::setStateGuess, docstring));


         //setStateGuessCovariance
        docstring =
                "\n"
                "    Set the covariance matrix of the current state estimation \n"
                "    takes " + stateSizeString.str() + " tuples of" + stateSizeString.str() + "\n"
                "    floating point mumbers as input \n"
                "\n";

        addCommand(std::string("setStateGuessCovariance"),
	     new
	     ::dynamicgraph::command::Setter <DGIMUAttitudeEstimation,dynamicgraph::Matrix>
	     (*this, &DGIMUAttitudeEstimation::setStateGuessCovariance, docstring));


        //setSensorsNoiseCovariance
        docstring =
                "\n"
                "    Set the covariance matrix of the sensor noise \n"
                "    takes " + measurementSizeString.str() + "tuples of" + measurementSizeString.str() + "\n"
                "    floating point mumbers as input \n"
                "\n";

        addCommand(std::string("setSensorsNoiseCovariance"),
	     new
	     ::dynamicgraph::command::Setter <DGIMUAttitudeEstimation,dynamicgraph::Matrix>
	     (*this, &DGIMUAttitudeEstimation::setSensorsNoiseCovariance, docstring));


        //setProcessNoiseCovariance
        docstring =
                "\n"
                "    Set the covariance matrix of the process noise \n"
                "    takes " + stateSizeString.str() + " tuples of" + stateSizeString.str() + "\n"
                "    floating point mumbers as input \n"
                "\n";

        addCommand(std::string("setProcessNoiseCovariance"),
	     new
	     ::dynamicgraph::command::Setter <DGIMUAttitudeEstimation,dynamicgraph::Matrix>
	     (*this, &DGIMUAttitudeEstimation::setProcessNoiseCovariance, docstring));


        filter_.setState(stateObservation::Vector::Zero(stateSize), attitudeSOUT.getTime());

        filter_.setFunctor(&imuFunctor_);

        ///The covariance matrix of the process noise and the measurement noise
        stateObservation::Matrix q = stateObservation::Matrix::Identity(stateSize,stateSize)*0.01;
        stateObservation::Matrix r = stateObservation::Matrix::Identity(measurementSize,measurementSize)*100;

        filter_.setQ(q);
        filter_.setR(r);

        stateObservation::Vector xh0=stateObservation::Vector::Zero(stateSize,1);

        stateObservation::Matrix p=stateObservation::Matrix::Identity(stateSize,stateSize)*0.1;

        filter_.setStateCovariance(p);

        filter_.setState(xh0,attitudeSOUT.getTime());

    }

    DGIMUAttitudeEstimation::~DGIMUAttitudeEstimation()
    {
    }

    dynamicgraph::Vector& DGIMUAttitudeEstimation::computeAttitude
                  (dynamicgraph::Vector & Attitude, const int& inTime)
    {
        const dynamicgraph::Vector & measurement = measurementSIN(inTime);
        const dynamicgraph::Vector & input = inputSIN(inTime-1);

        filter_.setMeasurement(convertVector<stateObservation::Vector>(measurement),inTime);
        filter_.setInput(convertVector<stateObservation::Vector>(input),inTime-1);

        Attitude = convertVector<dynamicgraph::Vector>(filter_.getEstimateState(inTime));

        return Attitude;
    }
}
