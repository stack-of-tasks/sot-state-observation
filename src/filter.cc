#include <sstream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-bind.h>

#include <sot-state-observation/filter.hh>

using namespace std;


namespace sotStateObservation
{
    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN ( Filter, "Filter" );

    Filter::Filter( const std::string & inName, unsigned n):
        Entity(inName),
        inputSIN_ (NULL, "Filter("+inName+")::input(vector)::input"),
        outputSOUT_ (NULL, "Filter("+inName+")::output(vector)::output"),
        on_(false), ioSize_(n), time_(0)
    {

        signalRegistration (inputSIN_ << outputSOUT_);

        std::string docstring;

        docstring  =
                "\n"
                "    Enable (true) or disable (false) the filter \n"
                "\n";

        addCommand(std::string("setOn"),
             new
             ::dynamicgraph::command::Setter <Filter,bool>
                (*this, &Filter::setOn, docstring));

        outputSOUT_.setFunction(boost::bind(&Filter::getOutput, this, _1, _2));

        stateObservation::Vector input;
        input.resize(ioSize_);
        inputSIN_.setConstant(convertVector<dynamicgraph::Vector>(input));
        inputSIN_.setTime (time_);

    }

    Filter::~Filter()
    {
    }

    Vector& Filter::getOutput(dynamicgraph::Vector& output, const int& time)
    {
        output=inputSIN_.access(time);
        return output;
    }
}

