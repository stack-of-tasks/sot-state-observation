#include <sstream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-bind.h>

#include <state-observation/tools/definitions.hpp>
#include <state-observation/tools/miscellaneous-algorithms.hpp>

#include <sot-state-observation/filter.hh>

using namespace std;

namespace sotStateObservation
{
    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN ( Filter, "Filter" );

    Filter::Filter( const std::string & inName):
        Entity(inName),
        inputSIN_ (NULL, "Filter("+inName+")::input(vector)::input"),
        outputSOUT_ (NULL, "Filter("+inName+")::output(vector)::output"),
        on_(false), time_(0), n_(10), distr_(10)
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

        docstring  =
                "\n"
                "    Set the wondow size \n"
                "\n";

        addCommand(std::string("setWindowSize"),
             new
             ::dynamicgraph::command::Setter <Filter,unsigned>
                (*this, &Filter::setWindowSize, docstring));

        outputSOUT_.setFunction(boost::bind(&Filter::getOutput, this, _1, _2));

    }

    Filter::~Filter()
    {
    }

    void Filter::updateU(const stateObservation::Vector& lastInput)
    {
        u_.push_back(lastInput);
        if(u_.size()>=n_) u_.pop_front();
    }

    void Filter::updateDistribution()
    {
        distr_.resize(n_);
        distr_.setOnes();
        distr_=(1/n_)*distr_;
    }

    dynamicgraph::Vector& Filter::getOutput(dynamicgraph::Vector& output, const int& time)
    {
        stateObservation::Vector lastInput=convertVector<stateObservation::Vector>(inputSIN_.access(time));

        // Update the input window
        updateU(lastInput);

        // Compute filtering
        output_.resize(lastInput.size()); output_.setZero();
        int i=0;
        for (iterator=u_.begin(); iterator != u_.end(); ++iterator)
        {
            output_+=distr_[i]*(*iterator);
            ++i;
        }

        output=convertVector<Vector>(output_);
        return output;
    }
}

