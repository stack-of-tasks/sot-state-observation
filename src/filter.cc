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

    Filter::Filter( const std::string & inName):
        Entity(inName)
    {

    }

    Filter::~Filter()
    {
    }


}

