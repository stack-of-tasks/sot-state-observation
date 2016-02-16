//
// Copyright (c) 2015,
// Alexis Mifsud
//
// CNRS
//
// This file is part of sot-dynamic.
// sot-dynamic is free software: you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of
// the License, or (at your option) any later version.
// sot-dynamic is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.  You should
// have received a copy of the GNU Lesser General Public License along
// with sot-dynamic.  If not, see <http://www.gnu.org/licenses/>.
//

#ifndef FILTER_HH
#define FILTER_HH

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/linear-algebra.h>

#include <sot/core/matrix-homogeneous.hh>
#include <state-observation/tools/miscellaneous-algorithms.hpp>
#include <sot-state-observation/tools/definitions.hh>

#include <queue>

namespace sotStateObservation
{
    using dynamicgraph::Signal;
    using dynamicgraph::SignalPtr;
    using dynamicgraph::SignalTimeDependent;
    using dynamicgraph::Vector;
    using dynamicgraph::Matrix;
    using dynamicgraph::Entity;

    using namespace sotStateObservation;
    using namespace stateObservation;

        /**
           \brief
        */
        class Filter :
            public dynamicgraph::Entity,
            private boost::noncopyable //
        {
        public:
            /**
            \brief Constructor by name
            */
            Filter(const std::string& inName);

            ~Filter();

            /// Each entity should provide the name of the class it belongs to
            virtual const std::string& getClassName (void) const
            {
                return CLASS_NAME;
            }

            /// Header documentation of the python class
            virtual std::string getDocString () const
            {
                return
                    "Entity that filter an input";
            }

            void setOn(const bool& b)
            {
                on_=b;
            }

            void setWindowSize(const unsigned & n){
                n_=n;
                updateDistribution();
            }

            unsigned getWindowSize() const
            {
                const unsigned & n(n_);
                return n;
            }

            void setFilter(const unsigned int & t)
            {
                filter_=t;
            }

            unsigned int getFilter() const
            {
                const unsigned int & filter(filter_);
                return filter;
            }

            dynamicgraph::Vector& getOutput(dynamicgraph::Vector& output, const int& time);

            /**
            \name Parameters
            @{
            */
        protected:
            /*
            \brief Class name
            */
            static const std::string CLASS_NAME;

        private:

            /// Methods
            void updateDistribution();

                // Differents distribution for different filtering
            stateObservation::Vector averageDistribution(const unsigned n);
            stateObservation::Vector gaussianDistribution(const unsigned n, const double mean, const double stddev);
            stateObservation::Vector updateGaussianDistribution(const unsigned n);

            /// Signals
            dynamicgraph::SignalPtr <Vector, int> inputSIN_;

            dynamicgraph::SignalPtr <Vector, int> outputSOUT_;

            /// Parameters

            bool on_;
            unsigned int time_;
            unsigned int filter_;

            unsigned int n_; // Size of the window
            std::list<stateObservation::Vector> u_; // list of maximum size n_ with vector of size ioSize_ as elements
            std::list<stateObservation::Vector>::iterator iterator;

            stateObservation::Vector distr_; // Filtering distribution
            stateObservation::Vector vec_;

            stateObservation::Vector output_;

      };

} // namespace sotStateObservation

#endif // FILTER_HH
