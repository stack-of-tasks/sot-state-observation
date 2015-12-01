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

#ifndef STACKOFVECTORS_HH
#define STACKOFVECTORS_HH

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/linear-algebra.h>

//#include <sot/core/matrix-rotation.hh>
//#include <sot/core/matrix-homogeneous.hh>
//#include <sot/core/multi-bound.hh>
#include <sot/core/vector-utheta.hh>
//#include <sot/core/vector-roll-pitch-yaw.hh>

#include <sot/core/matrix-homogeneous.hh>
#include <state-observation/tools/miscellaneous-algorithms.hpp>
#include <sot-state-observation/tools/definitions.hh>

namespace sotStateObservation
{
    using dynamicgraph::Signal;
    using dynamicgraph::SignalPtr;
    using dynamicgraph::SignalTimeDependent;
    using dynamicgraph::Vector;
    using dynamicgraph::Matrix;
    using dynamicgraph::Entity;
    //using dynamicgraph::sot::VectorMultiBound;
    using dynamicgraph::sot::MatrixHomogeneous;
    using dynamicgraph::sot::MatrixRotation;
    using dynamicgraph::sot::VectorUTheta;
    //using dynamicgraph::sot::VectorRollPitchYaw;

    using namespace sotStateObservation;
    using namespace stateObservation;

        /**
           \brief
        */
        class StackOfContacts :
            public dynamicgraph::Entity,
            private boost::noncopyable //
        {
        public:
            /**
            \brief Constructor by name
            */
            StackOfContacts(const std::string& inName);

            ~StackOfContacts();

            /// Each entity should provide the name of the class it belongs to
            virtual const std::string& getClassName (void) const
            {
                return CLASS_NAME;
            }

            /// Header documentation of the python class
            virtual std::string getDocString () const
            {
                return
                    "Entity that compute the stack of contacts";
            }

            /**
            \name Parameters
            @{
            */
        protected:
            /*
            \brief Class name
            */
            static const std::string CLASS_NAME;

            unsigned int& getNbSupport(unsigned int& nbSupport, const int& time);

            dynamicgraph::Vector& getSupportPos1(dynamicgraph::Vector& , const int& time);
            MatrixHomogeneous& getHomoSupportPos1(MatrixHomogeneous& , const int& time);
            dynamicgraph::Vector& getForceSupport1(dynamicgraph::Vector& , const int& time);

            dynamicgraph::Vector& getSupportPos2(dynamicgraph::Vector& , const int& time);
            MatrixHomogeneous& getHomoSupportPos2(MatrixHomogeneous& , const int& time);
            dynamicgraph::Vector& getForceSupport2(dynamicgraph::Vector& , const int& time);

        private:

            /// Methods
            void computeStack(const int& time);

            /// Signals
            SignalPtr <MatrixHomogeneous, int> leftFootPositionSIN_;
            SignalPtr <dynamicgraph::Vector, int> forceLeftFootSIN_;

            SignalPtr <MatrixHomogeneous, int> rightFootPositionSIN_;
            SignalPtr <dynamicgraph::Vector, int> forceRightFootSIN_;

            SignalTimeDependent <unsigned int, int> nbSupportSOUT_;

            SignalTimeDependent <dynamicgraph::Vector, int> supportPos1SOUT_;
            SignalTimeDependent <MatrixHomogeneous, int> homoSupportPos1SOUT_;
            SignalTimeDependent <dynamicgraph::Vector, int> forceSupport1SOUT_;

            SignalTimeDependent <dynamicgraph::Vector, int> supportPos2SOUT_;
            SignalTimeDependent <MatrixHomogeneous, int> homoSupportPos2SOUT_;
            SignalTimeDependent <dynamicgraph::Vector, int> forceSupport2SOUT_;

            /// Parameters
            double forceThreshold_;
      };

} // namespace sotStateObservation

#endif // STACKOFVECTORS_HH
