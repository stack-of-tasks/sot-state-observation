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
#include <sot/core/matrix-homogeneous.hh>
//#include <sot/core/multi-bound.hh>
#include <sot/core/vector-utheta.hh>
#include <sot/core/vector-roll-pitch-yaw.hh>

#include <sot/core/matrix-homogeneous.hh>
#include <state-observation/tools/miscellaneous-algorithms.hpp>
#include <sot-state-observation/tools/definitions.hh>

#include <Eigen/StdVector>
#include <algorithm>

namespace sotStateObservation
{
    using dynamicgraph::Signal;
    using dynamicgraph::SignalPtr;
    using dynamicgraph::SignalTimeDependent;
    using dynamicgraph::Vector;
    using dynamicgraph::Matrix;
    using dynamicgraph::Entity;
    using dynamicgraph::sot::MatrixHomogeneous;
    using dynamicgraph::sot::MatrixRotation;
    using dynamicgraph::sot::VectorUTheta;
    using dynamicgraph::sot::VectorRollPitchYaw;

    using namespace sotStateObservation;
    using namespace stateObservation;

        /**
           \brief
        */
        class Odometry :
            public dynamicgraph::Entity,
            private boost::noncopyable //
        {
        public:
            /**
            \brief Constructor by name
            */
            Odometry(const std::string& inName);

            ~Odometry();

            /// Each entity should provide the name of the class it belongs to
            virtual const std::string& getClassName (void) const
            {
                return CLASS_NAME;
            }

            /// Header documentation of the python class
            virtual std::string getDocString () const
            {
                return
                    "Entity that compute the stack of contacts and their position using odometry";
            }

            Vector& getFreeFlyer(Vector& freeFlyer, const int& time);
            Vector& getPivotPositionOut(Vector& pivotPositionOut, const int& time);

            void setLeftFootPosition(const Matrix & mL);
            void setRightFootPosition(const Matrix & mR);

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
            void getInputs(const int& time);
            void computeOdometry(const int& time);
            stateObservation::Matrix4 regulateOdometryWithRef(stateObservation::Matrix4 posEnc, stateObservation::Vector posRef, double alpha);
            stateObservation::Matrix4 homogeneousMatricesAverage(stateObservation::Matrix4 m1, stateObservation::Matrix4 m2, double alpha);

            /// Signals
            dynamicgraph::SignalPtr <MatrixHomogeneous, int> leftFootPositionSIN_;
            dynamicgraph::SignalPtr <Vector, int> forceLeftFootSIN_;

            dynamicgraph::SignalPtr <MatrixHomogeneous, int> rightFootPositionSIN_;
            dynamicgraph::SignalPtr <Vector, int> forceRightFootSIN_;

            dynamicgraph::SignalPtr <Matrix, int> leftFootPositionRefSIN_;
            dynamicgraph::SignalPtr <Matrix, int> rightFootPositionRefSIN_;

            dynamicgraph::SignalPtr <Vector, int> stackOfSupportContactsSIN_;

            dynamicgraph::SignalPtr <Vector, int>  freeFlyerSOUT_;

            dynamicgraph::SignalPtr <Vector, int> pivotPositionSOUT_;

            /// Parameters
            double forceThreshold_, time_;

            std::vector<stateObservation::Matrix4,Eigen::aligned_allocator<Matrix4> > inputHomoPosition_;
            std::vector<stateObservation::Vector6,Eigen::aligned_allocator<Vector6> > inputPosition_;
            std::vector<stateObservation::Matrix4,Eigen::aligned_allocator<Matrix4> > referenceHomoPosition_;
            std::vector<stateObservation::Vector6,Eigen::aligned_allocator<Vector6> > referencePosition_;
            std::vector<stateObservation::Vector6,Eigen::aligned_allocator<Vector6> > inputForces_;

            stateObservation::Vector stackOfSupportContacts_;
            double supportContactsNbr_;
            int pivotSupport_;

            std::vector<stateObservation::Matrix4,Eigen::aligned_allocator<Matrix4> > odometryHomoPosition_;
            stateObservation::Matrix4 odometryFreeFlyer_;

            stateObservation::Vector alpha_;

            // To optimize memory access
            stateObservation::Vector6 posUTheta_;
            stateObservation::Matrix3 rot_;
            stateObservation::Matrix4 homo_;
            stateObservation::AngleAxis aa_;

        public:
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      };

} // namespace sotStateObservation

#endif // STACKOFVECTORS_HH
