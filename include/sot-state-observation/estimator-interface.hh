//
// Copyright (c) 2016,
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

#ifndef ESTIMATORINTERFACE_HH
#define ESTIMATORINTERFACE_HH

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/linear-algebra.h>

#include <sot/core/matrix-homogeneous.hh>
#include <sot/core/vector-utheta.hh>
#include <sot/core/vector-roll-pitch-yaw.hh>

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

    struct contact
    {
      static const unsigned nbMax=4;
      // index for the contacts
      static const unsigned lf = 0;
      static const unsigned rf = 1;
      static const unsigned lh = 2;
      static const unsigned rh = 3;
    };

        /**
           \brief
        */
        class EstimatorInterface :
            public dynamicgraph::Entity,
            private boost::noncopyable //
        {
        public:
            /**
            \brief Constructor by name
            */
            EstimatorInterface(const std::string& inName);

            ~EstimatorInterface();

            /// Each entity should provide the name of the class it belongs to
            virtual const std::string& getClassName (void) const
            {
                return CLASS_NAME;
            }

            /// Header documentation of the python class
            virtual std::string getDocString () const
            {
                return
                    "Entity that compute the input and measurment vectors for the estimator plus the number of contacts";
            }

            void setForceThresholds(const Vector& forceThresholds)
            {
                forceThresholds_=convertVector<stateObservation::Vector>(forceThresholds);
            }

            dynamicgraph::Vector getForceThresholds() const
            {
                return convertVector<dynamicgraph::Vector>(forceThresholds_);
            }

            Vector& getInput(Vector& input, const int& time)
            {
                 if(time!=timeInput_) computeInput(time);
                 input=convertVector<dynamicgraph::Vector>(input_);
                 return input;
            }

            Vector& getMeasurement(Vector& measurement, const int& time)
            {
                if(time!=timeMeasurement_) computeMeasurement(time);
                measurement=convertVector<dynamicgraph::Vector>(measurement_);
                return measurement;
            }

            unsigned& getContactsNbr(unsigned& contactsNbr, const int& time)
            {
                 if(time!=timeStackOfContacts_) computeStackOfContacts(time);
                 contactsNbr = contactsNbr_;
                 return contactsNbr;
            }

            unsigned& getModeledContactsNbr(unsigned& modeledContactsNbr, const int& time)
            {
                 if(time!=timeStackOfContacts_) computeStackOfContacts(time);
                 modeledContactsNbr = modeledContactsNbr_;
                 return modeledContactsNbr;
            }

            unsigned& getUnmodeledContactsNbr(unsigned& unmodeledContactsNbr, const int& time)
            {
                 if(time!=timeStackOfContacts_) computeStackOfContacts(time);
                 unmodeledContactsNbr = unmodeledContactsNbr_;
                 return unmodeledContactsNbr;
            }

            void setLeftFootBias (const ::dynamicgraph::Vector & b)
            {
                bias_[0]=convertVector<stateObservation::Vector>(b);
            }

            void setRightFootBias (const ::dynamicgraph::Vector & b)
            {
              bias_[1]=convertVector<stateObservation::Vector>(b);
            }

            void setFDInertiaDot(const bool& b)
            {
              derivateInertiaFD_=b;
            }

            void setLastInertia(const dynamicgraph::Matrix & inert)
            {
                const stateObservation::Matrix& homoWaist=convertMatrix<stateObservation::Matrix>(positionWaistSIN.access(timeInput_));
                const stateObservation::Vector& comVector=convertVector<stateObservation::Vector>(comVectorSIN.access(timeInput_));
                computeInert(convertMatrix<stateObservation::Matrix>(inert),homoWaist,comVector,lastInertia_);
            }

            void setLeftHandSensorTransformation(const dynamicgraph::Vector & R)
            {
                forceSensorsTransformation_[contact::lh]=convertVector<stateObservation::Vector>(R);
            }


            dynamicgraph::Vector getLeftHandSensorTransformation() const
            {
                return convertVector<dynamicgraph::Vector>(forceSensorsTransformation_[contact::lh]);
            }

            void setRightHandSensorTransformation(const dynamicgraph::Vector & R)
            {
                forceSensorsTransformation_[contact::rh]=convertVector<stateObservation::Vector>(R);
            }

            dynamicgraph::Vector getRightHandSensorTransformation() const
            {
                return convertVector<dynamicgraph::Vector>(forceSensorsTransformation_[contact::rh]);
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

        private:

            /// Signals
            // Input signals

            /**
            \brief Positions and forces at contacts
            */
            dynamicgraph::SignalPtr <MatrixHomogeneous, int> positionLeftFootSIN_;
            dynamicgraph::SignalPtr <Vector, int> forceLeftFootSIN_;

            dynamicgraph::SignalPtr <MatrixHomogeneous, int> positionRightFootSIN_;
            dynamicgraph::SignalPtr <Vector, int> forceRightFootSIN_;

            dynamicgraph::SignalPtr <MatrixHomogeneous, int> positionLeftHandSIN_;
            dynamicgraph::SignalPtr <Vector, int> forceLeftHandSIN_;

            dynamicgraph::SignalPtr <MatrixHomogeneous, int> positionRightHandSIN_;
            dynamicgraph::SignalPtr <Vector, int> forceRightHandSIN_;

            /**
            \brief Com and derivatives
            */
            dynamicgraph::SignalPtr < ::dynamicgraph::Vector, int> comVectorSIN;

            /**
            \brief Inertia and derivative
            */
            dynamicgraph::SignalPtr < ::dynamicgraph::Matrix, int> inertiaSIN;
            dynamicgraph::SignalPtr < ::dynamicgraph::Vector, int> dinertiaSIN;

            /**
            \brief Angular momentum and derivative
            */
            dynamicgraph::SignalPtr < ::dynamicgraph::Vector, int> angMomentumSIN;
            dynamicgraph::SignalPtr < ::dynamicgraph::Vector, int> dangMomentumSIN;
            dynamicgraph::SignalPtr < ::dynamicgraph::Matrix, int> positionWaistSIN;

            /**
            \brief IMU vector
            */
            dynamicgraph::SignalPtr < ::dynamicgraph::Vector, int> imuVectorSIN;

            /**
            \brief IMU sensors
            */
            dynamicgraph::SignalPtr < ::dynamicgraph::Vector, int> accelerometerSIN;
            dynamicgraph::SignalPtr < ::dynamicgraph::Vector, int> gyrometerSIN;

            // Output signals
            dynamicgraph::SignalPtr <Vector, int> inputSOUT_;
            dynamicgraph::SignalPtr <Vector, int> measurementSOUT_;
            dynamicgraph::SignalPtr <unsigned, int> contactsNbrSOUT_;
            dynamicgraph::SignalPtr <unsigned, int> modeledContactsNbrSOUT_;
            dynamicgraph::SignalPtr <unsigned, int> unmodeledContactsNbrSOUT_;

            /// Methods
            void getForcesInControlFrame(const int& time);
            void getSensorsPositionsInControlFrame(const int& time);
            void computeStackOfContacts(const int& time);
            void computeInput(const int& inTime);
            void computeMeasurement(const int& time);

            // From input reconstructor
            void computeInert
                 (const stateObservation::Matrix & inertia, const stateObservation::Matrix & homoWaist,
                  const stateObservation::Vector&, stateObservation::Vector&);

            /// Parameters
            double timeStackOfContacts_, timeInput_, timeMeasurement_, timeSensorsPositions_, timeForces_;

            stateObservation::Vector forceThresholds_;
            stateObservation::Vector forceResidus_;
            std::vector<bool> modeled_;

            std::list<int> stackOfContacts_;
            std::list<int> stackOfModeledContacts_;
            std::list<int> stackOfUnmodeledContacts_;
            std::list<int>::iterator iterator;

            unsigned contactsNbr_, modeledContactsNbr_, unmodeledContactsNbr_;

            std::vector<stateObservation::Matrix4,Eigen::aligned_allocator_indirection<stateObservation::Matrix4> > inputHomoPosition_;
            std::vector<stateObservation::Vector6,Eigen::aligned_allocator_indirection<stateObservation::Vector6> > inputPosition_;
            std::vector<stateObservation::Vector6,Eigen::aligned_allocator_indirection<stateObservation::Vector6> > inputForces_;

            std::vector<stateObservation::Vector3,Eigen::aligned_allocator_indirection<stateObservation::Vector3> > forceSensorsTransformation_;

            stateObservation::Vector input_;
            stateObservation::Vector measurement_;

            // From input reconstructor
            std::vector<stateObservation::Vector> bias_;
            bool derivateInertiaFD_;
            stateObservation::Vector lastInertia_;
            double dt_;

            struct Optimization
            {
                // Compute stack of contacts
                bool found;

                // Get forces in control frame
                stateObservation::Vector6 forceResidusVector;
                stateObservation::Matrix3 Rct, Rc;
                stateObservation::Vector3 pc;
                stateObservation::Vector3 weight;
                stateObservation::Vector6 force;

            } op_;

        public:
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      };

} // namespace sotStateObservation

#endif // ESTIMATORINTERFACE_HH
