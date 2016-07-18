//
// Copyright (c) 2016,
// Alexis Mifsud
//
// CNRS
//
// This file is part of sot-state-observation.
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

            unsigned& getContactsModel(unsigned& contactsModel, const int& time)
            {
                 if(time!=timeContacts_) computeContacts(time);
                 contactsModel = contactsModel_;
                 return contactsModel;
            }

            unsigned& getConfig(unsigned& config, const int& time)
            {
                 if(time!=timeContacts_) computeContacts(time);
                 config=config_;
                 return config;
            }

            unsigned& getContactsNbr(unsigned& contactsNbr, const int& time)
            {
                 if(time!=timeContacts_) computeContacts(time);
                 contactsNbr = contactsNbr_;
                 return contactsNbr;
            }

            unsigned& getModeledContactsNbr(unsigned& modeledContactsNbr, const int& time)
            {
                 if(time!=timeContacts_) computeContacts(time);
                 modeledContactsNbr = modeledContactsNbr_;
                 return modeledContactsNbr;
            }

            unsigned& getUnmodeledContactsNbr(unsigned& unmodeledContactsNbr, const int& time)
            {
                 if(time!=timeContacts_) computeContacts(time);
                 unmodeledContactsNbr = unmodeledContactsNbr_;
                 return unmodeledContactsNbr;
            }

            unsigned& getSupportContactsNbr(unsigned& supportContactsNbr, const int& time)
            {
                 if(time!=timeContacts_) computeContacts(time);
                 supportContactsNbr = supportContactsNbr_;
                 return supportContactsNbr;
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

            void setLeftHandSensorTransformation(const dynamicgraph::Vector & v)
            {
                forceSensorsTransformation_[contact::lh]=convertVector<stateObservation::Vector>(v);
                
                stateObservation::Matrix3 Rr, Rp, Ry;
                Rr << 1, 0,         0,
                      0, cos(v(0)), -sin(v(0)),
                      0, sin(v(0)), cos(v(0));

                Rp << cos(v(1)), 0, sin(v(1)),
                      0,         1, 0,
                      -sin(v(1)), 0, cos(v(1));

                Ry << cos(v(2)), -sin(v(2)), 0,
                      sin(v(2)), cos(v(2)),  0,
                      0,         0,          1;
                forceSensorsTransfoMatrix_[contact::lh]=Ry*Rp*Rr;
            }

            dynamicgraph::Vector getLeftHandSensorTransformation() const
            {
                return convertVector<dynamicgraph::Vector>(forceSensorsTransformation_[contact::lh]);
            }

            void setRightHandSensorTransformation(const dynamicgraph::Vector & v)
            {
                forceSensorsTransformation_[contact::rh]=convertVector<stateObservation::Vector>(v);

                stateObservation::Matrix3 Rr, Rp, Ry;
                Rr << 1, 0,         0,
                0, std::cos(v(0)), -std::sin(v(0)),
                      0, std::sin(v(0)), std::cos(v(0));

                Rp << std::cos(v(1)), 0, std::sin(v(1)),
                      0,         1, 0,
                      -std::sin(v(1)), 0, std::cos(v(1));

                Ry << std::cos(v(2)), -std::sin(v(2)), 0,
                      std::sin(v(2)), std::cos(v(2)),  0,
                      0,         0,          1;
                forceSensorsTransfoMatrix_[contact::rh]=Ry*Rp*Rr;
            }

            dynamicgraph::Vector getRightHandSensorTransformation() const
            {
                return convertVector<dynamicgraph::Vector>(forceSensorsTransformation_[contact::rh]);
            }

            void setWithUnmodeledMeasurements(const bool & b)
            {
                withUnmodeledMeasurements_=b;
            }

            bool getWithUnmodeledMeasurements() const
            {
                return withUnmodeledMeasurements_;
            }

            dynamicgraph::Vector& getStackOfSupportContacts(dynamicgraph::Vector& stackOfSupportContacts, const int& time)
            {
                if(time!=timeStackOfContacts_) computeStackOfContacts(time);

                stackOfSupportContacts.resize(stackOfSupportContacts_.size());
                op_.i=0;
                for (iterator = stackOfSupportContacts_.begin(); iterator != stackOfSupportContacts_.end(); ++iterator)
                {
                    stackOfSupportContacts(op_.i)=*iterator;
                    ++op_.i;
                }
                return stackOfSupportContacts;
            }

            void setElastPendulumModel(const unsigned & i)
            {
                elastPendulumModel_=i;
            }

            MatrixHomogeneous& getPositionSupport1(MatrixHomogeneous& positionSupport1, const int& time)
            {
                if(time!=timeContacts_) computeContacts(time);
                if(stackOfSupportContacts_.size()>=1)
                {
                    iterator = stackOfSupportContacts_.begin();
                    positionSupport1=convertMatrix<dynamicgraph::Matrix>(inputHomoPosition_[*iterator]);
                }
                else
                {
                    positionSupport1.setIdentity();
                }
                return positionSupport1;
            }

            MatrixHomogeneous& getPositionSupport2(MatrixHomogeneous& positionSupport2, const int& time)
            {
                if(time!=timeContacts_) computeContacts(time);
                if(stackOfSupportContacts_.size()>=2)
                {
                    iterator = stackOfSupportContacts_.begin();
                    ++iterator;
                    positionSupport2=convertMatrix<dynamicgraph::Matrix>(inputHomoPosition_[*iterator]);
                }
                else
                {
                    positionSupport2.setIdentity();
                }
                return positionSupport2;
            }



            Vector& getVelocitySupport1(Vector& velocitySupport1, const int& time)
            {
                if(time!=timeContacts_) computeContacts(time);
                if(stackOfSupportContacts_.size()>=1)
                {
                    iterator = stackOfSupportContacts_.begin();
                    velocitySupport1=convertVector<dynamicgraph::Vector>(inputVelocity_[*iterator]);
                }
                else
                {
                    velocitySupport1.setZero();
                }
                return velocitySupport1;
            }

            Vector& getVelocitySupport2(Vector& velocitySupport2, const int& time)
            {
                if(time!=timeContacts_) computeContacts(time);
                if(stackOfSupportContacts_.size()>=2)
                {
                    iterator = stackOfSupportContacts_.begin();
                    ++iterator;
                    velocitySupport2=convertVector<dynamicgraph::Vector>(inputVelocity_[*iterator]);
                }
                else
                {
                    velocitySupport2.setZero();
                }
                return velocitySupport2;
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
            dynamicgraph::SignalPtr <Vector, int> velocityLeftFootSIN_;
            dynamicgraph::SignalPtr <Vector, int> forceLeftFootSIN_;

            dynamicgraph::SignalPtr <MatrixHomogeneous, int> positionRightFootSIN_;
            dynamicgraph::SignalPtr <Vector, int> velocityRightFootSIN_;
            dynamicgraph::SignalPtr <Vector, int> forceRightFootSIN_;

            dynamicgraph::SignalPtr <MatrixHomogeneous, int> positionLeftHandSIN_;
            dynamicgraph::SignalPtr <Vector, int> forceLeftHandSIN_;

            dynamicgraph::SignalPtr <MatrixHomogeneous, int> positionRightHandSIN_;
            dynamicgraph::SignalPtr <Vector, int> forceRightHandSIN_;

            dynamicgraph::SignalPtr <Vector, int> positionLeftStringSIN_;
            dynamicgraph::SignalPtr <Vector, int> positionRightStringSIN_;

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

            /**
            \brief Drift
            */
            dynamicgraph::SignalPtr < ::dynamicgraph::Vector, int> driftSIN;

            // Output signals
            dynamicgraph::SignalPtr <Vector, int> inputSOUT_;
            dynamicgraph::SignalPtr <Vector, int> measurementSOUT_;
            dynamicgraph::SignalPtr <unsigned, int> contactsModelSOUT_;
            dynamicgraph::SignalPtr <unsigned, int> configSOUT_;
            dynamicgraph::SignalPtr <unsigned, int> contactsNbrSOUT_;
            dynamicgraph::SignalPtr <unsigned, int> modeledContactsNbrSOUT_;
            dynamicgraph::SignalPtr <unsigned, int> unmodeledContactsNbrSOUT_;
            dynamicgraph::SignalPtr <unsigned, int> supportContactsNbrSOUT_;
            dynamicgraph::SignalPtr <Vector, int> stackOfSupportContactsSOUT_;
            dynamicgraph::SignalPtr <MatrixHomogeneous, int> positionSupport1SOUT_;
            dynamicgraph::SignalPtr <MatrixHomogeneous, int> positionSupport2SOUT_;            

            /// Methods
            void getForces(const int& time);
            void getForcesInControlFrame(const int& time);
            void getSensorsKineInControlFrame(const int& time);
            void getDrift(const int& time);
            void computeStackOfContacts(const int& time);
            void computeInput(const int& inTime);
            void computeMeasurement(const int& time);
            void computeContacts(const int& time);

            // From input reconstructor
            void computeInert
                 (const stateObservation::Matrix & inertia, const stateObservation::Matrix & homoWaist,
                  const stateObservation::Vector&, stateObservation::Vector&);

            /// Parameters
            double timeStackOfContacts_, timeInput_, timeMeasurement_, timeSensorsPositions_,
                   timeForces_, timeForcesInControlFrame_, timeDrift_, timeContacts_;

            stateObservation::Vector forceThresholds_;
            stateObservation::Vector forceResidus_;
            std::vector<bool> modeled_;
            std::vector<bool> support_;

            std::list<int> stackOfContacts_;
            std::list<int> stackOfModeledContacts_;
            std::list<int> stackOfUnmodeledContacts_;
            std::list<int> stackOfSupportContacts_;
            std::list<int>::iterator iterator;

            unsigned contactsNbr_, modeledContactsNbr_, unmodeledContactsNbr_, supportContactsNbr_;
            unsigned contactsModel_, elastPendulumModel_;
            unsigned config_;

            std::vector<stateObservation::Matrix4,Eigen::aligned_allocator_indirection<stateObservation::Matrix4> > inputHomoPosition_;
            std::vector<stateObservation::Vector6,Eigen::aligned_allocator_indirection<stateObservation::Vector6> > inputPosition_;
            std::vector<stateObservation::Vector6,Eigen::aligned_allocator_indirection<stateObservation::Vector6> > inputVelocity_;
            std::vector<stateObservation::Vector6,Eigen::aligned_allocator_indirection<stateObservation::Vector6> > inputForces_;
            std::vector<stateObservation::Vector6,Eigen::aligned_allocator_indirection<stateObservation::Vector6> > controlFrameForces_;

            std::vector<stateObservation::Vector3,Eigen::aligned_allocator_indirection<stateObservation::Vector3> > forceSensorsTransformation_;
            std::vector<stateObservation::Matrix3,Eigen::aligned_allocator_indirection<stateObservation::Matrix3> > forceSensorsTransfoMatrix_;

            stateObservation::Vector6 drift_;

            stateObservation::Vector input_;
            stateObservation::Vector measurement_;

            bool withUnmodeledMeasurements_;

            // From input reconstructor
            std::vector<stateObservation::Vector> bias_;
            bool derivateInertiaFD_;
            stateObservation::Vector lastInertia_;
            double dt_;

            struct Optimization
            {
                // Compute stack of contacts
                bool found;
                double contactForce;

                // Get forces in control frame
                double forceTransfo;
                stateObservation::Vector6 forceResidusVector;
                stateObservation::Matrix3 Rct, Rc;
                stateObservation::Vector3 weight;
                stateObservation::Vector6 force;
                stateObservation::Vector3 l;

                // Compute Input
                stateObservation::Vector contactKine;
                stateObservation::Vector bias;
                stateObservation::Vector inert;
                stateObservation::Vector dinert;
                double m;
                stateObservation::Vector com;
                stateObservation::Vector comddot;
                stateObservation::Vector dangMomentumOut;

                // General
                stateObservation::Vector3 pc;

                // Iterators
                int i;

            } op_;

        public:
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      };

} // namespace sotStateObservation

#endif // ESTIMATORINTERFACE_HH
