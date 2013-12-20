/*
 *  Copyright 2013 CNRS
 *
 *  Mehdi Benallegue
 */

#ifndef SOT_DYNAMIC_GRAPH_IMU_FLEXIBILITY_ESTIMATION_HH
#define SOT_DYNAMIC_GRAPH_IMU_FLEXIBILITY_ESTIMATION_HH

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/linear-algebra.h>

#include <sot/core/matrix-homogeneous.hh>

#include <state-observation/tools/miscellaneous-algorithms.hpp>
#include <state-observation/flexibility-estimation/fixed-contact-ekf-flex-estimator-imu.hpp>

#include <sot-state-observation/tools/definitions.hh>

namespace sotStateObservation
{
        /**
           \brief
        */
        class DGIMUFlexibilityEstimation :
            public dynamicgraph::Entity,
            private boost::noncopyable //
        {
        public:
            /**
            \brief Constructor by name
            */
            DGIMUFlexibilityEstimation(const std::string& inName);

            ~DGIMUFlexibilityEstimation();

            /// Each entity should provide the name of the class it belongs to
            virtual const std::string& getClassName (void) const
            {
                return CLASS_NAME;
            }

            /// Header documentation of the python class
            virtual std::string getDocString () const
            {
                return
                    "A state observer which takes an IMU and gives the flexibility state\n";
            }

            void setFlexibilityGuess (const ::dynamicgraph::Vector & xh0)
            {
                estimator_.setFlexibilityGuess(convertVector<stateObservation::Vector>(xh0));
            }

            void setFlexibilityGuessCovariance (const ::dynamicgraph::Matrix & p)
            {
                estimator_.setFlexibilityGuessCovariance(convertMatrix<stateObservation::Matrix>(p));
            }

            void setSamplingPeriod(const double & dt)
            {
                estimator_.setSamplingPeriod(dt);
            }

            void increment()
            {
                flexibilitySOUT(flexibilitySOUT.getTime()+1);
            }

            int getFlexTime() const
            {
                return flexibilitySOUT.getTime();
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
            /**
            */
            ::dynamicgraph::Vector& computeFlexibility
                        (::dynamicgraph::Vector & flexibility, const int& inTime);

            ::dynamicgraph::Vector& computeFlexPoseThetaU
                        (::dynamicgraph::Vector & flexibilityThetaU, const int& inTime);

            ::dynamicgraph::Vector& computeFlexPosition
                        (::dynamicgraph::Vector & flexibilityPosition, const int& inTime);

            ::dynamicgraph::Vector& computeFlexVelocity
                        (::dynamicgraph::Vector & flexibilityVelocity, const int& inTime);


            ::dynamicgraph::Vector& computeFlexAcceleration
                        (::dynamicgraph::Vector & flexibilityAcceleration, const int& inTime);

            ::dynamicgraph::Vector& computeFlexThetaU
                        (::dynamicgraph::Vector & flexibilityThetaU, const int& inTime);

            ::dynamicgraph::sot::MatrixHomogeneous& computeFlexTransformationMatrix
                        (::dynamicgraph::sot::MatrixHomogeneous & flexibilityTransformationMatrix, const int& inTime);

            ::dynamicgraph::Vector& computeFlexOmega
                        (::dynamicgraph::Vector & flexibilityOmega, const int& inTime);

            ::dynamicgraph::Vector& computeFlexOmegaDot
                        (::dynamicgraph::Vector & flexibilityOmegaDot, const int& inTime);

            ::dynamicgraph::Vector& computeFlexInverse
                        (::dynamicgraph::Vector & flexInverse, const int& inTime);

            ::dynamicgraph::sot::MatrixHomogeneous& computeFlexMatrixInverse
                        (::dynamicgraph::sot::MatrixHomogeneous & flexMatrixInverse, const int& inTime);




            /**
            \brief Measurement of the IMU
            */
            dynamicgraph::SignalPtr < ::dynamicgraph::Vector, int> measurementSIN;

            /**
            \brief Input of the dynamical system
            */
            dynamicgraph::SignalPtr < ::dynamicgraph::Vector, int> inputSIN;

            /**
            \brief Number of the contact points
            */
            dynamicgraph::SignalPtr < unsigned , int> contactsNbrSIN;

            /**
            \brief Position of the first contact
            */
            dynamicgraph::SignalPtr < ::dynamicgraph::Vector , int> contact1SIN;

            /**
            \brief Position of the second contact
            */
            dynamicgraph::SignalPtr < ::dynamicgraph::Vector , int> contact2SIN;

            /**
            \brief Position of the third contact
            */
            dynamicgraph::SignalPtr < ::dynamicgraph::Vector , int> contact3SIN;

            /**
            \brief Position of the fourth contact
            */
            dynamicgraph::SignalPtr < ::dynamicgraph::Vector , int> contact4SIN;

                        /**
            \brief Different parts of the vector of the flexibility estimation vector
            */

            dynamicgraph::SignalTimeDependent < ::dynamicgraph::Vector, int> flexPoseThetaUSOUT;

            dynamicgraph::SignalTimeDependent < ::dynamicgraph::Vector, int> flexPositionSOUT;

            dynamicgraph::SignalTimeDependent < ::dynamicgraph::Vector, int> flexVelocitySOUT;

            dynamicgraph::SignalTimeDependent < ::dynamicgraph::Vector, int> flexAccelerationSOUT;

            dynamicgraph::SignalTimeDependent < ::dynamicgraph::Vector, int> flexThetaUSOUT;

            dynamicgraph::SignalTimeDependent < ::dynamicgraph::sot::MatrixHomogeneous, int> flexTransformationMatrixSOUT;


            dynamicgraph::SignalTimeDependent < ::dynamicgraph::Vector, int> flexVelocityVectorSOUT;



            dynamicgraph::SignalTimeDependent < ::dynamicgraph::Vector, int> flexOmegaSOUT;

            dynamicgraph::SignalTimeDependent < ::dynamicgraph::Vector, int> flexOmegaDotSOUT;



            dynamicgraph::SignalTimeDependent < ::dynamicgraph::Vector, int> flexInverseSOUT;

            dynamicgraph::SignalTimeDependent < ::dynamicgraph::sot::MatrixHomogeneous, int> flexMatrixInverseSOUT;

            dynamicgraph::SignalTimeDependent < ::dynamicgraph::Vector,int> flexInversePoseThetaUSOUT;

            dynamicgraph::SignalTimeDependent < ::dynamicgraph::Vector,int> flexInverseVelocityVectorSOUT;


            /**
            \brief Estimation of the flexibility
            */
            dynamicgraph::SignalPtr < ::dynamicgraph::Vector, int> flexibilitySOUT;


            stateObservation::flexibilityEstimation::FixedContactEKFFlexEstimatorIMU estimator_;


            ///Sizes of the states for the state, the measurement, and the input vector
            static const unsigned stateSize=18;
            static const unsigned measurementSize=6;
            static const unsigned inputSize=15;

        };

} // namespace sotStateObservation

#endif //SOT_DYNAMIC_GRAPH_IMU_FLEXIBILITY_ESTIMATION_HH
