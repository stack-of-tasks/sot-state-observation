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
            Compute the control law
            */
            ::dynamicgraph::Vector& computeFlexibility
                        (::dynamicgraph::Vector & Flexibility, const int& inTime);

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
            \brief Estimation of the attitude
            */
            dynamicgraph::SignalTimeDependent < ::dynamicgraph::Vector, int> flexibilitySOUT;


            stateObservation::flexibilityEstimation::FixedContactEKFFlexEstimatorIMU estimator_;


            ///Sizes of the states for the state, the measurement, and the input vector
            static const unsigned stateSize=18;
            static const unsigned measurementSize=6;
            static const unsigned inputSize=15;

        };

} // namespace sotStateObservation

#endif //SOT_DYNAMIC_GRAPH_IMU_FLEXIBILITY_ESTIMATION_HH
