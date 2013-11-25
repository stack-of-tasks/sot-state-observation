/*
 *  Copyright 2013 CNRS
 *
 *  Mehdi Benallegue
 */

#ifndef SOT_DYNAMIC_GRAPH_IMU_ATTITTUDE_ESTIMATION_HH
#define SOT_DYNAMIC_GRAPH_IMU_ATTITTUDE_ESTIMATION_HH

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/linear-algebra.h>

#include <state-observation/dynamical-system/imu-dynamical-system.hpp>
#include <state-observation/dynamical-system/dynamical-system-simulator.hpp>
#include <state-observation/observer/extended-kalman-filter.hpp>
#include <state-observation/tools/miscellaneous-algorithms.hpp>

#include <sot-state-observation/tools/definitions.hh>

namespace sotStateObservation
{
        /**
           \brief Feedback controller for an inverted pendulum

           This class implements a feedback control for the inverted pendulum
           represented by class InvertedPendulum
        */
        class DynamicGraphIMUAttitudeEstimation : public dynamicgraph::Entity
        {
        public:
            /**
            \brief Constructor by name
            */
            DynamicGraphIMUAttitudeEstimation(const std::string& inName);

            ~DynamicGraphIMUAttitudeEstimation();

            /// Each entity should provide the name of the class it belongs to
            virtual const std::string& getClassName (void) const
            {
                return CLASS_NAME;
            }

            /// Header documentation of the python class
            virtual std::string getDocString () const
            {
                return
                    "A state observer which takes an IMU and gives the attitude\n";
            }

            void setStateGuess (const ::dynamicgraph::Vector & xh0, unsigned k)
            {
                filter_.setState(convertVector<stateObservation::Vector>(xh0),k);
            }

            void setStateGuessCovariance (const ::dynamicgraph::Matrix & p)
            {
                filter_.setStateCovariance(convertMatrix<stateObservation::Matrix>(p));
            }

            void setSensorsNoiseCovariance (const ::dynamicgraph::Matrix & q, unsigned k)
            {
                filter_.setQ(convertMatrix<stateObservation::Matrix>(q));
            }

            void setProcessNoiseCovariance (const ::dynamicgraph::Matrix & r, unsigned k)
            {
                filter_.setR(convertMatrix<stateObservation::Matrix>(r));
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
            ::dynamicgraph::Vector& computeAttitude
                        (::dynamicgraph::Vector & Attitude, const int& inTime);

            /**
            \brief Measurement of the IMU
            */
            dynamicgraph::SignalPtr < ::dynamicgraph::Vector, int> measurementSIN;

            /**
            \brief Input of the dynamical system
            */
            dynamicgraph::SignalPtr < ::dynamicgraph::Vector, int> inputSIN;

            /**
            \brief Estimation of the attitude
            */
            dynamicgraph::SignalTimeDependent < ::dynamicgraph::Vector, int> attitudeSOUT;

            ///initialization of the extended Kalman filter
            stateObservation::ExtendedKalmanFilter filter_;

            ///initalization of the functor
            stateObservation::IMUDynamicalSystem imuFunctor_;

        };

} // namespace sotStateObservation

#endif //SOT_DYNAMIC_GRAPH_IMU_ATTITTUDE_ESTIMATION_HH
