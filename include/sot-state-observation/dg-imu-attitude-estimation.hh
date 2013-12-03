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
           \brief
        */
        class DGIMUAttitudeEstimation :
            public dynamicgraph::Entity,
            private boost::noncopyable //
        {
        public:
            /**
            \brief Constructor by name
            */
            DGIMUAttitudeEstimation(const std::string& inName);

            ~DGIMUAttitudeEstimation();

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

            void setStateGuess (const ::dynamicgraph::Vector & xh0)
            {
                filter_.setState(convertVector<stateObservation::Vector>(xh0),attitudeSOUT.getTime());
            }

            void setStateGuessCovariance (const ::dynamicgraph::Matrix & p)
            {
                filter_.setStateCovariance(convertMatrix<stateObservation::Matrix>(p));
            }

            void setSensorsNoiseCovariance (const ::dynamicgraph::Matrix & r)
            {
                filter_.setR(convertMatrix<stateObservation::Matrix>(r));
            }

            void setProcessNoiseCovariance (const ::dynamicgraph::Matrix & q)
            {
                filter_.setQ(convertMatrix<stateObservation::Matrix>(q));
            }

            void setSamplingPeriod (const double & dt)
            {
                imuFunctor_.setSamplingPeriod(dt);
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

            ///Sizes of the states for the state, the measurement, and the input vector
            static const unsigned stateSize=18;
            static const unsigned measurementSize=6;
            static const unsigned inputSize=6;

        };

} // namespace sotStateObservation

#endif //SOT_DYNAMIC_GRAPH_IMU_ATTITTUDE_ESTIMATION_HH

