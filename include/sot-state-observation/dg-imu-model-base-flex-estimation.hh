/*
 *  Copyright 2013 CNRS
 *
 *  Mehdi Benallegue
 */

#ifndef SOT_DYNAMIC_GRAPH_IMU_MODEL_BASE_FLEX_ESTIMATION_HH
#define SOT_DYNAMIC_GRAPH_IMU_MODEL_BASE_FLEX_ESTIMATION_HH

#define SOT_STATE_OBSERVATION_CHECK_UNIQUENESS_IN_TIME

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/linear-algebra.h>

#include <sot/core/matrix-homogeneous.hh>

#include <state-observation/tools/miscellaneous-algorithms.hpp>
#include <state-observation/flexibility-estimation/model-base-ekf-flex-estimator-imu.hpp>

#include <sot-state-observation/tools/definitions.hh>

namespace sotStateObservation
{
        /**
           \brief
        */
        class DGIMUModelBaseFlexEstimation :
            public dynamicgraph::Entity,
            private boost::noncopyable //
        {
        public:
            /**
            \brief Constructor by name
            */
            DGIMUModelBaseFlexEstimation(const std::string& inName);

            ~DGIMUModelBaseFlexEstimation();

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

            void setFlexibilityCovariance (const ::dynamicgraph::Matrix & p)
            {
                estimator_.setFlexibilityCovariance(convertMatrix<stateObservation::Matrix>(p));
            }

            ::dynamicgraph::Matrix  getFlexibilityCovariance () const
            {
                return convertMatrix<dynamicgraph::Matrix>(estimator_.getFlexibilityCovariance());
            }

            void setSamplingPeriod(const double & dt)
            {
                estimator_.setSamplingPeriod(dt);
            }

            void setVirtualMeasurementsCovariance(const double & dt)
            {
                estimator_.setVirtualMeasurementsCovariance(dt);
            }

            double getVirtualMeasurementsCovariance() const
            {
                return estimator_.getVirtualMeasurementsCovariance();
            }

            void setProcessNoiseCovariance(const ::dynamicgraph::Matrix & q)
            {
                estimator_.setProcessNoiseCovariance(convertMatrix<stateObservation::Matrix>(q));
            }

            ::dynamicgraph::Matrix getProcessNoiseCovariance() const
            {
                return convertMatrix<dynamicgraph::Matrix>( estimator_.getProcessNoiseCovariance());
            }

            void setMeasurementNoiseCovariance(const ::dynamicgraph::Matrix & r)
            {
                estimator_.setMeasurementNoiseCovariance(convertMatrix<stateObservation::Matrix>(r));
            }

            ::dynamicgraph::Matrix getMeasurementNoiseCovariance() const
            {
                return convertMatrix<dynamicgraph::Matrix>( estimator_.getMeasurementNoiseCovariance());
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


            ::dynamicgraph::Vector& computeFlexPosition
                        (::dynamicgraph::Vector & flexibilityPosition, const int& inTime);

            ::dynamicgraph::Vector& computeFlexVelocity
                        (::dynamicgraph::Vector & flexibilityVelocity, const int& inTime);

            ::dynamicgraph::Vector& computeFlexAcceleration
                        (::dynamicgraph::Vector & flexibilityAcceleration, const int& inTime);

            ::dynamicgraph::Vector& computeFlexThetaU
                        (::dynamicgraph::Vector & flexibilityThetaU, const int& inTime);

            ::dynamicgraph::Vector& computeFlexOmega
                        (::dynamicgraph::Vector & flexibilityOmega, const int& inTime);

            ::dynamicgraph::Vector& computeFlexOmegaDot
                        (::dynamicgraph::Vector & flexibilityOmegaDot, const int& inTime);



            ::dynamicgraph::sot::MatrixHomogeneous& computeFlexTransformationMatrix
                        (::dynamicgraph::sot::MatrixHomogeneous & flexibilityTransformationMatrix,
                                                            const int& inTime);

            ::dynamicgraph::Vector& computeFlexPoseThetaU
                        (::dynamicgraph::Vector & flexibilityThetaU, const int& inTime);

            ::dynamicgraph::Vector& computeFlexVelocityVector
                        (::dynamicgraph::Vector & flexVelocityVector, const int& inTime);



            ::dynamicgraph::Vector& computeFlexInverse
                        (::dynamicgraph::Vector & flexInverse, const int& inTime);

            ::dynamicgraph::sot::MatrixHomogeneous& computeFlexMatrixInverse
                        (::dynamicgraph::sot::MatrixHomogeneous & flexMatrixInverse,
                                                            const int& inTime);

            ::dynamicgraph::Vector& computeFlexInversePoseThetaU
                        (::dynamicgraph::Vector & flexInversePoseThetaU, const int& inTime);

            ::dynamicgraph::Vector& computeFlexInverseVelocityVector
                        (::dynamicgraph::Vector & flexInverseVelocityVector, const int& inTime);

            ::dynamicgraph::Vector& computeFlexInverseVelocity
                        (::dynamicgraph::Vector & flexInverseVelocity, const int& inTime);

            ::dynamicgraph::Vector& computeFlexInverseOmega
                        (::dynamicgraph::Vector & flexInverseOmega, const int &inTime);

            ::dynamicgraph::Vector& computeSimulatedSensors
                        (::dynamicgraph::Vector & sensorSignal, const int &inTime);

            ::dynamicgraph::Vector& computeInovation
                        (::dynamicgraph::Vector & inovation, const int &inTime);

            ::dynamicgraph::Vector& computePredictedSensors
                        (::dynamicgraph::Vector & sensorSignal, const int &inTime);




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
            \brief Estimation of the flexibility
            */
            dynamicgraph::Signal < ::dynamicgraph::Vector, int> flexibilitySOUT;

            /**
            \brief Different parts of the vector of the flexibility estimation vector
            */
            dynamicgraph::SignalTimeDependent
                            < ::dynamicgraph::Vector, int> flexPositionSOUT;

            dynamicgraph::SignalTimeDependent
                            < ::dynamicgraph::Vector, int> flexVelocitySOUT;

            dynamicgraph::SignalTimeDependent
                            < ::dynamicgraph::Vector, int> flexAccelerationSOUT;

            dynamicgraph::SignalTimeDependent
                            < ::dynamicgraph::Vector, int> flexThetaUSOUT;

            dynamicgraph::SignalTimeDependent
                            < ::dynamicgraph::Vector, int> flexOmegaSOUT;

            dynamicgraph::SignalTimeDependent
                            < ::dynamicgraph::Vector, int> flexOmegaDotSOUT;

            /**
            \brief Transformed parts of the flexibility state vector
            */
            dynamicgraph::SignalTimeDependent
                            < ::dynamicgraph::Vector, int> flexPoseThetaUSOUT;

            dynamicgraph::SignalTimeDependent
                            < ::dynamicgraph::sot::MatrixHomogeneous, int>
                                                flexTransformationMatrixSOUT;
            dynamicgraph::SignalTimeDependent
                            < ::dynamicgraph::Vector, int> flexVelocityVectorSOUT;


            /**
            \brief Various parts of the inverse flexibility
            */
            dynamicgraph::SignalTimeDependent
                                < ::dynamicgraph::Vector, int> flexInverseSOUT;
            dynamicgraph::SignalTimeDependent
                                < ::dynamicgraph::sot::MatrixHomogeneous, int>
                                                        flexMatrixInverseSOUT;
            dynamicgraph::SignalTimeDependent
                                < ::dynamicgraph::Vector,int> flexInversePoseThetaUSOUT;
            dynamicgraph::SignalTimeDependent
                                < ::dynamicgraph::Vector,int> flexInverseVelocityVectorSOUT;
            dynamicgraph::SignalTimeDependent
                                < ::dynamicgraph::Vector,int> flexInverseVelocitySOUT;
            dynamicgraph::SignalTimeDependent
                                < ::dynamicgraph::Vector,int> flexInverseOmegaSOUT;
            /**
            \brief A simulation of the sensors' signals
            */
            dynamicgraph::SignalTimeDependent
                                < ::dynamicgraph::Vector,int> simulatedSensorsSOUT;

            dynamicgraph::SignalTimeDependent
                                < ::dynamicgraph::Vector,int> predictedSensorsSOUT;


            /**
            \brief A simulation of the sensors' signals
            */
            dynamicgraph::SignalTimeDependent
                                < ::dynamicgraph::Vector,int> inovationSOUT;


            stateObservation::flexibilityEstimation::ModelBaseEKFFlexEstimatorIMU estimator_;


            ///Sizes of the states for the state, the measurement, and the input vector
            static const unsigned stateSize=18;
            static const unsigned measurementSize=6;
            static const unsigned inputSize=51;

            unsigned contactNumber_;
#ifdef SOT_STATE_OBSERVATION_CHECK_UNIQUENESS_IN_TIME
            int currentTime_;
#endif

        };

} // namespace sotStateObservation

#endif //SOT_DYNAMIC_GRAPH_IMU_MODEL_BASE_FLEX_ESTIMATION_HH
