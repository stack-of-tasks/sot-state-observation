/*
 *  Copyright 2015 CNRS
 *
 *  Alexis MIFSUD
 */

#ifndef SOT_DYNAMIC_GRAPH_CALIBRATE
#define SOT_DYNAMIC_GRAPH_CALIBRATE
#define SOT_STATE_OBSERVATION_CHECK_UNIQUENESS_IN_TIME

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/linear-algebra.h>

#include <sot/core/matrix-homogeneous.hh>

#include <state-observation/tools/miscellaneous-algorithms.hpp>

#include <sot-state-observation/tools/definitions.hh>

namespace sotStateObservation
{
        /**
           \brief
        */
        class Calibrate :
            public dynamicgraph::Entity,
            private boost::noncopyable //
        {
        public:
            /**
            \brief Constructor by name
            */
            Calibrate(const std::string& inName);

            ~Calibrate();

            /// Each entity should provide the name of the class it belongs to
            virtual const std::string& getClassName (void) const
            {
                return CLASS_NAME;
            }

            /// Header documentation of the python class
            virtual std::string getDocString () const
            {
                return
                    "Entity that calibrate the inertial measurement unit";
            }

            dynamicgraph::Matrix getRa() const
            {
                return convertMatrix<dynamicgraph::Matrix>(R_.block(0,0,3,3));
            }

            dynamicgraph::Matrix getRg() const
            {
                return convertMatrix<dynamicgraph::Matrix>(R_.block(3,3,3,3));
            }

            dynamicgraph::Vector gettc() const
            {
                return convertVector<dynamicgraph::Vector>(tc_);
            }

            dynamicgraph::Vector gettcom() const
            {
                return convertVector<dynamicgraph::Vector>(tcom_);
            }


            void setRa(const dynamicgraph::Matrix & m)
            {
                R_.block(0,0,3,3)=convertMatrix<stateObservation::Matrix>(m);
            }

            void setRg(const dynamicgraph::Matrix & m)
            {
                R_.block(3,3,3,3)=convertMatrix<stateObservation::Matrix>(m);
            }

            void settc(const dynamicgraph::Vector & v)
            {
                tc_=convertVector<stateObservation::Vector>(v);
            }

            void settcom(const dynamicgraph::Vector & v)
            {
                tcom_=convertVector<stateObservation::Vector>(v);
            }

            void start(const int & nbStep)
            {
                calibrate_=true;
                nbStep_=nbStep;
                currentStep_=0;
                sumImuIn_.setZero();
                sumContactsPositionIn_.setZero();
                sumComIn_.setZero();
            }

            void reset()
            {
                R_.setIdentity();
                tc_.setZero();
                tcom_.setZero();
                sumImuIn_.setZero();
                sumContactsPositionIn_.setZero();
                sumComIn_.setZero();
            }

            void calibrate(const int& inTime);


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
            dynamicgraph::Vector& computeImu
                  (dynamicgraph::Vector & input, const int& inTime);

            dynamicgraph::Vector& computeContactsPosition
                    (dynamicgraph::Vector & contactsPositionOut, const int& inTime);

            dynamicgraph::Vector& computeCom
                    (dynamicgraph::Vector & comOut, const int& inTime);

            /**
            \brief input IMU vector
            */
            dynamicgraph::SignalPtr < ::dynamicgraph::Vector, int> imuSIN;

            /**
            \brief input contacts position
            */
            dynamicgraph::SignalPtr < ::dynamicgraph::Vector, int> contactsPositionSIN;

            /**
            \brief com
            */
            dynamicgraph::SignalPtr < ::dynamicgraph::Vector, int> comSIN;
            dynamicgraph::SignalPtr < ::dynamicgraph::Vector, int> comSOUT;

            /**
            \brief output IMU vector
            */
            dynamicgraph::SignalPtr < dynamicgraph::Vector, int> imuSOUT;

            /**
            \brief output contacts position
            */
            dynamicgraph::SignalPtr < ::dynamicgraph::Vector, int> contactsPositionSOUT;


            /**
            \brief number of contacts
            */
            dynamicgraph::SignalPtr < unsigned, int> contactsNbrSIN;

            // Rotational matrix between the measured orientation of the IMU and the one we use
            stateObservation::Matrix R_;
            stateObservation::Vector sumImuIn_;

            stateObservation::Vector tc_;
            stateObservation::Vector sumContactsPositionIn_;

            stateObservation::Vector tcom_;
            stateObservation::Vector sumComIn_;

            bool calibrate_;
            int nbStep_;
            int currentStep_;
            int inTime_;

        };

} // namespace sotStateObservation

#endif //SOT_DYNAMIC_GRAPH_CALIBRATE_IMU
