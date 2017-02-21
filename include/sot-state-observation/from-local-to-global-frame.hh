/*
 *  Copyright 2017 CNRS
 *
 *  Alexis MIFSUD
 */

#ifndef FROMLOCALTOGLOBALFRAME
#define FROMLOCALTOGLOBALFRAME

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
        class FromLocalToGlobalFrame :
            public dynamicgraph::Entity,
            private boost::noncopyable //
        {
        public:
            /**
            \brief Constructor by name
            */
            FromLocalToGlobalFrame(const std::string& inName);

            ~FromLocalToGlobalFrame();

            /// Each entity should provide the name of the class it belongs to
            virtual const std::string& getClassName (void) const
            {
                return CLASS_NAME;
            }

            /// Header documentation of the python class
            virtual std::string getDocString () const
            {
                return
                    "Entity that get an sin signal expressed in the local frame and express it in the global frame";
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

            void getMandatorySignals(const int& intTime);
            void computeSoutPos(const int& inTime);
            void computeSoutVel(const int& inTime);

            dynamicgraph::Vector& getSoutPos(dynamicgraph::Vector & sout, const int& inTime)
            {
                if(inTime!=timeComputeSoutPos_) computeSoutPos(inTime);
                sout=convertVector<dynamicgraph::Vector>(soutPos_);
                return sout;
            }

            dynamicgraph::Vector& getSoutVel(dynamicgraph::Vector & sout, const int& inTime)
            {
                computeSoutVel(inTime);
                sout=convertVector<dynamicgraph::Vector>(soutVel_);
                return sout;
            }

            dynamicgraph::Matrix& getSoutHomo(dynamicgraph::Matrix & soutHomo, const int& inTime)
            {
                computeSoutPos(inTime);
                soutHomo=convertMatrix<dynamicgraph::Matrix>(stateObservation::kine::vector6ToHomogeneousMatrix(soutPos_));
                return soutHomo;
            }

            /**
            \brief input signals (vector 6d)
            */
            dynamicgraph::SignalPtr < ::dynamicgraph::Vector, int> sinPosSIN;
            dynamicgraph::SignalPtr < ::dynamicgraph::Vector, int> sinVelSIN;

            /**
            \brief flexibility state
            */
            dynamicgraph::SignalPtr < ::dynamicgraph::Vector, int> flexStateSIN;

            /**
            \brief output signals in the global frame
            */
            dynamicgraph::Signal < dynamicgraph::Vector, int> soutPosSOUT;
            dynamicgraph::Signal < dynamicgraph::Vector, int> soutVelSOUT;
            dynamicgraph::Signal < dynamicgraph::Matrix, int> soutHomoSOUT;

            int inTime_;
            int timeGetMandatorySignals_, timeComputeSoutPos_;

            unsigned flexStateSize_;
            stateObservation::Matrix3 Rflex_;
            stateObservation::Vector3 omegaflex_;
            stateObservation::Vector3 tflex_;
            stateObservation::Vector3 wflex_;
            stateObservation::Vector3 dtflex_;

            stateObservation::Vector sinPos_;
            stateObservation::Vector sinVel_;

            stateObservation::Vector6 soutPos_;
            stateObservation::Vector6 soutVel_;
            stateObservation::Matrix4 soutHomo_;

        };

} // namespace sotStateObservation

#endif //FromLocalToGlobalFrame_IMU
