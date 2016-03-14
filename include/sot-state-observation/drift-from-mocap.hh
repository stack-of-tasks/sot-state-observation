#ifndef DG_DRIFT_FROM_MOCAP_HH
#define DG_DRIFT_FROM_MOCAP_HH


#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>

#include <sot/core/matrix-homogeneous.hh>
#include <sot/core/vector-utheta.hh>

#include <sot-state-observation/tools/definitions.hh>

namespace sotStateObservation
{
        /**
           \brief
        */
        class DriftFromMocap :
            public dynamicgraph::Entity,
            private boost::noncopyable //
        {
        public:
            /**
            \brief Constructor by name
            */
            DriftFromMocap(const std::string& inName);

            ~DriftFromMocap();

            /// Each entity should provide the name of the class it belongs to
            virtual const std::string& getClassName (void) const
            {
                return CLASS_NAME;
            }

            /// Header documentation of the python class
            virtual std::string getDocString () const
            {
                return
                    "Entity that rebuilds the 6D drift from the measurement of the Mocap";
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


            void init();

            ::dynamicgraph::sot::MatrixHomogeneous& computeDrift
              (::dynamicgraph::sot::MatrixHomogeneous & drift, const int& inTime);

            ::dynamicgraph::Vector& computeDriftVector
              (::dynamicgraph::Vector & drift, const int& inTime);

            /**
              \brief limb to global frame position
            */
            dynamicgraph::SignalPtr < ::dynamicgraph::sot::MatrixHomogeneous, int> limbGlobalSIN;
            /**
              \brief limb to lobal frame position
            */
            dynamicgraph::SignalPtr < ::dynamicgraph::sot::MatrixHomogeneous, int> limbLocalSIN;

            /**
              \brief drift
            */
            dynamicgraph::SignalTimeDependent < ::dynamicgraph::sot::MatrixHomogeneous, int> driftSOUT;

            dynamicgraph::SignalTimeDependent < ::dynamicgraph::Vector, int> driftVectorSOUT;

            dynamicgraph::sot::MatrixHomogeneous init_;

            dynamicgraph::sot::MatrixHomogeneous lastDrift_;

            bool initialized_;

        };
}



#endif
