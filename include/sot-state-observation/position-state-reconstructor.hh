#ifndef DG_POSITION_STATE_RECONSTRUCTOR_HH
#define DG_POSITION_STATE_RECONSTRUCTOR_HH

#include <deque>

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/linear-algebra.h>
#include <sot/core/matrix-homogeneous.hh>

#include <sot/core/flags.hh>

#include <sot-state-observation/tools/definitions.hh>


#include <state-observation/tools/miscellaneous-algorithms.hpp>


namespace sotStateObservation
{
        /**
           \brief
        */
        class PositionStateReconstructor :
            public dynamicgraph::Entity
        {
        public:
            /**
            \brief Constructor by name
            */
            PositionStateReconstructor(const std::string& inName);

            ~PositionStateReconstructor();

            /// Each entity should provide the name of the class it belongs to
            virtual const std::string& getClassName (void) const
            {
                return CLASS_NAME;
            }

            /// Header documentation of the python class
            virtual std::string getDocString () const
            {
                return
                    std::string("Recontruct the state vector of position orientation")
                              + "and their derivatives the orientation is in U-Theta"
                              + "form";
            }


            virtual void setCurrentValue (const dynamicgraph::Vector &value)
            {
                lastVector_= value;
            }

            virtual void setSampligPeriod (const double & dt)
            {
                dt_ = dt;
            }

            virtual double getSampligPeriod () const
            {
                return dt_;
            }

            virtual void setFiniteDifferencesInterval (const int & n)
            {
                derivationNumberOfSamples_=n;
            }

            virtual int getFiniteDifferencesInterval () const
            {
                return derivationNumberOfSamples_;
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
            stateObservation::Vector averageDistribution(const unsigned n);
            stateObservation::Vector gaussianDistribution(const unsigned n);

            /**
            Compute the control law
            */
            ::dynamicgraph::Vector& computeOutput
                        (::dynamicgraph::Vector & output, const int& inTime);

            /**
            \brief local to global frame position
            */
            dynamicgraph::SignalPtr < ::dynamicgraph::Vector, int> inputSIN;
            dynamicgraph::SignalPtr < dynamicgraph::sot::Flags, int> inputFormatSIN;
            dynamicgraph::SignalPtr < dynamicgraph::sot::Flags, int> outputFormatSIN;

            /**
            \brief Estimation of the attitude
            */
            dynamicgraph::SignalTimeDependent < ::dynamicgraph::Vector, int> outputSOUT;



            dynamicgraph::Vector lastVector_;

            std::deque < ::dynamicgraph::Vector> linearVelocities_;
            std::deque < ::dynamicgraph::Vector> angularVelocities_;
            std::deque < ::dynamicgraph::Vector> linearAccelerations_;
            std::deque < ::dynamicgraph::Vector> angularAccelerations_;

            double dt_;

            unsigned derivationNumberOfSamples_;

            stateObservation::Vector vec_;
            stateObservation::Vector distr_;

            std::deque<dynamicgraph::Vector>::iterator iterator;

        };

} // namespace sotStateObservation

#endif // DG_POSITION_STATE_RECONSTRUCTOR_HH
