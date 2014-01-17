/*
 *  Copyright 2013 CNRS
 *
 *  Mehdi Benallegue
 */

#ifndef MOVING_FRAME_TRANSFORMATION_HH
#define MOVING_FRAME_TRANSFORMATION_HH

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/linear-algebra.h>
#include <sot/core/matrix-homogeneous.hh>

#include <sot-state-observation/tools/definitions.hh>


#include <state-observation/tools/miscellaneous-algorithms.hpp>


namespace sotStateObservation
{
        /**
           \brief
        */
        class MovingFrameTransformation :
            public dynamicgraph::Entity
        {
        public:
            /**
            \brief Constructor by name
            */
            MovingFrameTransformation(const std::string& inName);

            ~MovingFrameTransformation();

            /// Each entity should provide the name of the class it belongs to
            virtual const std::string& getClassName (void) const
            {
                return CLASS_NAME;
            }

            /// Header documentation of the python class
            virtual std::string getDocString () const
            {
                return
                    std::string("Transform cordinates (lM0) and velocities (lV0)")
                              + "in a moving local frame into global frame."
                              + "The local frame is at cordinate gMl and velocity gVl";
            }

            virtual void setgVlFactor (const double & k)
            {
                velocityFactor_ = k;
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
            ::dynamicgraph::sot::MatrixHomogeneous& computegM0
                        (::dynamicgraph::sot::MatrixHomogeneous & computegM0, const int& inTime);

            ::dynamicgraph::Vector& computegV0
                        (::dynamicgraph::Vector & velocity, const int& intTime);

            /**
            \brief local to global frame position
            */
            dynamicgraph::SignalPtr < ::dynamicgraph::sot::MatrixHomogeneous, int> gMlSIN;

            /**
            \brief local to global velocity vector
            */
            dynamicgraph::SignalPtr < ::dynamicgraph::Vector, int> gVlSIN;

            /**
            \brief Homogeneous matrix in the local frame
            */
            dynamicgraph::SignalPtr < ::dynamicgraph::sot::MatrixHomogeneous, int> lM0SIN;

            /**
            \brief velocity vector in the local frame
            */
            dynamicgraph::SignalPtr < ::dynamicgraph::Vector, int> lV0SIN;


            /**
            \brief Estimation of the attitude
            */
            dynamicgraph::SignalTimeDependent < ::dynamicgraph::sot::MatrixHomogeneous, int> gM0SOUT;

            /**
            \brief Estimation of the attitude
            */
            dynamicgraph::SignalTimeDependent < ::dynamicgraph::Vector, int> gV0SOUT;

            double velocityFactor_;
        };

} // namespace sotStateObservation

#endif //MOVING_FRAME_TRANSFORMATION_HH

