//
// Copyright (c) 2015,
// Alexis Mifsud
//
// CNRS
//
// This file is part of sot-dynamic.
// sot-dynamic is free software: you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of
// the License, or (at your option) any later version.
// sot-dynamic is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.  You should
// have received a copy of the GNU Lesser General Public License along
// with sot-dynamic.  If not, see <http://www.gnu.org/licenses/>.
//

#include <sstream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-bind.h>

#include <math.h>

#include <state-observation/tools/definitions.hpp>
#include <state-observation/tools/miscellaneous-algorithms.hpp>

#include <sot-state-observation/estimator-interface.hh>

namespace sotStateObservation
{
    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN ( EstimatorInterface, "EstimatorInterface" );

    EstimatorInterface::EstimatorInterface( const std::string & inName):
        Entity(inName),
        inputSOUT_ (NULL, "EstimatorInterface("+inName+")::output(vector)::input"),
        measurementSOUT_ (NULL, "EstimatorInterface("+inName+")::output(vector)::measurement"),
        contactsNbrSOUT_ (NULL, "EstimatorInterface("+inName+")::output(unsigned)::contactsNbr"),
        positionLeftFootSIN_ (NULL, "Odometry("+inName+")::input(HomoMatrix)::position_lf"),
        forceLeftFootSIN_ (NULL, "Odometry("+inName+")::input(vector)::force_lf"),
        positionRightFootSIN_ (NULL, "Odometry("+inName+")::input(HomoMatrix)::position_rf"),
        forceRightFootSIN_ (NULL, "Odometry("+inName+")::input(vector)::force_rf"),
        positionLeftHandSIN_ (NULL, "Odometry("+inName+")::input(HomoMatrix)::position_lh"),
        forceLeftHandSIN_ (NULL, "Odometry("+inName+")::input(vector)::force_lh"),
        positionRightHandSIN_ (NULL, "Odometry("+inName+")::input(HomoMatrix)::position_rh"),
        forceRightHandSIN_ (NULL, "Odometry("+inName+")::input(vector)::force_rh"),
        time_(0),
        inputForces_(contact::nbMax),
        inputPosition_(contact::nbMax),
        inputHomoPosition_(contact::nbMax)
    {

        /// Signals
        // Input
        MatrixHomogeneous pos;
        stateObservation::Vector6 force;

        signalRegistration (positionLeftFootSIN_ << forceLeftFootSIN_);
        positionLeftFootSIN_.setConstant(pos);
        positionLeftFootSIN_.setTime (time_);
        forceLeftFootSIN_.setConstant(convertVector<dynamicgraph::Vector>(force));
        forceLeftFootSIN_.setTime (time_);

        signalRegistration (positionRightFootSIN_ << forceRightFootSIN_);
        positionRightFootSIN_.setConstant(pos);
        positionRightFootSIN_.setTime (time_);
        forceRightFootSIN_.setConstant(convertVector<dynamicgraph::Vector>(force));
        forceRightFootSIN_.setTime (time_);

        signalRegistration (positionLeftHandSIN_ << forceLeftHandSIN_);
        positionLeftHandSIN_.setConstant(pos);
        positionLeftHandSIN_.setTime (time_);
        forceLeftHandSIN_.setConstant(convertVector<dynamicgraph::Vector>(force));
        forceLeftHandSIN_.setTime (time_);

        signalRegistration (positionRightHandSIN_ << forceRightHandSIN_);
        positionRightHandSIN_.setConstant(pos);
        positionRightHandSIN_.setTime (time_);
        forceRightHandSIN_.setConstant(convertVector<dynamicgraph::Vector>(force));
        forceRightHandSIN_.setTime (time_);

        // Output
        signalRegistration (inputSOUT_);
        inputSOUT_.setFunction(boost::bind(&EstimatorInterface::getInput, this, _1, _2));

        signalRegistration (measurementSOUT_);
        measurementSOUT_.setFunction(boost::bind(&EstimatorInterface::getMeasurement, this, _1, _2));

        signalRegistration (contactsNbrSOUT_);
        contactsNbrSOUT_.setFunction(boost::bind(&EstimatorInterface::getContactsNbr, this, _1, _2));

        /// Parameters

        // ForceThresholds
        forceThresholds_.resize(contact::nbMax);
        forceThresholds_.setOnes();
        forceThresholds_*=0.02 * 56.8*stateObservation::cst::gravityConstant; // default value
        forceThresholds_[contact::lh]*=0.1;
        forceThresholds_[contact::rh]*=0.1;

        // Modeled
        modeled_.resize(contact::nbMax);
        forceThresholds_.setZero(); // default value
        forceThresholds_[contact::lf]=1;
        forceThresholds_[contact::rf]=1;
    }

    EstimatorInterface::~EstimatorInterface()
    {
    }

    void EstimatorInterface::computeStackOfContacts(const int& time)
    {

        inputForces_[contact::rf] = convertVector<stateObservation::Vector>(forceRightFootSIN_.access (time));
        inputHomoPosition_[contact::rf] = convertMatrix<stateObservation::Matrix4>(Matrix(positionRightFootSIN_.access (time)));

        inputForces_[contact::lf] = convertVector<stateObservation::Vector>(forceLeftFootSIN_.access (time));
        inputHomoPosition_[contact::lf] = convertMatrix<stateObservation::Matrix4>(Matrix(positionLeftFootSIN_.access (time)));

        bool found;

        for (int i=0; i<contact::nbMax;++i){
            inputPosition_[i]=kine::homogeneousMatrixToVector6(inputHomoPosition_[i]);

            found = (std::find(stackOfContacts_.begin(), stackOfContacts_.end(), i) != stackOfContacts_.end());

            if(inputForces_[i].norm()>forceThresholds_[i]) {
                if (!found) stackOfContacts_.push_back(i);
            } else {
                if(found) stackOfContacts_.remove(i);
            }
        }
    }

    Vector& EstimatorInterface::getInput(Vector& input, const int& time)
    {

        return input;
    }

    Vector& EstimatorInterface::getMeasurement(Vector& measurement, const int& time)
    {

        return measurement;
    }

    unsigned& EstimatorInterface::getContactsNbr(unsigned& contactsNbr, const int& time)
    {
        if(time!=time_) computeStackOfContacts(time);
        contactsNbr = stackOfContacts_.size();
        return contactsNbr;
    }

}

