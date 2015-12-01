#include <sstream>

#include <Eigen/Core>
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

#include <Eigen/Geometry>

#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-bind.h>

#include <math.h>

#include <state-observation/tools/definitions.hpp>
#include <state-observation/tools/miscellaneous-algorithms.hpp>

#include <sot-state-observation/stack-of-contacts.hh>


namespace sotStateObservation
{
    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN ( StackOfContacts, "StackOfContacts" );

    StackOfContacts::StackOfContacts( const std::string & inName):
        Entity(inName),
        leftFootPositionSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(HomoMatrix)::leftFootPosition"),
        rightFootPositionSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(HomoMatrix)::rightFootPosition"),
        forceLeftFootSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::force_lf"),
        forceRightFootSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::force_rf"),
        nbSupportSOUT_ ("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(unsigned)::nbSupport"),
        supportPos1SOUT_("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::supportPos1"),
        supportPos2SOUT_("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::supportPos2"),
        homoSupportPos1SOUT_(NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(HomoMatrix)::homoSupportPos1"),
        homoSupportPos2SOUT_(NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(HomoMatrix)::homoSupportPos2"),
        forceSupport1SOUT_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::forceSupport1"),
        forceSupport2SOUT_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::forceSupport2")
    {
        std::string docstring;

        signalRegistration (leftFootPositionSIN_ << forceLeftFootSIN_);
        signalRegistration (rightFootPositionSIN_ << forceRightFootSIN_);

        signalRegistration (nbSupportSOUT_);
        signalRegistration (supportPos1SOUT_ << homoSupportPos1SOUT_ << forceSupport1SOUT_);
        signalRegistration (supportPos2SOUT_ << homoSupportPos2SOUT_ << forceSupport2SOUT_);

        Vector rfconf(6);
        rfconf.setZero();
        Vector lfconf(6);
        lfconf.setZero();
        rfconf(0) = 0.009490463094;
        rfconf(1) = -0.095000000000;
        lfconf(0) = 0.009490463094;
        lfconf(1) = 0.095000000000;

        supportPos1SOUT_.setConstant (lfconf);
        supportPos1SOUT_.setTime (0);

        supportPos2SOUT_.setConstant (rfconf);
        supportPos2SOUT_.setTime (0);

        nbSupportSOUT_.setConstant (2);
        nbSupportSOUT_.setTime (0);

        stateObservation::Matrix leftFootPos;
        leftFootPos.resize(4,4);
        leftFootPos <<  1,1.94301e-07,2.363e-10,0.00949046,
                        -1.94301e-07,1,-2.70566e-12,0.095,
                        -2.363e-10,2.70562e-12,1,3.03755e-06,
                        0,0,0,1;
        leftFootPositionSIN_.setConstant(convertMatrix<dynamicgraph::Matrix>(leftFootPos));

        stateObservation::Matrix rightFootPos;
        rightFootPos.resize(4,4);
        rightFootPos <<  1,-9.18094e-18,-1.52169e-16,0.009496046,
                        9.184e-18,1,-1.10345e-16,-0.095,
                        1.68756e-16,1.10345e-16,1,2.55006e-07,
                        0,0,0,1;
        rightFootPositionSIN_.setConstant(convertMatrix<dynamicgraph::Matrix>(rightFootPos));

        stateObservation::Matrix homoSupportPos2;
        homoSupportPos2.resize(4,4);
        homoSupportPos2 <<  1,1.94301e-07,2.363e-10,0.00949046,
                        -1.94301e-07,1,-2.70566e-12,0.095,
                        -2.363e-10,2.70562e-12,1,3.03755e-06,
                        0,0,0,1;
        homoSupportPos2SOUT_.setConstant(convertMatrix<dynamicgraph::Matrix>(homoSupportPos2));

        stateObservation::Matrix homoSupportPos1;
        homoSupportPos1.resize(4,4);
        homoSupportPos1 <<  1,-9.18094e-18,-1.52169e-16,0.009496046,
                            9.184e-18,1,-1.10345e-16,-0.095,
                            1.68756e-16,1.10345e-16,1,2.55006e-07,
                            0,0,0,1;
        homoSupportPos1SOUT_.setConstant(convertMatrix<dynamicgraph::Matrix>(homoSupportPos1));

        stateObservation::Vector forceRightFoot;
        forceRightFoot.resize(6);
        forceRightFoot <<   45.1262,
                            -21.367,
                            361.344,
                            1.12135,
                            -14.5562,
                            1.89125;
        forceRightFootSIN_.setConstant(convertVector<dynamicgraph::Vector>(forceRightFoot));

        stateObservation::Vector forceLeftFoot;
        forceLeftFoot.resize(6);
        forceLeftFoot <<    44.6005,
                            21.7871,
                            352.85,
                            -1.00715,
                            -14.5158,
                            -1.72017;
        forceLeftFootSIN_.setConstant(convertVector<dynamicgraph::Vector>(forceLeftFoot));

        stateObservation::Vector forceSupport2;
        forceSupport2.resize(6);
        forceSupport2 <<   45.1262,
                            -21.367,
                            361.344,
                            1.12135,
                            -14.5562,
                            1.89125;
        forceSupport2SOUT_.setConstant(convertVector<dynamicgraph::Vector>(forceSupport2));

        stateObservation::Vector forceSupport1;
        forceSupport1.resize(6);
        forceSupport1 <<    44.6005,
                            21.7871,
                            352.85,
                            -1.00715,
                            -14.5158,
                            -1.72017;
        forceSupport1SOUT_.setConstant(convertVector<dynamicgraph::Vector>(forceSupport1));

    }

    StackOfContacts::~StackOfContacts()
    {
    }

    unsigned int& StackOfContacts::getNbSupport(unsigned int& nbSupport, const int& time)
    {
        computeStack(time);
        nbSupport=nbSupportSOUT_.access (time);
        return nbSupport;
    }

    dynamicgraph::Vector& StackOfContacts::getSupportPos1(dynamicgraph::Vector& supportPos1, const int& time)
    {
        computeStack(time);
        supportPos1=supportPos1SOUT_.access (time);
        return supportPos1;
    }

    MatrixHomogeneous& StackOfContacts::getHomoSupportPos1(MatrixHomogeneous& homoSupportPos1, const int& time)
    {
        computeStack(time);
        homoSupportPos1=homoSupportPos1SOUT_.access (time);
        return homoSupportPos1;
    }

    dynamicgraph::Vector& StackOfContacts::getForceSupport1(dynamicgraph::Vector& forceSupport1, const int& time)
    {
        computeStack(time);
        forceSupport1=forceSupport1SOUT_.access (time);
        return forceSupport1;
    }

    dynamicgraph::Vector& StackOfContacts::getSupportPos2(dynamicgraph::Vector& supportPos2, const int& time)
    {
        computeStack(time);
        supportPos2=supportPos2SOUT_.access (time);
        return supportPos2;
    }

    MatrixHomogeneous& StackOfContacts::getHomoSupportPos2(MatrixHomogeneous& homoSupportPos2, const int& time)
    {
        computeStack(time);
        homoSupportPos2=homoSupportPos2SOUT_.access (time);
        return homoSupportPos2;
    }

    dynamicgraph::Vector& StackOfContacts::getForceSupport2(dynamicgraph::Vector& forceSupport2, const int& time)
    {
        computeStack(time);
        forceSupport2=forceSupport2SOUT_.access (time);
        return forceSupport2;
    }


    void StackOfContacts::computeStack(const int& time)  //const MatrixHomogeneous& leftFootPosition, const MatrixHomogeneous& rightFootPosition, const Vector& forceLf, const Vector& forceRf, const int& time)
    {

        /// Forces signals
        const MatrixHomogeneous& leftFootPosition = leftFootPositionSIN_.access (time);
        const MatrixHomogeneous& rightFootPosition = rightFootPositionSIN_.access (time);
        const Vector& forceLf = forceLeftFootSIN_.access (time);
        const Vector& forceRf = forceRightFootSIN_.access (time);

        //feet position
        Vector rfpos(3);
        Vector lfpos(3);

        MatrixRotation rfrot;
        MatrixRotation lfrot;

        VectorUTheta rfuth;
        VectorUTheta lfuth;

        rightFootPosition.extract(rfpos);
        rightFootPosition.extract(rfrot);
        rfuth.fromMatrix(rfrot);

        leftFootPosition.extract(lfpos);
        leftFootPosition.extract(lfrot);
        lfuth.fromMatrix(lfrot);

        Vector rfconf(6);
        Vector lfconf(6);

        for (size_t i=0; i<3; ++i)
        {
          rfconf(i)   = rfpos(i);
          rfconf(i+3) = rfuth(i);
          lfconf(i)   = lfpos(i);
          lfconf(i+3) = lfuth(i);
        }

        // Express vertical component of force in global basis
        double flz = leftFootPosition (2,0) * forceLf (0) +
                     leftFootPosition(2,1) * forceLf (1) +
                     leftFootPosition (2,2) * forceLf (2);
        double frz = rightFootPosition (2,0) * forceRf (0) +
                     rightFootPosition(2,1) * forceRf (1) +
                     rightFootPosition (2,2) * forceRf (2);

        //compute the number of supports
        unsigned int nbSupport = 0;
        if (flz >= forceThreshold_)
        {
          supportPos1SOUT_.setConstant (lfconf);
          supportPos1SOUT_.setTime (time);
          homoSupportPos1SOUT_.setConstant (leftFootPosition);
          homoSupportPos1SOUT_.setTime (time);
          forceSupport1SOUT_.setConstant (forceLeftFootSIN_);
          forceSupport1SOUT_.setTime (time);
          nbSupport++;
        }

        if (frz >= forceThreshold_)
        {
          if (nbSupport==0)
          {
            supportPos1SOUT_.setConstant (rfconf);
            supportPos1SOUT_.setTime (time);
            homoSupportPos1SOUT_.setConstant (rightFootPosition);
            homoSupportPos1SOUT_.setTime (time);
            forceSupport1SOUT_.setConstant (forceRightFootSIN_);
            forceSupport1SOUT_.setTime (time);
          }
          else
          {
            supportPos2SOUT_.setConstant (rfconf);
            homoSupportPos2SOUT_.setConstant (rightFootPosition);
            forceSupport2SOUT_.setConstant (forceRightFootSIN_);
          }
          nbSupport++;
        }

        nbSupportSOUT_.setConstant (nbSupport);
        nbSupportSOUT_.setTime (time);
    }

}

