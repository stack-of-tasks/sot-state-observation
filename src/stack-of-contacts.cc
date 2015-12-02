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

#include <iostream>

namespace sotStateObservation
{
    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN ( StackOfContacts, "StackOfContacts" );

    StackOfContacts::StackOfContacts( const std::string & inName):
        Entity(inName),
        leftFootPositionSIN_ (NULL, "StackOfContacts("+inName+")::input(HomoMatrix)::leftFootPosition"),
        rightFootPositionSIN_ (NULL, "StackOfContacts("+inName+")::input(HomoMatrix)::rightFootPosition"),
        forceLeftFootSIN_ (NULL, "StackOfContacts("+inName+")::input(vector)::force_lf"),
        forceRightFootSIN_ (NULL, "StackOfContacts("+inName+")::input(vector)::force_rf"),
        nbSupportSOUT_ (NULL,"StackOfContacts("+inName+")::output(unsigned)::nbSupport"),
        supportPos1SOUT_(NULL,"StackOfContacts("+inName+")::output(vector)::supportPos1"),
        supportPos2SOUT_(NULL,"StackOfContacts("+inName+")::output(vector)::supportPos2"),
        homoSupportPos1SOUT_(NULL, "StackOfContacts("+inName+")::output(HomoMatrix)::homoSupportPos1"),
        homoSupportPos2SOUT_(NULL, "StackOfContacts("+inName+")::output(HomoMatrix)::homoSupportPos2"),
        forceSupport1SOUT_ (NULL, "StackOfContacts("+inName+")::output(vector)::forceSupport1"),
        forceSupport2SOUT_ (NULL, "StackOfContacts("+inName+")::output(vector)::forceSupport2"),
        forceThreshold_ (.036 * 56.8*stateObservation::cst::gravityConstant), time_(-1),
        nbSupport_(2), supportPos1_(6), supportPos2_(6), forceSupport1_(6), forceSupport2_(6)
    {
        std::string docstring;

        signalRegistration (leftFootPositionSIN_ << forceLeftFootSIN_);
        signalRegistration (rightFootPositionSIN_ << forceRightFootSIN_);

        signalRegistration (nbSupportSOUT_);
        signalRegistration (supportPos1SOUT_ << homoSupportPos1SOUT_ << forceSupport1SOUT_);
        signalRegistration (supportPos2SOUT_ << homoSupportPos2SOUT_ << forceSupport2SOUT_);

        nbSupportSOUT_.setFunction(boost::bind(&StackOfContacts::getNbSupport, this, _1, _2));

        supportPos1SOUT_.setFunction(boost::bind(&StackOfContacts::getSupportPos1, this, _1, _2));
        homoSupportPos1SOUT_.setFunction(boost::bind(&StackOfContacts::getHomoSupportPos1, this, _1, _2));
        forceSupport1SOUT_.setFunction(boost::bind(&StackOfContacts::getForceSupport1, this, _1, _2));

        supportPos2SOUT_.setFunction(boost::bind(&StackOfContacts::getSupportPos2, this, _1, _2));
        homoSupportPos2SOUT_.setFunction(boost::bind(&StackOfContacts::getHomoSupportPos2, this, _1, _2));
        forceSupport2SOUT_.setFunction(boost::bind(&StackOfContacts::getForceSupport2, this, _1, _2));

        stateObservation::Matrix leftFootPos;
        leftFootPos.resize(4,4);
        leftFootPos <<  1,1.94301e-07,2.363e-10,0.00949046,
                        -1.94301e-07,1,-2.70566e-12,0.095,
                        -2.363e-10,2.70562e-12,1,3.03755e-06,
                        0,0,0,1;
        leftFootPositionSIN_.setConstant(convertMatrix<dynamicgraph::Matrix>(leftFootPos));
        leftFootPositionSIN_.setTime (time_);

        stateObservation::Matrix rightFootPos;
        rightFootPos.resize(4,4);
        rightFootPos <<  1,-9.18094e-18,-1.52169e-16,0.009496046,
                        9.184e-18,1,-1.10345e-16,-0.095,
                        1.68756e-16,1.10345e-16,1,2.55006e-07,
                        0,0,0,1;
        rightFootPositionSIN_.setConstant(convertMatrix<dynamicgraph::Matrix>(rightFootPos));
        rightFootPositionSIN_.setTime (time_);

        stateObservation::Vector forceRightFoot;
        forceRightFoot.resize(6);
        forceRightFoot <<   45.1262,
                            -21.367,
                            361.344,
                            1.12135,
                            -14.5562,
                            1.89125;
        forceRightFootSIN_.setConstant(convertVector<dynamicgraph::Vector>(forceRightFoot));
        forceRightFootSIN_.setTime (time_);

        stateObservation::Vector forceLeftFoot;
        forceLeftFoot.resize(6);
        forceLeftFoot <<    44.6005,
                            21.7871,
                            352.85,
                            -1.00715,
                            -14.5158,
                            -1.72017;
        forceLeftFootSIN_.setConstant(convertVector<dynamicgraph::Vector>(forceLeftFoot));
        forceLeftFootSIN_.setTime (time_);

        supportPos1_.setZero();
        supportPos2_.setZero();
        forceSupport1_.setZero();
        forceSupport2_.setZero();
        homoSupportPos1_.setIdentity();
        homoSupportPos2_.setIdentity();
   }

    StackOfContacts::~StackOfContacts()
    {
    }

    unsigned int& StackOfContacts::getNbSupport(unsigned int& nbSupport, const int& time)
    {
        if(time!=time_) computeStack(time);
        nbSupport=nbSupport_;
        return nbSupport;
    }

    Vector& StackOfContacts::getSupportPos1(Vector& supportPos1, const int& time)
    {
        if(time!=time_) computeStack(time);
        supportPos1=supportPos1_;
        return supportPos1;
    }

    MatrixHomogeneous& StackOfContacts::getHomoSupportPos1(MatrixHomogeneous& homoSupportPos1, const int& time)
    {
        if(time!=time_) computeStack(time);
        homoSupportPos1=homoSupportPos1_;
        return homoSupportPos1;
    }

    Vector& StackOfContacts::getForceSupport1(Vector& forceSupport1, const int& time)
    {
        if(time!=time_) computeStack(time);
        forceSupport1=forceSupport1_;
        return forceSupport1;
    }

    Vector& StackOfContacts::getSupportPos2(Vector& supportPos2, const int& time)
    {
        if(time!=time_) computeStack(time);
        supportPos2=supportPos2_;
        return supportPos2;
    }

    MatrixHomogeneous& StackOfContacts::getHomoSupportPos2(MatrixHomogeneous& homoSupportPos2, const int& time)
    {
        if(time!=time_) computeStack(time);
        homoSupportPos2=homoSupportPos2_;
        return homoSupportPos2;
    }

    Vector& StackOfContacts::getForceSupport2(Vector& forceSupport2, const int& time)
    {
        if(time!=time_) computeStack(time);
        forceSupport2=forceSupport2_;
        return forceSupport2;
    }


    void StackOfContacts::computeStack(const int& time)
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

        double flz = leftFootPosition (2,0) * forceLf (0) +
                     leftFootPosition(2,1) * forceLf (1) +
                     leftFootPosition (2,2) * forceLf (2);
        double frz = rightFootPosition (2,0) * forceRf (0) +
                     rightFootPosition(2,1) * forceRf (1) +
                     rightFootPosition (2,2) * forceRf (2);

        nbSupport_ = 0;

        if (flz >= forceThreshold_)
        {
          supportPos1_=lfconf;
          homoSupportPos1_=leftFootPosition;
          forceSupport1_=forceLf;
          nbSupport_++;
        }

        if (frz >= forceThreshold_)
        {
          if (nbSupport_==0)
          {
              supportPos1_=rfconf;
              homoSupportPos1_=rightFootPosition;
              forceSupport1_=forceRf;
          }
          else
          {
              supportPos2_=rfconf;
              homoSupportPos2_=rightFootPosition;
              forceSupport2_=forceRf;
          }
          nbSupport_++;
        }

        time_=time;
    }

}

