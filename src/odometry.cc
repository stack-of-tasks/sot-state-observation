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

#include <sot-state-observation/odometry.hh>

namespace sotStateObservation
{
    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN ( Odometry, "Odometry" );

    Odometry::Odometry( const std::string & inName):
        Entity(inName),
        leftFootPositionSIN_ (NULL, "Odometry("+inName+")::input(HomoMatrix)::leftFootPosition"),
        rightFootPositionSIN_ (NULL, "Odometry("+inName+")::input(HomoMatrix)::rightFootPosition"),
        forceLeftFootSIN_ (NULL, "Odometry("+inName+")::input(vector)::force_lf"),
        forceRightFootSIN_ (NULL, "Odometry("+inName+")::input(vector)::force_rf"),
        nbSupportSOUT_ (NULL,"Odometry("+inName+")::output(unsigned)::nbSupport"),
        supportPos1SOUT_(NULL,"Odometry("+inName+")::output(vector)::supportPos1"),
        supportPos2SOUT_(NULL,"Odometry("+inName+")::output(vector)::supportPos2"),
        homoSupportPos1SOUT_(NULL, "Odometry("+inName+")::output(HomoMatrix)::homoSupportPos1"),
        homoSupportPos2SOUT_(NULL, "Odometry("+inName+")::output(HomoMatrix)::homoSupportPos2"),
        forceSupport1SOUT_ (NULL, "Odometry("+inName+")::output(vector)::forceSupport1"),
        forceSupport2SOUT_ (NULL, "Odometry("+inName+")::output(vector)::forceSupport2"),
        forceThreshold_ (.036 * 56.8*stateObservation::cst::gravityConstant), time_(0),
        candidatesForces_(contact::nbMax), candidatesPosition_(contact::nbMax), candidatesHomoPosition_(contact::nbMax)
    {
        std::string docstring;

        signalRegistration (leftFootPositionSIN_ << forceLeftFootSIN_);
        signalRegistration (rightFootPositionSIN_ << forceRightFootSIN_);

        signalRegistration (nbSupportSOUT_);
        signalRegistration (supportPos1SOUT_ << homoSupportPos1SOUT_ << forceSupport1SOUT_);
        signalRegistration (supportPos2SOUT_ << homoSupportPos2SOUT_ << forceSupport2SOUT_);

        nbSupportSOUT_.setFunction(boost::bind(&Odometry::getNbSupport, this, _1, _2));

        supportPos1SOUT_.setFunction(boost::bind(&Odometry::getSupportPos1, this, _1, _2));
        homoSupportPos1SOUT_.setFunction(boost::bind(&Odometry::getHomoSupportPos1, this, _1, _2));
        forceSupport1SOUT_.setFunction(boost::bind(&Odometry::getForceSupport1, this, _1, _2));

        supportPos2SOUT_.setFunction(boost::bind(&Odometry::getSupportPos2, this, _1, _2));
        homoSupportPos2SOUT_.setFunction(boost::bind(&Odometry::getHomoSupportPos2, this, _1, _2));
        forceSupport2SOUT_.setFunction(boost::bind(&Odometry::getForceSupport2, this, _1, _2));

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

        computeStackOfContacts(0);
   }

    Odometry::~Odometry()
    {
    }

    unsigned int& Odometry::getNbSupport(unsigned int& nbSupport, const int& time)
    {
        if(time!=time_) computeStackOfContacts(time);
        nbSupport = stackOfContacts_.size();
        return nbSupport;
    }

    Vector& Odometry::getSupportPos1(Vector& supportPos1, const int& time)
    {
        if(time!=time_) computeStackOfContacts(time);
        if (stackOfContacts_.size()>=1) {
            iterator = stackOfContacts_.begin();
            supportPos1=convertVector<dynamicgraph::Vector>(candidatesPosition_[*iterator]);
        } else {
            supportPos1.setZero();
        }
        return supportPos1;
    }

    MatrixHomogeneous& Odometry::getHomoSupportPos1(MatrixHomogeneous& homoSupportPos1, const int& time)
    {
        if(time!=time_) computeStackOfContacts(time);
        if (stackOfContacts_.size()>=1) {
            iterator = stackOfContacts_.begin();
            homoSupportPos1=candidatesHomoPosition_[*iterator];
        } else {
            homoSupportPos1.setIdentity();
        }
        return homoSupportPos1;
    }

    Vector& Odometry::getForceSupport1(Vector& forceSupport1, const int& time)
    {
        if(time!=time_) computeStackOfContacts(time);
        if (stackOfContacts_.size()>=1) {
            iterator = stackOfContacts_.begin();
            forceSupport1=convertVector<dynamicgraph::Vector>(candidatesForces_[*iterator]);
        } else {
            forceSupport1.setZero();
        }
        return forceSupport1;
    }

    Vector& Odometry::getSupportPos2(Vector& supportPos2, const int& time)
    {
        if(time!=time_) computeStackOfContacts(time);
        if (stackOfContacts_.size()>=2) {
            iterator = stackOfContacts_.begin();
            for(int i=1; i<2; ++i) ++iterator ;
            supportPos2=convertVector<dynamicgraph::Vector>(candidatesPosition_[*iterator]);
        } else {
            supportPos2.setZero();
        }
        return supportPos2;
    }

    MatrixHomogeneous& Odometry::getHomoSupportPos2(MatrixHomogeneous& homoSupportPos2, const int& time)
    {
        if(time!=time_) computeStackOfContacts(time);
        if (stackOfContacts_.size()>=2) {
            iterator = stackOfContacts_.begin();
            for(int i=1; i<2; ++i) ++iterator ;
            homoSupportPos2=candidatesHomoPosition_[*iterator];
        } else {
            homoSupportPos2.setIdentity();
        }
        return homoSupportPos2;
    }

    Vector& Odometry::getForceSupport2(Vector& forceSupport2, const int& time)
    {
        if(time!=time_) computeStackOfContacts(time);
        if (stackOfContacts_.size()>=2) {
            iterator = stackOfContacts_.begin();
            for(int i=1; i<2; ++i) ++iterator ;
            forceSupport2=convertVector<dynamicgraph::Vector>(candidatesForces_[*iterator]);
        } else {
            forceSupport2.setZero();
        }
        return forceSupport2;
    }


    void Odometry::computeStackOfContacts(const int& time)
    {
        candidatesHomoPosition_[contact::rf] = rightFootPositionSIN_.access (time);
        candidatesForces_[contact::rf] = convertVector<stateObservation::Vector>(forceRightFootSIN_.access (time));

        candidatesHomoPosition_[contact::lf] = leftFootPositionSIN_.access (time);
        candidatesForces_[contact::lf] = convertVector<stateObservation::Vector>(forceLeftFootSIN_.access (time));

        Vector pos(3);
        MatrixRotation rot;
        VectorUTheta uth;
        double fz;
        bool found;

        for (int i=0; i<contact::nbMax;++i){
            candidatesHomoPosition_[i].extract(pos);
            candidatesHomoPosition_[i].extract(rot);
            uth.fromMatrix(rot);
            candidatesPosition_[i] << convertVector<stateObservation::Vector>(pos),
                                      convertVector<stateObservation::Vector>(uth);

            fz =  candidatesHomoPosition_[i](2,0) * candidatesForces_[i](0) +
                  candidatesHomoPosition_[i](2,1) * candidatesForces_[i](1) +
                  candidatesHomoPosition_[i](2,2) * candidatesForces_[i](2);

            found = (std::find(stackOfContacts_.begin(), stackOfContacts_.end(), i) != stackOfContacts_.end());

            if(fz>forceThreshold_) {
                if (!found) stackOfContacts_.push_back(i);
            } else {
                if(found) stackOfContacts_.remove(i);
            }
        }

        time_=time;
    }
}

