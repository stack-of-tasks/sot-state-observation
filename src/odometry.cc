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

#include <sot-state-observation/odometry.hh>

namespace sotStateObservation
{
    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN ( Odometry, "Odometry" );

    Odometry::Odometry( const std::string & inName):
        Entity(inName),
        leftFootPositionSIN_ (NULL, "Odometry("+inName+")::input(HomoMatrix)::leftFootPosition"),
        rightFootPositionSIN_ (NULL, "Odometry("+inName+")::input(HomoMatrix)::rightFootPosition"),
        leftFootPositionRefSIN_ (NULL, "Odometry("+inName+")::input(HomoMatrix)::leftFootPositionRef"),
        rightFootPositionRefSIN_ (NULL, "Odometry("+inName+")::input(HomoMatrix)::rightFootPositionRef"),
        forceLeftFootSIN_ (NULL, "Odometry("+inName+")::input(vector)::force_lf"),
        forceRightFootSIN_ (NULL, "Odometry("+inName+")::input(vector)::force_rf"),
        robotStateInSIN_ (NULL, "Odometry("+inName+")::input(vector)::robotStateIn"),
        robotStateOutSOUT_ (NULL, "Odometry("+inName+")::output(vector)::robotStateOut"),
        nbSupportSOUT_ (NULL,"Odometry("+inName+")::output(unsigned)::nbSupport"),
        supportPos1SOUT_(NULL,"Odometry("+inName+")::output(vector)::supportPos1"),
        supportPos2SOUT_(NULL,"Odometry("+inName+")::output(vector)::supportPos2"),
        homoSupportPos1SOUT_(NULL, "Odometry("+inName+")::output(HomoMatrix)::homoSupportPos1"),
        homoSupportPos2SOUT_(NULL, "Odometry("+inName+")::output(HomoMatrix)::homoSupportPos2"),
        forceSupport1SOUT_ (NULL, "Odometry("+inName+")::output(vector)::forceSupport1"),
        forceSupport2SOUT_ (NULL, "Odometry("+inName+")::output(vector)::forceSupport2"),
        pivotPositionSOUT_ (NULL, "Odometry("+inName+")::output(Vector)::pivotPosition"),
        forceThreshold_ (.02 * 56.8*stateObservation::cst::gravityConstant), time_(0), pos_(3), posUTheta_(6), // .036 * 56.8*stateObservation::cst::gravityConstant
        candidatesForces_(contact::nbMax),
        candidatesPosition_(contact::nbMax), candidatesHomoPosition_(contact::nbMax),
        candidatesPositionRef_(contact::nbMax), candidatesHomoPositionRef_(contact::nbMax),
        odometryRelativePosition_(contact::nbMax)
    {

        signalRegistration (leftFootPositionSIN_ << forceLeftFootSIN_);
        signalRegistration (rightFootPositionSIN_ << forceRightFootSIN_);

        signalRegistration (leftFootPositionRefSIN_);
        signalRegistration (rightFootPositionRefSIN_);

        signalRegistration (robotStateInSIN_);
        signalRegistration (robotStateOutSOUT_);

        signalRegistration (nbSupportSOUT_);
        signalRegistration (supportPos1SOUT_ << homoSupportPos1SOUT_ << forceSupport1SOUT_);
        signalRegistration (supportPos2SOUT_ << homoSupportPos2SOUT_ << forceSupport2SOUT_);

        signalRegistration (pivotPositionSOUT_);

        std::string docstring;

        docstring  =
                "\n"
                "    Set left foot position input \n"
                "\n";

        addCommand(std::string("setLeftFootPosition"),
                   ::dynamicgraph::command::makeCommandVoid1(*this, & Odometry::setLeftFootPosition, docstring));

        docstring  =
                "\n"
                "    Set right foot position input \n"
                "\n";

        addCommand(std::string("setRightFootPosition"),
                   ::dynamicgraph::command::makeCommandVoid1(*this, & Odometry::setRightFootPosition, docstring));

        nbSupportSOUT_.setFunction(boost::bind(&Odometry::getNbSupport, this, _1, _2));

        supportPos1SOUT_.setFunction(boost::bind(&Odometry::getSupportPos1, this, _1, _2));
        homoSupportPos1SOUT_.setFunction(boost::bind(&Odometry::getHomoSupportPos1, this, _1, _2));
        forceSupport1SOUT_.setFunction(boost::bind(&Odometry::getForceSupport1, this, _1, _2));

        supportPos2SOUT_.setFunction(boost::bind(&Odometry::getSupportPos2, this, _1, _2));
        homoSupportPos2SOUT_.setFunction(boost::bind(&Odometry::getHomoSupportPos2, this, _1, _2));
        forceSupport2SOUT_.setFunction(boost::bind(&Odometry::getForceSupport2, this, _1, _2));

        robotStateOutSOUT_.setFunction(boost::bind(&Odometry::getRobotStateOut, this, _1, _2));
        pivotPositionSOUT_.setFunction(boost::bind(&Odometry::getPivotPositionOut, this, _1, _2));

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

        leftFootPositionRefSIN_.setConstant(convertMatrix<dynamicgraph::Matrix>(leftFootPos));
        leftFootPositionRefSIN_.setTime (time_);
        rightFootPositionRefSIN_.setConstant(convertMatrix<dynamicgraph::Matrix>(rightFootPos));
        rightFootPositionRefSIN_.setTime (time_);

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

        pos_.setZero();
        rot_.setIdentity();
        uth_.setZero();
        rpy_.setZero();
        posUTheta_.setZero();
        homo_.setIdentity();

        Vector robotState;
        robotState.resize(36);
        robotState.setZero();
        robotStateInSIN_.setConstant(robotState);
        robotStateInSIN_.setTime (time_);

        for (int i=0;i<contact::nbMax;++i) odometryRelativePosition_[i].setIdentity();
        pivotPosition_=convertMatrix<MatrixHomogeneous>(leftFootPos);
        computeOdometry(time_);
   }

    Odometry::~Odometry()
    {
    }

    unsigned int& Odometry::getNbSupport(unsigned int& nbSupport, const int& time)
    {
        if(time!=time_) computeOdometry(time);
        nbSupport = stackOfContacts_.size();
        return nbSupport;
    }

    Vector& Odometry::getSupportPos1(Vector& supportPos1, const int& time)
    {
        if(time!=time_) computeOdometry(time);
        if (stackOfContacts_.size()>=1) {
            iterator = stackOfContacts_.begin();
            supportPos1=convertVector<dynamicgraph::Vector>(posUThetaFromMatrixHomogeneous(odometryRelativePosition_[*iterator]*pivotPosition_));
            supportPos1.elementAt(2)=candidatesPositionRef_[*iterator][2]; // Position along the Z axis
            supportPos1.elementAt(3)=candidatesPositionRef_[*iterator][3]; // Orientation around the X axis
            supportPos1.elementAt(4)=candidatesPositionRef_[*iterator][4]; // Orientation around the Y axis
        } else {
            supportPos1.setZero();
        }
        return supportPos1;
    }

    MatrixHomogeneous& Odometry::getHomoSupportPos1(MatrixHomogeneous& homoSupportPos1, const int& time)
    {      
        if(time!=time_) computeOdometry(time);
        Vector supportPos1; supportPos1.setZero();
        supportPos1=getSupportPos1(supportPos1, time);
        homoSupportPos1=matrixHomogeneousFromPosUTheta(convertVector<stateObservation::Vector6>(supportPos1));
        return homoSupportPos1;
    }

    Vector& Odometry::getForceSupport1(Vector& forceSupport1, const int& time)
    {
        if(time!=time_) computeOdometry(time);
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
        if(time!=time_) computeOdometry(time);
        if (stackOfContacts_.size()>=2) {
            iterator = stackOfContacts_.begin();
            for(int i=1; i<2; ++i) ++iterator ;
            supportPos2=convertVector<dynamicgraph::Vector>(posUThetaFromMatrixHomogeneous(odometryRelativePosition_[*iterator]*pivotPosition_));
            supportPos2.elementAt(2)=candidatesPositionRef_[*iterator][2]; // Position along the Z axis
            supportPos2.elementAt(3)=candidatesPositionRef_[*iterator][3]; // Orientation around the X axis
            supportPos2.elementAt(4)=candidatesPositionRef_[*iterator][4]; // Orientation around the Y axis
        } else {
            supportPos2.setZero();
        }
        return supportPos2;
    }

    MatrixHomogeneous & Odometry::getHomoSupportPos2(MatrixHomogeneous & homoSupportPos2, const int& time)
    {
        if(time!=time_) computeOdometry(time);
        Vector supportPos2; supportPos2.setZero();
        supportPos2=getSupportPos2(supportPos2, time);
        homoSupportPos2=matrixHomogeneousFromPosUTheta(convertVector<stateObservation::Vector6>(supportPos2));
        return homoSupportPos2;
    }

    Vector& Odometry::getForceSupport2(Vector& forceSupport2, const int& time)
    {
        if(time!=time_) computeOdometry(time);
        if (stackOfContacts_.size()>=2) {
            iterator = stackOfContacts_.begin();
            for(int i=1; i<2; ++i) ++iterator ;
            forceSupport2=convertVector<dynamicgraph::Vector>(candidatesForces_[*iterator]);
        } else {
            forceSupport2.setZero();
        }
        return forceSupport2;
    }

    Vector& Odometry::getRobotStateOut(Vector& robotState, const int& time)
    {
        if(time!=time_) computeOdometry(time);

        stateObservation::Vector state; state.resize(36);
        stateObservation::Vector stateEncoders = convertVector<stateObservation::Vector>(robotStateInSIN_.access (time)).block(6,0,30,1);

        odometryFreeFlyer_.extract(pos_);
        odometryFreeFlyer_.extract(rot_);
        rpy_.fromMatrix(rot_);

        state << convertVector<stateObservation::Vector>(pos_),
                 convertVector<stateObservation::Vector>(rpy_),
                 stateEncoders;
        robotState = convertVector<Vector>(state);

        return robotState;
    }

    Vector& Odometry::getPivotPositionOut(Vector& pivotPositionOut, const int& time)
    {
        if(time!=time_) computeOdometry(time);
        pivotPositionOut=convertVector<dynamicgraph::Vector>(posUThetaFromMatrixHomogeneous(pivotPosition_));
        return pivotPositionOut;
    }


    void Odometry::setLeftFootPosition(const Matrix & mL)
    {
        leftFootPositionSIN_.setConstant(convertMatrix<MatrixHomogeneous>(mL));
        leftFootPositionSIN_.setTime (time_);
        pivotPosition_=convertMatrix<MatrixHomogeneous>(mL);
        computeOdometry(time_);
    }

    void Odometry::setRightFootPosition(const Matrix & mR)
    {
        rightFootPositionSIN_.setConstant(convertMatrix<MatrixHomogeneous>(mR));
        rightFootPositionSIN_.setTime (time_);
        computeOdometry(time_);
    }

    stateObservation::Vector6 Odometry::posUThetaFromMatrixHomogeneous (MatrixHomogeneous m)
    {
        m.extract(pos_);
        m.extract(rot_);
        uth_.fromMatrix(rot_);
        posUTheta_ << convertVector<stateObservation::Vector>(pos_),
                      convertVector<stateObservation::Vector>(uth_);
        return posUTheta_;
    }

    MatrixHomogeneous Odometry::matrixHomogeneousFromPosUTheta (stateObservation::Vector6 v)
    {
        pos_=convertVector<dynamicgraph::Vector>(v.block(0,0,3,1));
        uth_=convertVector<VectorUTheta>(v.block(3,0,3,1));
        rot_.fromVector(uth_);
        homo_.buildFrom(rot_,pos_);
        return homo_;
    }


    void Odometry::computeStackOfContacts(const int& time)
    {

        candidatesForces_[contact::rf] = convertVector<stateObservation::Vector>(forceRightFootSIN_.access (time));
        candidatesHomoPosition_[contact::rf] = rightFootPositionSIN_.access (time);
        candidatesHomoPositionRef_[contact::rf] = rightFootPositionRefSIN_.access (time);

        candidatesForces_[contact::lf] = convertVector<stateObservation::Vector>(forceLeftFootSIN_.access (time));
        candidatesHomoPosition_[contact::lf] = leftFootPositionSIN_.access (time);
        candidatesHomoPositionRef_[contact::lf] = leftFootPositionRefSIN_.access (time);

        double fz;
        bool found;

        for (int i=0; i<contact::nbMax;++i){
            candidatesPosition_[i]=posUThetaFromMatrixHomogeneous (candidatesHomoPosition_[i]);
            candidatesPositionRef_[i]=posUThetaFromMatrixHomogeneous (candidatesHomoPositionRef_[i]);

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
    }

    void Odometry::computeOdometry(const int& time){

                std::cout << "\ntimeOdo_=" << time_ << std::endl;

        computeStackOfContacts(time);

        /// Find the support used as pivot.
        bool pivotFound=false;
        iterator = stackOfContacts_.begin();
        while (pivotFound != true) {
            if(*iterator == contact::lf || *iterator == contact::rf) pivotFound=true;
        }
        int pivotContact=*iterator;

        /// Compute the pivot position from old pivot position
        pivotPosition_=pivotPosition_*odometryRelativePosition_[pivotContact];

        /// Compute odometryRelativeHomoPosition.
        for (int i=0; i<contact::nbMax; ++i){
            if(i==pivotContact){
                odometryRelativePosition_[i].setIdentity();
            } else {
                odometryRelativePosition_[i]=candidatesHomoPosition_[pivotContact].inverse()*candidatesHomoPosition_[i];
            }
        }

        /// Compute odometryFreeFlyer
            // reconstruction of freeFlyerInHomo
        MatrixRotation rot;
        Vector trans;
        MatrixHomogeneous freeFlyerInHomo; freeFlyerInHomo.setIdentity();
        stateObservation::Vector6 freeFlyerIn = convertVector<stateObservation::Vector>(robotStateInSIN_.access (time)).block(0,0,6,1);
        convertVector<VectorRollPitchYaw>(freeFlyerIn.block(3,0,3,1)).toMatrix(rot);
        trans=convertVector<Vector>(freeFlyerIn.block(0,0,3,1));
        freeFlyerInHomo.buildFrom(rot,trans);
            // reconstruction of odometryFreeFlyer
        odometryFreeFlyer_=pivotPosition_*candidatesHomoPosition_[pivotContact].inverse()*freeFlyerInHomo;

        time_=time;
    }
}

