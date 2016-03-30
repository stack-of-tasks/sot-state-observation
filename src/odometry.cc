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
#include <sot-state-observation/tools/definitions.hh>
#include <state-observation/tools/miscellaneous-algorithms.hpp>

#include <sot-state-observation/odometry.hh>

namespace sotStateObservation
{
    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN ( Odometry, "Odometry" );

    Odometry::Odometry( const std::string & inName):
        Entity(inName),
        leftFootPositionSIN_ (NULL, "Odometry("+inName+")::input(HomoMatrix)::leftFootPosition"),
        rightFootPositionSIN_ (NULL, "Odometry("+inName+")::input(HomoMatrix)::rightFootPosition"),
        leftFootPositionRefSIN_ (NULL, "Odometry("+inName+")::input(Matrix)::leftFootPositionRef"),
        rightFootPositionRefSIN_ (NULL, "Odometry("+inName+")::input(Matrix)::rightFootPositionRef"),
        forceLeftFootSIN_ (NULL, "Odometry("+inName+")::input(vector)::force_lf"),
        forceRightFootSIN_ (NULL, "Odometry("+inName+")::input(vector)::force_rf"),
        stackOfSupportContactsSIN_ (NULL, "Odometry("+inName+")::input(vector)::stackOfSupportContacts"),
        robotStateInSIN_ (NULL, "Odometry("+inName+")::input(vector)::robotStateIn"),
        robotStateOutSOUT_ (NULL, "Odometry("+inName+")::output(vector)::robotStateOut"),
        nbSupportSOUT_ (NULL,"Odometry("+inName+")::output(unsigned)::nbSupport"),
        supportPos1SOUT_(NULL,"Odometry("+inName+")::output(vector)::supportPos1"),
        supportPos2SOUT_(NULL,"Odometry("+inName+")::output(vector)::supportPos2"),
        homoSupportPos1SOUT_(NULL, "Odometry("+inName+")::output(HomoMatrix)::homoSupportPos1"),
        homoSupportPos2SOUT_(NULL, "Odometry("+inName+")::output(HomoMatrix)::homoSupportPos2"),
        forceSupport1SOUT_ (NULL, "Odometry("+inName+")::output(vector)::forceSupport1"),
        forceSupport2SOUT_ (NULL, "Odometry("+inName+")::output(vector)::forceSupport2"),
        forceSupportStackSOUT_ (NULL, "Odometry("+inName+")::output(vector)::forceSupportStack"),
        pivotPositionSOUT_ (NULL, "Odometry("+inName+")::output(vector)::pivotPosition"),
        forceThreshold_ (.02 * 56.8*stateObservation::cst::gravityConstant), time_(0),
        inputForces_(contact::nbMax),
        inputPosition_(contact::nbMax), inputHomoPosition_(contact::nbMax),
        referencePosition_(contact::nbMax), referenceHomoPosition_(contact::nbMax),
        odometryHomoPosition_(contact::nbMax), alpha_(contact::nbMax)
    {

        signalRegistration (leftFootPositionSIN_ << forceLeftFootSIN_);
        signalRegistration (rightFootPositionSIN_ << forceRightFootSIN_);

        signalRegistration (leftFootPositionRefSIN_);
        signalRegistration (rightFootPositionRefSIN_);

        signalRegistration (stackOfSupportContactsSIN_);

        signalRegistration (robotStateInSIN_);
        signalRegistration (robotStateOutSOUT_);

        signalRegistration (nbSupportSOUT_);
        signalRegistration (supportPos1SOUT_ << homoSupportPos1SOUT_ << forceSupport1SOUT_);
        signalRegistration (supportPos2SOUT_ << homoSupportPos2SOUT_ << forceSupport2SOUT_);
        signalRegistration (forceSupportStackSOUT_);

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

        forceSupportStackSOUT_.setFunction(boost::bind(&Odometry::getForceSupportStack, this, _1, _2));

        robotStateOutSOUT_.setFunction(boost::bind(&Odometry::getRobotStateOut, this, _1, _2));
        pivotPositionSOUT_.setFunction(boost::bind(&Odometry::getPivotPositionOut, this, _1, _2));

        stateObservation::Matrix leftFootPos;
        leftFootPos.resize(4,4);
        leftFootPos <<  0.999998, 6.69591e-07, 0.00184865, 0.00441951,
                        -1.39371e-06, 1, 0.000391702, 0.0940796,
                        -0.00184865, -0.000391703, 0.999998, -0.64868,
                        0, 0, 0, 1;
        leftFootPositionSIN_.setConstant(convertMatrix<dynamicgraph::Matrix>(leftFootPos));
        leftFootPositionSIN_.setTime (time_);
        inputHomoPosition_[contact::lf]=leftFootPos;
        inputPosition_[contact::lf]=kine::homogeneousMatrixToVector6(leftFootPos);

        stateObservation::Matrix rightFootPos;
        rightFootPos.resize(4,4);
        rightFootPos <<  0.999998, 5.65304e-06, 0.00198742, 0.00494195,
                        -5.78281e-06, 1, 6.52873e-05, -0.0958005,
                        -0.00198742, -6.52987e-05, 0.999998, -0.64868,
                        0, 0, 0, 1;
        rightFootPositionSIN_.setConstant(convertMatrix<dynamicgraph::Matrix>(rightFootPos));
        rightFootPositionSIN_.setTime (time_);
        inputHomoPosition_[contact::rf]=rightFootPos;
        inputPosition_[contact::rf]=kine::homogeneousMatrixToVector6(rightFootPos);

        stateObservation::Matrix leftFootPosRef;
        leftFootPosRef.resize(4,4);
        leftFootPosRef <<  1,1.94301e-07,2.363e-10,0.00949046,
                           -1.94301e-07,1,-2.70566e-12,0.095,
                           -2.363e-10,2.70562e-12,1,3.03755e-06,
                            0,0,0,1;
        leftFootPositionRefSIN_.setConstant(convertMatrix<dynamicgraph::Matrix>(leftFootPosRef));
        leftFootPositionRefSIN_.setTime (time_);
        referenceHomoPosition_[contact::lf]=leftFootPosRef;
        referencePosition_[contact::lf]=kine::homogeneousMatrixToVector6(referenceHomoPosition_[contact::lf]);

        stateObservation::Matrix rightFootPosRef;
        rightFootPosRef.resize(4,4);
        rightFootPosRef <<  1,-9.18094e-18,-1.52169e-16,0.009496046,
                            9.184e-18,1,-1.10345e-16,-0.095,
                            1.68756e-16,1.10345e-16,1,2.55006e-07,
                            0,0,0,1;
        rightFootPositionRefSIN_.setConstant(convertMatrix<dynamicgraph::Matrix>(rightFootPosRef));
        rightFootPositionRefSIN_.setTime (time_);
        referenceHomoPosition_[contact::rf]=rightFootPosRef;
        referencePosition_[contact::rf]=kine::homogeneousMatrixToVector6(referenceHomoPosition_[contact::rf]);


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
        inputForces_[contact::rf]=forceRightFoot;

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
        inputForces_[contact::lf]=forceLeftFoot;

        stateObservation::Vector v; v.resize(2);
        v << 0.0,
             1.0;
        stackOfSupportContactsSIN_.setConstant(convertVector<dynamicgraph::Vector>(v));
        stackOfSupportContactsSIN_.setTime(time_);

        posUTheta_.setZero();
        rot_.setZero();
        homo_.setIdentity();
        aa_=AngleAxis(0,stateObservation::Vector3::UnitZ());

        Vector robotState;
        robotState.resize(36);
        robotState.setZero();
        robotStateInSIN_.setConstant(robotState);
        robotStateInSIN_.setTime (time_);

        for (int i=0; i<contact::nbMax; ++i){
                odometryHomoPosition_[i]=referenceHomoPosition_[i];
        }

//        odometryFreeFlyer_ <<  1,0,0,0.0102748,
//                               0,1,0,0.00199113,
//                               0,0,1,0.648926,
//                               0,0,0,1;
//        computeOdometry(time_);

   }

    Odometry::~Odometry()
    {
    }

    unsigned int& Odometry::getNbSupport(unsigned int& nbSupport, const int& time)
    {
        if(time!=time_) computeOdometry(time);
        nbSupport = stackOfSupports_.size();
        return nbSupport;
    }

    Vector& Odometry::getSupportPos1(Vector& supportPos1, const int& time)
    {
        if(time!=time_) computeOdometry(time);

        supportPos1.resize(6);
        if (stackOfSupports_.size()>=1) {
            iterator_ = stackOfSupports_.begin();
            supportPos1=convertVector<dynamicgraph::Vector>(kine::homogeneousMatrixToVector6(odometryHomoPosition_[*iterator_]));
        } else {
            supportPos1.setZero();
        }
        return supportPos1;
    }

    MatrixHomogeneous& Odometry::getHomoSupportPos1(MatrixHomogeneous& homoSupportPos1, const int& time)
    {
        if(time!=time_) computeOdometry(time);
        if (stackOfSupports_.size()>=1) {
            iterator_ = stackOfSupports_.begin();
            homoSupportPos1=convertMatrix<MatrixHomogeneous>(odometryHomoPosition_[*iterator_]);
        } else {
            homoSupportPos1.setIdentity();
        }
        return homoSupportPos1;
    }

    Vector& Odometry::getForceSupport1(Vector& forceSupport1, const int& time)
    {
        if(time!=time_) computeOdometry(time);

        forceSupport1.resize(6);
        if (stackOfSupports_.size()>=1) {
            iterator_ = stackOfSupports_.begin();
            forceSupport1=convertVector<dynamicgraph::Vector>(inputForces_[*iterator_]);
        } else {
                        forceSupport1.setZero();
        }
        return forceSupport1;
    }

    Vector& Odometry::getForceSupportStack(Vector& forceSupportStack, const int& time)
    {
      if(time!=time_) computeOdometry(time);
      forceSupportStack.resize(stackOfSupports_.size()*6);

      int i = 0 ;
      for (iterator_=stackOfSupports_.begin();
            iterator_!=stackOfSupports_.end();
              ++iterator_)
      {
        setSubvector(forceSupportStack,i*6,convertVector<dynamicgraph::Vector>(inputForces_[*iterator_]));
        ++i;
      }
      return forceSupportStack;


    }

    Vector& Odometry::getSupportPos2(Vector& supportPos2, const int& time)
    {
        if(time!=time_) computeOdometry(time);

        supportPos2.resize(6);
        if (stackOfSupports_.size()>=2) {
            iterator_ = stackOfSupports_.begin();
            for(int i=1; i<2; ++i) ++iterator_ ;
            supportPos2=convertVector<dynamicgraph::Vector>(kine::homogeneousMatrixToVector6(odometryHomoPosition_[*iterator_]));
        } else {
            supportPos2.setZero();
        }
        return supportPos2;
    }

    MatrixHomogeneous & Odometry::getHomoSupportPos2(MatrixHomogeneous & homoSupportPos2, const int& time)
    {
        if(time!=time_) computeOdometry(time);
        if (stackOfSupports_.size()>=2) {
            iterator_ = stackOfSupports_.begin();
            for(int i=1; i<2; ++i) ++iterator_ ;
            homoSupportPos2=convertMatrix<MatrixHomogeneous>(odometryHomoPosition_[*iterator_]);
        } else {
            homoSupportPos2.setIdentity();
        }
        return homoSupportPos2;
    }

    Vector& Odometry::getForceSupport2(Vector& forceSupport2, const int& time)
    {
        if(time!=time_) computeOdometry(time);

        forceSupport2.resize(6);
        if (stackOfSupports_.size()>=2) {
            iterator_ = stackOfSupports_.begin();
            for(int i=1; i<2; ++i) ++iterator_ ;
            forceSupport2=convertVector<dynamicgraph::Vector>(inputForces_[*iterator_]);
        } else {
                        forceSupport2.setZero();
        }
        return forceSupport2;
    }

    Vector& Odometry::getRobotStateOut(Vector& robotState, const int& time)
    {
        if(time!=time_) computeOdometry(time);

        stateObservation::Vector3 rpy=(Matrix3(odometryFreeFlyer_.block(0,0,3,3))).eulerAngles(0, 1, 2);
        stateObservation::Vector robotStateIn = convertVector<stateObservation::Vector>(robotStateInSIN_.access (time));
        stateObservation::Vector stateEncoders(robotStateIn.tail<30>());

        stateObservation::Vector state(36);
        state << odometryFreeFlyer_.block(0,3,3,1),
                 rpy,
                 stateEncoders;
        robotState = convertVector<Vector>(state);

        return robotState;
    }

    Vector& Odometry::getPivotPositionOut(Vector& pivotPositionOut, const int& time)
    {
        if(time!=time_) computeOdometry(time);
        pivotPositionOut=convertVector<dynamicgraph::Vector>(kine::homogeneousMatrixToVector6(odometryHomoPosition_[pivotSupport_]));
        return pivotPositionOut;
    }


    void Odometry::setLeftFootPosition(const Matrix & mL)
    {
        odometryHomoPosition_[contact::lf]=referenceHomoPosition_[contact::lf];
    }

    void Odometry::setRightFootPosition(const Matrix & mR)
    {
        odometryHomoPosition_[contact::rf]=referenceHomoPosition_[contact::rf];
    }

    void Odometry::computeStackOfContacts(const int& time)
    {

        inputForces_[contact::rf] = convertVector<stateObservation::Vector>(forceRightFootSIN_.access (time));
        inputHomoPosition_[contact::rf] = convertMatrix<stateObservation::Matrix4>(Matrix(rightFootPositionSIN_.access (time)));
        referenceHomoPosition_[contact::rf] = convertMatrix<stateObservation::Matrix4>(rightFootPositionRefSIN_.access (time));

        inputForces_[contact::lf] = convertVector<stateObservation::Vector>(forceLeftFootSIN_.access (time));
        inputHomoPosition_[contact::lf] = convertMatrix<stateObservation::Matrix4>(Matrix(leftFootPositionSIN_.access (time)));
        referenceHomoPosition_[contact::lf] = convertMatrix<stateObservation::Matrix4>(leftFootPositionRefSIN_.access (time));

        double fz;
        bool found;

        for (int i=0; i<contact::nbMax;++i){
            inputPosition_[i]=kine::homogeneousMatrixToVector6(inputHomoPosition_[i]);
            referencePosition_[i]=kine::homogeneousMatrixToVector6(referenceHomoPosition_[i]);

            fz =  inputHomoPosition_[i](2,0) * inputForces_[i](0) +
                  inputHomoPosition_[i](2,1) * inputForces_[i](1) +
                  inputHomoPosition_[i](2,2) * inputForces_[i](2);

            found = (std::find(stackOfSupports_.begin(), stackOfSupports_.end(), i) != stackOfSupports_.end());

            if(fz>forceThreshold_) {
                if (!found) stackOfSupports_.push_back(i);
            } else {
                if(found) stackOfSupports_.remove(i);
            }
        }
    }

    stateObservation::Matrix4 Odometry::regulateOdometryWithRef(stateObservation::Matrix4 posEnc, stateObservation::Vector posRef, double alpha)
    {
        posUTheta_=kine::homogeneousMatrixToVector6(posEnc);
        posUTheta_.segment(2,3)=alpha*posRef.segment(2,3)+(1-alpha)*posUTheta_.segment(2,3);
        return kine::vector6ToHomogeneousMatrix(posUTheta_);
    }

    stateObservation::Matrix4 Odometry::homogeneousMatricesAverage(stateObservation::Matrix4 m1, stateObservation::Matrix4 m2, double alpha){

        // Rotational part
        aa_=AngleAxis(Matrix3(m1.block(0,0,3,3).inverse()*m2.block(0,0,3,3)));
        aa_=AngleAxis(alpha*aa_.angle(),aa_.axis());
        rot_=aa_.toRotationMatrix();
        rot_=m1.block(0,0,3,3)*rot_;
        homo_.block(0,0,3,3)=rot_;

        // Linear part
        homo_.block(0,3,3,1)=(1-alpha)*m1.block(0,3,3,1)+alpha*m2.block(0,3,3,1);

        return homo_;
    }

    void Odometry::computeOdometry(const int& time){

        computeStackOfContacts(time);

        /// Computation of alpha
        alpha_.setZero();
        double sum=0;
        double f=0;
        for (iterator_=stackOfSupports_.begin(); iterator_ != stackOfSupports_.end(); ++iterator_)
        {
            f=inputForces_[*iterator_].segment(0,3).norm();
            alpha_[*iterator_]=f;
            sum+=f;
        }
        alpha_=(1/sum)*alpha_;

        /// Find the pivot support.
//        pivotSupport_=std::distance(&alpha_[0], (std::max_element(&alpha_[0],&alpha_[alpha_.size()])));
        pivotSupport_=*(stackOfSupports_.begin());

        /// Compute odometryHomoPosition.
        for (int i=0; i<contact::nbMax; ++i){
            if(alpha_[i]!=0){//i==pivotSupport_){//
                odometryHomoPosition_[i]=regulateOdometryWithRef(odometryHomoPosition_[i], referencePosition_[i], 1);//alpha_[i]);//
            } else {
                odometryHomoPosition_[i]=odometryHomoPosition_[pivotSupport_]*inputHomoPosition_[pivotSupport_].inverse()*inputHomoPosition_[i];
            }
        }

        /// Compute odometryFreeFlyer
            // reconstruction of freeFlyerInHomo
        stateObservation::Matrix4 freeFlyerInHomo; freeFlyerInHomo.setIdentity();
//        stateObservation::Vector6 freeFlyerIn = convertVector<stateObservation::Vector>(robotStateInSIN_.access (time)).segment(0,6);
//        rot_= stateObservation::Matrix3( AngleAxis(freeFlyerIn[5], stateObservation::Vector3::UnitZ()) *
//                                         AngleAxis(freeFlyerIn[4], stateObservation::Vector3::UnitY()) *
//                                         AngleAxis(freeFlyerIn[3], stateObservation::Vector3::UnitX()));
//        freeFlyerInHomo.block(0,0,3,3)=rot_;
//        freeFlyerInHomo.block(0,3,3,1)=freeFlyerIn.segment(0,3);

            // reconstruction of odometryFreeFlyer
        if(stackOfSupports_.size()==1){
            odometryFreeFlyer_=odometryHomoPosition_[pivotSupport_]*inputHomoPosition_[pivotSupport_].inverse()*freeFlyerInHomo;
        } else if (stackOfSupports_.size()==2){
            stateObservation::Matrix4 odometryFreeFlyerL=odometryHomoPosition_[contact::lf]*inputHomoPosition_[contact::lf].inverse()*freeFlyerInHomo;
            stateObservation::Matrix4 odometryFreeFlyerR=odometryHomoPosition_[contact::rf]*inputHomoPosition_[contact::rf].inverse()*freeFlyerInHomo;
            odometryFreeFlyer_=homogeneousMatricesAverage(odometryFreeFlyerL, odometryFreeFlyerR, alpha_[contact::rf]);
        }
//        odometryFreeFlyer_=odometryHomoPosition_[pivotSupport_]*inputHomoPosition_[pivotSupport_].inverse()*freeFlyerInHomo;

        time_=time;
    }
}

