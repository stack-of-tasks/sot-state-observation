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
        leftFootPositionSIN_ (NULL, "Odometry("+inName+")::input(HomoMatrix)::leftFootPositionIn"),
        rightFootPositionSIN_ (NULL, "Odometry("+inName+")::input(HomoMatrix)::rightFootPositionIn"),
        leftFootPositionRefSIN_ (NULL, "Odometry("+inName+")::input(Matrix)::leftFootPositionRef"),
        rightFootPositionRefSIN_ (NULL, "Odometry("+inName+")::input(Matrix)::rightFootPositionRef"),
        forceLeftFootSIN_ (NULL, "Odometry("+inName+")::input(vector)::force_lf"),
        forceRightFootSIN_ (NULL, "Odometry("+inName+")::input(vector)::force_rf"),
        stackOfSupportContactsSIN_ (NULL, "Odometry("+inName+")::input(vector)::stackOfSupportContacts"),
        leftFootPositionSOUT_ (NULL, "Odometry("+inName+")::input(HomoMatrix)::leftFootPositionOut"),
        rightFootPositionSOUT_ (NULL, "Odometry("+inName+")::input(HomoMatrix)::rightFootPositionOut"),
        freeFlyerSOUT_ (NULL, "Odometry("+inName+")::output(vector)::freeFlyer"),
        pivotPositionSOUT_ (NULL, "Odometry("+inName+")::output(Vector)::pivotPosition"),
        forceThreshold_ (.02 * 56.8*stateObservation::cst::gravityConstant), time_(0),
        inputForces_(contact::nbMax),
        inputPosition_(contact::nbMax), inputHomoPosition_(contact::nbMax),
        referencePosition_(contact::nbMax), referenceHomoPosition_(contact::nbMax),
        odometryHomoPosition_(contact::nbMax)
    {

        signalRegistration (leftFootPositionSIN_ << forceLeftFootSIN_);
        signalRegistration (rightFootPositionSIN_ << forceRightFootSIN_);

        signalRegistration (leftFootPositionRefSIN_);
        signalRegistration (rightFootPositionRefSIN_);

        signalRegistration (stackOfSupportContactsSIN_);

        signalRegistration (leftFootPositionSOUT_);
        signalRegistration (rightFootPositionSOUT_);

        signalRegistration (freeFlyerSOUT_);
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

        leftFootPositionSOUT_.setFunction(boost::bind(&Odometry::getHomoLeftFootPos, this, _1, _2));
        rightFootPositionSOUT_.setFunction(boost::bind(&Odometry::getHomoRightFootPos, this, _1, _2));
        freeFlyerSOUT_.setFunction(boost::bind(&Odometry::getFreeFlyer, this, _1, _2));
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

        op_.posUTheta_.setZero();
        op_.rot_.setZero();
        op_.homo_.setIdentity();
        op_.aa_=AngleAxis(0,stateObservation::Vector3::UnitZ());

        op_.alpha.resize(contact::nbMax);

        for (int i=0; i<contact::nbMax; ++i){
                odometryHomoPosition_[i]=referenceHomoPosition_[i];
        }
   }

    Odometry::~Odometry()
    {
    }

    MatrixHomogeneous& Odometry::getHomoLeftFootPos(MatrixHomogeneous& homoLeftFootPos, const int& time)
    {
        if(time!=time_) computeOdometry(time);
        homoLeftFootPos=convertMatrix<dynamicgraph::Matrix>(odometryHomoPosition_[contact::lf]);
        return homoLeftFootPos;
    }

    MatrixHomogeneous& Odometry::getHomoRightFootPos(MatrixHomogeneous& homoRightFootPos, const int& time)
    {
        if(time!=time_) computeOdometry(time);
        homoRightFootPos=convertMatrix<dynamicgraph::Matrix>(odometryHomoPosition_[contact::rf]);
        return homoRightFootPos;
    }

    Vector& Odometry::getFreeFlyer(Vector& freeFlyer, const int& time)
    {
        if(time!=time_) computeOdometry(time);

        stateObservation::Vector3 rpy=(Matrix3(odometryFreeFlyer_.block(0,0,3,3))).eulerAngles(0, 1, 2);

        stateObservation::Vector ff(6);
        ff << odometryFreeFlyer_.block(0,3,3,1),
              rpy;

        freeFlyer = convertVector<Vector>(ff);
        return freeFlyer;
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

    void Odometry::getInputs(const int& time)
    {
        inputForces_[contact::rf] = convertVector<stateObservation::Vector>(forceRightFootSIN_.access (time));
        inputHomoPosition_[contact::rf] = convertMatrix<stateObservation::Matrix4>(Matrix(rightFootPositionSIN_.access (time)));
        referenceHomoPosition_[contact::rf] = convertMatrix<stateObservation::Matrix4>(rightFootPositionRefSIN_.access (time));

        inputForces_[contact::lf] = convertVector<stateObservation::Vector>(forceLeftFootSIN_.access (time));
        inputHomoPosition_[contact::lf] = convertMatrix<stateObservation::Matrix4>(Matrix(leftFootPositionSIN_.access (time)));
        referenceHomoPosition_[contact::lf] = convertMatrix<stateObservation::Matrix4>(leftFootPositionRefSIN_.access (time));

        for (int i=0; i<contact::nbMax;++i)
        {
            inputPosition_[i]=kine::homogeneousMatrixToVector6(inputHomoPosition_[i]);
            referencePosition_[i]=kine::homogeneousMatrixToVector6(referenceHomoPosition_[i]);
        }
    }

    stateObservation::Matrix4 Odometry::regulateOdometryWithRef(const stateObservation::Matrix4& posEnc, const stateObservation::Vector& posRef, double alpha)
    {
        op_.posUTheta_=kine::homogeneousMatrixToVector6(posEnc);
        op_.posUTheta_.segment(2,3)=alpha*posRef.segment(2,3)+(1-alpha)*op_.posUTheta_.segment(2,3);
        return kine::vector6ToHomogeneousMatrix(op_.posUTheta_);
    }

    stateObservation::Matrix4 Odometry::homogeneousMatricesAverage(const stateObservation::Matrix4& m1, const stateObservation::Matrix4& m2, double alpha){

        // Rotational part
        op_.aa_=AngleAxis(Matrix3(m1.block(0,0,3,3).inverse()*m2.block(0,0,3,3)));
        op_.aa_=AngleAxis(alpha*op_.aa_.angle(),op_.aa_.axis());
        op_.rot_=op_.aa_.toRotationMatrix();
        op_.rot_=m1.block(0,0,3,3)*op_.rot_;
        op_.homo_.block(0,0,3,3)=op_.rot_;

        // Linear part
        op_.homo_.block(0,3,3,1)=(1-alpha)*m1.block(0,3,3,1)+alpha*m2.block(0,3,3,1);

        return op_.homo_;
    }

    void Odometry::computeOdometry(const int& time){

        getInputs(time);

        /// Get stack and number of contacts
        stackOfSupportContacts_ = convertVector<stateObservation::Vector>(stackOfSupportContactsSIN_.access (time));
        supportContactsNbr_ = stackOfSupportContacts_.size();

        /// Find the pivot support.
//        pivotSupport_=std::distance(&alpha_[0], (std::max_element(&alpha_[0],&alpha_[alpha_.size()])));
        pivotSupport_=stackOfSupportContacts_[0];

        /// Compute odometryHomoPosition.
        op_.alpha.setZero();
        op_.sum=0;
        for (int i=0; i<supportContactsNbr_; ++i)
        {
            op_.f=inputForces_[stackOfSupportContacts_[i]].segment(0,3).norm();
            op_.alpha[stackOfSupportContacts_[i]]=op_.f;
            op_.sum+=op_.f;
        }
        op_.alpha=(1/op_.sum)*op_.alpha;

        for (int i=0; i<contact::nbMax; ++i){
            if(op_.alpha[i]!=0){//i==pivotSupport_){//
                odometryHomoPosition_[i]=regulateOdometryWithRef(odometryHomoPosition_[i], referencePosition_[i], 1);//alpha_[i]);//
            } else {
                odometryHomoPosition_[i]=odometryHomoPosition_[pivotSupport_]*inputHomoPosition_[pivotSupport_].inverse()*inputHomoPosition_[i];
            }
        }

        /// Compute odometryFreeFlyer
        if(supportContactsNbr_==1){
            odometryFreeFlyer_=odometryHomoPosition_[pivotSupport_]*inputHomoPosition_[pivotSupport_].inverse();
        } else if (supportContactsNbr_==2){
            stateObservation::Matrix4 odometryFreeFlyerL=odometryHomoPosition_[contact::lf]*inputHomoPosition_[contact::lf].inverse();
            stateObservation::Matrix4 odometryFreeFlyerR=odometryHomoPosition_[contact::rf]*inputHomoPosition_[contact::rf].inverse();
            odometryFreeFlyer_=homogeneousMatricesAverage(odometryFreeFlyerL, odometryFreeFlyerR, op_.alpha[contact::rf]);
        }
//        odometryFreeFlyer_=odometryHomoPosition_[pivotSupport_]*inputHomoPosition_[pivotSupport_].inverse();

        time_=time;
    }
}

