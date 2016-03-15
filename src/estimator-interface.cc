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
        modeledContactsNbrSOUT_ (NULL, "EstimatorInterface("+inName+")::output(unsigned)::modeledContactsNbr"),
        positionLeftFootSIN_ (NULL, "EstimatorInterface("+inName+")::input(HomoMatrix)::position_lf"),
        forceLeftFootSIN_ (NULL, "EstimatorInterface("+inName+")::input(vector)::force_lf"),
        positionRightFootSIN_ (NULL, "EstimatorInterface("+inName+")::input(HomoMatrix)::position_rf"),
        forceRightFootSIN_ (NULL, "EstimatorInterface("+inName+")::input(vector)::force_rf"),
        positionLeftHandSIN_ (NULL, "EstimatorInterface("+inName+")::input(HomoMatrix)::position_lh"),
        forceLeftHandSIN_ (NULL, "EstimatorInterface("+inName+")::input(vector)::force_lh"),
        positionRightHandSIN_ (NULL, "EstimatorInterface("+inName+")::input(HomoMatrix)::position_rh"),
        forceRightHandSIN_ (NULL, "EstimatorInterface("+inName+")::input(vector)::force_rh"),
        comVectorSIN(NULL , "EstimatorInterface("+inName+")::input(vector)::comVector"),
        inertiaSIN(NULL , "EstimatorInterface("+inName+")::input(matrix)::inertia"),
        dinertiaSIN(NULL , "EstimatorInterface("+inName+")::input(vector)::dinertia"),
        positionWaistSIN(NULL , "EstimatorInterface("+inName+")::input(matrix)::positionWaist"),
        angMomentumSIN(NULL , "EstimatorInterface("+inName+")::input(vector)::angMomentum"),
        dangMomentumSIN(NULL , "EstimatorInterface("+inName+")::input(vector)::dangMomentum"),
        imuVectorSIN(NULL , "EstimatorInterface("+inName+")::input(vector)::imuVector"),
        timeStackOfContacts_(0), timeInput_(0),
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
        positionLeftFootSIN_.setTime (timeStackOfContacts_);
        forceLeftFootSIN_.setConstant(convertVector<dynamicgraph::Vector>(force));
        forceLeftFootSIN_.setTime (timeStackOfContacts_);

        signalRegistration (positionRightFootSIN_ << forceRightFootSIN_);
        positionRightFootSIN_.setConstant(pos);
        positionRightFootSIN_.setTime (timeStackOfContacts_);
        forceRightFootSIN_.setConstant(convertVector<dynamicgraph::Vector>(force));
        forceRightFootSIN_.setTime (timeStackOfContacts_);

        signalRegistration (positionLeftHandSIN_ << forceLeftHandSIN_);
        positionLeftHandSIN_.setConstant(pos);
        positionLeftHandSIN_.setTime (timeStackOfContacts_);
        forceLeftHandSIN_.setConstant(convertVector<dynamicgraph::Vector>(force));
        forceLeftHandSIN_.setTime (timeStackOfContacts_);

        signalRegistration (positionRightHandSIN_ << forceRightHandSIN_);
        positionRightHandSIN_.setConstant(pos);
        positionRightHandSIN_.setTime (timeStackOfContacts_);
        forceRightHandSIN_.setConstant(convertVector<dynamicgraph::Vector>(force));
        forceRightHandSIN_.setTime (timeStackOfContacts_);

        signalRegistration (comVectorSIN);
        dynamicgraph::Vector comVector(3);
        comVectorSIN.setConstant(comVector);

        signalRegistration (inertiaSIN);
        dynamicgraph::Matrix inertia(6);
        inertiaSIN.setConstant(inertia);

        signalRegistration (dinertiaSIN);
        dynamicgraph::Vector dinertia(6);
        dinertiaSIN.setConstant(dinertia);

        signalRegistration (positionWaistSIN);
        dynamicgraph::Matrix positionWaist;
        positionWaistSIN.setConstant(positionWaist);

        signalRegistration (angMomentumSIN);
        dynamicgraph::Vector angMomentum(6);
        angMomentumSIN.setConstant(angMomentum);

        signalRegistration (dangMomentumSIN);
        dynamicgraph::Vector dangMomentum(6);
        dangMomentumSIN.setConstant(dangMomentum);

        signalRegistration (imuVectorSIN);
        dynamicgraph::Vector imuVector(15);
        imuVectorSIN.setConstant(imuVector);

        // Output
        signalRegistration (inputSOUT_);
        inputSOUT_.setFunction(boost::bind(&EstimatorInterface::getInput, this, _1, _2));

        signalRegistration (measurementSOUT_);
        measurementSOUT_.setFunction(boost::bind(&EstimatorInterface::getMeasurement, this, _1, _2));

        signalRegistration (contactsNbrSOUT_);
        contactsNbrSOUT_.setFunction(boost::bind(&EstimatorInterface::getContactsNbr, this, _1, _2));

        signalRegistration (modeledContactsNbrSOUT_);
        modeledContactsNbrSOUT_.setFunction(boost::bind(&EstimatorInterface::getModeledContactsNbr, this, _1, _2));

        /// Commands

        std::string docstring;

        docstring  =
                "\n"
                "    Set the force thresholds \n"
                "\n";

        addCommand(std::string("setForceThresholds"),
             new
             ::dynamicgraph::command::Setter <EstimatorInterface,dynamicgraph::Vector>
                (*this, &EstimatorInterface::setForceThresholds, docstring));
        docstring =
                "\n"
                "    Get the force thresholds \n"
                "\n";

        addCommand(std::string("getForceThresholds"),
             new
             ::dynamicgraph::command::Getter <EstimatorInterface,dynamicgraph::Vector>
                (*this, &EstimatorInterface::getForceThresholds, docstring));

        /// Parameters

        // ForceThresholds
        forceThresholds_.resize(contact::nbMax);
        forceThresholds_.setOnes();
        forceThresholds_*=0.02 * 56.8*stateObservation::cst::gravityConstant; // default value
        forceThresholds_[contact::lh]=15;
        forceThresholds_[contact::rh]=15;

        // Modeled
        modeled_.resize(contact::nbMax);
        modeled_[contact::lf]=true;
        modeled_[contact::rf]=true;
        modeled_[contact::lh]=false;
        modeled_[contact::rh]=false;

        // Outputs
        input_.resize(42);

        // From input reconstructor
        bias_[0].resize(6);
        bias_[1].resize(6);
        bias_[0].setZero();
        bias_[1].setZero();
        lastInertia_.setZero();
        dt_=5e-3;
    }

    EstimatorInterface::~EstimatorInterface()
    {
    }

    void EstimatorInterface::computeStackOfContacts(const int& time)
    {
        timeStackOfContacts_=time;

        inputForces_[contact::rf] = convertVector<stateObservation::Vector>(forceRightFootSIN_.access (time));
        inputHomoPosition_[contact::rf] = convertMatrix<stateObservation::Matrix4>(Matrix(positionRightFootSIN_.access (time)));

        inputForces_[contact::lf] = convertVector<stateObservation::Vector>(forceLeftFootSIN_.access (time));
        inputHomoPosition_[contact::lf] = convertMatrix<stateObservation::Matrix4>(Matrix(positionLeftFootSIN_.access (time)));

        inputForces_[contact::rh] = convertVector<stateObservation::Vector>(forceRightHandSIN_.access (time));
        inputHomoPosition_[contact::rh] = convertMatrix<stateObservation::Matrix4>(Matrix(positionRightHandSIN_.access (time)));

        inputForces_[contact::lh] = convertVector<stateObservation::Vector>(forceLeftHandSIN_.access (time));
        inputHomoPosition_[contact::lh] = convertMatrix<stateObservation::Matrix4>(Matrix(positionLeftHandSIN_.access (time)));

        bool found;

        for (int i=0; i<contact::nbMax;++i)
        {
            inputPosition_[i]=kine::homogeneousMatrixToVector6(inputHomoPosition_[i]);

            found = (std::find(stackOfContacts_.begin(), stackOfContacts_.end(), i) != stackOfContacts_.end());

            if(inputForces_[i].norm()>forceThresholds_[i])
            {
                if (!found)
                {
                    stackOfContacts_.push_back(i);
                    if(modeled_[i]) {
                        std::cout << "i=" << i << std::endl;
                        stackOfModeledContacts_.push_back(i); }
                    if(!modeled_[i]) { stackOfUnmodeledContacts_.push_back(i); }
                }
            }
            else
            {
                if(found)
                {
                    stackOfContacts_.remove(i);
                    if(modeled_[i]) {
                        std::cout << "iUnmodeld=" << i << std::endl;
                        stackOfModeledContacts_.remove(i); }
                    if(!modeled_[i]) { stackOfUnmodeledContacts_.remove(i); }
                }
            }
        }
    }

    void EstimatorInterface::computeInert(const dynamicgraph::Matrix & inertia, const dynamicgraph::Matrix & homoWaist, dynamicgraph::Vector& inert, const dynamicgraph::Vector& comVector)
    {

        double m=inertia(0,0); //<=== donne 56.8;
        //std::cout << "Masse=" << m << std::endl;

        dynamicgraph::Vector waist, com;
        waist.resize(3);
        com.resize(3);

        waist.elementAt(0)=homoWaist(0,3);
        waist.elementAt(1)=homoWaist(1,3);
        waist.elementAt(2)=homoWaist(2,3);

        com.elementAt(0)=comVector(0);
        com.elementAt(1)=comVector(1);
        com.elementAt(2)=comVector(2);

        // Inertia expressed at waist
        inert.elementAt(0)=inertia(3,3);
        inert.elementAt(1)=inertia(4,4);
        inert.elementAt(2)=inertia(5,5);
        inert.elementAt(3)=inertia(3,4);
        inert.elementAt(4)=inertia(3,5);
        inert.elementAt(5)=inertia(4,5);

        // From waist to com
        inert.elementAt(0) += -m*((com.elementAt(1)-waist.elementAt(1))*(com.elementAt(1)-waist.elementAt(1))+(com.elementAt(2)-waist.elementAt(2))*(com.elementAt(2)-waist.elementAt(2)));
        inert.elementAt(1) += -m*((com.elementAt(0)-waist.elementAt(0))*(com.elementAt(0)-waist.elementAt(0))+(com.elementAt(2)-waist.elementAt(2))*(com.elementAt(2)-waist.elementAt(2)));
        inert.elementAt(2) += -m*((com.elementAt(0)-waist.elementAt(0))*(com.elementAt(0)-waist.elementAt(0))+(com.elementAt(1)-waist.elementAt(1))*(com.elementAt(1)-waist.elementAt(1)));
        inert.elementAt(3) += m*(com.elementAt(0)-waist.elementAt(0))*(com.elementAt(1)-waist.elementAt(1));
        inert.elementAt(4) += m*(com.elementAt(0)-waist.elementAt(0))*(com.elementAt(2)-waist.elementAt(2));
        inert.elementAt(5) += m*(com.elementAt(1)-waist.elementAt(1))*(com.elementAt(2)-waist.elementAt(2));

        // From com to local frame
        inert.elementAt(0) -= -m*((com.elementAt(1))*(com.elementAt(1))+(com.elementAt(2))*(com.elementAt(2)));
        inert.elementAt(1) -= -m*((com.elementAt(0))*(com.elementAt(0))+(com.elementAt(2))*(com.elementAt(2)));
        inert.elementAt(2) -= -m*((com.elementAt(0))*(com.elementAt(0))+(com.elementAt(1))*(com.elementAt(1)));
        inert.elementAt(3) -= m*(com.elementAt(0))*(com.elementAt(1));
        inert.elementAt(4) -= m*(com.elementAt(0))*(com.elementAt(2));
        inert.elementAt(5) -= m*(com.elementAt(1))*(com.elementAt(2));

    }

    void EstimatorInterface::computeInertDot
            (const dynamicgraph::Matrix & inertia, const dynamicgraph::Vector & dinertia,
            const dynamicgraph::Matrix & homoWaist, dynamicgraph::Vector& dinert,
            const dynamicgraph::Vector& comVector)
    {
      //FIXE : THIS FUNCTION IS WRONG
        double m=inertia(0,0); //<=== donne 56.8;
        //std::cout << "Masse=" << m << std::endl;

        dynamicgraph::Vector waist, com, dcom;
        waist.resize(3);
        com.resize(3);
        dcom.resize(3);

        waist.elementAt(0)=homoWaist(0,3);
        waist.elementAt(1)=homoWaist(1,3);
        waist.elementAt(2)=homoWaist(2,3);

        com.elementAt(0)=comVector(0);
        com.elementAt(1)=comVector(1);
        com.elementAt(2)=comVector(2);

        dcom.elementAt(0)=comVector(3);
        dcom.elementAt(1)=comVector(4);
        dcom.elementAt(2)=comVector(5);

        // Inertia expressed at waist
        dinert = dinertia;
   }

   void EstimatorInterface::computeInput(const int& time)
   {
       timeInput_=time;

       const dynamicgraph::Matrix& inertia=inertiaSIN.access(time);
       const dynamicgraph::Matrix& homoWaist=positionWaistSIN.access(time);
       const dynamicgraph::Vector& comVector=comVectorSIN.access(time);
       const dynamicgraph::Vector& dinertia=dinertiaSIN.access(time);
       const dynamicgraph::Vector& angMomentum=angMomentumSIN.access(time);
       const dynamicgraph::Vector& dangMomentum=dangMomentumSIN.access(time);
       const dynamicgraph::Vector& imuVector=imuVectorSIN.access(time);
       unsigned contactsNbr;
       const unsigned& nbContacts=getModeledContactsNbr(contactsNbr,time);

       int i, u=0;

       // Modeled contacts position
       if(time!=timeStackOfContacts_) computeStackOfContacts(time);
       stateObservation::Vector contactPos; contactPos.resize(nbContacts*6); contactPos.setZero();
       i=0;
       for (iterator = stackOfModeledContacts_.begin(); iterator != stackOfModeledContacts_.end(); ++iterator)
       {
           contactPos.segment(i*6,6)=inputPosition_[*iterator];
           ++i;
       } i=0;
       dynamicgraph::Vector contactsPosition=convertVector<dynamicgraph::Vector>(contactPos);

       // Inertia and derivative
       dynamicgraph::Vector inert,dinert;
       inert.resize(6);
       computeInert(inertia,homoWaist,inert,comVector);
       if (derivateInertiaFD_)
       {
         if (lastInertia_.size()>0 && lastInertia_.norm()!=0)
           dinert = (1/dt_)*(inert - lastInertia_);
         else
         {
           dinert.resize(6);
           dinert.setZero();
         }
       }
       else
         computeInertDot(inertia,dinertia,homoWaist,dinert,comVector);
       lastInertia_ = inert;

       double m=inertia(0,0);

       // Com
       dynamicgraph::Vector com, comdot, comddot;
       com.resize(3);
       comdot.resize(3);
       comddot.resize(3);
       for (i=0;i<3;++i)
       {
         com(i) = comVector(i);
         comdot(i) = comVector(i+3);
         comddot(i) = comVector(i+6);
       }

       // Angular momentum and derivative
       dynamicgraph::Vector angMomentumOut, dangMomentumOut;
       angMomentumOut=angMomentum;
       dangMomentumOut = m*crossProduct(com,comddot);

       // Concatenate input
       dynamicgraph::Vector input;
       input.resize(42+6*nbContacts,true);
       input.setZero();

       for(i=0;i<3;++i){
           input.elementAt(u)=comVector(i);
           u++;
       }

       for(i=0;i<3;++i){
           input.elementAt(u)=comVector(i+3);
           u++;
       }

       for(i=0;i<3;++i){
           input.elementAt(u)=comVector(i+6);
           u++;
       }

       for(i=0;i<6;++i){
           input.elementAt(u)=inert(i);
           u++;
       }

       for(i=0;i<6;++i){
           input.elementAt(u)=dinert(i);
           u++;
       }

       for(i=0;i<3;++i){
           input.elementAt(u)=angMomentumOut(i);
           u++;
       }

       for(i=0;i<3;++i){
           input.elementAt(u)=dangMomentumOut(i);
           u++;
       }

       for(i=0;i<6;++i){
           input.elementAt(u)=imuVector(i);
           u++;
       }

       for(i=0;i<6;++i){
           input.elementAt(u)=imuVector(i+6);
           u++;
       }

       for(i=0;i<3;++i){
           input.elementAt(u)=imuVector(i+12);
           u++;
       }

       for(i=0;i<6*nbContacts;++i){

           input.elementAt(u)=contactsPosition(i)+bias_[i/6](i%6);
           u++;
       }

       input_=convertVector<stateObservation::Vector>(input);
   }

   Vector& EstimatorInterface::getInput(Vector& input, const int& time)
   {
        if(time!=timeInput_) computeInput(time);
        input=convertVector<dynamicgraph::Vector>(input_);
        return input;
   }

   Vector& EstimatorInterface::getMeasurement(Vector& measurement, const int& time)
   {

        return measurement;
   }

   unsigned& EstimatorInterface::getContactsNbr(unsigned& contactsNbr, const int& time)
   {
        if(time!=timeStackOfContacts_) computeStackOfContacts(time);
        contactsNbr = stackOfContacts_.size();
        return contactsNbr;
   }

   unsigned& EstimatorInterface::getModeledContactsNbr(unsigned& modeledContactsNbr, const int& time)
   {
        if(time!=timeStackOfContacts_) computeStackOfContacts(time);
        modeledContactsNbr = stackOfModeledContacts_.size();
        return modeledContactsNbr;
   }

}

