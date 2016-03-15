//
// Copyright (c) 2016,
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
        unmodeledContactsNbrSOUT_ (NULL, "EstimatorInterface("+inName+")::output(unsigned)::unmodeledContactsNbr"),
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
        accelerometerSIN(NULL , "EstimatorInterface("+inName+")::input(vector)::accelerometer"),
        gyrometerSIN(NULL , "EstimatorInterface("+inName+")::input(vector)::gyrometer"),
        timeStackOfContacts_(0), timeInput_(0), timeMeasurement_(0),
        inputForces_(contact::nbMax),
        inputPosition_(contact::nbMax),
        inputHomoPosition_(contact::nbMax),
        bias_(contact::nbMax)
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

        signalRegistration (accelerometerSIN);
        dynamicgraph::Vector accelerometer(3);
        accelerometerSIN.setConstant(accelerometer);

        signalRegistration (gyrometerSIN);
        dynamicgraph::Vector gyrometer(3);
        gyrometerSIN.setConstant(gyrometer);

        // Output
        signalRegistration (inputSOUT_);
        inputSOUT_.setFunction(boost::bind(&EstimatorInterface::getInput, this, _1, _2));

        signalRegistration (measurementSOUT_);
        measurementSOUT_.setFunction(boost::bind(&EstimatorInterface::getMeasurement, this, _1, _2));

        signalRegistration (contactsNbrSOUT_);
        contactsNbrSOUT_.setFunction(boost::bind(&EstimatorInterface::getContactsNbr, this, _1, _2));

        signalRegistration (modeledContactsNbrSOUT_);
        modeledContactsNbrSOUT_.setFunction(boost::bind(&EstimatorInterface::getModeledContactsNbr, this, _1, _2));

        signalRegistration (unmodeledContactsNbrSOUT_);
        unmodeledContactsNbrSOUT_.setFunction(boost::bind(&EstimatorInterface::getUnmodeledContactsNbr, this, _1, _2));

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

        docstring =
                "\n"
                "    Set if the derivative of inertia matrix is computed using finite differences"
                "\n";

        addCommand(std::string("setFDInertiaDot"),
             new
             ::dynamicgraph::command::Setter <EstimatorInterface,bool>
                (*this, &EstimatorInterface::setFDInertiaDot, docstring));

        docstring =
                "\n"
                "    Set bias for left foot"
                "\n";

        addCommand(std::string("setLeftFootBias"),
             new
             ::dynamicgraph::command::Setter <EstimatorInterface,dynamicgraph::Vector>
                (*this, &EstimatorInterface::setLeftFootBias, docstring));

        docstring =
                "\n"
                "    Set bias for right foot"
                "\n";

        addCommand(std::string("setRightFootBias"),
             new
             ::dynamicgraph::command::Setter <EstimatorInterface,dynamicgraph::Vector>
                (*this, &EstimatorInterface::setRightFootBias, docstring));

        docstring =
                "\n"
                "    Set lastInertia_\n"
                "\n";

        addCommand(std::string("setLastInertia"),
             new
             ::dynamicgraph::command::Setter <EstimatorInterface,dynamicgraph::Matrix>
                (*this, &EstimatorInterface::setLastInertia, docstring));

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

        // From input reconstructor
        for (int i=0; i<contact::nbMax;++i) { bias_[i].resize(6); bias_[i].setZero(); }
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
                    if(modeled_[i]) { stackOfModeledContacts_.remove(i); }
                    if(!modeled_[i]) { stackOfUnmodeledContacts_.remove(i); }
                }
            }

            // Update all contacts numbers.
            contactsNbr_=stackOfContacts_.size();
            modeledContactsNbr_=stackOfModeledContacts_.size();
            unmodeledContactsNbr_=stackOfUnmodeledContacts_.size();
        }
    }

    void EstimatorInterface::computeInert(const stateObservation::Matrix & inertia,
                                          const stateObservation::Matrix & homoWaist,
                                          const stateObservation::Vector& comVector,
                                          stateObservation::Vector& inert)
    {

        double m=inertia(0,0); //<=== donne 56.8;
        //std::cout << "Masse=" << m << std::endl;

        stateObservation::Vector waist=homoWaist.block(0,3,3,1);
        stateObservation::Vector com=comVector.segment(0,3);

        // Inertia expressed at waist
        inert(0)=inertia(3,3);
        inert(1)=inertia(4,4);
        inert(2)=inertia(5,5);
        inert(3)=inertia(3,4);
        inert(4)=inertia(3,5);
        inert(5)=inertia(4,5);

        // From waist to com
        inert(0) += -m*((com(1)-waist(1))*(com(1)-waist(1))+(com(2)-waist(2))*(com(2)-waist(2)));
        inert(1) += -m*((com(0)-waist(0))*(com(0)-waist(0))+(com(2)-waist(2))*(com(2)-waist(2)));
        inert(2) += -m*((com(0)-waist(0))*(com(0)-waist(0))+(com(1)-waist(1))*(com(1)-waist(1)));
        inert(3) += m*(com(0)-waist(0))*(com(1)-waist(1));
        inert(4) += m*(com(0)-waist(0))*(com(2)-waist(2));
        inert(5) += m*(com(1)-waist(1))*(com(2)-waist(2));

        // From com to local frame
        inert(0) -= -m*((com(1))*(com(1))+(com(2))*(com(2)));
        inert(1) -= -m*((com(0))*(com(0))+(com(2))*(com(2)));
        inert(2) -= -m*((com(0))*(com(0))+(com(1))*(com(1)));
        inert(3) -= m*(com(0))*(com(1));
        inert(4) -= m*(com(0))*(com(2));
        inert(5) -= m*(com(1))*(com(2));

    }

   void EstimatorInterface::computeInput(const int& time)
   {
       timeInput_=time;
       if(time!=timeStackOfContacts_) computeStackOfContacts(time);

       const stateObservation::Matrix& inertia=convertMatrix<stateObservation::Matrix>(inertiaSIN.access(time));
       const stateObservation::Matrix& homoWaist=convertMatrix<stateObservation::Matrix>(positionWaistSIN.access(time));
       const stateObservation::Vector& comVector=convertVector<stateObservation::Vector>(comVectorSIN.access(time));
       const stateObservation::Vector& dinertia=convertVector<stateObservation::Vector>(dinertiaSIN.access(time));
       const stateObservation::Vector& angMomentum=convertVector<stateObservation::Vector>(angMomentumSIN.access(time));
       const stateObservation::Vector& dangMomentum=convertVector<stateObservation::Vector>(dangMomentumSIN.access(time));
       const stateObservation::Vector& imuVector=convertVector<stateObservation::Vector>(imuVectorSIN.access(time));

       unsigned contactsNbr;
       const unsigned& nbModeledContacts=getModeledContactsNbr(contactsNbr,time);

       int i, u=0;

       // Modeled contacts position
       if(time!=timeStackOfContacts_) computeStackOfContacts(time);
       stateObservation::Vector contactPosition; contactPosition.resize(nbModeledContacts*6); contactPosition.setZero();
       stateObservation::Vector bias; bias.resize(6*nbModeledContacts); bias.setZero();
       i=0;
       for (iterator = stackOfModeledContacts_.begin(); iterator != stackOfModeledContacts_.end(); ++iterator)
       {
           contactPosition.segment(i*6,6)=inputPosition_[*iterator];
           bias.segment(i*6,6)=bias_[*iterator];
           ++i;
       }

       // Inertia and derivative
       stateObservation::Vector inert,dinert;
       inert.resize(6);
       computeInert(inertia,homoWaist,comVector,inert);
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
         dinert=dinertia;
       lastInertia_ = inert;

       double m=inertia(0,0);

       // Com
       stateObservation::Vector com, comddot;
       com=comVector.segment(0,3);
       comddot=comVector.segment(6,3);

       // Angular momentum and derivative
       stateObservation::Vector dangMomentumOut = m*kine::skewSymmetric(com)*comddot;

       // Concatenate input
       input_.resize(42+6*nbModeledContacts);
       input_.segment(0,9)=comVector;
       input_.segment(9,6)=inert;
       input_.segment(15,6)=dinert;
       input_.segment(21,3)=angMomentum;
       input_.segment(24,3)=dangMomentumOut;
       input_.segment(27,15)=imuVector;
       input_.segment(42,6*nbModeledContacts)=contactPosition+bias;

   }

   void EstimatorInterface::computeMeasurement(const int& time)
   {
       timeMeasurement_=time;
       if(time!=timeStackOfContacts_) computeStackOfContacts(time);

       const stateObservation::Vector& accelerometer=convertVector<stateObservation::Vector>(accelerometerSIN.access(time));
       const stateObservation::Vector& gyrometer=convertVector<stateObservation::Vector>(gyrometerSIN.access(time));

       unsigned contactsNbr;
       const unsigned& nbModeledContacts=getModeledContactsNbr(contactsNbr,time);
       const unsigned& nbUnmodeledContacts=getContactsNbr(contactsNbr,time)-getModeledContactsNbr(contactsNbr,time);

       measurement_.resize(6+nbModeledContacts*6+6); measurement_.setZero();
       measurement_.segment(0,3)=accelerometer;
       measurement_.segment(3,3)=gyrometer;
       int i=0;
       for (iterator = stackOfModeledContacts_.begin(); iterator != stackOfModeledContacts_.end(); ++iterator)
       {
           measurement_.segment(6+i*6,6)=inputForces_[*iterator];
           ++i;
       }

   }

}

