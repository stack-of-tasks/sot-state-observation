#include <sstream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-bind.h>

#include <sot-state-observation/input_reconstructor.hh>


// basic file operations
#include <iostream>
#include <fstream>
using namespace std;


namespace sotStateObservation
{
    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN ( InputReconstructor, "InputReconstructor" );

    InputReconstructor::InputReconstructor( const std::string & inName):
        Entity(inName),
        comVectorSIN(0x0 , "InputReconstructor("+inName+")::input(vector)::comVector"),
        inertiaSIN(0x0 , "InputReconstructor("+inName+")::input(matrix)::inertia"),
        dinertiaSIN(0x0 , "InputReconstructor("+inName+")::input(vector)::dinertia"),
        positionWaistSIN(0x0 , "InputReconstructor("+inName+")::input(matrix)::positionWaist"),
        angMomentumSIN(0x0 , "InputReconstructor("+inName+")::input(vector)::angMomentum"),
        dangMomentumSIN(0x0 , "InputReconstructor("+inName+")::input(vector)::dangMomentum"),
        imuVectorSIN(0x0 , "InputReconstructor("+inName+")::input(vector)::imuVector"),
        nbContactsSIN(0x0 , "InputReconstructor("+inName+")::input(unsigned)::nbContacts"),
        contactsPositionSIN(0x0 , "InputReconstructor("+inName+")::input(vector)::contactsPosition"),
        inputSOUT("InputReconstructor("+inName+")::output(vector)::input")
    {
        // Input signal
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

        signalRegistration (nbContactsSIN);

        signalRegistration (contactsPositionSIN);

        // Output that is input
        signalRegistration (inputSOUT);
        dynamicgraph::Vector input(42);
        inputSOUT.setConstant(input);

        inputSOUT.setFunction(boost::bind(&InputReconstructor::computeInput, this, _1, _2));

//        std::cout << "input reconstructor" << std::endl;
    }

    InputReconstructor::~InputReconstructor()
    {
    }

    void InputReconstructor::computeInert(const dynamicgraph::Matrix & inertia, const dynamicgraph::Matrix & homoWaist, dynamicgraph::Vector& inert, const dynamicgraph::Vector& comVector)
    {

        double m=inertia(0,0); //<=== donne 56.8;
        //std::cout << "Masse=" << m << std::endl;

        dynamicgraph::Vector waist, com, localframe;
        waist.resize(3);
        com.resize(3);
        localframe.resize(3);

        waist.elementAt(0)=homoWaist(0,3);
        waist.elementAt(1)=homoWaist(1,3);
        waist.elementAt(2)=homoWaist(2,3);

        com.elementAt(0)=comVector(0);
        com.elementAt(1)=comVector(1);
        com.elementAt(2)=comVector(2);

        localframe.elementAt(0)=0;
        localframe.elementAt(1)=0;
        localframe.elementAt(2)=0;

        // Inertia expressed at waist
        inert.elementAt(0)=inertia(3,3);
        inert.elementAt(1)=inertia(4,4);
        inert.elementAt(2)=inertia(5,5);
        inert.elementAt(3)=inertia(3,4);
        inert.elementAt(4)=inertia(3,5);
        inert.elementAt(5)=inertia(4,5);

               // std::cout << "\n\n\n\n\n INERTIA="<< inert << "\n\n\n\n\n" << std::endl;

        // From waist to com
        inert.elementAt(0) -= -m*((com.elementAt(1)-waist.elementAt(1))*(com.elementAt(1)-waist.elementAt(1))+(com.elementAt(2)-waist.elementAt(2))*(com.elementAt(2)-waist.elementAt(2)));
        inert.elementAt(1) -= -m*((com.elementAt(0)-waist.elementAt(0))*(com.elementAt(0)-waist.elementAt(0))+(com.elementAt(2)-waist.elementAt(2))*(com.elementAt(2)-waist.elementAt(2)));
        inert.elementAt(2) -= -m*((com.elementAt(0)-waist.elementAt(0))*(com.elementAt(0)-waist.elementAt(0))+(com.elementAt(1)-waist.elementAt(1))*(com.elementAt(1)-waist.elementAt(1)));
        inert.elementAt(3) -= m*(com.elementAt(1)-waist.elementAt(1))*(com.elementAt(2)-waist.elementAt(2));
        inert.elementAt(4) -= m*(com.elementAt(0)-waist.elementAt(0))*(com.elementAt(2)-waist.elementAt(2));
        inert.elementAt(5) -= m*(com.elementAt(0)-waist.elementAt(0))*(com.elementAt(1)-waist.elementAt(1));

             //   std::cout << "\n\n\n\n\n INERTIA="<< inert << "\n\n\n\n\n" << std::endl;

        // From com to local frame
        inert.elementAt(0) -= -m*((localframe.elementAt(1)-com.elementAt(1))*(localframe.elementAt(1)-com.elementAt(1))+(localframe.elementAt(2)-com.elementAt(2))*(localframe.elementAt(2)-com.elementAt(2)));
        inert.elementAt(1) -= -m*((localframe.elementAt(0)-com.elementAt(0))*(localframe.elementAt(0)-com.elementAt(0))+(localframe.elementAt(2)-com.elementAt(2))*(localframe.elementAt(2)-com.elementAt(2)));
        inert.elementAt(2) -= -m*((localframe.elementAt(0)-com.elementAt(0))*(localframe.elementAt(0)-com.elementAt(0))+(localframe.elementAt(1)-com.elementAt(1))*(localframe.elementAt(1)-com.elementAt(1)));
        inert.elementAt(3) -= m*(localframe.elementAt(1)-com.elementAt(1))*(localframe.elementAt(2)-com.elementAt(2));
        inert.elementAt(4) -= m*(localframe.elementAt(0)-com.elementAt(0))*(localframe.elementAt(2)-com.elementAt(2));
        inert.elementAt(5) -= m*(localframe.elementAt(0)-com.elementAt(0))*(localframe.elementAt(1)-com.elementAt(1));

          //      std::cout << "\n\n\n\n\n INERTIA="<< inert << "\n\n\n\n\n" << std::endl;


//        inert.elementAt(0)=48.2378;
//        inert.elementAt(1)=48.2378;
//        inert.elementAt(2)=2.87339;
//        inert.elementAt(3)=0.0;
//        inert.elementAt(4)=0.0;
//        inert.elementAt(5)=0.0;
   }

    dynamicgraph::Vector& InputReconstructor::computeInput(dynamicgraph::Vector & input, const int& inTime)
    {

        const dynamicgraph::Matrix& inertia=inertiaSIN.access(inTime);
        const dynamicgraph::Matrix& homoWaist=positionWaistSIN.access(inTime);
        const dynamicgraph::Vector& comVector=comVectorSIN.access(inTime);
        const dynamicgraph::Vector& dinertia=dinertiaSIN.access(inTime);
        const dynamicgraph::Vector& angMomentum=angMomentumSIN.access(inTime);
        const dynamicgraph::Vector& dangMomentum=dangMomentumSIN.access(inTime);
        const dynamicgraph::Vector& imuVector=imuVectorSIN.access(inTime);
        const unsigned& nbContacts=nbContactsSIN.access(inTime);
        const dynamicgraph::Vector& contactsPosition=contactsPositionSIN.access(inTime);

        int i, u=0,k;

        dynamicgraph::Vector inert;
        inert.resize(6);

        computeInert(inertia,homoWaist,inert,comVector);
//        std::cout << "Com: "<< comVector << std::endl;
//        std::cout << "Inertia: ="<< inert << std::endl;

        input.resize(42+6*nbContacts,true);
        for(i=0;i<9;++i){
            input.elementAt(u)=comVector(i);
            u++;
        }

        for(i=0;i<6;++i){
            input.elementAt(u)=inert(i);
            u++;
        }

        for(i=0;i<6;++i){
            input.elementAt(u)=dinertia(i);
            u++;
        }

        for(i=0;i<3;++i){
            input.elementAt(u)=angMomentum(i);
            u++;
        }

        for(i=0;i<3;++i){
            input.elementAt(u)=dangMomentum(i);
            u++;
        }

        for(i=0;i<15;++i){
            input.elementAt(u)=imuVector(i);
            u++;
        }


        for(i=0;i<6*nbContacts;++i){
            input.elementAt(u)=contactsPosition(i);
//            if(i==2)
//            {
//                input.elementAt(u) += -0.0015;
//            }
            u++;
        }

        //cout << "contacts Position: " << contactsPosition << endl;
        //std::cout << "input" << input << std::endl;


        return input;

    }

}

