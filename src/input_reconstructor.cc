#include <sstream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-bind.h>

#include <sot-state-observation/input_reconstructor.hh>


// basic file operations
//#include <iostream>
//#include <fstream>
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
        inputSOUT("InputReconstructor("+inName+")::output(vector)::input"),
        lastInertia_(6), config_(3),derivateInertiaFD_(false)
    {
        bias_[0].resize(6);
        bias_[1].resize(6);

        bias_[0].setZero();
        bias_[1].setZero();
        lastInertia_.setZero();

        currentTime = 0;

        dt_=5e-3;


        inputSOUT.addDependency(  comVectorSIN );
        inputSOUT.addDependency(  inertiaSIN );
        inputSOUT.addDependency(  dinertiaSIN );
        inputSOUT.addDependency( positionWaistSIN );
        inputSOUT.addDependency(  angMomentumSIN );
        inputSOUT.addDependency(  dangMomentumSIN );
        inputSOUT.addDependency(  imuVectorSIN );
        inputSOUT.addDependency(  nbContactsSIN );
        inputSOUT.addDependency(  contactsPositionSIN );
        inputSOUT.addDependency(  inputSOUT);




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
        nbContactsSIN.setConstant(1);

        signalRegistration (contactsPositionSIN);
        dynamicgraph::Vector contactsPosition(6);
        contactsPositionSIN.setConstant(contactsPosition);


        std::string docstring;


        //setMeasurementNoiseCovariance
        docstring =
                "\n"
                "    Set if the derivative of inertia matrix is computed using finite differences"
                "\n";

        addCommand(std::string("setFDInertiaDot"),
	     new
	     ::dynamicgraph::command::Setter <InputReconstructor,bool>
                (*this, &InputReconstructor::setFDInertiaDot, docstring));

        //setMeasurementNoiseCovariance
        docstring =
                "\n"
                "    Set bias1"
                "\n";

        addCommand(std::string("setFootBias1"),
	     new
	     ::dynamicgraph::command::Setter <InputReconstructor,dynamicgraph::Vector>
                (*this, &InputReconstructor::setFootBias1, docstring));

        //setMeasurementNoiseCovariance
        docstring =
                "\n"
                "    Set bias2"
                "\n";

        addCommand(std::string("setFootBias2"),
	     new
	     ::dynamicgraph::command::Setter <InputReconstructor,dynamicgraph::Vector>
                (*this, &InputReconstructor::setFootBias2, docstring));

                //setMeasurementNoiseCovariance
        docstring =
                "\n"
                "    Set sampling period"
                "\n";

        addCommand(std::string("setSamplingPeriod"),
	     new
	     ::dynamicgraph::command::Setter <InputReconstructor,double>
                (*this, &InputReconstructor::setSamplingPeriod, docstring));

        docstring =
                "\n"
                "    Set config"
                "\n";

        addCommand(std::string("setConfig"),
             new
             ::dynamicgraph::command::Setter <InputReconstructor,dynamicgraph::Vector>
                (*this, &InputReconstructor::setConfig, docstring));

        docstring =
                "\n"
                "    Set lastInertia_\n"
                "\n";

        addCommand(std::string("setLastInertia"),
             new
             ::dynamicgraph::command::Setter <InputReconstructor,dynamicgraph::Matrix>
                (*this, &InputReconstructor::setLastInertia, docstring));




//        // Output that is input
        signalRegistration (inputSOUT);

        inputSOUT.setFunction(boost::bind(&InputReconstructor::computeInput, this, _1, _2));

        stateObservation::Vector3 True;
        True.setOnes();
        setConfig(convertVector<dynamicgraph::Vector>(True));
    }

    InputReconstructor::~InputReconstructor()
    {
    }

    void InputReconstructor::computeInert(const dynamicgraph::Matrix & inertia, const dynamicgraph::Matrix & homoWaist, dynamicgraph::Vector& inert, const dynamicgraph::Vector& comVector)
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

    void InputReconstructor::computeInertDot
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

    dynamicgraph::Vector& InputReconstructor::computeInput(dynamicgraph::Vector & input, const int& inTime)
    {
        if (currentTime==inTime)
        {
          input =  inputSOUT.accessCopy();

          return input;
        }

        currentTime = inTime;

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


        input.resize(42+6*nbContacts,true);
        input.setZero();

        for(i=0;i<3;++i){
            input.elementAt(u)=comVector(i);
            u++;
        }

        for(i=0;i<3;++i){
            if(config_[0] & config_[1]) input.elementAt(u)=comVector(i+3);
            u++;
        }

        for(i=0;i<3;++i){
            if(config_[0] & config_[1] & config_[2]) input.elementAt(u)=comVector(i+6);
            u++;
        }

        for(i=0;i<6;++i){
            input.elementAt(u)=inert(i);
            u++;
        }

        for(i=0;i<6;++i){
            if(config_[0] & config_[1]) input.elementAt(u)=dinert(i);
            u++;
        }

        double m=inertia(0,0);

        dynamicgraph::Vector angMomentumOut, dangMomentumOut;
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

        angMomentumOut=angMomentum;
        dangMomentumOut = m*crossProduct(com,comddot);

        for(i=0;i<3;++i){
            if(config_[0] & config_[1]) input.elementAt(u)=angMomentumOut(i);
            u++;
        }


        for(i=0;i<3;++i){
            if(config_[0] & config_[1] & config_[2]) input.elementAt(u)=dangMomentumOut(i);
            u++;
        }

        for(i=0;i<6;++i){
            input.elementAt(u)=imuVector(i);
            u++;
        }

        for(i=0;i<6;++i){
            if(config_[0] & config_[1]) input.elementAt(u)=imuVector(i+6);
            u++;
        }

        for(i=0;i<3;++i){
            if(config_[0] & config_[1] & config_[2]) input.elementAt(u)=imuVector(i+12);
            u++;
        }

        for(i=0;i<6*nbContacts;++i){

            input.elementAt(u)=contactsPosition(i)+bias_[i/6](i%6);
            u++;
        }

        return input;

    }

}

