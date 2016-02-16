#include <sstream>
#include <numeric>

#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>

#include <dynamic-graph/factory.h>
#include <jrl/mal/matrixabstractlayervector3jrlmath.hh>
#include <sot-state-observation/position-state-reconstructor.hh>



namespace sotStateObservation
{
    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN ( PositionStateReconstructor,
                                "PositionStateReconstructor" );

    PositionStateReconstructor::PositionStateReconstructor
                                        ( const std::string & inName):
        lastVector_(18),
        Entity(inName),
        dt_(0.005),
        inputSIN(0x0 ,
                "PositionStateReconstructor("+inName+")::input(vector)::sin"),
        inputFormatSIN(0x0 ,
                "PositionStateReconstructor("+inName+")::input(flags)::inputFormat"),
        outputFormatSIN(0x0 ,
                "PositionStateReconstructor("+inName+")::input(flags)::outputFormat"),
        outputSOUT( inputSIN<<inputFormatSIN<<outputFormatSIN,
                "PositionStateReconstructor("+inName+")::output(vector)::sout")
    {
        derivationNumberOfSamples_=1;

        signalRegistration (inputSIN);
        signalRegistration (inputFormatSIN);
        signalRegistration (outputFormatSIN);
        signalRegistration (outputSOUT);

        lastVector_.setZero();

        ::dynamicgraph::Vector initV(3);
        initV.setZero();

        linearVelocities_.push_back(initV);
        angularVelocities_.push_back(initV);
        linearAccelerations_.push_back(initV);
        angularAccelerations_.push_back(initV);

        dynamicgraph::Vector input(size_t(0));
        dynamicgraph::Vector ouput(size_t(0));

        inputSIN.setConstant(input);
        outputSOUT.setConstant(ouput);

        outputSOUT.setFunction(boost::bind(&PositionStateReconstructor::computeOutput,
				    this, _1, _2));

        std::string docstring;

        //setCurrentValue
        docstring =
                "\n"
                "    Sets the current value of the vector.\n"
                "    Its the basis of the computation of \n"
                "    the derivations and integration of the output vector\n"
                "    (takes a 18x1 vector)\n"
                "\n";

        addCommand(std::string("setCurrentValue"),
	     new
	     dynamicgraph::command::Setter <PositionStateReconstructor,dynamicgraph::Vector>
            (*this, &PositionStateReconstructor::setCurrentValue, docstring));

        //setSampligPeriod
        docstring =
                "\n"
                "    Sets the sampling period.\n"
                "    takes a floating point number\n"
                "\n";

        addCommand(std::string("setSampligPeriod"),
	     new
	     dynamicgraph::command::Setter <PositionStateReconstructor,double>
            (*this, &PositionStateReconstructor::setSampligPeriod, docstring));

        //getSampligPeriod
        docstring =
                "\n"
                "    Gets the sampling period.\n"
                "    gives a floating point number\n"
                "\n";

        addCommand(std::string("getSampligPeriod"),
	     new
	     dynamicgraph::command::Getter <PositionStateReconstructor,double>
            (*this, &PositionStateReconstructor::getSampligPeriod, docstring));

        //setFiniteDifferencesInterval
        docstring =
                "\n"
                "    Sets the number of sampling periods which \n"
                "    defines the finite difference derivation interval.\n"
                "    takes an integer number\n"
                "\n";

        addCommand(std::string("setFiniteDifferencesInterval"),
	     new
	     dynamicgraph::command::Setter <PositionStateReconstructor,int>
            (*this, &PositionStateReconstructor::setFiniteDifferencesInterval, docstring));

        //getFiniteDifferencesInterval
        docstring =
                "\n"
                "    Gets the number of sampling periods which \n"
                "    defines the finite difference derivation interval.\n"
                "    takes an integer number\n"
                "\n";

        addCommand(std::string("getFiniteDifferencesInterval"),
	     new
	     dynamicgraph::command::Getter <PositionStateReconstructor,int>
            (*this, &PositionStateReconstructor::getFiniteDifferencesInterval, docstring));



    }

    PositionStateReconstructor::~PositionStateReconstructor()
    {
    }

    stateObservation::Vector PositionStateReconstructor::averageDistribution(const unsigned n)
    {
        vec_.resize(n);
        vec_.setOnes();
        vec_=double(1./double(n))*vec_;
        return vec_;
    }

    stateObservation::Vector PositionStateReconstructor::gaussianDistribution(const unsigned n)
    {
        double mean=0;
        double stddev=std::sqrt((n-mean)*(n-mean)/4.6); // the first element of the window correspond to 10% of the last element.

        vec_.resize(n);
        double sum=0.;
        for(int i=0; i<vec_.size();++i)
        {
            vec_[i]=(1./(stddev*std::sqrt(6.28)))*std::exp(-0.5*((i-mean)/stddev)*((i-mean)/stddev));
            sum+=vec_[i];
        }
        vec_=(1./sum)*vec_;
        return vec_;
    }

    dynamicgraph::Vector& PositionStateReconstructor::computeOutput
                (dynamicgraph::Vector & output, const int& inTime)
    {

        dynamicgraph::Vector inputSig (inputSIN(inTime));

        dynamicgraph::sot::Flags inputFormat (inputFormatSIN(inTime));
        dynamicgraph::sot::Flags outputFormat (outputFormatSIN(inTime));

        dynamicgraph::Vector input (18);
        dynamicgraph::Vector zero (3);

        zero.setZero();

        size_t inputIndex = 0;
        for (size_t i=0; i<6; ++i)
        {
            if (inputFormat(i))
            {
                setSubvector(input, 3*i, getSubvector(inputSig,inputIndex,3));
                inputIndex += 3;
            }
            else
            {
                setSubvector(input, 3*i, zero);
            }

        }

        size_t outputIndex = 0;
        size_t outputSize = 0;

        /////////////////////
        //Linear position
        dynamicgraph::Vector position(3);
        bool posSet=true;

        if (inputFormat(0))
        {
            position = getSubvector(input,0,3);
        }
        else
        {
            if (inputFormat(2))
            {
                position = getSubvector(lastVector_,0,3)
                            + dt_*getSubvector(input,6,3);
            }
            else
            {
                if (inputFormat(4))
                    position = getSubvector(lastVector_,0,3)
                            + dt_*getSubvector(lastVector_,6,3)
                                + 0.5*dt_*dt_*getSubvector(input,12,3);
                else
                {
                    position.setZero();
                    posSet=false;
                }
            }
        }
        if (outputFormat(0))
        {
            if (posSet)
            {
                outputSize +=3;
                output.resize(outputSize,false);
                setSubvector(output, outputIndex, position);
                outputIndex +=3;
            }
            else
                throw std::runtime_error
                        ("There is nothing to reconstruct the position !");
        }

        /////////////////////
        //orientation
        dynamicgraph::Vector orientation(3);
        bool oriSet=true;

        if (inputFormat(1))
                orientation = getSubvector(input,3,3);
        else
        {
            if (inputFormat(3))
            {
                stateObservation::Vector
                        velocity
                            (convertVector<stateObservation::Vector>
                                (getSubvector(input,9,3)));

                stateObservation::Vector
                        lastOrientation
                            (convertVector<stateObservation::Vector>
                                (getSubvector(lastVector_,3,3)));

                stateObservation::Quaternion
                    dr (stateObservation::kine::rotationVectorToAngleAxis
                                                    (velocity*dt_));

                stateObservation::AngleAxis
                    r (dr* stateObservation::kine::rotationVectorToAngleAxis
                                                    (lastOrientation));

                orientation=convertVector<dynamicgraph::Vector>(r.angle()*r.axis());
            }
            else
            {
                if (inputFormat(5))
                {
                    stateObservation::Vector
                        velocity
                            (convertVector<stateObservation::Vector>
                                (dt_ *getSubvector(input,15,3)+
                                        getSubvector(lastVector_,9,3)));

                    stateObservation::Vector
                        lastOrientation
                            (convertVector<stateObservation::Vector>
                                (getSubvector(lastVector_,3,3)));

                    stateObservation::Quaternion
                        dr (stateObservation::kine::rotationVectorToAngleAxis
                                                (velocity*dt_));

                    stateObservation::AngleAxis
                        r (dr* stateObservation::kine::rotationVectorToAngleAxis
                                                (lastOrientation));

                    orientation= convertVector<dynamicgraph::Vector>(r.angle()*r.axis());
                }
                else
                {
                    orientation.setZero();
                    oriSet=false;
                }
            }
        }
        if (outputFormat(1))
        {
            if (oriSet)
            {
                outputSize +=3;
                output.resize(outputSize,false);
                setSubvector(output, outputIndex, orientation);
                outputIndex +=3;
            }
            else
                throw std::runtime_error
                        ("There is nothing to reconstruct the orientation !");
        }

        /////////////////////
        //Linear velocity
        dynamicgraph::Vector linearVelocity(3);
        dynamicgraph::Vector curfddLinVel(3);
        bool linVelSet=true;

        if (inputFormat(2))
            linearVelocity = getSubvector(input,6,3);
        else
        {
            if (inputFormat(0))
            {
                curfddLinVel = stateObservation::tools::derivate
                        (getSubvector(lastVector_,0,3),getSubvector(input,0,3),dt_) ;

                linearVelocities_.push_back(curfddLinVel);

                while (linearVelocities_.size()>derivationNumberOfSamples_)
                    linearVelocities_.pop_front();

                linearVelocity = 1.0 /linearVelocities_.size()*
                    std::accumulate(linearVelocities_.begin(),
                        linearVelocities_.end(),zero);
            }
            else
            {
                if (inputFormat(4))
                    linearVelocity= getSubvector(lastVector_,6,3)
                                        + dt_*getSubvector(input,12,3);
                else
                {
                    linearVelocity.setZero();
                    linVelSet=false;
                }
            }
        }
        if (outputFormat(2))
        {
            if (linVelSet)
            {
                outputSize +=3;
                output.resize(outputSize,false);
                setSubvector(output, outputIndex, linearVelocity);
                outputIndex +=3;
            }
            else
                throw std::runtime_error
                        ("There is nothing to reconstruct the linear velocity !");
        }

        /////////////////////
        //Angular velocity
        dynamicgraph::Vector angularVelocity(3);
        dynamicgraph::Vector curfddAngVel(3);
        bool angVelSet = true;

        if (inputFormat(3))
            angularVelocity= getSubvector(input,9,3);
        else
        {
            if (inputFormat(1))
            {
                curfddAngVel=convertVector<dynamicgraph::Vector>(
                            stateObservation::kine::derivateRotationFD
                        (convertVector<stateObservation::Vector>(getSubvector(lastVector_,3,3)),
                         convertVector<stateObservation::Vector>(getSubvector(input,3,3)),
                        dt_) );

                angularVelocities_.push_back(curfddAngVel);

                while (angularVelocities_.size()>derivationNumberOfSamples_)
                    angularVelocities_.pop_front();

                angularVelocity = 1.0 / angularVelocities_.size() *
                    std::accumulate( angularVelocities_.begin(),
                        angularVelocities_.end(),zero) ;
            }
            else
            {
                if (inputFormat(5))
                    angularVelocity= getSubvector(lastVector_,9,3)
                                        + dt_*getSubvector(input,15,3);
                else
                {
                    angularVelocity.setZero();
                    angVelSet=false;
                }
            }
        }
        if (outputFormat(3))
        {
            if (angVelSet)
            {
                outputSize +=3;
                output.resize(outputSize,false);
                setSubvector(output, outputIndex, angularVelocity);
                outputIndex +=3;
            }
            else
                throw std::runtime_error
                        ("There is nothing to reconstruct the angular velocity !");
        }

        /////////////////////
        //Linear acceleration
        dynamicgraph::Vector linearAcceleration(3);
        dynamicgraph::Vector curfddLinAcc(3);
        bool linAccSet = true;

        if (inputFormat(4))
            linearAcceleration= getSubvector(input,12,3);
        else
        {
            if (linVelSet)
            {
                curfddLinAcc = stateObservation::tools::derivate
                        (getSubvector(lastVector_,6,3), linearVelocity, dt_);

                linearAccelerations_.push_back(curfddLinAcc);

                while (linearAccelerations_.size()>derivationNumberOfSamples_)
                        linearAccelerations_.pop_front();

                linearAcceleration = 1.0 / linearAccelerations_.size() *
                    std::accumulate(linearAccelerations_.begin(),
                        linearAccelerations_.end(), zero);
            }
            else
            {
                linearAcceleration.setZero();
                linAccSet=false;
            }
        }
        if (outputFormat(4))
        {
            if (linAccSet)
            {
                outputSize +=3;
                output.resize(outputSize,false);
                setSubvector(output, outputIndex, linearAcceleration);
                outputIndex +=3;
            }
            else
                throw std::runtime_error
                    ("There is nothing to reconstruct the linear acceleration !");
        }

        /////////////////////
        //Angular acceleration
        dynamicgraph::Vector angularAcceleration(3);
        dynamicgraph::Vector curfddAngAcc(3);
        bool angAccSet = true;

        if (inputFormat(5))
            angularAcceleration = getSubvector(input,15,3);
        else
        {
            if (angVelSet)
            {
                    curfddAngAcc = stateObservation::tools::derivate
                        (getSubvector(lastVector_,9,3), angularVelocity, dt_);

                    angularAccelerations_.push_back(curfddAngAcc);

                    while (angularAccelerations_.size()>derivationNumberOfSamples_)
                        angularAccelerations_.pop_front();

                    angularAcceleration = 1.0 / angularAccelerations_.size() *
                        std::accumulate( angularAccelerations_.begin(),
                           angularAccelerations_.end(), zero);
            }
            else
            {
                angularAcceleration.setZero();
                angAccSet=false;
            }
        }
        if (outputFormat(5))
        {
            if (angAccSet)
            {
                outputSize +=3;
                output.resize(outputSize,false);
                setSubvector(output, outputIndex, angularAcceleration);
                outputIndex +=3;
            }
            else
                throw std::runtime_error
                        ("There is nothing to reconstruct the angular acceleration !");
        }

        setSubvector(lastVector_, 0, position);
        setSubvector(lastVector_, 3, orientation);
        setSubvector(lastVector_, 6, linearVelocity);
        setSubvector(lastVector_, 9, angularVelocity);
        setSubvector(lastVector_, 12, linearAcceleration);
        setSubvector(lastVector_, 15, angularAcceleration);

        return output;

    }
}
