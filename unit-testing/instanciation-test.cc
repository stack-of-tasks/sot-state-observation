#include <iostream>

#include <sot-state-observation/dg-imu-attitude-estimation.hh>
#include <sot-state-observation/dg-imu-flexibility-estimation.hh>


using namespace sotStateObservation;

struct instanciator
{
    instanciator():
        f("Hey")
        ,
        a("Ho")
    {
        std::cout << "Instanciation succeeded" << std::endl;


    }

    DGIMUFlexibilityEstimation f;
    DGIMUAttitudeEstimation a;
};

int main()
{
    instanciator i;
    return 0;
}
