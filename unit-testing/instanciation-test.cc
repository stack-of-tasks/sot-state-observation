#include <iostream>

#include <sot-state-observation/dg-imu-attitude-estimation.hh>
#include <sot-state-observation/dg-imu-flexibility-estimation.hh>
#include <sot-state-observation/moving-frame-transformation.hh>


using namespace sotStateObservation;

struct instanciator
{
    instanciator():
        f("Hey")
        ,
        a("Ho")
        ,
        t("Hu")
    {
        std::cout << "Instanciation succeeded" << std::endl;


    }

    DGIMUFlexibilityEstimation f;
    DGIMUAttitudeEstimation a;
    MovingFrameTransformation t;
};

int main()
{
    instanciator i;
    return 0;
}
