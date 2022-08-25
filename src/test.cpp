#include <iostream>
#include "../include/offb_ctrl.h"
#include <Eigen/Dense>
#include <unistd.h>
#include <chrono>

int main(int argc, char** argv)
{
    double voltage = 11.1;
    if(argc > 1)
        voltage = std::stod(argv[1]);
    double mass = 195.5; 
    Eigen::Vector3d pos_gain = Eigen::Vector3d(1.0, 1.0, 1.0);
    Eigen::Vector3d vel_gain = Eigen::Vector3d(1.0, 1.0, 1.0); 
    std::vector<double> thrust_coeff{202.33, 145.56, -8.0219};
    std::vector<double> volt_coeff{-0.1088, 2.1964};
    double max_acc{10.0};
    double throttle_offset{0.06};
    double throttle_limit{0.5};
    std::vector<double> thrust_original{0.02, 0.01};
    offboard_controller::OffbCtrl controller(mass, pos_gain, vel_gain, thrust_coeff, volt_coeff, max_acc, throttle_offset, throttle_limit, thrust_original);

    // std::cout << "mass is: " << controller.getMass() << std::endl;
    // std::cout << "pos gain is: " << controller.getPosGains() << std::endl;
    // std::cout << "vel gain is: " << controller.getVelGains() << std::endl;
    // std::cout << "thrust coeff is: " << controller.getThrustCoeff() << std::endl;
    // std::cout << "volt coeff is: " << controller.getVoltCoeff() << std::endl;
    // std::cout << "getThrottleOffset is: " << controller.getThrottleOffset() << std::endl;
    // std::cout << "getMaxAcc is: " << controller.getMaxAcc() << std::endl;

    Eigen::Vector3d acc_des;
    Eigen::Vector4d att_des;
    double throttle_des;

    bool exit=false;
    auto runtime_start = std::chrono::steady_clock::now();

    while(!exit)
    {
        auto loop_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = loop_time-runtime_start;
        if(elapsed_seconds.count() > 100)
            exit = true;

        
        acc_des = controller.calDesiredAcceleration(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0));
        att_des = controller.calDesiredAttitude(acc_des, 0.0);
        throttle_des = controller.calDesiredThrottle(acc_des, Eigen::Vector4d(0.0,0.0,0.0,-1), voltage, true);
        std::cout << "computed throttle is " << throttle_des << std::endl;
        sleep(1);
    }


    return 0;
}