#ifndef DR_HH
#define DR_HH

#include <iostream>
#include <csignal>
#include <signal.h>
#include <chrono>
#include <thread>
#include <wiringPi.h>
#include <softPwm.h>
#include <thread>
#include <phidget22.h>
#include <mutex>
#include <malloc.h>
#include <limits.h>
#include <sys/mman.h>  
#include <cmath>
#include <pid.h>

#include <iostream>
#include <fstream>

#include <stdio.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <string.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 
#include "Iir.h"
#include "random"
#include "time.h"

#include <ctime>
#include <iostream>
#include <string>
#include <boost/array.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::udp;
using boost::asio::ip::address;

std::ofstream myfile;

#define IPADDRESS_REMOTE "192.169.1.2"
#define IPADDRESS_LOCAL "192.169.1.1"
#define UDP_PORT 44100

// #define DEBUG_PID

std::mutex mtx_enc1, mtx_enc2, mtx_enc3, mtx_enc4 ; 
bool stopping_ = false;

constexpr double radius    = 0.1016;
constexpr double lxy       = 0.71;
// constexpr double lxy       = -0.11;

class azrael_mobile_driver
{

    // safety data
    int safe_ = 1;

    // acceleration-limited body-frame velocity command (ramped toward buffer_in each control cycle)
    double cmdvel_x = 0.0;
    double cmdvel_y = 0.0;
    double cmdvel_z = 0.0;


    double vel_enc1 = 0.0, vel_enc2 = 0.0, vel_enc3 = 0.0, vel_enc4 = 0.0;
    double vel_enc1_f = 0.0, vel_enc2_f = 0.0, vel_enc3_f = 0.0, vel_enc4_f = 0.0;

    Iir::Butterworth::LowPass<2> f_vel_1;
    Iir::Butterworth::LowPass<2> f_vel_2;
    Iir::Butterworth::LowPass<2> f_vel_3;
    Iir::Butterworth::LowPass<2> f_vel_4;

    // PID pid_w1 = PID(5,30.0,0.,0.001,&vel_enc1,&mtx_enc1);
    // PID pid_w2 = PID(5,30.0,0.,0.001,&vel_enc2,&mtx_enc2);
    // PID pid_w3 = PID(5,30.0,0.,0.001,&vel_enc3,&mtx_enc3);
    // PID pid_w4 = PID(5,30.0,0.,0.001,&vel_enc4,&mtx_enc4);

    PID pid_w1 = PID(10,5.0,0.,0.001,&vel_enc1_f,&mtx_enc1);
    PID pid_w2 = PID(10,5.0,0.,0.001,&vel_enc2_f,&mtx_enc2);
    PID pid_w3 = PID(10,5.0,0.,0.001,&vel_enc3_f,&mtx_enc3);
    PID pid_w4 = PID(10,5.0,0.,0.001,&vel_enc4_f,&mtx_enc4);

    double velx_odom = 0.0;
    double vely_odom = 0.0;
    double velw_odom = 0.0;
    double posx_odom = 0.0;
    double posy_odom = 0.0;
    double posw_odom = 0.0;


    int state1 = HIGH;
    int state2 = HIGH;
    int state3 = HIGH;
    int state4 = HIGH;
    
    PhidgetEncoderHandle encoder0;
    PhidgetEncoderHandle encoder1;
    PhidgetEncoderHandle encoder2;
    PhidgetEncoderHandle encoder3;

    double v1in_ = 0.0;
    double v2in_ = 0.0;
    double v3in_ = 0.0;
    double v4in_ = 0.0;

    //Socket
    // int sockfd; 
    double buffer_in[3] = {0.0,0.0,0.0};
    // [vel1,vel2,vel3,vel4,pwm1,pwm2,pwm3,pwm4] - vel in rad/s, pwm as commanded duty cycle fraction (0-1)
    double buffer_out[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    // struct sockaddr_in     servaddr; 
    std::mutex mtx_receive_;
    
    boost::asio::io_context io_context;
   
    udp::socket * socket;
    udp::endpoint remote_endpoint;
    udp::endpoint local_endpoint;


public:
  
    // void pins_setup();
    azrael_mobile_driver();
    ~azrael_mobile_driver();

    void setWheelsSpeed();
    void control_thread();
    void odometry();
    void socket_send();
    void socket_receive();

    //   void safety_task();


};

azrael_mobile_driver* mobile_robot_driver;


#endif
