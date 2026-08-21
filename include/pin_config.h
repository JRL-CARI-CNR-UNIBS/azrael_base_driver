#ifndef PINS_HH
#define PINS_HH

//PINS
constexpr int PWM_dir_1 = 24;   
constexpr int PWM_dir_2 = 25;   

constexpr int PWM_dir_3 = 28;   
constexpr int PWM_dir_4 = 29;   

constexpr int PWM_pin_1 = 22;   
constexpr int PWM_pin_2 = 23;   

constexpr int PWM_pin_3 = 26;   
constexpr int PWM_pin_4 = 27;   



//MOTION PROFILE

constexpr int MAX_PWM_RANGE = 100;
// constexpr int MIN_VEL_TH    = 2;
// constexpr double max_speed  = 10;
// constexpr double perc_ramp  = 0.4;
// constexpr double acc_delay  = 10;

constexpr double CONTROL_LOOP_DT = 0.002; // s, nominal control loop period (500 Hz)
constexpr double MAX_LIN_ACCEL   = 1.0;   // m/s^2, max accel applied to vx/vy commands
constexpr double MAX_ANG_ACCEL   = 2.0;   // rad/s^2, max accel applied to angular velocity command


#endif
