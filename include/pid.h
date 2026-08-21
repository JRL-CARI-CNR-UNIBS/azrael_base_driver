#include <stddef.h>
#include <stdint.h>
#include <cmath>

// constexpr int ffw    = 0;
constexpr int ffw    = 14;


class PID {
  public:

    PID(double kp, double ki, double kd, double Ts,double* curr_vel_ptr_in,std::mutex* lock_in, double fc=0.0)
        : kp(kp), ki(ki), kd(kd), alpha(calcAlphaEMA(fc * Ts)), Ts(Ts),_lock(lock_in), curr_vel_pnt(curr_vel_ptr_in) {}


    double calcAlphaEMA(double fn)
    {
        if (fn <= 0)
            return 1;
        const double c = std::cos(2 * double(M_PI) * fn);
        return c - 1 + std::sqrt(c * c - 4 * c + 3);
    }

    double update_state() {
            _lock->lock();
            double error = setpoint - std::abs(*curr_vel_pnt);
            _lock->unlock();
            double ef = alpha * error + (1 - alpha) * old_ef;
            double derivative = (ef - old_ef) / Ts;
            double new_integral = integral + error * Ts;

            double control_u =  (ffw * setpoint) +  kp * error + ki * integral + kd * derivative;

            // Clamp the output
            if (control_u > max_output)
            {
                control_u = max_output;
            }
            else if (control_u < 0)
            {
                control_u = 0;
            }
            else
            {
                integral = new_integral;
            }
            old_ef = ef;

            if (control_u < 0)
            {
                std::cout << "FORCING CONTROL TO 0 from  " << (ffw * setpoint) << " " << control_u << "\n";
                control_u = (ffw * setpoint);
            }

            return control_u;
        }

    void setSetpoint(double setpoint) {
            this->setpoint = setpoint;
        }

    private:
        double setpoint = 0;
        double kp, ki, kd, alpha, Ts;
        double max_output = 90;
        double integral = 0;
        double old_ef = 0;
        double* curr_vel_pnt  = nullptr;
        std::mutex* _lock;
};
