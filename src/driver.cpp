#include<driver.h>
#include<pin_config.h>

void my_handler(int signum){
    stopping_ = true;
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    softPwmWrite(PWM_pin_1,0);
    softPwmWrite(PWM_pin_2,0);
    softPwmWrite(PWM_pin_3,0);
    softPwmWrite(PWM_pin_4,0);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));	
    softPwmWrite(PWM_pin_1,0);
    softPwmWrite(PWM_pin_2,0);
    softPwmWrite(PWM_pin_3,0);
    softPwmWrite(PWM_pin_4,0);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));	
    softPwmStop   (PWM_pin_1) ;
    softPwmStop   (PWM_pin_2) ;
    softPwmStop   (PWM_pin_3) ;
    softPwmStop   (PWM_pin_4) ;
    pinMode(PWM_pin_1,OUTPUT);
    pinMode(PWM_pin_2,OUTPUT);
    pinMode(PWM_pin_3,OUTPUT);
    pinMode(PWM_pin_4,OUTPUT);
    digitalWrite(PWM_pin_1,0);
    digitalWrite(PWM_pin_2,0);
    digitalWrite(PWM_pin_3,0);
    digitalWrite(PWM_pin_4,0);
    // socket->close();
    delete(mobile_robot_driver);
}

static void CCONV onEncoder0_PositionChange(PhidgetEncoderHandle ch, void * ctx, int positionChange, double timeChange, int indexTriggered) {
    if (std::isnan((double)positionChange/timeChange))
    {
        std::cout <<"PD0";
        return;
    } 
    mtx_enc1.lock();
	*((double *) ctx) = ((double)positionChange * -2 * M_PI) / (timeChange  * 0.001 * 520 * 4)  ;
    mtx_enc1.unlock();
}

static void CCONV onEncoder1_PositionChange(PhidgetEncoderHandle ch, void * ctx, int positionChange, double timeChange, int indexTriggered) {
    if (std::isnan((double)positionChange/timeChange))
    {
        std::cout <<"PD1";
        return;
    } 
    mtx_enc2.lock();
	*((double *) ctx) = ((double)positionChange * 2 * M_PI) / (timeChange  * 0.001 * 520 * 4)  ;
    mtx_enc2.unlock();
}

static void CCONV onEncoder2_PositionChange(PhidgetEncoderHandle ch, void * ctx, int positionChange, double timeChange, int indexTriggered) {
    if (std::isnan((double)positionChange/timeChange))
    {
        std::cout <<"PD2";
        return;
    } 
    mtx_enc3.lock();
	*((double *) ctx) = ((double)positionChange * 2 * M_PI) / (timeChange  * 0.001 * 520 * 4) ;
    mtx_enc3.unlock();
}

static void CCONV onEncoder3_PositionChange(PhidgetEncoderHandle ch, void * ctx, int positionChange, double timeChange, int indexTriggered) {
    if (std::isnan((double)positionChange/timeChange))
    {
        std::cout <<"PD3";
        return;
    } 
    mtx_enc4.lock();
	*((double *) ctx) = ((double)positionChange * -2 * M_PI) / (timeChange  * 0.001 * 520 * 4)  ;
    mtx_enc4.unlock();
}

azrael_mobile_driver::~azrael_mobile_driver()
{
    std::cout << " azrael_mobile_driver Destructor\n";
    #ifdef DEBUG_PID
    myfile.close();
    #endif
    // close(this->sockfd); 

    Phidget_close((PhidgetHandle)this->encoder0);
	PhidgetEncoder_delete(&this->encoder0);
    Phidget_close((PhidgetHandle)this->encoder1);
	PhidgetEncoder_delete(&this->encoder1);
    Phidget_close((PhidgetHandle)this->encoder2);
	PhidgetEncoder_delete(&this->encoder2);
    Phidget_close((PhidgetHandle)this->encoder3);
	PhidgetEncoder_delete(&this->encoder3);
}

azrael_mobile_driver::azrael_mobile_driver()
{
    std::cout << "Constructor \n";
    pinMode(PWM_pin_1,OUTPUT);
    pinMode(PWM_pin_2,OUTPUT);
    pinMode(PWM_pin_3,OUTPUT);
    pinMode(PWM_pin_4,OUTPUT);
    softPwmCreate (PWM_pin_1, 0, MAX_PWM_RANGE) ;
    softPwmCreate (PWM_pin_2, 0, MAX_PWM_RANGE) ;
    softPwmCreate (PWM_pin_3, 0, MAX_PWM_RANGE) ;
    softPwmCreate (PWM_pin_4, 0, MAX_PWM_RANGE) ;
    pinMode (PWM_dir_1, OUTPUT) ;
    pinMode (PWM_dir_2, OUTPUT) ;
    pinMode (PWM_dir_3, OUTPUT) ;
    pinMode (PWM_dir_4, OUTPUT) ;

    PhidgetEncoder_create(&this->encoder0);
    PhidgetEncoder_create(&this->encoder1);
    PhidgetEncoder_create(&this->encoder2);
    PhidgetEncoder_create(&this->encoder3);

    Phidget_setChannel((PhidgetHandle)this->encoder0, 0);
    Phidget_setChannel((PhidgetHandle)this->encoder1, 1);
    Phidget_setChannel((PhidgetHandle)this->encoder2, 2);
    Phidget_setChannel((PhidgetHandle)this->encoder3, 3);

    PhidgetEncoder_setOnPositionChangeHandler(this->encoder0, onEncoder0_PositionChange, (void *)&this->vel_enc1);
    PhidgetEncoder_setOnPositionChangeHandler(this->encoder1, onEncoder1_PositionChange, (void *)&this->vel_enc2);
    PhidgetEncoder_setOnPositionChangeHandler(this->encoder2, onEncoder2_PositionChange, (void *)&this->vel_enc3);
    PhidgetEncoder_setOnPositionChangeHandler(this->encoder3, onEncoder3_PositionChange, (void *)&this->vel_enc4);


    Phidget_openWaitForAttachment((PhidgetHandle)this->encoder0, 0);
    Phidget_openWaitForAttachment((PhidgetHandle)this->encoder1, 0);
    Phidget_openWaitForAttachment((PhidgetHandle)this->encoder2, 0);
    Phidget_openWaitForAttachment((PhidgetHandle)this->encoder3, 0);

    PhidgetEncoder_setIOMode(this->encoder0,ENCODER_IO_MODE_OPEN_COLLECTOR_10K);
    PhidgetEncoder_setIOMode(this->encoder1,ENCODER_IO_MODE_OPEN_COLLECTOR_10K);
    PhidgetEncoder_setIOMode(this->encoder2,ENCODER_IO_MODE_OPEN_COLLECTOR_10K);
    PhidgetEncoder_setIOMode(this->encoder3,ENCODER_IO_MODE_OPEN_COLLECTOR_10K);


    PhidgetEncoder_setPositionChangeTrigger(this->encoder0, 0);
    PhidgetEncoder_setPositionChangeTrigger(this->encoder1, 0);
    PhidgetEncoder_setPositionChangeTrigger(this->encoder2, 0);
    PhidgetEncoder_setPositionChangeTrigger(this->encoder3, 0);

    int interval = 8;

    PhidgetEncoder_setDataInterval(this->encoder0, interval);
    PhidgetEncoder_setDataInterval(this->encoder1, interval);
    PhidgetEncoder_setDataInterval(this->encoder2, interval);
    PhidgetEncoder_setDataInterval(this->encoder3, interval);

    //Filt encoder

    // const float samplingrate = 31.25; // Hz
    const float samplingrate = 500; // Hz
    const float cutoff_frequency = 30; // Hz
    f_vel_1.setup (samplingrate, cutoff_frequency);
    f_vel_2.setup (samplingrate, cutoff_frequency);
    f_vel_3.setup (samplingrate, cutoff_frequency);
    f_vel_4.setup (samplingrate, cutoff_frequency);


    //Socket

    // if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
    //     perror("socket creation failed"); 
    //     exit(EXIT_FAILURE); 
    // } 

    // memset(&servaddr, 0, sizeof(servaddr)); 
        
    // // Filling server information 
    // this->servaddr.sin_family = AF_INET; 
    // this->servaddr.sin_port = htons(44100); 
    // this->servaddr.sin_addr.s_addr = inet_addr("192.169.1.2");

    socket = new udp::socket(io_context);
    remote_endpoint = udp::endpoint(address::from_string(IPADDRESS_REMOTE), UDP_PORT);
    local_endpoint = udp::endpoint(address::from_string(IPADDRESS_LOCAL), UDP_PORT);
    socket->open(udp::v4());
    socket->bind(local_endpoint);
    io_context.run();

    std::cout << "Constructor End\n";

}

static double rampToward(double current, double target, double max_delta)
{
    double delta = target - current;
    if (delta > max_delta)
    {
        delta = max_delta;
    }
    else if (delta < -max_delta)
    {
        delta = -max_delta;
    }
    return current + delta;
}

void azrael_mobile_driver::control_thread()
{
    srand (time(NULL));
    double time_sin  = 0.0;
    double time_save = 0.0;
    int cnt = 0;

    std::chrono::time_point<std::chrono::system_clock> init_time = std::chrono::system_clock::now();
    std::chrono::time_point<std::chrono::system_clock> end_time  = std::chrono::system_clock::now();


    while(!stopping_)
    {   

        {
            std::scoped_lock lock(mtx_receive_);
            cmdvel_x = rampToward(cmdvel_x, buffer_in[0], MAX_LIN_ACCEL * CONTROL_LOOP_DT);
            cmdvel_y = rampToward(cmdvel_y, buffer_in[1], MAX_LIN_ACCEL * CONTROL_LOOP_DT);
            cmdvel_z = rampToward(cmdvel_z, buffer_in[2], MAX_ANG_ACCEL * CONTROL_LOOP_DT);
        }

        v1in_ = ( cmdvel_x - cmdvel_y - (lxy * cmdvel_z)) * (1.0/radius);
        v2in_ = (-cmdvel_x - cmdvel_y + (lxy * cmdvel_z)) * (1.0/radius);
        v3in_ = ( cmdvel_x - cmdvel_y + (lxy * cmdvel_z)) * (1.0/radius);
        v4in_ = (-cmdvel_x - cmdvel_y - (lxy * cmdvel_z)) * (1.0/radius);


        this->pid_w1.setSetpoint(std::abs(v1in_));
        this->pid_w2.setSetpoint(std::abs(v2in_));
        this->pid_w3.setSetpoint(std::abs(v3in_));
        this->pid_w4.setSetpoint(std::abs(v4in_));

        mtx_enc1.lock();
        this->vel_enc1_f = this->f_vel_1.filter(this->vel_enc1);
        double snap_enc1 = this->vel_enc1;
        mtx_enc1.unlock();
        mtx_enc2.lock();
        double snap_enc2 = this->vel_enc2;
        this->vel_enc2_f = this->f_vel_2.filter(this->vel_enc2);
        mtx_enc2.unlock();
        mtx_enc3.lock();
        double snap_enc3 = this->vel_enc3;
        this->vel_enc3_f = this->f_vel_3.filter(this->vel_enc3);
        mtx_enc3.unlock();
        mtx_enc4.lock();
        double snap_enc4 = this->vel_enc4;
        this->vel_enc4_f = this->f_vel_4.filter(this->vel_enc4);
        mtx_enc4.unlock();
        
        int v1dir = (v1in_ < 0) ? HIGH : LOW;
        int v2dir = (v2in_ < 0) ? LOW : HIGH;
        int v3dir = (v3in_ < 0) ? LOW : HIGH;
        int v4dir = (v4in_ < 0) ? HIGH : LOW;

        //TODO tristate assign
        digitalWrite(PWM_dir_1,v1dir);
        digitalWrite(PWM_dir_2,v2dir);
        digitalWrite(PWM_dir_3,v3dir);
        digitalWrite(PWM_dir_4,v4dir);

        int pwm1 = (int)pid_w1.update_state();
        int pwm2 = (int)pid_w2.update_state();
        int pwm3 = (int)pid_w3.update_state();
        int pwm4 = (int)pid_w4.update_state();

        // std::cout << pwm1 << "," << pwm2 << "," << pwm3 << "," << pwm4 << "\n";

        {
            std::scoped_lock lock(mtx_buffer_out_);
            buffer_out[0] = snap_enc1;
            buffer_out[1] = snap_enc2;
            buffer_out[2] = snap_enc3;
            buffer_out[3] = snap_enc4;
            buffer_out[4] = ((v1in_ < 0) ? -1.0 : 1.0) * (pwm1 / (double)MAX_PWM_RANGE);
            buffer_out[5] = ((v2in_ < 0) ? -1.0 : 1.0) * (pwm2 / (double)MAX_PWM_RANGE);
            buffer_out[6] = ((v3in_ < 0) ? -1.0 : 1.0) * (pwm3 / (double)MAX_PWM_RANGE);
            buffer_out[7] = ((v4in_ < 0) ? -1.0 : 1.0) * (pwm4 / (double)MAX_PWM_RANGE);
        }

        softPwmWrite (PWM_pin_1,  pwm1) ;
        softPwmWrite (PWM_pin_2,  pwm2) ;
        softPwmWrite (PWM_pin_3,  pwm3) ;
        softPwmWrite (PWM_pin_4,  pwm4) ;

        end_time  = std::chrono::system_clock::now();
        
        // time_sin = time_sin + (std::chrono::duration_cast<std::chrono::microseconds>(end_time - init_time).count() * 1e-6);

        auto micros = static_cast<std::chrono::microseconds::rep>(CONTROL_LOOP_DT * 1e6) - std::chrono::duration_cast<std::chrono::microseconds>(end_time - init_time).count();
        if(micros > 0)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(micros));
        }

        init_time = end_time;
    }
}

void azrael_mobile_driver::odometry()
{
    auto current_time = std::chrono::high_resolution_clock::now();
    auto last_time    = std::chrono::high_resolution_clock::now();

    while(!stopping_)
    {
        last_time = current_time;
        current_time = std::chrono::high_resolution_clock::now();

        double dt = std::chrono::duration_cast<std::chrono::nanoseconds>(current_time-last_time).count() / 1e9;
        mtx_enc1.lock();
        mtx_enc2.lock();
        mtx_enc3.lock();
        mtx_enc4.lock();
        this->velx_odom = (      this->vel_enc1 + this->vel_enc2 + this->vel_enc3 + this->vel_enc4 ) * (radius * 0.5);
        this->vely_odom = ( -1 * this->vel_enc1 + this->vel_enc2 + this->vel_enc3 - this->vel_enc4 ) * (radius * 0.5);
        this->velw_odom = ( -1 * this->vel_enc1 + this->vel_enc2 - this->vel_enc3 + this->vel_enc4 ) * (radius / ( 4 * lxy));
        mtx_enc1.unlock();
        mtx_enc2.unlock();
        mtx_enc3.unlock();
        mtx_enc4.unlock();
        this->posx_odom += (this->velx_odom * cos(this->posw_odom) - this->vely_odom * sin(this->posw_odom)) * dt;
        this->posy_odom += (this->velx_odom * sin(this->posw_odom) + this->vely_odom * cos(this->posw_odom)) * dt;
        this->posw_odom += this->velw_odom * dt;


        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        // std::cout << "enc :" << this->vel_enc1 << " " << this->vel_enc2 << " " << this->vel_enc3 << " " << this->vel_enc4 << "\n";

    }
}

void azrael_mobile_driver::socket_send()
{
    std::chrono::time_point<std::chrono::system_clock> init_time_udp = std::chrono::system_clock::now();

    while(!stopping_)
    {
	// buffer_out[0] = this->vel_enc1;
	// buffer_out[1] = this->vel_enc2;
	// buffer_out[2] = this->vel_enc3;
	// buffer_out[3] = this->vel_enc4;
    // sendto(this->sockfd, (const void *)buffer_out, sizeof(double) * 4, MSG_WAITALL, (const struct sockaddr *) &servaddr, sizeof(servaddr)); 
    boost::system::error_code err;
    double snapshot[8];
    {
        std::scoped_lock lock(mtx_buffer_out_);
        std::copy(std::begin(buffer_out), std::end(buffer_out), std::begin(snapshot));
    }
    auto sent = socket->send_to(boost::asio::buffer(snapshot), remote_endpoint, 0, err);

    std::this_thread::sleep_for(std::chrono::microseconds(20000));
    // auto micros = 20000 - std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - init_time_udp).count();
    // if(micros > 0)
    // {
    //     std::this_thread::sleep_for(std::chrono::microseconds(micros));
    // }
    // init_time_udp = std::chrono::system_clock::now();
    }
}

void azrael_mobile_driver::socket_receive()
{
    int n; 
    unsigned int len;
    std::chrono::time_point<std::chrono::system_clock> init_time_udp = std::chrono::system_clock::now();

    while(!stopping_)
    {

    {
        std::scoped_lock lock(mtx_receive_);
        // n = recvfrom(this->sockfd, (void *)buffer_in, sizeof(double) * 3, MSG_WAITALL, (struct sockaddr *) &servaddr, &len); 
        socket->receive_from(boost::asio::buffer(buffer_in), local_endpoint);
    }

    std::this_thread::sleep_for(std::chrono::microseconds(20000));
    // auto micros = 10000 - std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - init_time_udp).count();
    // if(micros > 0)
    // {
    //     std::this_thread::sleep_for(std::chrono::microseconds(micros));
    // }
    // init_time_udp = std::chrono::system_clock::now();
    }
}

int main(int argc, char **argv)
{      

    #ifdef DEBUG_PID
    std::cout << "DEBUG PID MODE ----  DEBUG PID MODE \n DEBUG PID MODE ----  DEBUG PID MODE \n";
    myfile.open ("log.txt");
    #endif

    //vx = atof(argv[1]);
    //vy = atof(argv[2]);
    //vw = atof(argv[3]);

    /* Lock memory */
    if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
            printf("mlockall failed: %m\n");
            exit(-2);
    }


    std::cout << "Wiring pi setup .\n";
    if (wiringPiSetup () == -1) exit (1) ;

    struct sigaction sigIntHandler;

    printf("Sig setting\n");
    signal(SIGINT, my_handler);

    std::cout << "Class init \n";
    mobile_robot_driver = new azrael_mobile_driver();

    std::cout << "Thread started .\n";
    std::thread control_t(&azrael_mobile_driver::control_thread, mobile_robot_driver);
    // std::thread    odom_t(&azrael_mobile_driver::odometry, mobile_robot_driver);
    std::thread    sock_t_send(&azrael_mobile_driver::socket_send, mobile_robot_driver);
    std::thread    sock_t_receive(&azrael_mobile_driver::socket_receive, mobile_robot_driver);

    sched_param sch;
    int policy;
    pthread_getschedparam(control_t.native_handle(), &policy, &sch);
    sch.sched_priority = 80;

    pthread_setschedparam(control_t.native_handle(), SCHED_FIFO, &sch);
    // pthread_setschedparam(odom_t.native_handle(), SCHED_RR, &sch);
    pthread_setschedparam(sock_t_send.native_handle(), SCHED_FIFO, &sch);
    pthread_setschedparam(sock_t_receive.native_handle(), SCHED_FIFO, &sch);
    




    control_t.join();
    // odom_t.join();
    sock_t_receive.join();
    sock_t_send.join();

    std::cout << "Thread joined.\n";

    return 0;
}
