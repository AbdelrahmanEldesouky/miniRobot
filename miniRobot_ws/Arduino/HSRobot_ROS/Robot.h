#ifndef __ROBOT__
#define __ROBOT__

#include "Motor.h" 

struct Robot
{
    double dl, dr, vr, vl;
    
    Motor right = Motor(right_motor_en, right_motor_forword, right_motor_backword, right_motor);    
    Motor left = Motor(left_motor_en, left_motor_forword, left_motor_backword, left_motor); 

    //vc: Target linear velocity of the robot (cm/sec)
    //wc: Target angular velocity of the robot (rad/sec)
    //The function uses inverse kinematics to set the desired angular velocities of the two motors
    void setTarget(double vel, double omega)
    {
        // update motor speed 
        vr = (((2.0 * vel) + (omega * Robot_Width)) / (2.0 * Wheel_Diameter)) ; 
        vl = (((2.0 * vel) - (omega * Robot_Width)) / (2.0 * Wheel_Diameter)) ; 
        right.setTargetSpeed(vr);
        left.setTargetSpeed(vl);
    }

    //Modifies the PWM of the left and right motors to track the desired linear and angular velocities.
    //Stores the distance covered by the left and right wheels to be published later to ROS.
    void update()
    {
        dl = left.update();
        dr = right.update(); 
    }
};

#endif
