#ifndef __MOTOR__
#define __MOTOR__

#include <Encoder.h>
#include <math.h>

#define MAXPWM 255
#define MINSPEED 2
#define TPR (13.0*30.0)  //Ticks per rev.
#define KP 5.0
#define KI 0.1
#define KD 0


/******** Motors Pins Connection ********/
#define right_motor_en             5
#define right_motor_forword        6
#define right_motor_backword       7
#define right_ChannelA_encoder     2
#define right_ChannelB_encoder     3

#define left_motor_en              8
#define left_motor_forword         9
#define left_motor_backword        10
#define left_ChannelA_encoder      19
#define left_ChannelB_encoder      18

#define right_motor                 0
#define left_motor                  1

/******** Mechanical Spasefication ********/
#define Wheel_Diameter            3.25
#define Robot_Width               30.0

/******** Encoder Definitions ********/
Encoder Right_Encoder (right_ChannelA_encoder, right_ChannelB_encoder) ; 
Encoder Left_Encoder (left_ChannelA_encoder, left_ChannelB_encoder) ; 


struct Motor
{
    byte pwmPin, fPin, bPin, motor ;    
    int pwmVal = 0 ;
    double Speed, targetSpeed, ticksPerSec = 0 ;
    long oldt, newt, oldPos, newPos, ticks = 0;
    double E, de, olde, e = 0 ;

    /*
     * Assign the values to pwmPin1 and pwmPin2 such that when the PWM is applied to 
     * pin1 and pin2 is set to 0 the motor moves in the forward direction.
     * 
     * Assign the values to encoderPin1 and encoderPin2 such that when the motor
     * moves forward, the encoder ticks get incremented by a positive value.
     */
    Motor(byte copy_pwmPin, byte copy_fPin, byte copy_bPin, byte copy_motor)
    {
        oldt = millis();
        pwmPin = copy_pwmPin ;
        fPin = copy_fPin ;
        bPin = copy_bPin ;
        motor = copy_motor ; 
        
        pinMode (pwmPin, OUTPUT) ; 
        pinMode (fPin, OUTPUT) ; 
        pinMode (bPin, OUTPUT) ;

        digitalWrite (pwmPin, LOW) ; 
        digitalWrite (fPin, LOW) ;
        analogWrite (bPin, LOW) ;
    }

    // Sets the desired angular velocity of the motor in rad/sec
    void setTargetSpeed(double copy_targetSpeed)
    {
        targetSpeed = copy_targetSpeed;
        if(abs(targetSpeed) < MINSPEED)
        {
          targetSpeed = pwmVal = E = olde = 0 ;
        }
    }

    // Computes the current speed.
    // Modifies the PWM value to track the target speed.
    // Returns the distance covered by the wheel since the last call in (cm).
    double update()
    {
        if (motor == right_motor)
        {
          newPos = Right_Encoder.read();
        }
        else 
        {
          newPos = Left_Encoder.read();
        }
        
        newt = millis()-oldt;
        ticks = newPos-oldPos;
        oldPos = newPos; 
        oldt = millis();
        
        ticksPerSec = ticks*(1000.0/newt);
        Speed = (ticksPerSec/TPR)*2*M_PI;
        e = targetSpeed - Speed;
        de = e-olde;
        olde = e;
        E += e;
        pwmVal += (int)(KP*e + KD*de + KI*E);

        if(targetSpeed > 0)
        {
            pwmVal = constrain(pwmVal, 0, MAXPWM);
            
            // forword
            digitalWrite (fPin, HIGH) ; 
            digitalWrite (bPin, LOW) ;
        }
        else
        {
            pwmVal = constrain(pwmVal, -MAXPWM, 0);
            
            // forword
            digitalWrite (fPin, LOW) ; 
            digitalWrite (bPin, HIGH) ;
        }

        //if(abs(targetSpeed) < 1) pwm = 0 ;
        
        analogWrite(pwmPin, abs(pwmVal));

        return 2*M_PI*Wheel_Diameter*(ticks/TPR);
    }
};

#endif
