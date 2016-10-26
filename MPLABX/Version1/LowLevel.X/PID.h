/* 
 * File:   PID.h
 * Author: Arsapol
 *
 * Created on October 22, 2016, 10:23 PM
 */

#ifndef PID_H
#define	PID_H

#ifdef	__cplusplus
extern "C" {
#endif

    /*typedef struct {
        float error;

        float integral;
        float derivative;

        float previous_error;

        float distance_set;
        float current_distance;

        float output;

        float KP;
        float KI;
        float KD;
    } PID;

    float PID_Calc(PID *pid) {
        pid->error = pid->distance_set - pid->current_distance;
        //PID control
        pid->integral += pid->error;
        pid->derivative = pid->error - pid->previous_error;

        pid->output = (pid->KP * pid->error) + (pid->KI * pid->integral) + (pid->KD * pid->derivative);

        pid->previous_error = pid->error;

        return pid->output;
    }*/

#define PI 3.14159265359
#define CPR 400.0
#define Diameter 0.8 //unit centimeter - ****************Need
#define Position_Per_Pulse (2.0*PI*(Diameter/2.0))/CPR //cm
#define Speed PI*Diameter/CPR
    
#define CPR_BY_QEI 800.0
#define Lead_Pitch 2.0 //unit mm
#define xPOSITION Lead_Pitch/CPR_BY_QEI

#define MAX_SPD 180 //cm/s - ***************************Need Test
#define MAX_CPR MAX_SPD

    //New

    typedef struct {
        float dState; // Last position input
        float iState; // Integrator state
        float iMax, iMin; // Maximum and minimum allowable integrator state
        float iGain, // integral gain
        pGain, // proportional gain
        dGain; // derivative gain
    } SPID;
    
    void PID_Init(SPID *pid, float Kp, float Ki, float Kd){
        pid->dState = 0; // Last position input
        pid->iState = 0; // Integrator state
        pid->iMax = 0;
        pid->iMin = 0; // Maximum and minimum allowable integrator state
        pid->iGain = Ki; // integral gain
        pid->pGain = Kp; // proportional gain
        pid->dGain = Kd; // derivative gain
    }

    float UpdatePID(SPID * pid, float error, float position) {
        float pTerm, dTerm, iTerm;
        pTerm = pid->pGain * error;
        // calculate the proportional term
        // calculate the integral state with appropriate limiting
        pid->iState += error;
        if (pid->iState > pid->iMax) pid->iState = pid->iMax;
        else if (pid->iState < pid->iMin) pid->iState = pid->iMin;
        iTerm = pid->iGain * pid->iState; // calculate the integral term
        dTerm = pid->dGain * (position - pid->dState);
        pid->dState = position;
        return pTerm + iTerm - dTerm;
    }

    /*typedef struct {
        boolean usingFeedForward;
        boolean inAuto;

        //Actual tuning parameters used in PID calculation.
        float Kc_;
        float tauR_;
        float tauD_;

        //Raw tuning parameters.
        float pParam_;
        float iParam_;
        float dParam_;

        //The point we want to reach.
        float setPoint_;
        //The thing we measure.
        float processVariable_;
        float prevProcessVariable_;
        //The output that affects the process variable.
        float controllerOutput_;
        float prevControllerOutput_;

        //We work in % for calculations so these will scale from
        //real world values to 0-100% and back again.
        float inMin_;
        float inMax_;
        float inSpan_;
        float outMin_;
        float outMax_;
        float outSpan_;

        //The accumulated error, i.e. integral.
        float accError_;
        //The controller output bias.
        float bias_;

        //The interval between samples.
        float tSample_;

        //Controller output as a real world value.
        volatile float realOutput_;
    } SPID;

    void setSetPoint(SPID *pid, float sp) {
        pid->setPoint_ = sp;
    }

    void setProcessValue(SPID *pid, float pv) {
        pid->processVariable_ = pv;
    }

    void setInputLimits(SPID *pid, float inMin, float inMax) {

        //Make sure we haven't been given impossible values.
        if (inMin >= inMax) {
            return;
        }

        //Rescale the working variables to reflect the changes.
        pid->prevProcessVariable_ *= (inMax - inMin) / pid->inSpan_;
        pid->accError_ *= (inMax - inMin) / pid->inSpan_;

        //Make sure the working variables are within the new limits.
        if (pid->prevProcessVariable_ > 1) {
            pid->prevProcessVariable_ = 1;
        } else if (pid->prevProcessVariable_ < 0) {
            pid->prevProcessVariable_ = 0;
        }

        pid->inMin_ = inMin;
        pid->inMax_ = inMax;
        pid->inSpan_ = inMax - inMin;

    }

    void setOutputLimits(SPID *pid, float outMin, float outMax) {

        //Make sure we haven't been given impossible values.
        if (outMin >= outMax) {
            return;
        }

        //Rescale the working variables to reflect the changes.
        pid->prevControllerOutput_ *= (outMax - outMin) / pid->outSpan_;

        //Make sure the working variables are within the new limits.
        if (pid->prevControllerOutput_ > 1) {
            pid->prevControllerOutput_ = 1;
        } else if (pid->prevControllerOutput_ < 0) {
            pid->prevControllerOutput_ = 0;
        }

        pid->outMin_ = outMin;
        pid->outMax_ = outMax;
        pid->outSpan_ = outMax - outMin;

    }

    void setTunings(SPID *pid, float Kc, float tauI, float tauD) {

        //Verify that the tunings make sense.
        if (Kc == 0.0 || tauI < 0.0 || tauD < 0.0) {
            return;
        }

        //Store raw values to hand back to user on request.
        pid->pParam_ = Kc;
        pid->iParam_ = tauI;
        pid->dParam_ = tauD;

        float tempTauR;

        if (tauI == 0.0) {
            tempTauR = 0.0;
        } else {
            tempTauR = (1.0 / tauI) * pid->tSample_;
        }

        //For "bumpless transfer" we need to rescale the accumulated error.
        if (pid->inAuto) {
            if (tempTauR == 0.0) {
                pid->accError_ = 0.0;
            } else {
                pid->accError_ *= (pid->Kc_ * pid->tauR_) / (Kc * tempTauR);
            }
        }

        pid->Kc_ = Kc;
        pid->tauR_ = tempTauR;
        pid->tauD_ = tauD / pid->tSample_;

    }

    void PID_Init(SPID *pid, float Kc, float tauI, float tauD, float interval) {
        pid->usingFeedForward = false;
        pid->inAuto = false;

        //Default the limits to the full range of I/O: 3.3V
        //Make sure to set these to more appropriate limits for
        //your application.
        setInputLimits(&pid, 0.0, 3.3);
        setOutputLimits(&pid, 0.0, 3.3);

        pid->tSample_ = interval;

        setTunings(&pid, Kc, tauI, tauD);

        pid->setPoint_ = 0.0;
        pid->processVariable_ = 0.0;
        pid->prevProcessVariable_ = 0.0;
        pid->controllerOutput_ = 0.0;
        pid->prevControllerOutput_ = 0.0;

        pid->accError_ = 0.0;
        pid->bias_ = 0.0;

        pid->realOutput_ = 0.0;
    }

    float compute(SPID *pid) {

        //Pull in the input and setpoint, and scale them into percent span.
        float scaledPV = (pid->processVariable_ - pid->inMin_) / pid->inSpan_;

        if (scaledPV > 1.0) {
            scaledPV = 1.0;
        } else if (scaledPV < 0.0) {
            scaledPV = 0.0;
        }

        float scaledSP = (pid->setPoint_ - pid->inMin_) / pid->inSpan_;
        if (scaledSP > 1.0) {
            scaledSP = 1;
        } else if (scaledSP < 0.0) {
            scaledSP = 0;
        }

        float error = scaledSP - scaledPV;

        //Check and see if the output is pegged at a limit and only
        //integrate if it is not. This is to prevent reset-windup.
        if (!(pid->prevControllerOutput_ >= 1 && error > 0) && !(pid->prevControllerOutput_ <= 0 && error < 0)) {
            pid->accError_ += error;
        }

        //Compute the current slope of the input signal.
        float dMeas = (scaledPV - pid->prevProcessVariable_) / pid->tSample_;

        float scaledBias = 0.0;

        if (pid->usingFeedForward) {
            scaledBias = (pid->bias_ - pid->outMin_) / pid->outSpan_;
        }

        //Perform the PID calculation.
        pid->controllerOutput_ = scaledBias + pid->Kc_ * (error + (pid->tauR_ * pid->accError_) - (pid->tauD_ * dMeas));

        //Make sure the computed output is within output constraints.
        if (pid->controllerOutput_ < 0.0) {
            pid->controllerOutput_ = 0.0;
        } else if (pid->controllerOutput_ > 1.0) {
            pid->controllerOutput_ = 1.0;
        }

        //Remember this output for the windup check next time.
        pid->prevControllerOutput_ = pid->controllerOutput_;
        //Remember the input for the derivative calculation next time.
        pid->prevProcessVariable_ = scaledPV;

        //Scale the output from percent span back out to a real world number.
        return ((pid->controllerOutput_ * pid->outSpan_) + pid->outMin_);

    }*/
    
    //In main
    /*if (sec_flag == TRUE) {
            sec_flag = FALSE;
            seconds++;
            //printf("%d\r\n", seconds);
        }
          now = seconds;

        if (now - prev >= RATE * 1000) {
            leftPulses = En_x.Pulses * Speed;
            leftVelocity = (leftPulses - leftPrevPulses) / RATE;
            leftPrevPulses = leftPulses;
            setProcessValue(&Motor_X, leftVelocity);
            leftPwmDuty += compute(&Motor_X);

            if (leftPwmDuty > 1.0) {
                leftPwmDuty = 1.0;
            } else if (leftPwmDuty < -1.0) {
                leftPwmDuty = -1.0;
            }

            //leftMotor = leftPwmDuty;

            rightPulses = En_y.Pulses * Speed;
            rightVelocity = (rightPulses - rightPrevPulses) / RATE;
            rightPrevPulses = rightPulses;
            setProcessValue(&Motor_Y, rightVelocity);
            rightPwmDuty += compute(&Motor_Y);

            if (rightPwmDuty > 1.0) {
                rightPwmDuty = 1.0;
            } else if (rightPwmDuty < -1.0) {
                rightPwmDuty = -1.0;
            }

            //rightMotor = rightPwmDuty;
#ifdef DEBUG
            printf("left,right pulse duty : %f,%f\n", leftPulses, rightPulses);
            //printf("left,right pwm duty : %f,%f\n",leftPwmDuty,rightPwmDuty);
            printf("left,right Velocity : %f,%f\n", leftVelocity, rightVelocity);
#endif
            if (leftPwmDuty > 0.1) {
                output_high(MOTOR_X_DIR); //leftDirection = 1;
                //leftBrake = 0;
                Control_Motor(DRIVE_MOTOR_X, leftPwmDuty); //leftMotor = leftPwmDuty;
            } else if (leftPwmDuty < -0.1) {
                output_low(MOTOR_X_DIR); //leftDirection = 0;
                //leftBrake = 1;
                Control_Motor(DRIVE_MOTOR_X, (-1) * leftPwmDuty); //leftMotor = (-1) * leftPwmDuty;
            } else {
                output_low(MOTOR_X_DIR); //leftDirection = 0;
                //leftBrake = 0;
                Control_Motor(DRIVE_MOTOR_X, 0); //leftMotor = 0.0;
            }

            if (rightPwmDuty > 0.1) {
                output_high(MOTOR_Y_DIR); //rightDirection = 1;
                //rightBrake = 0;
                Control_Motor(DRIVE_MOTOR_Y, rightPwmDuty); //rightMotor = rightPwmDuty;
            } else if (rightPwmDuty < -0.1) {
                output_low(MOTOR_Y_DIR); //rightDirection = 0;
                //rightBrake = 1;
                Control_Motor(DRIVE_MOTOR_Y, (-1) * rightPwmDuty); //rightMotor = (-1) * rightPwmDuty;
            } else {
                output_low(MOTOR_Y_DIR); //rightDirection = 0;
                //rightBrake = 0;
                Control_Motor(DRIVE_MOTOR_Y, 0); //rightMotor = 0.0;
            }

            prev = now;
        }*/

#ifdef	__cplusplus
}
#endif

#endif	/* PID_H */

