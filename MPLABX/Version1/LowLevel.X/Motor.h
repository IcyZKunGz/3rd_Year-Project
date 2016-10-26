/* 
 * File:   Motor.h
 * Author: Arsapol
 *
 * Created on October 22, 2016, 11:41 PM
 */
#ifndef MOTOR_H
#define	MOTOR_H

#ifdef	__cplusplus
extern "C" {
#endif
    
    typedef struct {
        int B_signal;
        int Pulses;
    } Encoder;
    
    void Encoder_Init(Encoder *en){
        en->B_signal = 0;
        en->Pulses = 0;
    }
    


#ifdef	__cplusplus
}
#endif

#endif	/* MOTOR_H */

