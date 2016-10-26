/* 
 * File:   config.h
 * Author: Arsapol
 *
 * Created on October 23, 2016, 2:08 AM
 */

#ifndef CONFIG_H
#define	CONFIG_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#define DRIVE_MOTOR_X   1
#define DRIVE_MOTOR_Y   2
    
#define MOTOR_HZ        5

#define MOTOR_X_PWM     PIN_B0
#define MOTOR_Y_PWM     PIN_B1
//#define MOTOR_Z_PWM     PIN_B2
    
#define MOTOR_X_DIR     PIN_B4
#define MOTOR_Y_DIR     PIN_B5
//#define MOTOR_Z_DIR     PIN_B6
    
#define ENCODER_X_A     0
#define ENCODER_Y_A     1    
#define ENCODER_X_B     PIN_B3
#define ENCODER_Y_B     PIN_B2
//#define ENCODER_Z_B     PIN_B13
    
//#define TIMES_PER_ROUND 5
    
    /*
 ****************Note******************
 * INT0[Ex] - Encoder Axis X
 * INT1[Ex] - Encoder Axis Y
 * INT2[In] - PWM Timer2
 * INT3[Ex] -                                  (deleted) Encoder Axis Z
 * INT4[ ] - 
 * 
 * PIN_A0  - 
 * PIN_A1  - 
 * PIN_A2  - 
 * PIN_A3  - 
 * PIN_A4  - 
 * PIN_B0  - [OUT] AN2 : PWM_MOTOR_X
 * PIN_B1  - [OUT] AN3 : PWM_MOTOR_Y
 * PIN_B2  - [IN]  AN5 : B_Y (Encoder signal B) /                               
 * PIN_B3  - [IN]  AN5 : B_X (Encoder signal B) /
 * PIN_B4  - [OUT] DIR_X /
 * PIN_B5  - [OUT] DIR_Y /
 * PIN_B6  -                                   
 * PIN_B7  - [IN] INT0 : Read Encoder X
 * PIN_B8  - [IN] INT1 : Read Encoder Y 
 * PIN_B9  -                                   
 * PIN_B10 - 
 * PIN_B11 - 
 * PIN_B12 - [IN]  AN12 : B_Y (Encoder signal B) /
 * PIN_B13 - RX                                  
 * PIN_B14 - TX
 * PIN_B15 - 
 * PIN_B16 - 
 */


#ifdef	__cplusplus
}
#endif

#endif	/* CONFIG_H */

