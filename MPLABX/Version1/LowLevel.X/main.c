#include <24FJ48GA002.h>
#device ADC=10
#include <PID.h>
#include <Motor.h>
#include <config.h>
#include <stdio.h>
#include <stdlib.h> 
#fuses FRC_PLL
#use delay (clock = 32000000)
//#use delay (clock = 16000000) For Use in Hardware

#PIN_SELECT U1RX = PIN_B12
#PIN_SELECT U1TX = PIN_B13
#use rs232(baud=9600, UART1)

#use fixed_io(b_outputs= MOTOR_X_DIR, MOTOR_Y_DIR) //DIR x,y

#PIN_SELECT OC1 = MOTOR_X_PWM //PWM_MOTOR_X
#PIN_SELECT OC2 = MOTOR_Y_PWM //PWM_MOTOR_Y
#PIN_SELECT INT1 = PIN_B8 //Y Axis - X is INT0

#define RATE  0.1
#define Kc   0.5
#define Ti    0.0
#define Td    0.05
SPID Motor_X, Motor_Y;
Encoder En_x, En_y;
float plantCommand[2]={0}, position[2]={0}, drive[2]={0}; //plantCommand = goal position - x,y

int Z_Position = 0;

char SM_Buffer[20];
int SM_Index = 0;
float data[2] = {0};
int SM_Id = 1;
boolean isX = FALSE;
boolean state = FALSE;

void SM_RxD(char c) {
    switch (SM_Id) {
        case 1:
            if (c == ':') { //X data
                isX = TRUE;
                SM_Index = 0;
                SM_Id = 2;
            } else if (c == '!') { //Y data
                isX = FALSE;
                SM_Id = 2;
            }
            SM_Index = 0;
            break;

        case 2:
            if (c == '#') {
                SM_Buffer[SM_Index] = '\0';
                state = TRUE;
                SM_Id = 1;
            } else {
                SM_Buffer[SM_Index] = c;
                SM_Index++;
            }
            break;
    }
}

#INT_RDA

void UART1_Isr() {
    char c = getc();
    SM_RxD(c);
}

void Init_ADC() {
    setup_adc_ports(sAN0, VSS_VDD);
    setup_adc(ADC_CLOCK_DIV_32);
    set_adc_channel(0);
    delay_us(10);
}

void Change_ADC_Read_Channel(int channel) {
    set_adc_channel(channel);
    delay_us(10);
}

void Control_Motor(int motor_num, float speed_velo) {
    if (motor_num == DRIVE_MOTOR_X) {
        if (speed_velo >= 0) output_high(MOTOR_X_DIR);
        else output_low(MOTOR_X_DIR);
        set_pwm_duty(DRIVE_MOTOR_X, speed_velo);
    } else if (motor_num == DRIVE_MOTOR_Y) {
        if (speed_velo >= 0) output_high(MOTOR_Y_DIR);
        else output_low(MOTOR_Y_DIR);
        set_pwm_duty(DRIVE_MOTOR_Y, speed_velo);
    }
}

#INT_EXT0

void INT_EXT_INPUT0(void) { //X Axis
    //Read Pulse from Encoder and compare pulse A and B
    En_x.B_signal = input(ENCODER_X_B);
    if (En_x.B_signal == 0) En_x.Pulses += 1;
    else En_x.Pulses -= 1;

    position[0] = En_x.Pulses * Position_Per_Pulse; //Times of twist for 1 round ****************Need Test for TIMES_PER_ROUND
    drive[0] = UpdatePID(&Motor_X, (plantCommand[0] - position[0]), position[0]);
    Control_Motor(DRIVE_MOTOR_X, drive[0]);
}

#INT_EXT1

void INT_EXT_INPUT1(void) { //Y Axis
    //Read Pulse from Encoder and compare pulse A and B 
    En_y.B_signal = input(ENCODER_Y_B);
    if (En_y.B_signal == 0) En_y.Pulses += 1;
    else En_y.Pulses -= 1;

    position[1] = En_y.Pulses * Position_Per_Pulse; //Times of twist for 1 round ****************Need Test for TIMES_PER_ROUND
    drive[1] = UpdatePID(&Motor_Y, (plantCommand[1] - position[1]), position[1]);
    Control_Motor(DRIVE_MOTOR_Y, drive[1]);
}

void Init_Interrupts() {
    enable_interrupts(INT_EXT0);
    ext_int_edge(ENCODER_X_A, L_TO_H); // Rising Edge
    enable_interrupts(INT_EXT1);
    ext_int_edge(ENCODER_Y_A, L_TO_H); // Rising Edge
    enable_interrupts(INT_RDA);
}

void Init_MotorPWM() {
    setup_timer3(TMR_INTERNAL | TMR_DIV_BY_64, MOTOR_HZ);
    setup_compare(DRIVE_MOTOR_X, COMPARE_PWM | COMPARE_TIMER3);
    set_pwm_duty(DRIVE_MOTOR_X, 0);
    setup_compare(DRIVE_MOTOR_Y, COMPARE_PWM | COMPARE_TIMER3);
    set_pwm_duty(DRIVE_MOTOR_Y, 0);
}

void PID_X_Y() {
    position[0] = En_x.Pulses * Position_Per_Pulse; //Times of twist for 1 round ****************Need Test for TIMES_PER_ROUND
    drive[0] = UpdatePID(&Motor_X, (plantCommand[0] - position[0]), position[0]);
    Control_Motor(DRIVE_MOTOR_X, drive[0]);

    position[1] = En_y.Pulses * Position_Per_Pulse; //Times of twist for 1 round ****************Need Test for TIMES_PER_ROUND
    drive[1] = UpdatePID(&Motor_Y, (plantCommand[1] - position[1]), position[1]);
    Control_Motor(DRIVE_MOTOR_Y, drive[1]);
}

/*volatile float leftPulses = 0;
volatile float leftPrevPulses = 0;
volatile float leftPwmDuty = 1.0;
volatile float leftVelocity = 0.0;

volatile float rightPulses = 0;
volatile float rightPrevPulses = 0;
volatile float rightPwmDuty = 1.0;
volatile float rightVelocity = 0.0;*/


void main() {
    disable_interrupts(GLOBAL);
    clear_interrupt(INT_RDA);
    Init_Interrupts();
    Init_MotorPWM();
    enable_interrupts(GLOBAL);

    //ADC
    Init_ADC();
    Change_ADC_Read_Channel(0);

    //Encoder
    Encoder_Init(&En_x);
    Encoder_Init(&En_y);

    //PID
    PID_Init(&Motor_X, Kc, Ti, Td);
    PID_Init(&Motor_Y, Kc, Ti, Td);

    int Z_Position = 0;

    printf("Main\r\n");
    while (TRUE) {
        int temp = read_adc();
        if(temp != 0)
            Z_Position = temp;
        
        if (state == TRUE) {
            int int_buffer = atoi(SM_Buffer);
            if (isX == TRUE)
                data[0] = int_buffer / 1000.0;
            else
                data[1] = int_buffer / 1000.0;
            plantCommand[0] = data[0];
            plantCommand[1] = data[1];
            
            PID_X_Y();
            
            state = FALSE;
            
            printf("%e  %e  %d\r\n",plantCommand[0],plantCommand[1],Z_Position);
        }

    }
}

