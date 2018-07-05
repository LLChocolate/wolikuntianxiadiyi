#include "myPWM.h"
void Motot1_Init(void)
{
  gpio_init(PORTA,13,GPO,HIGH);//DIR
  FTM_PWM_init(MOTOR_1,1000,50000);//����Ƶ��Ϊ10k
}
void Motot2_Init(void)
{
  gpio_init(PORTE,26,GPO,HIGH);//DIR
  FTM_PWM_init(MOTOR_2,1000,65530);//����Ƶ��Ϊ10k
}

void Servo_Init(void)
{
  FTM_PWM_init(STEER_,50,SERVO_MIDDLE);//��ʼ0.5ms�ߵ�ƽ��ת0��
}

void PWM_DISENABLE(FTMn ftmn, CHn ch)
{
    FTM_PWM_Duty(ftmn,ch,0);//�ر�PWM
}

