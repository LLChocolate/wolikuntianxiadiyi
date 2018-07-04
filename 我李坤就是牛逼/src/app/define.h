#ifndef _DEFINE_H
#define _DEFINE_H

#include  "common.h"
/*************************************************************************
*  ģ�����ƣ�defineģ��
*  ����˵����Include �û��Զ���ĺ�
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2012-2-14
*************************************************************************/

//#define LCD_DISPLAY

//��·״̬��־
typedef enum
{
  Nstart,//δ��ʼ״̬
  Straight,//��ֱ��
  Left_turn,//��ת
  Right_turn,//��ת
//  Cross,//ʮ��
  Uphill,//�µ�
  
}Road_Status;


#define Speed_filter_Period       5//��ֵ�˲�����

//������
#define MOTOR_1                         FTM0,CH4//����ӿ�
#define MOTOR_2                         FTM0,CH3//����ӿ�
//#define STEER_                          FTM1,CH0//����ӿ�
#define MOTOR1_DIR                       PTA13_OUT//����������
#define MOTOR2_DIR                       PTA12_OUT//����������
typedef struct{
  u8 Dir;
  s16 Speed;
}Motor_Status;          //���״̬�ṹ��

#define SERVO_MIDDLE    1462

typedef struct
{
  u16 Middle;
  u16 Now_duty;
}
Servo_Str;


//ADC

#define MYADC_1                          ADC0,AD13
#define MYADC_2                          ADC0,AD12
#define MYADC_3                          ADC1,AD10
#define MYADC_4                          ADC0,AD13
#define MYADC_5                          ADC1,AD15
#define MYADC_6                          ADC1,AD13
#define MYADC_7                          ADC1,AD10
#define MYADC_8                          ADC0,AD12
//����
#define SW1              PTB0_IN
#define SW2              PTA29_IN
#define SW3              PTA28_IN
#define SW4              PTA27_IN
#define SW5              PTA26_IN
#define SW6              PTA25_IN
#define SW7              PTA24_IN
#define SW8              PTA19_IN

//����
#define KEY1             PTB1_IN
#define KEY1_PRES        1
#define KEY2             PTA17_IN
#define KEY2_PRES        2
#define KEY3             PTA15_IN
#define KEY3_PRES        3   
#define KEY_Stop         PTD15_IN
#define KEY_Stop_PRES    4   

//������
#define BEEP PTD6_OUT

//��ˮ��
#define LED1             PTD5_OUT
#define LED2             PTD3_OUT


//PID�ṹ��
typedef struct                    //�ṹ�壬���PID��ر���
{
  float P;                        //����P
  float I;                        //����I
  float D;                        //����D
  float error[3];                 //���洢����
  float delat;                    //ÿ�εĵ��ڽ��
  float derr;                     //һ�����
  float dderr;                    //�������
  float result;                      //PID�����������������ʽ�����Գ�ֵ����Ϊ����ƽ��ʱ�����ֵ����Ҫ��������0Ҫ����ը��
  
  float target;                   //PID���ڵ�Ŀ��ֵ     
  float feedback;
  float UP_Limit;
  float LOW_Limit;
}PID_Struct;

typedef struct 
{
  float x_mid;
  float x_now;
  float p_mid ;
  float p_now;
  float kg;
  float ProcessNoise_Q;
  float MeasureNoise_R;
  float x_last1;
  float p_last1;
}Kalman_Date;

typedef struct
{
  float m_filter;
  float ResrcData_mem[2];
  float output_mem[2];
}Filter_1st_Str;

////ϵͳ״̬�ṹ��
//typedef struct
//{
//  long int Time_1ms;
//  
//}System_Status

#define     pit_delay_ms(PITn,ms)          pit_delay(PITn,ms * bus_clk_khz);        //PIT��ʱ ms
#define     pit_delay_us(PITn,us)          pit_delay(PITn,us * bus_clk_khz/1000);   //PIT��ʱ us

typedef enum{
  Unlock,
  Left_Lock,
  Right_Lock
}Black_View_Run_Status;


#define ALL_LINE            (240)
typedef struct
{
  int center[ALL_LINE];
  u8 halfwidth[ALL_LINE];
  int black_L[ALL_LINE];
  int black_R[ALL_LINE];
  u8 getLeft_flag[ALL_LINE];
  u8 getRight_flag[ALL_LINE];
  u8 hang_use;
}Image_hangData;


typedef struct
{
  u8 Three_Lie[3];
  u8 Three_lie_end[3];
}Image_lieData;


enum ISLAND_STATE
{
  NoIsland,
  Left_Island_pre,
  Right_Island_pre,
  Left_Island_in,
  Right_Island_in,
  Left_Island_out,
  Right_Island_out,
  Left_Wait_Next,
  Right_Wait_Next
};

#define Island_Center_Period_Const  (5)
typedef struct
{
  enum ISLAND_STATE State;
  
  u16 black_L[10];
  u16 black_R[10];
  
  u8 Image_Start_hang;
  
  u8  Correct_hang;     //�������ߵ���
  int In_Center;
  u8  In2Stay_cnt;
  
  int Out_Center;//�����������ı�׼
  
  u8  Stay_hang_use;
  u8  Stay2Out_flag;
  u16 Stay2Out_flag_delay;
  u16 Stay2Out_flag_delay_const;
  u8  Out_Allow_flag;
  
  u8  Stay2Out_cnt;
  
  u8  Out_center_Period;
  int Out_Center_[Island_Center_Period_Const];
  
  u8  Next_Island_flag;         //����һ������֮���ʱ����
  u16 Next_Island_flag_delay;
  u16 Next_Island_flag_delay_const;
}Island_Data;

typedef struct
{
  u8 Flag;
  u8 Delay;
  u8 Delay_const;
}Beep_Str;

enum Cross_STATE
{
  NoCross,//û��ʮ��
  R2Cross_Pre, //����������б��ʮ��
  L2Cross_Pre, //����������б��ʮ��
  R2Cross_True,//���������б��ʮ��
  L2Cross_True,//���������б��ʮ��
  Str2Cross//ֱ��ʮ��

};

typedef struct
{
  enum Cross_STATE State;
  int In_center;
  u8  Test_hang;
  u16  Cross_delay_cnt;
  u16  Cross_delay_cnt_const;
  u8   Cross_delay_flag;
}Cross_Data;


typedef struct
{
  float Run_Volts;
  u8 Run_flag;//���ܱ�־
}Cap_Run_Str;


typedef struct
{
  u16 _2Over_cnt;
  u16 _2Over_cnt_const;
  u8 test_allow_flag;
  u8 Start_Line_cnt;
}Start_line_Str;
#endif //_DEFINE_H