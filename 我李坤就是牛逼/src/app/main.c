#include "include.h"
#include "stdlib.h"
//u8 B_f=0;

u32 L_AD_Ave = 0;
u32 R_AD_Ave = 0;
u32 L_AD_Sum = 5000;
u32 R_AD_Sum = 5000;
Motor_Status Motor1;//���״̬�ṹ��
Motor_Status Motor2;//���״̬�ṹ��
int Speed_L_sum=0;
int Speed_R_sum=0;
s32 Duty_Motor1;//���ռ�ձ�
s32 Duty_Motor2;//���ռ�ձ�
PID_Struct Motor1_PID;//pid�ṹ��
PID_Struct Motor2_PID;//pid�ṹ��
PID_Struct Diff_PID;//����PID
PID_Struct Diff_Straight;//ֱ��PID
float Speed_goal1=25;//���ת��Ŀ��ֵ
float Speed_goal2=25;//���ת��Ŀ��ֵ
u16 Diff_goal=0;
int Diff_error=0;
u16 Master_Speed=25;
u16 ADC_Value;//ADCֵ
u16 Speed_stand;
int Weight_mean=0;
//***************************��ʱ����־λ*************************
u8 button_timeout=255;//����ʱ���־
long int Time_1ms=0;//ʱ����
//****************************************************************
u8 Road_Status_Flag=0;
u8 Switch_Status;//����״̬
u8 Key_status;//����״̬
u8 LCD_DISPLAY_FLAG=1;
u8 Motor_enable_Flag=1;
u8 Slow_Flag=0;
u8 Reduct_Flag=0;
u8 Blue_Start_Flag=0;//��������
u8 Key_Start_Flag=0;
//***********************��������ʱȫ�ֱ���**************************
float stand_p;
float stand_d;
int P_TEMP1=550;//201
int I_TEMP1=70;
int D_TEMP1=0;
int P_TEMP2=200;//90
int I_TEMP2=50;
int D_TEMP2=0;
signed int Speed_goal1_TEMP=80;//���ת��Ŀ��ֵ
signed int Speed_goal2_TEMP=80;//���ת��Ŀ��ֵ
u8 Image_Flag=1;
u8 LED_timeout=50;
u8 Dir_temp=1;
u16 hang=0;
int DIFF_UP=110;
int DIFF_DOWN=-110;
long int  temp_cnt=0;
u8 DIFF_PID_CHANGE_FLAG=0;
u16 temp_CNT_WATCH=0;
u8 Speed_max_to_min_diff;
u8 Acc_Limit=40;
float Acc_K=1;
u16 stand_AD_L = 0xffff;
u16 stand_AD_R = 0xffff;
u16 stand_AD   = 0Xffff;
PID_Struct Servo_PID;
int center_temp_test = 160;
u16 duty_temp=32767;
Cap_Run_Str Run_={.Run_Volts = 10};
Start_line_Str Start_line = {._2Over_cnt = 10000,
                            ._2Over_cnt_const = 10000};
//******************************************************************
void main()
{
  u8 Key;
  u16 i=0,j;
  System_Init();
  Brush_Color=Black;
  Motor1_PID.target=Speed_goal1;
  Motor2_PID.target=Speed_goal2;
  Duty_Motor1=10000;
  Duty_Motor2=10000;
while(1)
{
  Motor2_PID.P=P_TEMP2;
  Motor2_PID.I=I_TEMP2;
  Key = KEY_Scan();
  if(Key == KEY1_PRES)
  {
    LCD_init(FALSE);
    Disp_single_colour(White);
    LCD_DISPLAY_FLAG = 1;
  }
  else if(Key == KEY2_PRES)
  {
    LCD_DISPLAY_FLAG = 0;
  }
  else if(Key == KEY3_PRES)
  {
    Beep_Once(&Image_Island_Test_Beep);
    Key_Start_Flag = 1;
  }
  else if(Key == KEY_Stop_PRES)
  {

  }
//����ͷ�ɼ�һ��
//ͼ����  
    if(Image_Flag==1)
  {
    Image_Flag=0;
    ov7725_get_img();//ת�����������������ճ��ж� 
    image_process();
    if(LCD_DISPLAY_FLAG==1)
    {
      Send_Image_to_LCD(Image_fire);
      Send_Image_to_LCD(Image_fire);
      LCD_Draw_Line(Image_lie.Three_Lie[0],Image_lie.Three_lie_end[0],Image_lie.Three_Lie[0],160);  
      LCD_Draw_Line(Image_lie.Three_Lie[1],Image_lie.Three_lie_end[1],Image_lie.Three_Lie[1],160);  
      LCD_Draw_Line(Image_lie.Three_Lie[2],Image_lie.Three_lie_end[2],Image_lie.Three_Lie[2],160);
      LCD_Draw_Line(0,Island.Image_Start_hang,319,Island.Image_Start_hang);
//        LCD_Draw_Line(0,Image_hang.hang_use,319,Image_hang.hang_use);
      LCD_Draw_Line(0,Start_Point,319,Start_Point);
      LCD_Put_Int(100,100,"",L_AD_Ave,Red,White);
      LCD_Put_Int(100,120,"",R_AD_Ave,Red,White);
    }
    if(Memory_use_Flag==1)//�ı仺����
    {
      Image_fire=&Image_fire_Memory2[0];
    }
    else if(Memory_use_Flag==2)
    {
      Image_fire=&Image_fire_Memory1[0];
    }
  }
//  SCI_Send_Datas(UART5);
  if(Blue_Start_Flag==0)
  {
    cmt_pwm_duty(SERVO_MIDDLE);
    FTM_PWM_Duty(MOTOR_2,0xfff0);
  }
  
}
while(1);
}