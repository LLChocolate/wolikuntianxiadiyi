#ifndef __IMAGE_PROCESS_H__
#define __IMAGE_PROCESS_H__

#include "common.h"
//Ѱ�Һ��߲�Ѱ������ֵ
#define CENTER_             160
#define HALF_WIDTH          98
#define HALF_NEAR           115
#define VN_HALF             170
#define Island_HALF         106
#define Far_Point           85            //1m��  76���
#define Far_Half            80
#define Start_Point         100         //45cm��
#define Near_Point          150            //����
#define VN_Point            180
#define Island_Point        125

#define Image_Point(x,y)    (Image_fire[(x)][(y)/8]>>(7-((y)%8))&1)
#define AD_Near_hang        120
#define Turn_Point          100

#define Center_correct(hang)   ((hang-Start_Point)*(1-2)*1.0/(Far_Point-Start_Point)-1)


#define _1m_Left        108
#define _1m_Right       198
#define Near_Left       76
#define Near_Right      230


#define  Centerdiff_const    10
#define  Center_Filter_Period 5


#define Cross_Center_Period_Const  (10)

typedef enum
{
  turn_left  = 1,
  turn_right = 2,
  go_str     = 3
}_1m_Road_Status;
void image_process(void);
u8 get_black_line(unsigned char *ImageData_in,int hang);//��׽���� 
u8 get_black_line_without_Iteration(unsigned char *ImageData_in,int hang);
u8 CenterlineToDiff(int center);
void get_three_lie(void);
u8 Str_Cross_Test(void);
u8 double_AD(void);
u8 In_double_AD(void);
u8 Out_double_AD(void);
u8 Island_process(void);
u8 Elec_Island(void);
u8 In_Island(void);
int In_Island_center(u8* hang);
u8 Stay_Island(void);
u8 Out_Island(void);
u8 Image_Island_Test(void);
int Test_Far_Lie();
int Out_Island_Test(int* start_end, int* end_end);
int Stay2Out_test();
int Wait_Next_Island();
int Wait_Next_center(u8* hang);
u8 Cross_process(void);
u8 In_Cross_test();//б��ʮ�ּ��
u8 In_Cross();//б��ʮ��
u8 Cross_center_test(int* start_end, int* end_end);//�ͳ�����ʱ�����ĵ�Ĵ���һ��
u8 Cross_curve_test();
u8 Out_Cross(void);
u8 Cross_pre_test(void);
u8 Str_Cross(void);
u8 Find_Start_line(void);
u8 Start_Line_process(void);
u8 Cross_find_far_center();//��б��ʮ��ʱ��Ѱ����Զ������Ϊ��׼Ѱ�ұ߽�
u8 Power_Square_test(void);
int find_lie_end(int lie,int start_hang);//����ĳ�еľ�ͷ
#endif