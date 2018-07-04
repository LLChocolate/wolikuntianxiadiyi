#include "image_process.h"
#include "include.h"
#include "stdlib.h"
u8 ImageData[320];
float island_addline_k = 0;
float Cur_error = 0;

u8 diff_done_flag = 0;

u16 Start_line_delay = 0xffff;
u8  Start_line_flag  = 0;
u8  Start_line_cnt   = 0;

u8  road_filter_flag = 0;
Filter_1st_Str Center_Filter = {0.8,{0,0},{0,0}};
Filter_1st_Str In_Island_Center_Filter = {0.6,{0,0},{0,0}};
//Kalman_Date Center_Filter={0,0,0,0,0,0.1,40,0,0};

u8 Cross_flag;
u16 Cross_flag_delay;
u16 Cross_flag_delay_const = 1000;

Image_hangData Image_hang;
Image_lieData  Image_lie={{80,160,240},{0}};
Island_Data    Island={
                  .Correct_hang = 150,
                  .Stay_hang_use = Start_Point+10,//�������Ϲ����
                  .Image_Start_hang = 67,
                  .Next_Island_flag_delay_const = 2000,
                  .Stay2Out_flag_delay_const = 800
                    };
Cross_Data     Cross={
                  .Cross_delay_cnt = 100,
                  .Cross_delay_cnt_const = 100,
                  .Test_hang = 180
                    };
void image_process(void)
{
//  u8 max_temp;
//  u8 min_temp;
  diff_done_flag = 0;//ÿ��ѭ���Ŀ�ͷ��diff��ֵ��־����
  Image_hang.hang_use = 0;
  get_three_lie();
  if(Image_lie.Three_lie_end[1]<65)
    road_filter_flag = 1;
  else
    road_filter_flag = 0;
  Island_process();
  Cross_process();
//  Start_Line_process();
//  Out_Island();
//  if(Power_Square_test()==1)
//  {
//    Blue_Start_Flag = 0;
//  }
//  Out_Island();
    Slow_Flag=0;
    if(Island.State!=NoIsland)
    {
      
    }
    else
    {
      Slow_Flag=1;
      Image_hang.hang_use = 0;
      get_black_line(Image_fire[Start_Point],Start_Point);//45cm�����ĵ�
      if(Image_lie.Three_lie_end[0]>Image_hang.hang_use+5
         &&Image_lie.Three_lie_end[1]>Image_hang.hang_use+5
           &&Image_lie.Three_lie_end[2]>Image_hang.hang_use+5)//ȥ����ߵ�Ӱ��
      {
        get_black_line(Image_fire[Image_lie.Three_lie_end[1]+3],Image_lie.Three_lie_end[1]+3);
      }
      CenterlineToDiff(Image_hang.center[Image_hang.hang_use]);
    }
    if(LCD_DISPLAY_FLAG==1)
    {
      LCD_Put_Int(250,100,"cen:",Image_hang.center[Image_hang.hang_use],Red,White);
      LCD_Put_Int(250,120,"half",Image_hang.halfwidth[Image_hang.hang_use],Red,White);
      LCD_Put_Int(250,140,"L",Image_hang.getLeft_flag[Image_hang.hang_use],Red,White);
      LCD_Put_Int(250,160,"L_V",Image_hang.black_L[Image_hang.hang_use],Red,White);
      LCD_Put_Int(250,180,"R",Image_hang.getRight_flag[Image_hang.hang_use],Red,White);
      LCD_Put_Int(250,200,"R_V",Image_hang.black_R[Image_hang.hang_use],Red,White);
      switch(Island.State)
      {
      case NoIsland:
        LCD_PutString(250,220,"N",Red,White);
        break;
      case Left_Island_pre:
        LCD_PutString(250,220,"Lp",Red,White);
        break;
      case Right_Island_pre:
        LCD_PutString(250,220,"Rp",Red,White);
        break;
      case Left_Island_in:
        LCD_PutString(250,220,"Li",Red,White);
        break;
      case Right_Island_in:
        LCD_PutString(250,220,"Ri",Red,White);
        break;
      case Left_Island_out:
        LCD_PutString(250,220,"Lo",Red,White);
        break;
      case Right_Island_out:
        LCD_PutString(250,220,"Ro",Red,White);
        break;
      case Left_Wait_Next:
        LCD_PutString(250,220,"LW",Red,White);
        break;
      case Right_Wait_Next:
        LCD_PutString(250,220,"RW",Red,White);
        break;
      default:
        break;
      }
    }
    DIFF_PID_CHANGE_FLAG=0;//ʹ��ֱ��PID
}

u8 get_black_line(unsigned char *ImageData_in,int hang)//��׽����  
{
  int Middle=160;  //�����м�Ĭ��ΪCENNTER
  int ccd_start=10,ccd_end=310;  //ccdɨ�����10���յ�310   
  int Left_Count=0,Right_Count=0;//���Ҽ���Ϊ0
  int getleft_flag=0,getright_flag=0;//�ҵ����ұ�־0
  int _black_R,_black_L;//�������Ҷ�
  int _halfwidth = 100;//����һ����Ĭ��80
  static unsigned char first_run=0;//���ܵ�0
  u8 middle_black_flag = 0;
  int i=0;
  
  if(hang>239||hang<0)
    return 0;
  
  if(first_run==0)  //���ܵ���0��
  {
    
    first_run++;//���ܵ��1
  }
  else//���first_run!=0
  {
    Middle = (Image_hang.center[hang]+Image_lie.Three_Lie[1])/2;
    _halfwidth = Image_hang.halfwidth[hang];//halfwidth[hang];//һ��Ϊhalfwidth[hang]
  }
  for(i=0;i<40;i++)
    for(int k=0;k<8;k++)
      ImageData[i*8+k] = (ImageData_in[i]>>(7-k))&0x01;
  
  Right_Count = Middle;//�Ѻ����м�ֵ�����Ҽ������
  while(!(ImageData[Right_Count+3]==1 
          && ImageData[Right_Count+2]==1
            && ImageData[Right_Count+1]==1)
        && Right_Count < ccd_end)//�������Ч����û���ҵ����������ڵ�
    Right_Count++;//���м�λ�ÿ�ʼ���������������������㶼�Ǻڵ�ͣ
  
  if(Right_Count<ccd_end)//�������Ч��Χ��
  {
    _black_R = Right_Count;
    Image_hang.getRight_flag[hang]=1;
    getright_flag=1;
  }
  else
  {
    getright_flag=0;
    Image_hang.getRight_flag[hang]=0;
  }
  Left_Count = Middle;
  while(!(ImageData[Left_Count-3]==1 
          && ImageData[Left_Count-2]==1
            && ImageData[Left_Count-1]==1)
        && Left_Count > ccd_start)	  
    Left_Count--;
  if(Left_Count > ccd_start)
  {
    _black_L = Left_Count; 
    Image_hang.getLeft_flag[hang]=1;
    getleft_flag=1;
  } 
  else
  {
    getleft_flag=0;
    Image_hang.getLeft_flag[hang]=0;
  }
  if(Left_Count==Middle||Right_Count==Middle)//�м䲿��Ϊ��ɫ
  {
    getright_flag=0;
    Image_hang.getRight_flag[hang]=0;
    getleft_flag=0;
    Image_hang.getLeft_flag[hang]=0;//flag������
    middle_black_flag = 1;
  }
  else
  {
    middle_black_flag = 0;
  }
  

  if(middle_black_flag==1&&hang<210)
  {
    Image_hang.center[hang+3] = Middle;
    Image_hang.halfwidth[hang+3]=_halfwidth+8;
    get_black_line(Image_fire[hang+3],hang+3);
  }
  else if(getleft_flag==0 && getright_flag==0)//���ұ߽綼û���ҵ�
  {
    
  }
  else if(getleft_flag!=1 && getright_flag==1)//�ҵ��ұ߽�
  {
    Middle = _black_R-_halfwidth;//�����м�λ��Ϊ�ұ߽�-���߿��һ��
    _black_L = _black_R - _halfwidth*2;//�������λ��Ϊ�ұ߽�-���߿�
  }
  else if(getleft_flag==1 && getright_flag!=1)//�ҵ���߽�
  {
    Middle = _black_L+_halfwidth;
    _black_R = _black_L + _halfwidth*2;
  }
  else if(getleft_flag==1 && getright_flag==1) //���ұ߽綼�ҵ�
  {
    _halfwidth=(int)((_black_R - _black_L)/2.0); //�����⵽�����Ҳ�ֵ����160��ȡ�м�λ��
    
      
    if(_halfwidth < 100)//����޷� 
      _halfwidth = 100;
    else if(_halfwidth >140)
      _halfwidth = 140; 
    Middle = (int)((_black_R + _black_L)/2.0);
  }
  if(Middle<35) //���ĵ��޷� 
    Middle=35;
  else if(Middle>285)
    Middle=285;
  
  //data record ��¼������������
  Image_hang.center[hang] = Middle + Center_correct(hang);
  if(Image_hang.hang_use<hang)
  {
    Image_hang.hang_use = hang;
  }
  if(_black_L>319)_black_L=319;
  else if(_black_L<0)_black_L=0;
  Image_hang.black_L[hang] = _black_L;
  if(_black_R>319)_black_R=319;
  else if(_black_R<0)_black_R=0;
  Image_hang.black_R[hang] = _black_R;
  Image_hang.halfwidth[hang] = _halfwidth;
  return 0;
}

u8 get_black_line_without_Iteration(unsigned char *ImageData_in,int hang)//��׽����  
{
  int Middle=160;  //�����м�Ĭ��ΪCENNTER
  int ccd_start=10,ccd_end=310;  //ccdɨ�����10���յ�310   
  int Left_Count=0,Right_Count=0;//���Ҽ���Ϊ0
  int getleft_flag=0,getright_flag=0;//�ҵ����ұ�־0
  int _black_R,_black_L;//�������Ҷ�
  int _halfwidth = 100;//����һ����Ĭ��80
  static unsigned char first_run=0;//���ܵ�0
  u8 middle_black_flag = 0;
  int i=0;
  
  if(hang>239||hang<0)
    return 0;
  
  if(first_run==0)  //���ܵ���0��
  {
    
    first_run++;//���ܵ��1
  }
  else//���first_run!=0
  {
    Middle = Image_hang.center[hang];
    _halfwidth = Image_hang.halfwidth[hang];//halfwidth[hang];//һ��Ϊhalfwidth[hang]
  }
  for(i=0;i<40;i++)
    for(int k=0;k<8;k++)
      ImageData[i*8+k] = (ImageData_in[i]>>(7-k))&0x01;
  
  Right_Count = Middle;//�Ѻ����м�ֵ�����Ҽ������
  while(!(ImageData[Right_Count+3]==1 
          && ImageData[Right_Count+2]==1
            && ImageData[Right_Count+1]==1)
        && Right_Count < ccd_end)//�������Ч����û���ҵ����������ڵ�
    Right_Count++;//���м�λ�ÿ�ʼ���������������������㶼�Ǻڵ�ͣ
  
  if(Right_Count<ccd_end)//�������Ч��Χ��
  {
    _black_R = Right_Count;
    Image_hang.getRight_flag[hang]=1;
    getright_flag=1;
  }
  else
  {
    getright_flag=0;
    Image_hang.getRight_flag[hang]=0;
  }
  Left_Count = Middle;
  while(!(ImageData[Left_Count-3]==1 
          && ImageData[Left_Count-2]==1
            && ImageData[Left_Count-1]==1)
        && Left_Count > ccd_start)	  
    Left_Count--;
  if(Left_Count > ccd_start)
  {
    _black_L = Left_Count; 
    Image_hang.getLeft_flag[hang]=1;
    getleft_flag=1;
  } 
  else
  {
    getleft_flag=0;
    Image_hang.getLeft_flag[hang]=0;
  }

  if(getleft_flag==0 && getright_flag==0)//���ұ߽綼û���ҵ�
  {
    
  }
  else if(getleft_flag!=1 && getright_flag==1)//�ҵ��ұ߽�
  {
    Middle = _black_R-_halfwidth;//�����м�λ��Ϊ�ұ߽�-���߿��һ��
    _black_L = _black_R - _halfwidth*2;//�������λ��Ϊ�ұ߽�-���߿�
  }
  else if(getleft_flag==1 && getright_flag!=1)//�ҵ���߽�
  {
    Middle = _black_L+_halfwidth;
    _black_R = _black_L + _halfwidth*2;
  }
  else if(getleft_flag==1 && getright_flag==1) //���ұ߽綼�ҵ�
  {
    _halfwidth=(int)((_black_R - _black_L)/2.0); //�����⵽�����Ҳ�ֵ����160��ȡ�м�λ��
    
      
    if(_halfwidth < 100)//����޷� 
      _halfwidth = 100;
    else if(_halfwidth >140)
      _halfwidth = 140; 
    Middle = (int)((_black_R + _black_L)/2.0);
  }
  if(Middle<35) //���ĵ��޷� 
    Middle=35;
  else if(Middle>285)
    Middle=285;
  
  //data record ��¼������������
  Image_hang.center[hang] = Middle + Center_correct(hang);
  if(_black_L>319)_black_L=319;
  else if(_black_L<0)_black_L=0;
  Image_hang.black_L[hang] = _black_L;
  if(_black_R>319)_black_R=319;
  else if(_black_R<0)_black_R=0;
  Image_hang.black_R[hang] = _black_R;
  Image_hang.halfwidth[hang] = _halfwidth;
  return 0;
}


void get_three_lie(void)
{
  u16 left_lie=Image_lie.Three_Lie[0],middle_lie=Image_lie.Three_Lie[1],right_lie=Image_lie.Three_Lie[2];
  u8 Left_flag=0,Right_flag=0,middle_flag=0;
  u8 Left_point,Middle_point,Right_point;
  Left_point=239;
  while(!(Image_Point(Left_point,left_lie)==1
          &&Image_Point(Left_point-1,left_lie)==1
            &&Image_Point(Left_point-2,left_lie)==1)&&Left_point>=2)
    Left_point--;
  if(Left_point!=239)//���߳�ʼ��Ϊ��
  {
    Left_flag=1;
  }
  Image_lie.Three_lie_end[0]=Left_point;
  Middle_point=239;
  while(!(Image_Point(Middle_point,middle_lie)==1
          &&Image_Point(Middle_point-1,middle_lie)==1
            &&Image_Point(Middle_point-2,middle_lie)==1)&&Middle_point>=2)
    Middle_point--;
  if(Middle_point!=239)//���߳�ʼ��Ϊ��
  {
    middle_flag=1;
  }
  
  Image_lie.Three_lie_end[1]=Middle_point;
  Right_point=239;
  while(!(Image_Point(Right_point,right_lie)==1
          &&Image_Point(Right_point-1,right_lie)==1
            &&Image_Point(Right_point-2,right_lie)==1)&&Right_point>=2)
    Right_point--;
  if(Right_point!=239)//���߳�ʼ��Ϊ��
  {
    Right_flag=1;
  }
  Image_lie.Three_lie_end[2]=Right_point;
}


u8 CenterlineToDiff(int center)
{
  int real_center;
  int In_Island_center;
  if(diff_done_flag == 1)return 0;
  

//*********************�����ĵ��ֵ�˲�*************************************  
  
  real_center = filter_1st(center,&Center_Filter);
  In_Island_center = filter_1st(center,&In_Island_Center_Filter);
  diff_done_flag = 1;
  if(Island.State==Left_Island_pre
     ||Island.State==Right_Island_pre)
    Diff_error = 160 - real_center;
  else if(Island.State==Left_Island_in
     ||Island.State==Left_Island_in)
    Diff_error = 160 - In_Island_center;
  else
    Diff_error = 160 - center;
  
  return 0;
}


u8 Str_Cross_Test(void)
{
  u8 i,cnt=0;
  for(i=0;i<5;i++)
  {
    if(sum_point(Image_fire[Start_Point+i*5],40)<=5)
      cnt++;
  }
  if(cnt>1&&(Image_lie.Three_lie_end[0]<120||Image_lie.Three_lie_end[2]<120))//����û�м��
  {
    return 1;
  }
  else 
    return 0;
}

u8 double_AD(void)
{
  if((L_AD_Ave>1900)&&(R_AD_Ave>1900))
  {
    return 1;
  }
  else 
  {
    return 0;
  }
}

u8 In_double_AD(void)
{
  if(((L_AD_Ave>900)&&(R_AD_Ave>1900))
     ||((L_AD_Ave>1900)&&(R_AD_Ave>900))
     ||(L_AD_Ave+R_AD_Ave>3200))
  {
    return 1;
  }
  else 
  {
    return 0;
  }
}

u8 Out_double_AD(void)
{
  if((R_AD_Ave>2000)
     ||(L_AD_Ave>2000))
  {
    return 1;
  }
  else 
  {
    return 0;
  }
}

u8 Island_process(void)
{
  Elec_Island();//��ż�⣬ֻ���뻷�ͳ�����ʱ����
  In_Island();//�뻷��
  Stay_Island();//�ڻ�����
  Out_Island();//������
  Wait_Next_Island();//��ֹ�ٴν��뻷��
  return 0;
}

u8 Elec_Island(void)
{
  u8 doublt_island = 0;
  if(Island.State == NoIsland)//�޻���ʱ��⻷��
  {
    doublt_island = Image_Island_Test();
    if(doublt_island!=0&&In_double_AD()==1)
    {
      Island.State = doublt_island;
      LED1 = 0;
    }
  }
  
  if(Island.State == Left_Island_out)
  {
    if(Out_double_AD()==1)
    {
      Island.State = Left_Wait_Next;//�ȴ���һ��������ʱ����
      Island.Next_Island_flag = 1;
      Island.Next_Island_flag_delay = Island.Next_Island_flag_delay_const;
      LED1 = 1;
    }
  }
  else if(Island.State == Right_Island_out)
  {
    if(Out_double_AD()==1)
    {
      Island.State = Right_Wait_Next;//�ȴ���һ��������ʱ����
      Island.Next_Island_flag = 1;
      Island.Next_Island_flag_delay = Island.Next_Island_flag_delay_const;
      LED1 = 1;
    }
  }
  return 0;
}

u8 In_Island(void)
{
  int center;
  int center_use;
  u8  Impulse_hang = 0;
  if(Island.State!=Left_Island_pre&&Island.State!=Right_Island_pre)//���ڴ�״̬��
    return 1;//ֱ�ӷ���
  else 
  {
    center = In_Island_center(&Impulse_hang);//Ѱ�����ĵ�
    if(center == -1)//Ѱ��ʧ�ܻ�������ȫ���뻷��
    {
      Island.In2Stay_cnt ++;
      CenterlineToDiff(Island.In_Center);//ʹ����һ�εľ�ֵ
      if(Island.In2Stay_cnt>=13)//����10��Ѱ��ʧ�ܣ���Ϊ���뻷��������״̬
      {
        if(Island.State==Left_Island_pre)
        {
          Island.State = Left_Island_in;
          Island.Stay2Out_flag = 1;
          Island.Stay2Out_flag_delay = Island.Stay2Out_flag_delay_const;
        }
        else if(Island.State==Right_Island_pre)
        {
          Island.State = Right_Island_in;
          Island.Stay2Out_flag = 1;
          Island.Stay2Out_flag_delay = Island.Stay2Out_flag_delay_const;
        }
      }
    }
    else
    {
      if(Island.State==Left_Island_pre)
      {
        if(center>Image_lie.Three_Lie[1]+50)//ͻ���̫����
        {
          Island.In2Stay_cnt++;
          Image_hang.hang_use = 0;
          get_black_line(Image_fire[Start_Point],Start_Point);//45cm�����ĵ�
          if(Image_lie.Three_lie_end[0]>Image_hang.hang_use+5
             &&Image_lie.Three_lie_end[1]>Image_hang.hang_use+5
               &&Image_lie.Three_lie_end[2]>Image_hang.hang_use+5)//ȥ����ߵ�Ӱ��
          {
            get_black_line(Image_fire[Image_lie.Three_lie_end[1]+3],Image_lie.Three_lie_end[1]+3);
          }
          center_use = Image_hang.center[Image_hang.hang_use];
        }
        else
        {
          Island.In2Stay_cnt = 0;//����
          if(Impulse_hang<85)
            center_use = ((center - (center - 319)*(Impulse_hang - Start_Point)*1.0/(Impulse_hang - Island.Correct_hang)) + 0)/2 + 20;//���ߣ����������ƣ�
          else
            center_use = ((center - (center - 319)*(Impulse_hang - Start_Point)*1.0/(Impulse_hang - Island.Correct_hang)) + 0)/2 + 13;//���ߣ����������ƣ�
          if(center_use<35)
            center_use = 35;
        }
        Island.In_Center = center_use;//������һ�ε����ĵ�
        CenterlineToDiff(center_use);
      }
      else if(Island.State==Right_Island_pre)
      {
        if(center<Image_lie.Three_Lie[1]-50)//ͻ���̫����
        {
          Island.In2Stay_cnt++;
          Image_hang.hang_use = 0;
          get_black_line(Image_fire[Start_Point],Start_Point);//45cm�����ĵ�
          if(Image_lie.Three_lie_end[0]>Image_hang.hang_use+5
             &&Image_lie.Three_lie_end[1]>Image_hang.hang_use+5
               &&Image_lie.Three_lie_end[2]>Image_hang.hang_use+5)//ȥ����ߵ�Ӱ��
          {
            get_black_line(Image_fire[Image_lie.Three_lie_end[1]+3],Image_lie.Three_lie_end[1]+3);
          }
          center_use = Image_hang.center[Image_hang.hang_use];
        }
        else
        {
          Island.In2Stay_cnt = 0;//����
          center_use = ((center - (center - 0)*(Impulse_hang - Start_Point)*1.0/(Impulse_hang - Island.Correct_hang)) + 319)/2;
          if(center_use>285)
            center_use = 285;
        }
        Island.In_Center = center_use;//������һ�ε����ĵ�
        CenterlineToDiff(center_use);
      }
      //�����ã���ʾ���ĵ�
      if(LCD_DISPLAY_FLAG==1)
      {
        LCD_DrawBigPoint(center_use,Start_Point,Blue);//���еߵ�
      }
    }
  }
  return 0;
}

int In_Island_center(u8* hang)//�뻷��ʱѰ��ͻ���+����
{
  int Middle;
  int center;
  int ccd_start=10,ccd_end=310;  //ccdɨ�����10���յ�310   
  int Left_Count=0,Right_Count=0;//���Ҽ���Ϊ0
  int Diff_L[19],Diff_R[19];//һ�ײ��
  u16 In_black_L[20];
  u16 In_black_R[20];
  u8  Impulse_L_Flag = 0,Impulse_R_Flag = 0;//һ�ײ���г��ֽ�Ծ
  u8 i = 0,j = 0;
  u8 *ImageData_in;
  
  Middle = Test_Far_Lie();//��65�п�ʼ��255��
  if(Island.State == Right_Island_pre)
  {
    if(Middle>200)return -1;//ǰ��ֱ���Ѿ���������
  }
  else if(Island.State == Left_Island_pre)
  {
    if(Middle<120)return -1;//ǰ��ֱ���Ѿ���������
  }
  
  for(i=0;i<20;i++)//20��
  {
    ImageData_in = Image_fire[i*3+Island.Image_Start_hang];
    for(j=0;j<40;j++)
      for(u8 k=0;k<8;k++)
        ImageData[j*8+k] = (ImageData_in[j]>>(7-k))&0x01;
    if(Island.State == Right_Island_pre)
    {
      Right_Count = Middle;//�Ѻ����м�ֵ�����Ҽ������
      while(!(ImageData[Right_Count+3]==1 
              && ImageData[Right_Count+2]==1
                && ImageData[Right_Count+1]==1)
            && Right_Count < ccd_end)//�������Ч����û���ҵ����������ڵ�
        Right_Count++;//���м�λ�ÿ�ʼ���������������������㶼�Ǻڵ�ͣ
      if(Right_Count<ccd_end)//�������Ч��Χ��
      {
        In_black_R[i] = Right_Count;
      }
      else if(Right_Count<Image_lie.Three_Lie[1]+10)
      {
        In_black_R[i] = ccd_end;
      }
      else
      {
        In_black_R[i] = ccd_end;
      }
    }
    else if(Island.State == Left_Island_pre)
    {
      Left_Count = Middle;
      while(!(ImageData[Left_Count-3]==1 
              && ImageData[Left_Count-2]==1
                && ImageData[Left_Count-1]==1)
            && Left_Count > ccd_start)	  
        Left_Count--;
      if(Left_Count > ccd_start)
      {
        In_black_L[i] = Left_Count; 
      }
      else if(Left_Count>Image_lie.Three_Lie[1]-10)
      {
        In_black_L[i] = ccd_start;
      }
      else
      {
        In_black_L[i] = ccd_start;
      }
    }
  }
  
  if(Island.State == Right_Island_pre)
  {
    for(i=0;i<19;i++)
    {
      Diff_R[i] = In_black_R[i+1] - In_black_R[i];
      if(Diff_R[i]>30)
      {
        Impulse_R_Flag = 1;
        center = In_black_R[i];//������ת����
        *hang   = i*3+Island.Image_Start_hang;
        break;
      }
    }
  }
  else if(Island.State == Left_Island_pre)
  {
    for(i=0;i<19;i++)
    {
      Diff_L[i] = In_black_L[i+1] - In_black_L[i];
      if(Diff_L[i]<-30)
      {
        Impulse_L_Flag = 1;//���ֳ弤
        center = In_black_L[i];//������ת����
        *hang   = i*3+Island.Image_Start_hang;
        break;
      }
    }
  }
  if(Impulse_R_Flag==0&&Impulse_L_Flag==0)
  {
    return -1;//û�г���
  }
  else 
  {
    return center;
  }
}

u8 Stay_Island(void)
{
  static u8  center_Period;
  static int Center_[Island_Center_Period_Const];
  static int Center_Sum = 0;
  if(Island.State!=Left_Island_in&&Island.State!=Right_Island_in)
    return 1;
  
//������ͨ���Ѱ������
  Image_hang.hang_use = 0;
  get_black_line(Image_fire[Island.Stay_hang_use],Island.Stay_hang_use);//
  if(Image_lie.Three_lie_end[0]>Image_hang.hang_use+5
     &&Image_lie.Three_lie_end[1]>Image_hang.hang_use+5
       &&Image_lie.Three_lie_end[2]>Image_hang.hang_use+5)//ȥ����ߵ�Ӱ��
  {
    get_black_line(Image_fire[Image_lie.Three_lie_end[1]+3],Image_lie.Three_lie_end[1]+3);
  }
  CenterlineToDiff(Image_hang.center[Image_hang.hang_use]);
  
  if(center_Period > (Island_Center_Period_Const - 1))
    center_Period = 0;

  Center_Sum  -= Center_[center_Period];
  Center_[center_Period] = Image_hang.center[Image_hang.hang_use];
  Center_Sum  += Center_[center_Period];
  center_Period++;  
  
  if(Island.Out_Allow_flag==1)//����ı�״̬
  {
    if(Stay2Out_test()==1)
      Island.Stay2Out_cnt ++;
    else 
      Island.Stay2Out_cnt = 0;
    if(Island.Stay2Out_cnt>1)//�������β⵽ͻ���״̬�ı�
    {
      if(Island.State==Left_Island_in)
      {
        Island.Out_Center = (Center_Sum + 35*Island_Center_Period_Const)/2;
        Island.State = Left_Island_out;
      }
      else if(Island.State==Right_Island_in)
      {
        Island.Out_Center = (Center_Sum + 285*Island_Center_Period_Const)/2;
        Island.State = Right_Island_out;
      }
    }
  }

  return 0;
}

u8 Out_Island(void)
{
  static int Start_End, End_End;
  int center_impulse;//�洢ͻ�������
  int center_use;
  
  //����������־
  if(Island.State!=Right_Island_out&&Island.State!=Left_Island_out)
    return 1;
  
  center_impulse = Out_Island_Test(&Start_End,&End_End);
  if(Island.State==Right_Island_out)//����
  {
    center_use = ((center_impulse - (center_impulse - 0)*(End_End - Start_Point)*1.0/(End_End - Start_End)) + 319)/2-20;
    if((center_impulse!=-1) && (center_use>Image_lie.Three_Lie[1]+60))//��Ч�Լ���
    {
//      Beep_Once(&Image_Island_Test_Beep);
      if(center_use > 285)
        center_use = 285;
      CenterlineToDiff(center_use);
      if(Island.Out_center_Period > (Island_Center_Period_Const - 1))
        Island.Out_center_Period = 0;
    
      Island.Out_Center  -= Island.Out_Center_[Island.Out_center_Period];
      Island.Out_Center_[Island.Out_center_Period] = center_use;
      Island.Out_Center  += Island.Out_Center_[Island.Out_center_Period];
      Island.Out_center_Period++;  
//�����ã���ʾ���ĵ�
      if(LCD_DISPLAY_FLAG==1)
      {
        LCD_DrawBigPoint(center_use,Start_Point,Cyan);//���еߵ�
      }
    }
    else if((center_impulse!=-1)&&End_End<Start_Point-15)
    {
      Image_hang.hang_use = 0;
      get_black_line(Image_fire[Start_Point],Start_Point);//45cm�����ĵ�
      if(Image_lie.Three_lie_end[0]>Image_hang.hang_use+5
         &&Image_lie.Three_lie_end[1]>Image_hang.hang_use+5
           &&Image_lie.Three_lie_end[2]>Image_hang.hang_use+5)//ȥ����ߵ�Ӱ��
      {
        get_black_line(Image_fire[Image_lie.Three_lie_end[1]+3],Image_lie.Three_lie_end[1]+3);
      }
      if(Image_hang.center[Image_hang.hang_use]>Image_lie.Three_Lie[1]+60)
        CenterlineToDiff(Image_hang.center[Image_hang.hang_use]);
      else if(Island.Out_Center/Island_Center_Period_Const>Image_lie.Three_Lie[1]+60)
        CenterlineToDiff(Island.Out_Center/Island_Center_Period_Const);
      else 
      {
        CenterlineToDiff(Image_lie.Three_Lie[1]+80);
//        Beep_Once(&Image_Island_Test_Beep);
      }
    }
    else//����ʧ�ܣ�ʹ��֮ǰ��������ĵ�
    {
      if(Island.Out_Center/Island_Center_Period_Const>Image_lie.Three_Lie[1]+60)
        CenterlineToDiff(Island.Out_Center/Island_Center_Period_Const);
      else 
      {
        CenterlineToDiff(Image_lie.Three_Lie[1]+80);
//        Beep_Once(&Image_Island_Test_Beep);
      }
//�����ã���ʾ���ĵ�
      if(LCD_DISPLAY_FLAG==1)
      {
        LCD_DrawBigPoint(Island.Out_Center/Island_Center_Period_Const,Start_Point,Magenta);//���еߵ�
//        LCD_DrawBigPoint(Island.Out_Center,Start_Point,Magenta);//���еߵ�
      }
    }
  }
  else if(Island.State==Left_Island_out)
  {
    center_use = ((center_impulse - (center_impulse - 319)*(End_End - Start_Point)*1.0/(End_End - Start_End)) + 0)/2+25;
    if((center_impulse!=-1)&&(center_use<Image_lie.Three_Lie[1]-60))//��Ч�Լ���
    {
      if(center_use < 35)
        center_use = 35;
      CenterlineToDiff(center_use);
      if(Island.Out_center_Period > (Island_Center_Period_Const - 1))
        Island.Out_center_Period = 0;
    
      Island.Out_Center  -= Island.Out_Center_[Island.Out_center_Period];
      Island.Out_Center_[Island.Out_center_Period] = center_use;
      Island.Out_Center  += Island.Out_Center_[Island.Out_center_Period];
      Island.Out_center_Period++;  
//�����ã���ʾ���ĵ�
      if(LCD_DISPLAY_FLAG==1)
      {
        LCD_DrawBigPoint(center_use,Start_Point,Cyan);//���еߵ�
      }
    }
    else if((center_impulse!=-1)&&End_End<Start_Point-15)
    {
      Image_hang.hang_use = 0;
      get_black_line(Image_fire[Start_Point],Start_Point);//45cm�����ĵ�
      if(Image_lie.Three_lie_end[0]>Image_hang.hang_use+5
         &&Image_lie.Three_lie_end[1]>Image_hang.hang_use+5
           &&Image_lie.Three_lie_end[2]>Image_hang.hang_use+5)//ȥ����ߵ�Ӱ��
      {
        get_black_line(Image_fire[Image_lie.Three_lie_end[1]+3],Image_lie.Three_lie_end[1]+3);
      }
      if(Image_hang.center[Image_hang.hang_use]<Image_lie.Three_Lie[1]-60)
        CenterlineToDiff(Image_hang.center[Image_hang.hang_use]);
      else if(Island.Out_Center/Island_Center_Period_Const<Image_lie.Three_Lie[1]-60)
        CenterlineToDiff(Island.Out_Center/Island_Center_Period_Const);
      else 
      {
        CenterlineToDiff(Image_lie.Three_Lie[1]-80);
//        Beep_Once(&Image_Island_Test_Beep);
      }
    }
    else//����ʧ�ܣ�ʹ��֮ǰ��������ĵ�
    {
      if(Island.Out_Center/Island_Center_Period_Const<Image_lie.Three_Lie[1]-60)
        CenterlineToDiff(Island.Out_Center/Island_Center_Period_Const);
      else 
      {
        CenterlineToDiff(Image_lie.Three_Lie[1]-80);
//        Beep_Once(&Image_Island_Test_Beep);
      }
//�����ã���ʾ���ĵ�
      if(LCD_DISPLAY_FLAG==1)
      {
        LCD_DrawBigPoint(Island.Out_Center/Island_Center_Period_Const,Start_Point,Magenta);//���еߵ�
//        LCD_DrawBigPoint(Island.Out_Center,Start_Point,Magenta);//���еߵ�
      }
    }
  }
  return 0;
}

u8 Image_Island_Test(void)//��׽����  
{
  int ccd_start=10,ccd_end=310;  //ccdɨ�����10���յ�310   
  int Left_Count=0,Right_Count=0;//���Ҽ���Ϊ0
  int Diff_L[9],Diff_R[9];//һ�ײ��
  int DDiff_L[8],DDiff_R[8];//���ײ��
  int   Liner_L_cnt  = 0,Liner_R_cnt  = 0;
  u8    Liner_L_flag = 0,Liner_R_flag = 0;
  u8    Impulse_L_Flag = 0,Impulse_R_Flag = 0;//һ�ײ���г��ֽ�Ծ
  u8 i = 0,j = 0;
  u8 *ImageData_in;
  
  for(i=0;i<10;i++)//10��
  {
    ImageData_in = Image_fire[i*4+Island.Image_Start_hang];
    for(j=0;j<40;j++)
      for(u8 k=0;k<8;k++)
        ImageData[j*8+k] = (ImageData_in[j]>>(7-k))&0x01;
    
    Right_Count = Image_lie.Three_Lie[1];//�Ѻ����м�ֵ�����Ҽ������
    while(!(ImageData[Right_Count+3]==1 
            && ImageData[Right_Count+2]==1
              && ImageData[Right_Count+1]==1)
          && Right_Count < ccd_end)//�������Ч����û���ҵ����������ڵ�
      Right_Count++;//���м�λ�ÿ�ʼ���������������������㶼�Ǻڵ�ͣ
    if(Right_Count<ccd_end)//�������Ч��Χ��
    {
      Island.black_R[i] = Right_Count;
    }
    else if(Right_Count<Image_lie.Three_Lie[1]+10)
    {
      Island.black_R[i] = ccd_end;
    }
    else
    {
      Island.black_R[i] = ccd_end;
    }
    Left_Count = Image_lie.Three_Lie[1];
    while(!(ImageData[Left_Count-3]==1 
            && ImageData[Left_Count-2]==1
              && ImageData[Left_Count-1]==1)
          && Left_Count > ccd_start)	  
      Left_Count--;
    if(Left_Count > ccd_start)
    {
      Island.black_L[i] = Left_Count; 
    }
    else if(Left_Count>Image_lie.Three_Lie[1]-10)
    {
      Island.black_L[i] = ccd_start;
    }
    else
    {
      Island.black_L[i] = ccd_start;
    }
  }
  for(i=0;i<9;i++)
  {
    Diff_L[i] = Island.black_L[i+1] - Island.black_L[i];
    Diff_R[i] = Island.black_R[i+1] - Island.black_R[i];
  }
  for(i=0;i<8;i++)
  {
    DDiff_L[i] = Diff_L[i+1] - Diff_L[i];
    DDiff_R[i] = Diff_R[i+1] - Diff_R[i];
    if(Abs_(DDiff_L[i])<5&&Abs_(Diff_L[i])>0)
      Liner_L_cnt++;
    if(Abs_(DDiff_R[i])<5&&Abs_(Diff_R[i])>0)
      Liner_R_cnt++;
    if(DDiff_L[i]<-23
       &&Island.black_L[i]>30
       &&Liner_L_cnt>i/2
       &&Liner_L_cnt>0)
    {
      Impulse_L_Flag=1;//���ֳ弤
    }
    if(DDiff_R[i]>23
       &&Island.black_R[i]<289
       &&Liner_R_cnt>i/2
       &&Liner_R_cnt>0)
    {
      Impulse_R_Flag=1;
    }
  }
  if(Liner_L_cnt>5)Liner_L_flag = 1;
  if(Liner_R_cnt>5)Liner_R_flag = 1;
  
  if(Impulse_R_Flag&&Impulse_L_Flag)
    return 0;
  else if(Liner_L_flag&&Impulse_R_Flag==1)
    return Right_Island_pre;
  else if(Liner_R_flag&&Impulse_L_Flag==1)
    return Left_Island_pre;
  else 
    return 0;
}

int Test_Far_Lie()//���뻷��ʱ����Զ�����ڵ��У��Ӵ��п�ʼ������Ѱ��ͻ���
{
  u8 Far_Lie[20];
  u8 Temp_point;
  u8 i;
  for(i=0;i<20;i++)
  {
    Temp_point=180;
    while(!(Image_Point(Temp_point,65+i*10)==1
          &&Image_Point(Temp_point-1,65+i*10)==1
            &&Image_Point(Temp_point-2,65+i*10)==1)&&Temp_point>=10)
    Temp_point--;
    Far_Lie[i] = Temp_point;
  }
  return min_u8_index(Far_Lie,20)*10+65;
}

int Out_Island_Test(int* start_end, int* end_end)//��ʼ�����е��յ��ĩβ�е��յ�
{
  u8 Far_Lie[30];
  int Diff_Far_Lie[29];//һ�ײ��
  int DDiff_Far_Lie[28];//���ײ��
  int Liner_cnt  = 0;
  int out_center = -1;
  u8  Impulse_flag = 0;
  
  u8 Temp_point;
  u8 i;
  if(Island.State==Right_Island_out)
  {
    for(i=0;i<30;i++)
    {
      Temp_point=200;
      while(!(Image_Point(Temp_point,0+i*4)==1
            &&Image_Point(Temp_point-1,0+i*4)==1
              &&Image_Point(Temp_point-2,0+i*4)==1)&&Temp_point>=10)
      Temp_point--;
      Far_Lie[i] = Temp_point;
    }
    for(i=0;i<29;i++)
    {
      Diff_Far_Lie[i] = Far_Lie[i+1] - Far_Lie[i];
    }
    for(i=0;i<28;i++)
    {
      DDiff_Far_Lie[i] = Diff_Far_Lie[i+1] - Diff_Far_Lie[i];
      if(Abs_(DDiff_Far_Lie[i])<5)//���Ա�־
        Liner_cnt++;
      if(DDiff_Far_Lie[i]<-15&&Liner_cnt>i/2&&Liner_cnt>0)//֮ǰ���ԣ����ֳ弤
      {
        Impulse_flag = 1;
        break;
      }
    }
    if(Impulse_flag==1)
      out_center = 4*i;//��¼ͻ���
  }
  else if(Island.State==Left_Island_out)
  {
    for(i=0;i<30;i++)
    {
      Temp_point=200;
      while(!(Image_Point(Temp_point,319-i*4)==1
            &&Image_Point(Temp_point-1,319-i*4)==1
              &&Image_Point(Temp_point-2,319-i*4)==1)&&Temp_point>=10)
      Temp_point--;
      Far_Lie[i] = Temp_point;
    }
    for(i=0;i<29;i++)
    {
      Diff_Far_Lie[i] = Far_Lie[i+1] - Far_Lie[i];
    }
    for(i=0;i<28;i++)
    {
      DDiff_Far_Lie[i] = Diff_Far_Lie[i+1] - Diff_Far_Lie[i];
      if(Abs_(DDiff_Far_Lie[i])<5)//���Ա�־
        Liner_cnt++;
      if(DDiff_Far_Lie[i]<-15&&Liner_cnt>i/2&&Liner_cnt>0)//֮ǰ���ԣ����ֳ弤
      {
        Impulse_flag = 1;
        break;
      }
    }
    if(Impulse_flag==1)
      out_center = 319-4*i;//��¼ͻ���
  }
  *start_end = Far_Lie[0];
  *end_end   = Far_Lie[i];
  return out_center;
}

int Stay2Out_test()
{
  u8 Far_Lie[20];
  int Diff_Far_Lie[19];//һ�ײ��
  int DDiff_Far_Lie[18];//���ײ��
  int Liner_cnt  = 0;
  u8 Impulse_flag = 0;
  
  u8 Temp_point;
  u8 i;
  if(Island.State==Right_Island_in)
  {
    for(i=0;i<20;i++)
    {
      Temp_point=200;
      while(!(Image_Point(Temp_point,30+i*4)==1
            &&Image_Point(Temp_point-1,30+i*4)==1
              &&Image_Point(Temp_point-2,30+i*4)==1)&&Temp_point>=10)
      Temp_point--;
      Far_Lie[i] = Temp_point;
    }
    for(i=0;i<19;i++)
    {
      Diff_Far_Lie[i] = Far_Lie[i+1] - Far_Lie[i];
    }
    for(i=0;i<18;i++)
    {
      DDiff_Far_Lie[i] = Diff_Far_Lie[i+1] - Diff_Far_Lie[i];
      if(Abs_(DDiff_Far_Lie[i])<5)//���Ա�־
        Liner_cnt++;
      if(DDiff_Far_Lie[i]<-15&&Liner_cnt>i/2&&Liner_cnt>0)//֮ǰ���ԣ����ֳ弤
      {
        Impulse_flag = 1;
        break;
      }
    }
  }
  else if(Island.State==Left_Island_in)
  {
    for(i=0;i<20;i++)
    {
      Temp_point=200;
      while(!(Image_Point(Temp_point,289-i*4)==1
            &&Image_Point(Temp_point-1,289-i*4)==1
              &&Image_Point(Temp_point-2,289-i*4)==1)&&Temp_point>=10)
      Temp_point--;
      Far_Lie[i] = Temp_point;
    }
    for(i=0;i<19;i++)
    {
      Diff_Far_Lie[i] = Far_Lie[i+1] - Far_Lie[i];
    }
    for(i=0;i<18;i++)
    {
      DDiff_Far_Lie[i] = Diff_Far_Lie[i+1] - Diff_Far_Lie[i];
      if(Abs_(DDiff_Far_Lie[i])<5)//���Ա�־
        Liner_cnt++;
      if(DDiff_Far_Lie[i]<-15&&Liner_cnt>i/2&&Liner_cnt>0)//֮ǰ���ԣ����ֳ弤
      {
        Impulse_flag = 1;
        break;
      }
    }
  }
  return Impulse_flag;
}

int Wait_Next_Island()
{
  int center;
  u8 hang_use;
  if(Island.State!=Left_Wait_Next&&Island.State!=Right_Wait_Next)//���ڴ�״̬��
    return 1;//ֱ�ӷ���
  
  center = Wait_Next_center(&hang_use);
  if(center==-1)//������ʱѰ��ͻ���ʧ��
  {
    Image_hang.hang_use = 0;
    get_black_line(Image_fire[Start_Point],Start_Point);//45cm�����ĵ�
    if(Image_lie.Three_lie_end[0]>Image_hang.hang_use+5
       &&Image_lie.Three_lie_end[1]>Image_hang.hang_use+5
         &&Image_lie.Three_lie_end[2]>Image_hang.hang_use+5)//ȥ����ߵ�Ӱ��
    {
      get_black_line(Image_fire[Image_lie.Three_lie_end[1]+3],Image_lie.Three_lie_end[1]+3);
    }
    CenterlineToDiff(Image_hang.center[Image_hang.hang_use]);
  }
  else
  {
    Image_hang.hang_use = 0;
    get_black_line(Image_fire[hang_use],hang_use);
    CenterlineToDiff(Image_hang.center[Image_hang.hang_use]);
  }
  return 0;
}


int Wait_Next_center(u8* hang)//������ʱ��ֹ�ٴν��뻷����Ѱ��ͻ���
{
  int Middle;
  int center;
  int ccd_start=10,ccd_end=310;  //ccdɨ�����10���յ�310   
  int Left_Count=0,Right_Count=0;//���Ҽ���Ϊ0
  int Diff_L[19],Diff_R[19];//һ�ײ��
  int DDiff_L[18],DDiff_R[18];//���ײ��
  int Liner_L_cnt  = 0,Liner_R_cnt  = 0;
  u16 Next_black_L[20];
  u16 Next_black_R[20];
  u8  Impulse_L_Flag = 0,Impulse_R_Flag = 0;//һ�ײ���г��ֽ�Ծ
  u8 i = 0,j = 0;
  u8 *ImageData_in;
  
  Middle = Test_Far_Lie();//��65�п�ʼ��255��
  if(Island.State == Right_Island_pre)
  {
    if(Middle>200)return -1;//ǰ��ֱ���Ѿ���������
  }
  else if(Island.State == Left_Island_pre)
  {
    if(Middle<120)return -1;//ǰ��ֱ���Ѿ���������
  }
  
  for(i=0;i<20;i++)//20��
  {
    ImageData_in = Image_fire[i*3+Island.Image_Start_hang];
    for(j=0;j<40;j++)
      for(u8 k=0;k<8;k++)
        ImageData[j*8+k] = (ImageData_in[j]>>(7-k))&0x01;
    if(Island.State == Right_Wait_Next)
    {
      Right_Count = Middle;//�Ѻ����м�ֵ�����Ҽ������
      while(!(ImageData[Right_Count+3]==1
              && ImageData[Right_Count+2]==1
                && ImageData[Right_Count+1]==1)
            && Right_Count < ccd_end)//�������Ч����û���ҵ����������ڵ�
        Right_Count++;//���м�λ�ÿ�ʼ���������������������㶼�Ǻڵ�ͣ
      if(Right_Count<ccd_end)//�������Ч��Χ��
      {
        Next_black_R[i] = Right_Count;
      }
      else if(Right_Count<Image_lie.Three_Lie[1]+10)
      {
        Next_black_R[i] = ccd_end;
      }
      else
      {
        Next_black_R[i] = ccd_end;
      }
    }
    else if(Island.State == Left_Wait_Next)
    {
      Left_Count = Image_lie.Three_Lie[1];
      while(!(ImageData[Left_Count-3]==1 
              && ImageData[Left_Count-2]==1
                && ImageData[Left_Count-1]==1)
            && Left_Count > ccd_start)	  
        Left_Count--;
      if(Left_Count > ccd_start)
      {
        Next_black_L[i] = Left_Count; 
      }
      else if(Left_Count>Image_lie.Three_Lie[1]-10)
      {
        Next_black_L[i] = ccd_start;
      }
      else
      {
        Next_black_L[i] = ccd_start;
      }
    }
  }
  for(i=0;i<19;i++)
  {
    if(Island.State == Right_Wait_Next)
    {
      Diff_R[i] = Next_black_R[i+1] - Next_black_R[i];
    }
    else if(Island.State == Left_Wait_Next)
    {
      Diff_L[i] = Next_black_L[i+1] - Next_black_L[i];
    }
  }
  for(i=0;i<18;i++)
  {
    if(Island.State == Right_Wait_Next)
    {
      DDiff_R[i] = Diff_R[i+1] - Diff_R[i];
      if(Abs_(DDiff_R[i])<5)Liner_R_cnt++;
      if(DDiff_R[i]<-30&&Liner_R_cnt>i-3&&Liner_R_cnt>1)
      {
        Impulse_R_Flag = 1;
        center = Next_black_R[i];//������ת����
        *hang   = i*3+Island.Image_Start_hang;
        break;
      }
    }
    else if(Island.State == Left_Wait_Next)
    {
      DDiff_L[i] = Diff_L[i+1] - Diff_L[i];
      if(Abs_(DDiff_L[i])<5)Liner_L_cnt++;
      if(DDiff_L[i]> 30&&Liner_L_cnt>i-3&&Liner_L_cnt>1)
      {
        Impulse_L_Flag = 1;//���ֳ弤
        center = Next_black_L[i];//������ת����
        *hang   = i*3+Island.Image_Start_hang;
        break;
      }
    }
  }
  if(Impulse_R_Flag==0&&Impulse_L_Flag==0)
  {
    return -1;//û�г���
  }
  else 
  {
    return center;
  }
}

u8 Cross_process(void)
{
  if((Island.State!=NoIsland)
     ||In_double_AD()==1)//�������ȼ����
    return 1;
  Cross.State = NoCross;//���״̬
  In_Cross_test();//б��ʮ�ּ��
  In_Cross();//б��ʮ��
//  Out_Cross();
  Str_Cross();
  return 0;
}

u8 In_Cross_test()//б��ʮ�ּ��
{
  u8 Cross_curve_flag = 0;
  
  if(Cross.State==R2Cross_True
     ||Cross.State==L2Cross_True
     ||Cross.State==Str2Cross)
  return 1;
  
  Cross_pre_test();
  Cross_curve_flag = Cross_curve_test();
  return 0;
}

u8 In_Cross(void)//б��ʮ��
{
  int Start_End, End_End;
  int center_impulse;//�洢ͻ�������
  int center_use;
  static u8  center_Period = 0;
  static u16 Center_[Cross_Center_Period_Const];
  
  //���б��ʮ�ֱ�־
  if(Cross.State!=R2Cross_True&&Cross.State!=L2Cross_True)
    return 1;
  
  center_impulse = Cross_center_test(&Start_End,&End_End);
  if(Cross.State==R2Cross_True)//����
  {
    Cross.Cross_delay_flag = 1;
    Cross.Cross_delay_cnt = Cross.Cross_delay_cnt_const;
    center_use = ((center_impulse - (center_impulse - 0)*(End_End - Start_Point)*1.0/(End_End - Start_End)) + 319)/2+50;
    if(center_use>Image_lie.Three_Lie[1]+10)//��Ч�Լ���
    {
      if(center_use > 285)
        center_use = 285;
      CenterlineToDiff(center_use);
      if(center_Period > (Cross_Center_Period_Const - 1))
        center_Period = 0;
    
      Cross.In_center  -= Center_[center_Period];
      Center_[center_Period] = center_use;
      Cross.In_center  += Center_[center_Period];
      center_Period++;  
//�����ã���ʾ���ĵ�
      if(LCD_DISPLAY_FLAG==1)
      {
        LCD_DrawBigPoint(center_use,Start_Point,Cyan);//���еߵ�
      }
    }
    else if((Image_lie.Three_lie_end[0]-10)>Image_lie.Three_lie_end[1]
            ||(Image_lie.Three_lie_end[1]-10)>Image_lie.Three_lie_end[2])
    {
      //������ͨ���Ѱ������
      Image_hang.hang_use = 0;
      get_black_line(Image_fire[Start_Point],Start_Point);//45cm�����ĵ�
      if(Image_lie.Three_lie_end[0]>Image_hang.hang_use+5
         &&Image_lie.Three_lie_end[1]>Image_hang.hang_use+5
           &&Image_lie.Three_lie_end[2]>Image_hang.hang_use+5)//ȥ����ߵ�Ӱ��
      {
        get_black_line(Image_fire[Image_lie.Three_lie_end[1]+3],Image_lie.Three_lie_end[1]+3);
      }
      if(Image_hang.center[Image_hang.hang_use]>170)//�����е���ȷ��
        CenterlineToDiff(Image_hang.center[Image_hang.hang_use]);
      else
        CenterlineToDiff(Cross.In_center/Cross_Center_Period_Const);
    }
    else//����ʧ�ܣ�ʹ��֮ǰ��������ĵ�
    {
      CenterlineToDiff(Cross.In_center/Cross_Center_Period_Const);
//�����ã���ʾ���ĵ�
      if(LCD_DISPLAY_FLAG==1)
      {
        LCD_DrawBigPoint(Cross.In_center/Cross_Center_Period_Const,Start_Point,Magenta);//���еߵ�
      }
    }
  }
  else if(Cross.State==L2Cross_True)
  {
    Cross.Cross_delay_flag = 1;
    Cross.Cross_delay_cnt = Cross.Cross_delay_cnt_const;
    center_use = ((center_impulse - (center_impulse - 319)*(End_End - Start_Point)*1.0/(End_End - Start_End)) + 0)/2 - 50;
    if(center_use<Image_lie.Three_Lie[1]-10)//��Ч�Լ���
    {
      if(center_use < 35)
        center_use = 35;
      CenterlineToDiff(center_use);
      if(center_Period > (Cross_Center_Period_Const - 1))
        center_Period = 0;
    
      Cross.In_center  -= Center_[center_Period];
      Center_[center_Period] = center_use;
      Cross.In_center  += Center_[center_Period];
      center_Period++;  
//�����ã���ʾ���ĵ�
      if(LCD_DISPLAY_FLAG==1)
      {
        LCD_DrawBigPoint(center_use,Start_Point,Cyan);//���еߵ�
      }
    }
    else if((Image_lie.Three_lie_end[2]-10)>Image_lie.Three_lie_end[1]
            ||(Image_lie.Three_lie_end[1]-10)>Image_lie.Three_lie_end[0])
    {
      //������ͨ���Ѱ������
      Image_hang.hang_use = 0;
      get_black_line(Image_fire[Start_Point],Start_Point);//45cm�����ĵ�
      if(Image_lie.Three_lie_end[0]>Image_hang.hang_use+5
         &&Image_lie.Three_lie_end[1]>Image_hang.hang_use+5
           &&Image_lie.Three_lie_end[2]>Image_hang.hang_use+5)//ȥ����ߵ�Ӱ��
      {
        get_black_line(Image_fire[Image_lie.Three_lie_end[1]+3],Image_lie.Three_lie_end[1]+3);
      }
      if(Image_hang.center[Image_hang.hang_use]<150)//�����е���ȷ��
        CenterlineToDiff(Image_hang.center[Image_hang.hang_use]);
      else
        CenterlineToDiff(Cross.In_center/Cross_Center_Period_Const);
    }
    else//����ʧ�ܣ�ʹ��֮ǰ��������ĵ�
    {
      CenterlineToDiff(Cross.In_center/Cross_Center_Period_Const);
//�����ã���ʾ���ĵ�
      if(LCD_DISPLAY_FLAG==1)
      {
        LCD_DrawBigPoint(Cross.In_center/Cross_Center_Period_Const,Start_Point,Magenta);//���еߵ�
      }
    }
  }
  return 0;
}

u8 Cross_curve_test()
{
  int ccd_start=10,ccd_end=310;  //ccdɨ�����10���յ�310   
  int Left_Count=0,Right_Count=0;//���Ҽ���Ϊ0
  int L_black[60],R_black[60];//���ұ߽�
  int Diff_L[59],Diff_R[59];//һ�ײ��
  int DDiff_L[58],DDiff_R[58];//���ײ��
  int   Liner_L_cnt  = 0,Liner_R_cnt  = 0;
  int Middle = 160;
  u8    Liner_L_flag = 0,Liner_R_flag = 0;
  u8    Turn_L_Flag = 0,Turn_R_Flag = 0;//����ת�۵�
  u8    Turn_L_index= 0,Turn_R_index= 0;//����ת�۵��λ��
  int   Turn_L_early_cnt = 0,Turn_R_early_cnt = 0;//����ת�۵�֮ǰ���ز���
  int   Turn_L_late_cnt = 0 ,Turn_R_late_cnt = 0 ;//����ת�۵�֮����ز���
  u8  Impulse_L_Flag = 0,Impulse_R_Flag = 0;//һ�ײ���г��ֽ�Ծ
  int Impulse_L_index = 0,Impulse_R_index = 0;//һ�ײ�ֳ��ֽ�Ծ��λ��
  u8 i = 0,j = 0;
  u8 *ImageData_in;
//  Middle = Cross_find_far_center();
  for(i=0;i<60;i++)//10��
  {
    ImageData_in = Image_fire[Cross.Test_hang-i*2];
    for(j=0;j<40;j++)
      for(u8 k=0;k<8;k++)
        ImageData[j*8+k] = (ImageData_in[j]>>(7-k))&0x01;
    
    if(Cross.State==L2Cross_Pre)//��ת���ұ߽�
    {
      Right_Count = 35;
      while(!(ImageData[Right_Count+3]==1
              && ImageData[Right_Count+2]==1
                && ImageData[Right_Count+1]==1)
            && Right_Count < ccd_end)//�������Ч����û���ҵ����������ڵ�
        Right_Count++;//���м�λ�ÿ�ʼ���������������������㶼�Ǻڵ�ͣ
      if(Right_Count< ccd_end)//�������Ч��Χ��
      {
        R_black[i] = Right_Count;
      }
      else
      {
        R_black[i] = ccd_end;
      }
    }
    else if(Cross.State==R2Cross_Pre)//��ת���ұ߽�
    {
      Left_Count = 285;
      while(!(ImageData[Left_Count-3]==1
              && ImageData[Left_Count-2]==1
                && ImageData[Left_Count-1]==1)
            && Left_Count > ccd_start)	  
        Left_Count--;
      if(Left_Count > ccd_start)
      {
        L_black[i] = Left_Count; 
      }
      else
      {
        L_black[i] = ccd_start;
      }
    }
  }
  if(Cross.State==L2Cross_Pre)
  {
    for(i=0;i<59;i++)
    {
      Diff_R[i] = R_black[i+1] - R_black[i];
      if(Diff_R[i]<-100)
      {
        Impulse_R_Flag = 1;
        Impulse_R_index = i;
      }
    }
    for(i=0;i<59;i++)
    {
      if(Turn_R_Flag==0&&Diff_R[i]<=0)//δ����ת�۵�
        Turn_R_early_cnt++;
      else if(Turn_R_Flag==0
              &&i<50            //��ֹ�ڴ����
              &&Diff_R[i]>0
              &&Diff_R[i+1]>0
              &&Diff_R[i+2]>0)//����ת�۵�
      {
        Turn_R_Flag = 1;
        Turn_R_late_cnt++;
        Turn_R_index = i;//�ҵ�ת�۵��λ��
      }
      else if(Turn_R_Flag==1&&Diff_R[i]>0)
      {
        Turn_R_late_cnt++;
      }
    }
    
  }
  else if(Cross.State==R2Cross_Pre)
  {
    for(i=0;i<59;i++)
    {
      Diff_L[i] = L_black[i+1] - L_black[i];
      if(Diff_L[i]>100)
      {
        Impulse_L_Flag = 1;
        Impulse_L_index = i;
      }
    }
    for(i=0;i<59;i++)
    {
      if(Turn_L_Flag==0&&Diff_L[i]>=0)//δ����ת�۵�
        Turn_L_early_cnt++;
      else if(Turn_L_Flag==0
              &&i<50            //��ֹ�ڴ����
              &&Diff_L[i]<0
              &&Diff_L[i+1]<0
              &&Diff_L[i+2]<0)//����ת�۵�
      {
        Turn_L_Flag = 1;
        Turn_L_late_cnt++;
        Turn_L_index = i;//�ҵ�ת�۵��λ��
      }
      else if(Turn_L_Flag==1&&Diff_L[i]<0)
      {
        Turn_L_late_cnt++;
      }
    }
  }
  if(Cross.State==L2Cross_Pre)
  {
    for(i=0;i<58;i++)
    {
      DDiff_R[i] = Diff_R[i+1] - Diff_R[i];
      if(Abs_(DDiff_R[i])<5)Liner_R_cnt++;
    }
    if(Liner_R_cnt>30)Liner_R_flag = 1;
  }
  else if(Cross.State==R2Cross_Pre)
  {
    for(i=0;i<58;i++)
    {
      DDiff_L[i] = Diff_L[i+1] - Diff_L[i];
      if(Abs_(DDiff_L[i])<5)Liner_L_cnt++;
    }
    if(Liner_L_cnt>30)Liner_L_flag = 1;
  }
  if(Cross.State==L2Cross_Pre)
  {
    if(Liner_R_flag
       &&Turn_R_Flag
       &&Turn_R_index>5
//       &&Turn_R_early_cnt>Turn_R_index/2
       &&Turn_R_index<45
       &&Impulse_R_Flag
//       &&Turn_R_late_cnt>(Impulse_R_index-Turn_R_index)/2
         )
    {
      Cross.State = L2Cross_True;
      return 1;
    }
  }
  else if(Cross.State==R2Cross_Pre)
  {
    if(
       Liner_L_flag
       &&Turn_L_Flag
       &&Turn_L_index>5
//       &&Turn_L_early_cnt>Turn_L_index/2
       &&Turn_L_index<45
       &&Impulse_L_Flag
//       &&Turn_L_late_cnt>(Impulse_L_index-Turn_L_index)/2
         )
    {
      Cross.State = R2Cross_True;
      return 1;
    }
  }
  return 0;
}

u8 Cross_find_far_center()//��б��ʮ��ʱ��Ѱ����Զ������Ϊ��׼Ѱ�ұ߽�
{
  u8 Far_Lie[18];
  u8 Temp_point;
  u8 i;
  if(Cross.State==L2Cross_Pre)
  {
    for(i=0;i<18;i++)
    {
      Temp_point=220;
      while(!(Image_Point(Temp_point,5+i*8)==1
            &&Image_Point(Temp_point-1,5+i*8)==1
              &&Image_Point(Temp_point-2,5+i*8)==1)&&Temp_point>=10)
      Temp_point--;
      Far_Lie[i] = Temp_point;
    }
    return min_u8_index(Far_Lie,20)*10+5;
  }
  else if(Cross.State==R2Cross_Pre)
  {
    for(i=0;i<18;i++)
    {
      Temp_point=220;
      while(!(Image_Point(Temp_point,165+i*8)==1
            &&Image_Point(Temp_point-1,165+i*8)==1
              &&Image_Point(Temp_point-2,165+i*8)==1)&&Temp_point>=10)
      Temp_point--;
      Far_Lie[i] = Temp_point;
    }
    return min_u8_index(Far_Lie,20)*10+165;
  }
}

//u8 Out_Cross_far_Liner_line(void)

u8 Cross_center_test(int* start_end, int* end_end)//�ͳ�����ʱ�����ĵ�Ĵ���һ��
{
  u8 Far_Lie[25];
  int Diff_Far_Lie[24];//һ�ײ��
  int DDiff_Far_Lie[23];//���ײ��
  int Liner_cnt  = 0;
  int out_center = 160;
  
  u8 Temp_point;
  u8 i;
  if(Cross.State==R2Cross_True)
  {
    for(i=0;i<25;i++)
    {
      Temp_point=239;
      while(!(Image_Point(Temp_point,0+i*4)==1
            &&Image_Point(Temp_point-1,0+i*4)==1
              &&Image_Point(Temp_point-2,0+i*4)==1)&&Temp_point>=10)
      Temp_point--;
      Far_Lie[i] = Temp_point;
    }
    for(i=0;i<24;i++)
    {
      Diff_Far_Lie[i] = Far_Lie[i+1] - Far_Lie[i];
    }
    for(i=0;i<23;i++)
    {
      DDiff_Far_Lie[i] = Diff_Far_Lie[i+1] - Diff_Far_Lie[i];
      if(Abs_(DDiff_Far_Lie[i])<5)//���Ա�־
        Liner_cnt++;
      if(DDiff_Far_Lie[i]<-15&&Liner_cnt>i/2&&Liner_cnt>0)//֮ǰ���ԣ����ֳ弤
      {
        break;
      }
    }
    out_center = 4*i;//��¼ͻ���
  }
  else if(Cross.State==L2Cross_True)
  {
    for(i=0;i<25;i++)
    {
      Temp_point=239;
      while(!(Image_Point(Temp_point,319-i*4)==1
            &&Image_Point(Temp_point-1,319-i*4)==1
              &&Image_Point(Temp_point-2,319-i*4)==1)&&Temp_point>=10)
      Temp_point--;
      Far_Lie[i] = Temp_point;
    }
    for(i=0;i<24;i++)
    {
      Diff_Far_Lie[i] = Far_Lie[i+1] - Far_Lie[i];
    }
    for(i=0;i<23;i++)
    {
      DDiff_Far_Lie[i] = Diff_Far_Lie[i+1] - Diff_Far_Lie[i];
      if(Abs_(DDiff_Far_Lie[i])<5)//���Ա�־
        Liner_cnt++;
      if(DDiff_Far_Lie[i]<-15&&Liner_cnt>i/2&&Liner_cnt>0)//֮ǰ���ԣ����ֳ弤
      {
        break;
      }
    }
    out_center = 319-4*i;//��¼ͻ���
  }
  *start_end = Far_Lie[0];
  *end_end   = Far_Lie[i];
  return out_center;
}

u8 Out_Cross(void)
{
  if(Cross.State==NoCross&&Cross.Cross_delay_flag==1)
  {
    get_black_line(Image_fire[Image_lie.Three_lie_end[1]+5],Image_lie.Three_lie_end[1]+5);
    if(Image_hang.getLeft_flag[Image_hang.hang_use]
       &&Image_hang.getRight_flag[Image_hang.hang_use]
       &&Abs_(Image_hang.black_R[Image_hang.hang_use]+Image_hang.black_L[Image_hang.hang_use]-2*Image_lie.Three_Lie[1])<50
         )
    {
      Cross.Cross_delay_flag=0;
    }
    else
      CenterlineToDiff(Cross.In_center/Cross_Center_Period_Const);
  }
  return 0;
}


u8 Cross_pre_test(void)
{
  u8 Temp_point;
  u8 i;
  int L_Far_Lie[10],R_Far_Lie[10];
  int L_Diff_Far_Lie[9],R_Diff_Far_Lie[9];//һ�ײ��
  int L_DDiff_Far_Lie[8],R_DDiff_Far_Lie[8];//���ײ��
  int L_Liner_cnt = 0,R_Liner_cnt = 0;
  
  for(i=0;i<10;i++)
  {
    Temp_point=239;
    while(!(Image_Point(Temp_point,0+i*3)==1
          &&Image_Point(Temp_point-1,0+i*3)==1
            &&Image_Point(Temp_point-2,0+i*3)==1)&&Temp_point>=130)
    Temp_point--;
    L_Far_Lie[i] = Temp_point;
    
    Temp_point=239;
    while(!(Image_Point(Temp_point,319-i*3)==1
          &&Image_Point(Temp_point-1,319-i*3)==1
            &&Image_Point(Temp_point-2,319-i*3)==1)&&Temp_point>=130)
    Temp_point--;
    R_Far_Lie[i] = Temp_point;
  }
  for(i=0;i<9;i++)
  {
    L_Diff_Far_Lie[i] = L_Far_Lie[i+1] - L_Far_Lie[i];
    R_Diff_Far_Lie[i] = R_Far_Lie[i+1] - R_Far_Lie[i];
  }
  for(i=0;i<8;i++)
  {
    L_DDiff_Far_Lie[i] = L_Diff_Far_Lie[i+1] - L_Diff_Far_Lie[i];
    R_DDiff_Far_Lie[i] = R_Diff_Far_Lie[i+1] - R_Diff_Far_Lie[i];
    if(Abs_(L_DDiff_Far_Lie[i])<5&&Abs_(L_Diff_Far_Lie[i])>0)//���Ա�־
      L_Liner_cnt++;
    if(Abs_(R_DDiff_Far_Lie[i])<5&&Abs_(R_Diff_Far_Lie[i])>0)//���Ա�־
      R_Liner_cnt++;
  }
  if(L_Liner_cnt>4&&R_Liner_cnt>4)
  {
    return 1;
  }
  else if(L_Liner_cnt>4)
    Cross.State = R2Cross_Pre;
  else if(R_Liner_cnt>4)
    Cross.State = L2Cross_Pre;
  return 0;
}


u8 Str_Cross(void)
{
  int center_temp;
  int test_hang;
  if(Cross.State==L2Cross_True||Cross.State==R2Cross_True)
    return 1;
  if(Str_Cross_Test()==1)//ֱ��ʮ��
  {
    center_temp = Test_Far_Lie();
    test_hang = find_lie_end(center_temp,150)+20;
    Image_hang.center[test_hang] = center_temp;
    get_black_line_without_Iteration(Image_fire[test_hang],test_hang);//
    CenterlineToDiff(Image_hang.center[test_hang]);
  }
  return 0;
}

u8 Find_Start_line(void)
{
  u8 i;
  const u8 Start_line = 150;
  int cnt = 0;
  u8 line_cnt = 0;
  u8 dir = 0;
  u8 find_flag = 0;

  for(i=0;i<4;i++)
  {
    cnt = 0;
    if(Image_Point(Start_line+i*10,0)==0)//��һ�����ǰ׵�
    {
      dir = 1;//�Һڵ�
    }
    else//��һ�����Ǻڵ�
    {
      dir = 0;//�Ұ׵�
    }
    while(cnt<320)
    {
      if((Image_Point(Start_line+i*10,cnt+3)==dir)&&(Image_Point(Start_line+i*10,cnt+2)==dir)&&(Image_Point(Start_line+i*10,cnt+1)==dir))
      {
        line_cnt++;
        dir =! dir;
      }
      cnt++;
    }
    if(line_cnt>12)
    {
      find_flag ++;
    }
  }
  return find_flag;
}

u8 Start_Line_process(void)
{
  int center_temp;
  if(Start_line.test_allow_flag==1&&Find_Start_line()>1)
  {
    Start_line.test_allow_flag = 0;//��ձ�־
    Start_line._2Over_cnt = Start_line._2Over_cnt_const;//���¸�ֵ
    Start_line.Start_Line_cnt++;
    center_temp = Test_Far_Lie();
    Image_hang.center[Far_Point] = center_temp;
    get_black_line(Image_fire[Far_Point],Far_Point);
    CenterlineToDiff(Image_hang.center[Far_Point]);
  }
  if(Start_line.Start_Line_cnt==1)
  {
    Blue_Start_Flag = 0;//�رյ��
  }
  return 0;
}

u8 Power_Square_test(void)
{
  int Far_Lie[40];
  int Diff_Far_Lie[39];//һ�ײ��
  int DDiff_Far_Lie[38];//���ײ��
  int Left_Count=0,Right_Count=319;//���Ҽ���Ϊ0
  int L_black[10],R_black[10];//���ұ߽�
  int Diff_L[9],Diff_R[9];//һ�ײ��
  int DDiff_L[8],DDiff_R[8];//���ײ��
  u8  Liner_L_flag = 0,Liner_R_flag = 0;
  int Liner_L_cnt  = 0,Liner_R_cnt = 0;
  int Liner_cnt =0;
  u8  find_impulse_flag = 0;
  int first_impulse_index = 55;
  int second_impulse_index= 250;
  int stand_hang;
  u8 *ImageData_in;
  int ccd_start = 10,ccd_end = 310;
  
  u8 Temp_point;
  u8 i,j;
  for(i=0;i<40;i++)
  {
    Temp_point=200;
    while(!(Image_Point(Temp_point,55+i*5)==1
          &&Image_Point(Temp_point-1,55+i*5)==1
            &&Image_Point(Temp_point-2,55+i*5)==1)&&Temp_point>=10)
    Temp_point--;
    Far_Lie[i] = Temp_point;
  }
  for(i=0;i<39;i++)
  {
    Diff_Far_Lie[i] = Far_Lie[i+1] - Far_Lie[i];
  }
  for(i=0;i<38;i++)
  {
    DDiff_Far_Lie[i] = Diff_Far_Lie[i+1] - Diff_Far_Lie[i];
    if(Abs_(DDiff_Far_Lie[i])<5
       &&Abs_(Diff_Far_Lie[i])<5
       &&Far_Lie[i]>120)//���Ա�־
      Liner_cnt++;
    if(find_impulse_flag==0&&DDiff_Far_Lie[i]<-40)
    {
      find_impulse_flag++;
      first_impulse_index = i;
    }
    else if(find_impulse_flag==1&&DDiff_Far_Lie[i]<-40)
    {
      find_impulse_flag++;
      second_impulse_index = i;
    }
  }
  if(Liner_cnt>15&&find_impulse_flag==2)//ɨ�赽�����εײ�
  {
    stand_hang = ave_s16(&Far_Lie[first_impulse_index+1],(second_impulse_index-first_impulse_index-1));//�����������������
    for(i=0;i<10;i++)//10��
    {
      ImageData_in = Image_fire[stand_hang-5-i];
      for(j=0;j<40;j++)
        for(u8 k=0;k<8;k++)
          ImageData[j*8+k] = (ImageData_in[j]>>(7-k))&0x01;
      Right_Count = first_impulse_index*5+55 - 10;
      while(!(ImageData[Right_Count+3]==1
              && ImageData[Right_Count+2]==1
                && ImageData[Right_Count+1]==1)
            && Right_Count < ccd_end)//�������Ч����û���ҵ����������ڵ�
        Right_Count++;//���м�λ�ÿ�ʼ���������������������㶼�Ǻڵ�ͣ
      if(Right_Count< ccd_end)//�������Ч��Χ��
      {
        R_black[i] = Right_Count;
      }
      else
      {
        R_black[i] = ccd_end;
      }
      Left_Count = second_impulse_index*5+55 + 10;
      while(!(ImageData[Left_Count-3]==1
              && ImageData[Left_Count-2]==1
                && ImageData[Left_Count-1]==1)
            && Left_Count > ccd_start)	  
        Left_Count--;
      if(Left_Count > ccd_start)
      {
        L_black[i] = Left_Count; 
      }
      else
      {
        L_black[i] = ccd_start;
      }
    }
    for(i=0;i<9;i++)
    {
      Diff_L[i] = L_black[i+1]-L_black[i];
      Diff_R[i] = R_black[i+1]-R_black[i];
    }
    for(i=0;i<8;i++)
    {
      DDiff_L[i] = Diff_L[i+1] - Diff_L[i];
      DDiff_R[i] = Diff_R[i+1] - Diff_R[i];
      if(Abs_(DDiff_L[i])<5)
        Liner_L_cnt++;
      if(Abs_(DDiff_R[i])<5)
        Liner_R_cnt++;
    }
    if(Liner_L_cnt>5)
      Liner_L_flag = 1;
    if(Liner_R_cnt>5)
      Liner_R_flag = 1;
    if(Liner_L_flag&&Liner_R_flag)
      return 1;
    else
      return 0;
  }
  return 0;
}

int find_lie_end(int lie,int start_hang)
{
  int find_point=start_hang;
  while(!(Image_Point(find_point,lie)==1
          &&Image_Point(find_point-1,lie)==1
            &&Image_Point(find_point-2,lie)==1)&&find_point>=2)
    find_point--;
  return find_point;
}