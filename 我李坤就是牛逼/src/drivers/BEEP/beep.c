#include "beep.h"

Beep_Str Image_Island_Test_Beep={.Delay_const=20};
void myBEEP_Init(void)
{
  gpio_init(PORTE,24,GPO,LOW);
  BEEP_Open_once();
} 

void BEEP_Open_once(void)
{
  BEEP=1;
  delayms(10);
  BEEP=0;
} 

void Beep_Once(Beep_Str* Beep)
{
  BEEP = 1;
  Beep->Flag = 1;
  Beep->Delay = Beep->Delay_const;
}

void Beep_ISR(Beep_Str* Beep)
{
  if(Beep->Flag == 1)
  {
    if(Beep->Delay==0)
    {
      BEEP = 0;
      Beep->Flag = 0;
    }
    Beep->Delay--;
  }
}