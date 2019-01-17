/***********************************************************
*	加密相关的代码
*
*
*
************************************************************/
#include "extern.h"


u32 IDAddr0 = 0x2aaa9fe;//和加密id有关
u16 IDCal1;
u16 IDCal2;//out
u16 IDCal3;//out
u16 IDCal4;
u16 IDCal5;//out
u16 IDCal6;//out
u16 IDCal7;//id加密相关 out
u16 IDCal8;//id加密相关 out

u32 IDAddr1 		= 0x15554ff;//id相关，通过这个计算一下，得到的id
u8 	IDFlagWrite3;//out
u32 IDAddr2 		= 0x444433;//id相关，通过这个计算一下，得到的id
u8 	IDFlagWrite4;
u32 IDAddr3 		= 0x444433;//id相关，通过这个计算一下，得到的id
u8 	IDFlagWrite5;//out
u32 IDAddr4 		= 0x444433;//id相关，通过这个计算一下，得到的id
u8 	IDFlagWrite6;//out

u16 IDFlashBuf2[16];
u16 IDFlashBuf3[16];
u16 IDFlashBuf4[16];
u16 IDFlashBuf5[16];
u16 IDFlashBuf6[16];

//有修改
u16 const IDFlashAddr16000[16] __attribute__ ((at(0x8016000)))={
		0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
		0x003F, 0x0000, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
u16 const IDFlashAddr1B000[16] __attribute__ ((at(0x801B000)))={
		0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
		0xFFFF, 0xFFFF, 0x033C, 0x0000, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF
}; 	
u16 const IDFlashAddr2F000[16] __attribute__ ((at(0x802F000)))={
		0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0x00D9, 0x0000,
		0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF
}; 	
u16 const IDFlashAddr33C00[16] __attribute__ ((at(0x8033C00)))={
		0x1234, 0x0000, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
		0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF
}; 	
u16 const IDFlashAddr34000[16] __attribute__ ((at(0x8034000)))={
		0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0x3828, 0x0000,
		0xFFFF, 0xFFFF, 0x0328, 0x0000, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF
}; 


void IDReadFun2(void);//读参数 
void IDWriteFun2(void);//写参数 
void IDReadFun3(void);//读参数
void IDWriteFun3(void);//写参数
void IDResetFun3(void);
void IDReadFun4(void);//读参数
void IDWriteFun4(void);//写参数
void IDResetFun4(void);
void IDReadFun5(void);//读参数
void IDWriteFun5(void);//写参数
void IDResetFun5(void);
void IDReadFun6(void);//读参数
void IDWriteFun6(void);//写参数
void IDResetFun6(void);
//id加密代码
//-------------------------------------------------------------
void IDReadFun2()
{
  u16 *pv_addr;
  u16 *pf_addr;
	u16 i;

  i = 16;
  pv_addr = IDFlashBuf2;
  pf_addr = (u16*)IDFlashAddr34000;
  while ( i-- )
  {
    *pv_addr++ = *pf_addr++;
  }
  IDCal1 = IDFlashBuf2[6];
  IDCal4 = IDFlashBuf2[8];
  IDCal2 = IDFlashBuf2[10];
  IDCal5 = IDFlashBuf2[12];
  IDCal6 = IDFlashBuf2[13];
}

//-------------------------------------------------------------
void IDWriteFun2()
{//写参数
  u16* pv_addr; 
  u16* pf_addr; 
	u16 i; 

  i = 16;
  pv_addr = IDFlashBuf2;
  pf_addr = (u16*)IDFlashAddr34000;
  IDFlashBuf2[0] = 0x101;
  IDFlashBuf2[1] = 0x1234;
  IDFlashBuf2[2] = 0xffff;
  IDFlashBuf2[3] = 0x1384;
  IDFlashBuf2[4] = 0xfffe;
  IDFlashBuf2[5] = 0xF038;
  IDFlashBuf2[6] = IDCal1;
  IDFlashBuf2[8] = IDCal4;
  IDFlashBuf2[10] = IDCal2;
  IDFlashBuf2[12] = IDCal7;
  IDFlashBuf2[13] = IDCal8;
  FLASH_Unlock();
  if ( FLASH_ErasePage((u32)IDFlashAddr34000) == FLASH_COMPLETE )
  {
    while ( i-- )
    {
      if ( FLASH_ProgramHalfWord((u32)pf_addr, *pv_addr) != FLASH_COMPLETE )
      {
        FLASH_Lock();
        return ;
      }
      pf_addr++;
      pv_addr++;
    }
    for ( i = 16; i > 0; i-- )
      IDFlashBuf2[i - 1] = 0;
    FLASH_Lock();
  }
  else
  {
    FLASH_Lock();
  }
}


//-------------------------------------------------------------
void IDReadFun3()
{//拷贝数据
  u16* pv_addr;
  u16* pf_addr;
	u16 i;

  i = 16;
  pv_addr = IDFlashBuf3;
  pf_addr = (u16*)IDFlashAddr33C00;
  while ( i-- )
  {
    *pv_addr++ = *pf_addr++;
  }
}

//-------------------------------------------------------------
void IDWriteFun3()
{
  u16* pv_addr; 
  u16* pf_addr; 
  u16 i; 

  i = 16;
  pv_addr = IDFlashBuf3;
  pf_addr = (u16*)IDFlashAddr33C00;
  IDFlashBuf3[0] = 0x101;
  IDFlashBuf3[1] = 0;
  IDFlashBuf3[2] = 0xffff;
  IDFlashBuf3[3] = 0x1384;
  IDFlashBuf3[4] = 0xfffe;
  IDFlashBuf3[5] = 0xf038;
  IDFlashBuf3[6] = 0;
  IDFlashBuf3[7] = 0xffff;
  IDFlashBuf3[9] = 0xfffe;
  IDFlashBuf3[10] = 0xf038;
  IDFlashBuf3[11] = 0xfffe;
  IDFlashBuf3[12] = 0xf038;
  IDFlashBuf3[13] = 0xfffe;
  IDFlashBuf3[14] = 0xf038;
  IDFlashBuf3[15] = 0xf038;
  FLASH_Unlock();
  if ( FLASH_ErasePage((u32)IDFlashAddr33C00) == FLASH_COMPLETE )
  {
    while ( i-- )
    {
      if ( FLASH_ProgramHalfWord((u32)pf_addr, *pv_addr) != FLASH_COMPLETE )
      {
        FLASH_Lock();
        return;
      }
      pf_addr++;
      pv_addr++;
    }
    for ( i = 16; i > 0; i-- )
      IDFlashBuf3[i - 1] = 0;
    FLASH_Lock();
  }
  else
  {
    FLASH_Lock();
  }
}


//-------------------------------------------------------------
void IDResetFun3()
{
  u16 val; 

  IDReadFun3();
  if ( IDFlashBuf3[0] == 0x1234 )
  {
    IDFlashBuf3[8] =  ((*(u8 *)(24 * IDAddr1 + 1)) << 8) + (*(u8 *)(24 * IDAddr1 + 0xB)) - 257;
		IDWriteFun3();
  }
  else
  {
    if ( IDFlagWrite3 != 1 )
    {
      IDReadFun3();
      val = IDFlashBuf3[8] + 257;
      IDFlashBuf3[8] = (*(u8 *)(24 * IDAddr1 + 1)) << 8;
      IDFlashBuf3[8] += (*(u8 *)(24 * IDAddr1 + 0xB));
      if ( IDFlashBuf3[8] == val )
      {
        IDFlagWrite3 = 1;
      }
    }
  }
}
//-------------------------------------------------------------
void IDReadFun4()
{
  u16* pv_addr; 
  u16* pf_addr; 
	u16 i; 

  i = 16;
  pv_addr = IDFlashBuf4;
  pf_addr = (u16*)IDFlashAddr2F000;
  while ( i-- )
  {
    *pv_addr++ = *pf_addr++;
  }
}

//-------------------------------------------------------------
void IDWriteFun4()
{
  u16* pv_addr;
  u16* pf_addr; 
	u16 i;

  i = 16;
  pv_addr = IDFlashBuf4;
  pf_addr = (u16*)IDFlashAddr2F000;
  IDFlashBuf4[0] = 0xffff;
  IDFlashBuf4[1] = 0xffff;
  IDFlashBuf4[2] = 0xffff;
  IDFlashBuf4[3] = 4996;
  IDFlashBuf4[5] = 0xf038;
  IDFlashBuf4[6] = 0;
  IDFlashBuf4[7] = 0xffff;
  IDFlashBuf4[8] = 4996;
  IDFlashBuf4[9] = 0xfffe;
  IDFlashBuf4[10] = 0xf038;
  IDFlashBuf4[11] = 0xfffe;
  IDFlashBuf4[12] = 0xf038;
  IDFlashBuf4[13] = 0xfffe;
  IDFlashBuf4[14] = 0xf038;
  IDFlashBuf4[15] = 0xf038;
  FLASH_Unlock();
  if ( FLASH_ErasePage((u32)IDFlashAddr2F000) == FLASH_COMPLETE )
  {
    while ( i-- )
    {
      if ( FLASH_ProgramHalfWord((u32)pf_addr, *pv_addr) != FLASH_COMPLETE )
      {
        FLASH_Lock();
        return ;
      }
      pf_addr++;
      pv_addr++;
    }
    for ( i = 16; i > 0; i-- )
      IDFlashBuf4[i - 1] = 0;
    FLASH_Lock();
  }
  else
  {
    FLASH_Lock();
  }
}


//-------------------------------------------------------------
void IDResetFun4()
{
  u16 val; 

  IDReadFun4();
  if ( IDFlashBuf4[6] == 217 )
  {
    IDFlashBuf4[4] = (*(u8 *)(120 * IDAddr2 + 2) << 8) + *(u8 *)(120 * IDAddr2 + 6) + 257;
    IDWriteFun4();
  }
  else
  {
    if ( IDFlagWrite4 != 1 )
    {
      IDReadFun4();
      val = IDFlashBuf4[4] - 257;
      IDFlashBuf4[4] = *(u8 *)(120 * IDAddr2 + 2) << 8;
      IDFlashBuf4[4] += *(u8 *)(120 * IDAddr2 + 6);
      if ( IDFlashBuf4[4] == val )
      {
        IDFlagWrite4 = 1;
      }
      else
      {
        GPIO_ResetBits(GPIOA, 32);//关机
      }
    }
  }
}

//-------------------------------------------------------------
void IDReadFun5()
{
  u16* pv_addr;
  u16 *pf_addr;
	u16 i; 

  i = 16;
  pv_addr = IDFlashBuf5;
  pf_addr = (u16*)IDFlashAddr16000;
  while ( i-- )
  {
    *pv_addr++ = *(u16 *)pf_addr++;
  }
}

//-------------------------------------------------------------
void IDWriteFun5()
{
  u16* pv_addr; 
  u16* pf_addr; 
	u16 i;

  i = 16;
  pv_addr = IDFlashBuf5;
  pf_addr = (u16*)IDFlashAddr16000;
  IDFlashBuf5[0] = 0xffff;
  IDFlashBuf5[1] = 0xffff;
  IDFlashBuf5[2] = 0xffff;
  IDFlashBuf5[3] = 4996;
  IDFlashBuf5[4] = 0xfffe;
  IDFlashBuf5[6] = 0;
  IDFlashBuf5[7] = 0xffff;
  IDFlashBuf5[8] = 4996;
  IDFlashBuf5[9] = 0xfffe;
  IDFlashBuf5[10] = 0xf038;
  IDFlashBuf5[11] = 0xfffe;
  IDFlashBuf5[12] = 0xf038;
  IDFlashBuf5[13] = 0xfffe;
  IDFlashBuf5[14] = 0xf038;
  IDFlashBuf5[15] = 0xf038;
  FLASH_Unlock();
  if ( FLASH_ErasePage((u32)IDFlashAddr16000) == FLASH_COMPLETE )
  {
    while ( i-- )
    {
      if ( FLASH_ProgramHalfWord((u32)pf_addr, *pv_addr) != 4 )
      {
        FLASH_Lock();
				return;
      }
      pf_addr++;
      pv_addr++;
    }
    for ( i = 16; i > 0; i-- )
      IDFlashBuf5[i - 1] = 0;
    FLASH_Lock();
  }
  else
  {
    FLASH_Lock();
  }
}


//-------------------------------------------------------------
void IDResetFun5()
{
  u16 val; 

	val = 0x3c;
  IDReadFun5();
	val += 2;
	val ++;
  if ( IDFlashBuf5[8] == val )
  {
    IDFlashBuf5[5] = (*(u8 *)(120 * IDAddr3 + 2) << 8) + *(u8 *)(120 * IDAddr3 + 6) + 4112;
    IDWriteFun5();
  }
  else
  {
    if ( IDFlagWrite5 != 1 )
    {
      IDReadFun5();
      val = IDFlashBuf5[5] - 0x1010;
      IDFlashBuf5[5] = *(u8 *)(120 * IDAddr3 + 2) << 8;
      IDFlashBuf5[5] += *(u8 *)(120 * IDAddr3 + 6);
      if ( IDFlashBuf5[5] == val )
      {
        IDFlagWrite5 = 1;
      }
    }
  }
}


//-------------------------------------------------------------
void IDReadFun6()
{//读flash
  u16* pv_addr;
  u16* pf_addr;
	u16 i; 

  i = 16;
  pv_addr = IDFlashBuf6;
  pf_addr = (u16*)IDFlashAddr1B000;
  while ( i-- )
  {
    *pv_addr++ = *pf_addr++;
  }
}

//-------------------------------------------------------------
void IDWriteFun6()
{//写flash 重置数据
  u16 *pv_addr; 
  u16* pf_addr; 
  u16 i; 

  i = 16;
  pv_addr = IDFlashBuf6;
  pf_addr = (u16*)IDFlashAddr1B000;
  IDFlashBuf6[0] = 0xffff;
  IDFlashBuf6[1] = 0xffff;
  IDFlashBuf6[2] = 0xffff;
  IDFlashBuf6[3] = 0x1384;
  IDFlashBuf6[4] = 0xfffe;
  IDFlashBuf6[6] = 0;
  IDFlashBuf6[7] = 0xffff;
  IDFlashBuf6[8] = 0x1384;
  IDFlashBuf6[9] = 0xfffe;
  IDFlashBuf6[10] = 0xf038;
  IDFlashBuf6[11] = 0xfffe;
  IDFlashBuf6[12] = 0xf038;
  IDFlashBuf6[13] = 0xfffe;
  IDFlashBuf6[14] = 0xf038;
  FLASH_Unlock();
  if ( FLASH_ErasePage((u32)IDFlashAddr1B000) == FLASH_COMPLETE )
  {//擦除数据
    while ( i-- )
    {
      if ( FLASH_ProgramHalfWord((u32)pf_addr, *pv_addr) != 4 )
      {
        FLASH_Lock();
        return;
      }
      pf_addr++;
      pv_addr++;
    }
    for ( i = 16; i > 0; i-- )
      IDFlashBuf6[i - 1] = 0;
    FLASH_Lock();
  }
  else
  {
    FLASH_Lock();
  }
}


//-------------------------------------------------------------
void IDResetFun6()
{
  u16 val; //

  IDReadFun6();
  if ( IDFlashBuf6[10] == 828 )
  {//0x1FFFF7E8 还是和id相关
    IDFlashBuf6[5] = (*(u8 *)(120 * IDAddr4 + 3) << 8) + *(u8 *)(120 * IDAddr4 + 7) + 8500;
    IDWriteFun6();//数据重置
  }
  else if ( IDFlagWrite6 != 1 )
	{
		IDReadFun6();//重新读取参数
		val = IDFlashBuf6[5] - 8500;
		IDFlashBuf6[5] = *(u8 *)(120 * IDAddr4 + 3) << 8;
		IDFlashBuf6[5] += *(u8 *)(120 * IDAddr4 + 7);
		if ( IDFlashBuf6[5] == val )
		{
			IDFlagWrite6 = 1;
		}
  }
}


