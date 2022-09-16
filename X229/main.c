/* =========================================================================
 * Project:       ADC_Polling
 * File:          main.c
 * Description:   ADC Convert With Polling AIN0~AIN8 and Internal 1/4 VDD.
 *   1. Set ADC clock frequency is 1MHz , Sample pulse width is 8 ADC clock,
 *	    ADC conversion time = (8+12+2)*1us = 22us , ADC conversion rate = 1/22us = 45.5KHz
 *   2. Polling one of PA0(AIN0) to PB3(AIN8)  or internal 1/4 VDD as ADC analog input.
 *   3. Store the ADC convert result bit11~ bit4 to RAM "R_AINx_DATA_HB", bit3~ bit0 to RAM "R_AINx_DATA_LB[3:0]" (x=0~4)
 *   4. Store internal 1/4VDD ADC convert result bit11~ bit4 to RAM "R_Quarter_VDD_DATA_HB", bit3~ bit0 to RAM "R_Quarter_VDD_DATA_LB[3:0]"	                    
 ;	*noticedon't use PA2 & PA4 to connect Q-Link, please use PB4 & PB5
 
; Author:  		 David Wei      
; Version:       V1.0 
; Date:          2021/05/07
 =========================================================================*/
#include <ny8.h>
#include "ny8_constant.h"


// 宏定义
#define C_PWM_LB_DUTY_00H	0x00
#define C_PWM_LB_DUTY_40H	0x40
#define C_PWM_LB_DUTY_01H	0x01
#define C_PWM_LB_DUTY_FFH	0xFF
// *    PWM3(PA2) 	: Period = 3.91KHz (Duty: 768/1024)
// *    PWM2(PB2) 	: Period = 3.91KHz (Duty: 1023/1024)
// *	PWM1(PB3)		: Period = 3.91KHz (Duty: 1/1024)
#define ADC_NUM 2

#define Key					PA5
#define MotorPWM			PB3
#define LED_Red				PB1
#define LED_Green			PA2
#define LED_Blue			PA4

#define ON						1
#define OFF						0

#define PowerOFF			0
#define PowerOn				1

#define MotorOFF			0
#define MotorON				1

#define MotorPwmOFF				0
#define MotorPwmON				1

#define PwmDutyMax 100

#define BatLevelLess20 					1
#define BatLevelMore21Less40 		2
#define BatLevelMore41less60    3
#define BatLevelMore60Less80		4
#define BatLevelMore80less100		5
#define BatLevel100							6

#define LedStatus_Onestep				1
#define LedStatus_Twostep				2
#define LedStatus_Threestep			3
#define LedStatus_Fourstep			4
#define LedStatus_Fivestep			5
#define LedStatus_Sixstep				6
#define LedStatus_Sevenstep				7


#define volt3_13V  2632
#define volt3_3V   2775
#define volt3_53V  2986
#define volt3_73V  3136
#define volt3_86V  3245

#define chgless20		1		// <3.3V							//1灯闪
#define chgMore20		2		// 3.3V < x < 3.53V	//1亮2灯闪
#define chgMore40		3		// 3.53V < x < 3.73V	//1亮2亮3灯闪
#define chgMore60		4		// 3.73V < x < 3.86V	//1亮2亮3亮4灯闪
#define chgMore80		5		// 3.86V < x < 4.2V		//1亮2亮3亮4亮5亮
#define chgMore100	6		// 4.2V == x						//全闪


#define Dischgless10	1 // x < 3.13V		//1灯闪
#define DischgLess20	2	// 3.13V < x < 3.3V	1灯亮
#define DischgMore20	3	// 3.3V < x < 3.53V	1.2灯亮
#define DischgMore40	4	// 3.53V < x < 3.73V 1.2.3灯亮
#define DischgMore60	5	// 3.73V < x < 3.86V	1.2.3.4灯亮
#define DischgMore80	6	// 3.86V < x 					1.2.3.4.5灯亮


#define BoostFeedbackValue  1015	//0.495V
#define BoostDeadzone 			105//21 = 0.01V 


#define RedLed	1
#define WieghtLed 0
//变量
volatile unsigned char u8CntAdSamp=0;
volatile unsigned char u8CntFeedbackAdSamp=0;
volatile unsigned char SystemFlag1=0;
volatile unsigned char SystemFlag2=0;
volatile unsigned char SystemFlag3=0;
volatile unsigned char Timecnt_2ms=0;
volatile unsigned char Timecnt_10ms=0;
volatile unsigned char Timecnt_50ms=0;
volatile unsigned char Timecnt_100ms=0;
volatile unsigned char SleepWakeupcnt=0;
volatile unsigned char PoweronDelay2Stimecnt=0;
volatile unsigned char SystemStatus=0;
volatile unsigned char MotorStatus=0;
volatile unsigned char LedStatus=0;
volatile unsigned char MotorRuntime=0;
volatile unsigned char SinglePengWucnt=0; //按下一次按键, 喷雾次数
volatile unsigned char Send_Timer_Cnt_10S=0;
volatile unsigned char Send_Timer_Cnt_50S=0;
volatile unsigned char Send_Timer_Cnt_10Min=0;
volatile unsigned char Send_Timer_Cnt_4Hl=0;
volatile unsigned char Send_Timer_Cnt_4Hh=0;	
volatile unsigned char DumpSwitchDetectCnt=0;	
volatile unsigned int PWM1DUTYCnt=0;
volatile unsigned int	MotorTotaltimeCntL=0;
volatile unsigned char	MotorTotaltimeCntH=0;	
volatile unsigned char	Pwm_LED_Red_Duty = 0;	
volatile unsigned char	Pwm_LED_Green_Duty = 0;	
volatile unsigned char	Pwm_LED_Blue_Duty = 0;

volatile unsigned char WorkMode=0;
volatile unsigned char DisplayBuffer=0;
volatile unsigned char DisplaySclCnt=0;
volatile unsigned char BatLevel=0;
volatile unsigned char BatLevelTmep=0;
volatile unsigned char LedOffTimeCnt=0;
volatile unsigned char LedFlashtimeCnt=0;
volatile unsigned char chargeDetectTimecnt=0;
volatile unsigned char chargeFullDetectTimecnt=0;
volatile unsigned int Keypresstime=0;
volatile unsigned int BKeypresstime=0;
volatile unsigned int CKeypresstime=0;
volatile unsigned int DKeypresstime=0;
volatile unsigned long  MotorRunlongtimeCnt=0;
volatile unsigned int  MotorRunintvaltimeCnt=0; //定时喷雾计时
volatile unsigned int MotorSingleRunintvaltimeCnt=0; //单击喷雾计时
volatile unsigned int  R_AIN0_DATA=0;	
unsigned long Ad_Sum=0;
unsigned int u16AdcResult=0;
unsigned int u16AdcResult1=0;
unsigned char OverCurrentTimeCnt=0;
unsigned char MotorOnDelayTimeCnt=0;
unsigned char Led_Pwm_Cnt_Cycle=0;
unsigned char Led_Pwm_Cnt_DutyBuffer=0;
unsigned char DelayToLowSpeedTimeCnt = 0;
unsigned char keyShortPressTimerCnt=0; //多击按键间隔计时
unsigned int  PengwuRunTimeCnt=0;
unsigned char PengWucnt=0; //一个定时周期喷雾次数
unsigned char short_key_PerssCnt=0; //多击按键次数


__sbit f_Power_ON_AD = SystemFlag1 : 0;
__sbit f_short_key_WuHua_on = SystemFlag1 : 1; //单击雾化标志
__sbit f_intervalstop = SystemFlag1 : 2;
__sbit KeyLongPressflag = SystemFlag1 : 3;
__sbit f_EnableSleep = SystemFlag1 : 4;
__sbit Time_50S_Flag = SystemFlag1 : 5;
__sbit Time_10S_Flag = SystemFlag1 : 6;	
__sbit StartWuHuaFLag = SystemFlag1 : 7;

__sbit f_Timer2ms = SystemFlag2 : 0;
__sbit f_Timer10ms = SystemFlag2 : 1;
__sbit f_Timer50ms = SystemFlag2 : 2;
__sbit f_Timer100ms = SystemFlag2 : 3;
__sbit f_Key2PressEnable = SystemFlag2 : 4; //单击按键标志
__sbit f_fisrtPress = SystemFlag2 : 5;
__sbit ModeLedOnFlag = SystemFlag2 : 6;
__sbit f_SystemonInit = SystemFlag2 : 7;
	
__sbit f_wakeupfromCharge = SystemFlag3 : 0;
__sbit f_Enable_SinglePengwuStatus = SystemFlag3 : 1;
__sbit f_StoptimeUp = SystemFlag3 : 2;
__sbit f_PwmMax = SystemFlag3 : 3;
__sbit f_long_bkey_on = SystemFlag3 : 4;
__sbit f_long_ckey_on	= SystemFlag3 : 5;
__sbit f_long_dkey_on	= SystemFlag3 : 6;
__sbit f_intervalstopsingle	= SystemFlag3 : 7;

//typedef union
//{
//	unsigned int Result[ADC_NUM];
//}Unio_AdcResultTypedef;
//Unio_AdcResultTypedef u16Adc;

const unsigned char TAB_AD_CH[ADC_NUM] = {5,6};

#define UPDATE_REG(x)	__asm__("MOVR _" #x ",F")


//函数定义
int ADCConvert(char B_CH);
void BufferClean(void);
void Init(void);
void IO_Init(void);
void Pwm_Init(void);
void delay(int count);
void ADCInit(void);
void Timer0Init(void);
void F_wait_eoc(void);
void Adc_Sample(void);
void Keyscan(void);
void OffAllExtDevice(void);
void Sleep(void);
void AllLedOffFunc(void);
void AllLedONFunc(void);
void chargeLedDisplay(void);
void LedDisplayFunc(void);
void FangdianLedDisplay(void);
void Flashled(unsigned char LedType);
void MotorFunc(void);
void LedDisplay(void);
void MotorIntvialRunFunc(int Runtime,int intervalstoptime,int Stoptime);
void ReadFeedbackVoltageAD(void);
void keyHandle(void);   
//--------------- IO init --------------------------------------------
//--------------------------------------------------------------------------
void IO_Init(void)
{
	DISI();
//   ; PORTB I/O state 
//    BWUCON = C_PB2_Wakeup;					// Enable PB1 input change wakeup function
  IOSTB =  0x00;				// Set PB3 & PB2 to input mode,others set to output mode
	//BPHCON = (char)~( C_PB3_PHB | C_PB2_PHB );
  PORTB = 0x00;                           		// PB5 & PB4 output low / PB1 & PB0 output high
    
//   ; PORTA I/O state
	PORTA = 0x00;									// PA3 & PA2 output high	
	//AWUCON = C_PA1_Wakeup | C_PA3_Wakeup;					// Enable PB1 input change wakeup function	
	IOSTA =  C_PA5_Input;				// PA3 & PA2 set output mode ; PA1 & PA0 set input mode
	APHCON = (char)~(C_PA5_PHB); 		// Enable PA1、PA0 Pull-High Resistor,others disable

}
//--------------- Timer0 init --------------------------------------------
void Timer0Init(void)
{
	PCON1 = C_TMR0_Dis;						// Disable Timer0
	TMR0 = 213;								// Load 0x00 to TMR0 (Initial Timer0 register)
	
	T0MD = C_PS0_TMR0 | C_PS0_Div4 ;		// Prescaler0 is assigned to Timer0, Prescaler0 dividing rate = 1:8,clock source is instruction clock	
	INTE = C_INT_TMR0;	
	
	PCON1 = C_TMR0_En;						// Enable Timer0
	ENI();									// Enable all unmasked interrupts
}
//--------------- ADC init --------------------------------------------
//--------------------------------------------------------------------------
void ADCInit(void)
{
	//----- Initial ADC-----	  
	ADMD  = C_ADC_En | C_ADC_CH_Dis;		// Enable ADC power, Disable global ADC input channelt (SFR "ADMD")
 
 //----- ADC high reference voltage source select-----
	ADVREFH = C_Vrefh_VDD;
 	//ADVREFH = C_Vrefh_4V;					// ADC reference high voltage is supplied by internal 4V  (Note: ADC clock freq. must be equal or less 1MHz)
 	//ADVREFH = C_Vrefh_3V;					// ADC reference high voltage is supplied by internal 3V  (Note: ADC clock freq. must be equal or less 500KHz)
 	//ADVREFH = C_Vrefh_2V;					// ADC reference high voltage is supplied by internal 2V  (Note: ADC clock freq. must be equal or less 250KHz)
 
//----- ADC clock frequency select----------------------------	 
	//ADR	 = C_Ckl_Div1;					// ADC clock=Fcpu/1, Clear ADIF, disable ADC interrupt	
	ADR	 = C_Ckl_Div2;					// ADC clock=Fcpu/2, Clear ADIF, disable ADC interrupt	
	//ADR	  = C_Ckl_Div8;						// ADC clock=Fcpu/8, Clear ADIF, disable ADC interrupt	
	//ADR	 = C_Ckl_Div16;					// ADC clock=Fcpu/16, Clear ADIF, disable ADC interrupt	
 
//----- ADC Sampling pulse width select-------------	 
 	ADCR  = C_Sample_1clk | C_12BIT;		// Sample pulse width=1 adc clock, ADC select 12-bit conversion ( Note: ADC clock freq. must be equal or less 500KHz)
 	//ADCR  = C_Sample_2clk | C_12BIT;		// Sample pulse width=2 adc clock, ADC select 12-bit conversion ( Note: ADC clock freq. must be equal or less 1MHz)
 	//ADCR  = C_Sample_4clk | C_12BIT;		// Sample pulse width=4 adc clock, ADC select 12-bit conversion ( Note: ADC clock freq. must be equal or less 1.25MHz)
 	//ADCR  = C_Sample_8clk | C_12BIT; 		// Sample pulse width=8 adc clock, ADC select 12-bit conversion ( Note: ADC clock freq. must be equal or less 2MHz)	

//--------------------------------------------------	
	PACON = C_ADC_PB1;// Set AIN0(PA0) as pure ADC input for reducing power consumption (SFR "PACON")
	delay(50);								// Delay 0.56ms(Instruction clock=4MHz/2T) for waiting ADC stable 
}
void Pwm_Init(void)
{
	TMRH = 0x00;
	TMR1  = 0x64;	// Move FFH to TMR1 LB register ( TMR1[9:0]=3FFH )
	
    PWM1DUTY = 0;			// Move 01H to PWM1DUTY LB register ( PWM1DUTY[9:0]=001H )
    PWM2DUTY = 0;			// Move FFH to PWM2DUTY LB register ( PWM2DUTY[9:0]=3FFH )
    PWM3DUTY = 0;			// Move FFH to PWM3DUTY LB register ( PWM3DUTY[9:0]=300H )
    
    T1CR2	 = C_PS1_Dis | C_TMR1_ClkSrc_Inst;	// Prescaler 1:1 , Timer1 clock source is instruction clock
    T1CR1	 = C_PWM1_En | C_PWM1_Active_Hi | C_TMR1_Reload | C_TMR1_En;	// Enable PWM1 , Active_High , Non-Stop mode ,reloaded from TMR1[9:0] , enable Timer1
   	T1CR1 &=~(C_PWM1_En);
   	
   	P2CR1    = C_PWM2_En | C_PWM2_Active_Hi;
    P2CR1 &=~(C_PWM2_En);
    
    P3CR1	 = C_PWM3_En | C_PWM3_Active_Hi;	// Enable PWM3 , Active_High , Non-Stop mode ,reloaded from TMR3[9:0] , enable Timer3
    P2CR1 &=~(C_PWM3_En);
    
    
    TM4RH   = 00;//C_TMR4_Data_b9 | C_TMR4_Data_b8 | C_PWM4_Duty_b9 | C_PWM4_Duty_b8;				
    TMR4	= 0x11;						// Move FFH to TMR3 LB register  ( TMR3[9:0]=3FFH )
   
    PWM4DUTY = 0x09;			// Move 00H to PWM3DUTY LB register ( PWM3DUTY[9:0]=300H )
    T4CR2	 = C_PS4_Dis | C_TMR4_ClkSrc_Inst;	// Prescaler 1:1 , Timer3 clock source is instruction clock  
    T4CR1	 = C_PWM4_En | C_PWM4_Active_Hi | C_TMR4_Reload | C_TMR4_En;	// Enable PWM3 , Active_High , Non-Stop mode ,reloaded from TMR3[9:0] , enable Timer3 
	T4CR1  &= ~(C_PWM4_En);
}
//--------------- System init --------------------------------------------
//--------------------------------------------------------------------------
void System_Init(void)
{
	IO_Init();
	Pwm_Init();
	//ADCInit();
	Timer0Init();
}

//=========================================================================
//  AD 转换
//=========================================================================
int ADCConvert(char B_CH)
{
	char i;
  	ADMD  = 0x90 | B_CH;				// Select AIN0(PA0) pad as ADC input
	for(i = 0;i < 50;i++);           //延时10几个us ，等待稳定		 
  	ADMDbits.START = 1;					// Start a ADC conversion session
  	//F_wait_eoc();							// Wait for ADC conversion complete	
  	while(ADMDbits.EOC==0);
  	
  	R_AIN0_DATA  = ADD; 
  	R_AIN0_DATA  = R_AIN0_DATA << 4;
  	R_AIN0_DATA |= ( 0x0F & ADR); 
 	return R_AIN0_DATA;
}
//=========================================================================
//void F_wait_eoc(void)
//{
//   while(ADMDbits.EOC==0)
//   ;
//}

//========================================================
//ad 数据读取
//========================================================
void Adc_Sample(void)
{
	//if(u8CntAdSamp >= 200)
	//{
	//	Ad_Sum -= u16AdcResult;
	//	f_Power_ON_AD = 1;
	//}
	//else
	//{
	//	u8CntAdSamp++;
	//}
	//Ad_Sum += ADCConvert(5);
	//u16AdcResult = Ad_Sum / u8CntAdSamp;
	u16AdcResult = ADCConvert(6);
	R_AIN0_DATA = 0;
}


//-----------------------------------------------------------
//	触摸按键检测
//-----------------------------------------------------------
void Keyscan(void)
{
	//if(u16AdcResult < 300)	//power key
	if(!Key)	//power key
	{
		if(f_short_key_WuHua_on==0)
		{
			Keypresstime++;
			if(Keypresstime>=2)		//消抖25ms
			{
				Keypresstime=0;
				f_short_key_WuHua_on = 1;
			}
		}	
		else
		{
			if(KeyLongPressflag == 0)
			{
				Keypresstime++;
				if(Keypresstime>=200)
				{
					KeyLongPressflag = 1;
					if(SystemStatus == PowerOFF)
					{
						KeyLongPressflag = 1;	
						SystemStatus = PowerOn;
						StartWuHuaFLag = 1;
						f_EnableSleep = 0;		//开机
						WorkMode = 1;
					}
					else
					{
						SystemStatus = PowerOFF;
						f_EnableSleep = 1;		//关机睡眠
						WorkMode = 0;
					}
				}
			}
		}	
	}
	else
	{
		if((f_short_key_WuHua_on)&&(KeyLongPressflag == 0))
		{
			//StartWuHuaFLag = 1;
			f_EnableSleep = 0;
			if(SystemStatus == PowerOn)
			{
				f_Key2PressEnable = 1;
				if(f_fisrtPress==0)
				{
					f_fisrtPress = 1;
					keyShortPressTimerCnt = 0;	//双击计时清除
				}
				if(SystemStatus == PowerOn)
					short_key_PerssCnt++;
			}
		}
		f_short_key_WuHua_on = 0;
		KeyLongPressflag = 0;
	}
}

//-----------------------------------------------------------
//		模式按键点击双击处理
//-----------------------------------------------------------
void keyHandle(void)   
{
	if(SystemStatus == PowerOn)   
	{
		if(keyShortPressTimerCnt <= 40)		//1S内按键2次检测
		{
			if(short_key_PerssCnt >= 2)
			{
				keyShortPressTimerCnt = 0;
				short_key_PerssCnt = 0;
				f_Key2PressEnable = 0;
				f_fisrtPress = 0;
				StartWuHuaFLag = 1;
				MotorRunintvaltimeCnt = 0;  //清除喷雾 计时变量
				WorkMode++;
				if(WorkMode > 2)		//切换档位模式
					WorkMode = 1;
			}
		}
		else
		{			
			keyShortPressTimerCnt = 0;
			short_key_PerssCnt = 0;
			f_fisrtPress = 0;
			if(f_Key2PressEnable)		//单击按键 喷雾一次
			{
				f_Key2PressEnable = 0;
			//	StartWuHuaFLag = 1;
				f_Enable_SinglePengwuStatus = 1;
				MotorSingleRunintvaltimeCnt = 0;
			}
		}
	}
}
//-----------------------------------------------------------
//一次喷Runtime, 多次间隔intervalstoptime(0为只喷1次), 一次喷雾周期Stoptime
void MotorIntvialRunFunc(int Runtime,int intervalstoptime,int Stoptime)
{
	if(f_Enable_SinglePengwuStatus)
	{
		if((MotorStatus == MotorOFF)&&(f_intervalstopsingle==0))
		{
			MotorStatus = MotorON;	//开启马达
			T4CR1  |= (C_PWM4_En);
		}
		if(MotorSingleRunintvaltimeCnt >= PengwuRunTimeCnt)
		{
			if(intervalstoptime > 0)
			{
				f_intervalstopsingle=1;
				MotorSingleRunintvaltimeCnt = 0;
				SinglePengWucnt++;
				if(SinglePengWucnt>=2)
				{
					SinglePengWucnt=0;
					f_intervalstopsingle=0;
					f_Enable_SinglePengwuStatus = 0;
				}
			}
			else
			{
				f_intervalstopsingle=0;
				MotorSingleRunintvaltimeCnt = 0;
				f_Enable_SinglePengwuStatus = 0;
			}	
			if(f_StoptimeUp)
			{
				MotorRunintvaltimeCnt = 0;
				f_StoptimeUp = 0;
			}
			MotorStatus = MotorOFF;	
			MotorPWM = MotorPwmOFF;
			T4CR1  &= ~(C_PWM4_En);
		}
		else
		{
			if(f_intervalstopsingle) //是否多次喷雾
			{
				if(MotorSingleRunintvaltimeCnt >= intervalstoptime)
				{
					MotorSingleRunintvaltimeCnt = 0;
					f_intervalstopsingle = 0;
					
					MotorStatus = MotorON;	//开启马达
					//MotorPWM = MotorPwmON;
					T4CR1  |= (C_PWM4_En);
				}
			}
		}
	}
	else
	{
		if(MotorStatus == MotorON)
		{
			PengwuRunTimeCnt = Runtime;
			if(MotorRunintvaltimeCnt >= Runtime)
			{
				MotorRunintvaltimeCnt = 0;
				if(intervalstoptime > 0)
				{
					PengWucnt++;
					if(PengWucnt>=2)
					{
						PengWucnt=0;
						f_intervalstop=0;
					}
					else
					{
						f_intervalstop=1;
					}
				}
				else
					f_intervalstop=0;
					
				MotorStatus = MotorOFF;	
				MotorPWM = MotorPwmOFF;
				T4CR1  &= ~(C_PWM4_En);
			}
		}	
		else
		{
			if(f_intervalstop) //是否多次喷雾
			{
				if(MotorRunintvaltimeCnt >= intervalstoptime)
				{
					MotorRunintvaltimeCnt = 0;
					f_intervalstop = 0;
					
					f_StoptimeUp = 1;
					MotorStatus = MotorON;	//开启马达
					//MotorPWM = MotorPwmON;
					T4CR1  |= (C_PWM4_En);
				}
			}
			else
			{
				if(MotorRunintvaltimeCnt >= Stoptime)
				{
					MotorRunintvaltimeCnt = 0;
					f_StoptimeUp = 1;
					MotorStatus = MotorON;	//开启马达
					//MotorPWM = MotorPwmON;
					T4CR1  |= (C_PWM4_En);
				}	
			}	
		}
	}
}
//-----------------------------------------------------------
void MotorONFunc(void)
{
	MotorStatus = MotorON; 
	//MotorPWM = MotorPwmON;
	T4CR1  |= C_PWM4_En;
	MotorOnDelayTimeCnt =0;
	MotorRunlongtimeCnt=0;
	MotorRunintvaltimeCnt = 0;
	MotorTotaltimeCntL = 0;
	MotorTotaltimeCntH = 0;	
}

//-----------------------------------------------------------
void MotorOFFFunc(void)
{
	MotorStatus = MotorOFF;	
	MotorPWM = MotorPwmOFF;
	T4CR1  &= ~(C_PWM4_En);
	MotorOnDelayTimeCnt=0;
	MotorRunlongtimeCnt=0;
	MotorRunintvaltimeCnt = 0;
	MotorTotaltimeCntL = 0;
	MotorTotaltimeCntH = 0;	
}
//-----------------------------------------------------------
void MotorFunc(void)
{
	if(SystemStatus == PowerOn)
	{
		switch(WorkMode)
		{
			case 0:
				if(f_EnableSleep)
				{
					MotorStatus = MotorOFF;	
					MotorPWM = MotorPwmOFF;
					SystemStatus = PowerOFF;
					f_EnableSleep = 1;		//关机睡眠
					T4CR1  &= ~(C_PWM4_En);
				}
				break;
			case 1:
				if(StartWuHuaFLag)
				{
					StartWuHuaFLag = 0;
					//PWM4DUTY = 0x29;
					MotorONFunc();
					
					PWM1DUTY = 0;	//R
					PWM2DUTY = 100;	//B
					PWM3DUTY = 50;	//G   冰蓝色	
					Pwm_LED_Red_Duty = 0;	
					Pwm_LED_Green_Duty = 50;	
					Pwm_LED_Blue_Duty = 100;
					f_PwmMax = 1;
					
					P2CR1 |= (C_PWM2_En);
					P3CR1 |= (C_PWM3_En);
				}
				MotorIntvialRunFunc(50, 0, 3000); //喷一次, 一次喷5s, 间隔0.5s, 一次喷雾周期5min
				break;
			case 2:	
				if(StartWuHuaFLag)
				{
					StartWuHuaFLag = 0;
					//PWM4DUTY = 0x29;
					MotorONFunc();
					
					
					PWM1DUTY = 0;	//R
					PWM2DUTY = 50;	//B
					PWM3DUTY = 100;	//G   浅绿色
					Pwm_LED_Red_Duty = 0;	
					Pwm_LED_Green_Duty = 100;	
					Pwm_LED_Blue_Duty = 25;
					f_PwmMax = 1;
					
					P2CR1 |= (C_PWM2_En);
					P3CR1 |= (C_PWM3_En);
				}
				MotorIntvialRunFunc(30, 5, 1800); //喷2次, 一次喷3s, 间隔0.5s, 一次喷雾周期3min
				break;
			default:
				break;
		}	
      	LedDisplay();
	}
	else
	{
		T1CR1 &=~(C_PWM1_En);
		P2CR1 &=~(C_PWM2_En);
		P3CR1 &=~(C_PWM3_En);
		LED_Red	= OFF;			
		LED_Green	= OFF;			
		LED_Blue	= OFF;			
		Pwm_LED_Red_Duty = 0;	
		Pwm_LED_Green_Duty = 0;	
		Pwm_LED_Blue_Duty = 0;
		f_PwmMax = 0;
		MotorStatus = MotorOFF;	
		MotorPWM = MotorPwmOFF;
		T4CR1  &= ~(C_PWM4_En);
		f_Enable_SinglePengwuStatus = 0;
		MotorSingleRunintvaltimeCnt = 0;
		MotorRunintvaltimeCnt=0;
		f_intervalstop=0;
		f_StoptimeUp = 0;
		PengWucnt=0;
	}
}
//--------------------------------------------------------------------------
//
//--------------------------------------------------------------------------
void LedDisplay(void)
{
  switch(WorkMode)
  {
	case 1:
		if(f_PwmMax)
		{
		 	if(Pwm_LED_Blue_Duty >= 1)
			{
				Pwm_LED_Blue_Duty = Pwm_LED_Blue_Duty - 1;
				if((Pwm_LED_Blue_Duty % 2)==0)
				{
					if(Pwm_LED_Green_Duty >=1)
					 Pwm_LED_Green_Duty = Pwm_LED_Green_Duty - 1;
					else
					 Pwm_LED_Green_Duty = 0;  
				}
			}
			else
			{
				Pwm_LED_Green_Duty = 0;
			 	Pwm_LED_Blue_Duty = 0; 
			 	f_PwmMax = 0;
			}
		}
		else
		{
			if(Pwm_LED_Blue_Duty < 100)
			{
				Pwm_LED_Blue_Duty = Pwm_LED_Blue_Duty + 1;
				if((Pwm_LED_Blue_Duty % 2)==0)
				{
					if(Pwm_LED_Green_Duty < 50)
					 Pwm_LED_Green_Duty = Pwm_LED_Green_Duty + 1;
				}
			}
			else
			{
				Pwm_LED_Green_Duty = 50;
			 	Pwm_LED_Blue_Duty = 100; 
				f_PwmMax = 1;
			}
		}
	break;
	
	case 2:
		if(f_PwmMax)
		{
		 	if(Pwm_LED_Green_Duty >= 1)
			{
				Pwm_LED_Green_Duty = Pwm_LED_Green_Duty - 1;
				if((Pwm_LED_Green_Duty % 4)==0)
				{
					if(Pwm_LED_Blue_Duty >=1)
					 Pwm_LED_Blue_Duty = Pwm_LED_Blue_Duty - 1;
					else
					 Pwm_LED_Blue_Duty = 0;  
				}
			}
			else
			{
				Pwm_LED_Green_Duty = 0;
			 	Pwm_LED_Blue_Duty = 0; 
			 	f_PwmMax = 0;
			}
		}
		else
		{
			if(Pwm_LED_Green_Duty < 100)
			{
				Pwm_LED_Green_Duty = Pwm_LED_Green_Duty + 1;
				if((Pwm_LED_Green_Duty % 4)==0)
				{
					if(Pwm_LED_Blue_Duty < 50)
					 Pwm_LED_Blue_Duty = Pwm_LED_Blue_Duty + 1;
				}
			}
			else
			{
				Pwm_LED_Green_Duty = 100;
			 	Pwm_LED_Blue_Duty = 25; 
				f_PwmMax = 1;
			}
		}
	break;
	default:
	break;
  }
  PWM2DUTY = Pwm_LED_Blue_Duty;	//B
  PWM3DUTY = Pwm_LED_Green_Duty;	//G   冰蓝色	
}

//--------------- isr function --------------------------------------------
//! interrupt service routine   200uS 中断一次
//--------------------------------------------------------------------------
void isr(void) __interrupt(0)
{
	if(INTFbits.T0IF)
	{ 
		TMR0 = 213;
		INTF= (char)~(C_INT_TMR0);			// Clear T0IF flag bit		
		//PORTB ^= (1<<1);					// PA3 Toggle
		//if(++Timecnt_2ms >= 10)
		if(++Timecnt_2ms >= 20)
		{
			Timecnt_2ms = 0;
			f_Timer2ms = 1;
			if(++Timecnt_10ms >= 5)//10ms
			{
				Timecnt_10ms = 0;
				f_Timer10ms = 1;
				if(f_fisrtPress)
				{
					if(keyShortPressTimerCnt<0xff)	//用于检测按键双击
						keyShortPressTimerCnt++;
				}
				SleepWakeupcnt++;
				if(++Timecnt_50ms >= 5)//50ms
				{
					Timecnt_50ms = 0;
					f_Timer50ms = 1;
					if(++Timecnt_100ms >= 2)//100ms
					{
						Timecnt_100ms = 0;
						f_Timer100ms = 1;
						MotorRunlongtimeCnt++;
						MotorSingleRunintvaltimeCnt++;
						MotorRunintvaltimeCnt++;  
						if(MotorOnDelayTimeCnt<6)
						MotorOnDelayTimeCnt++;
					}
				}
			}
		}
	}
}


//--------------- Main function --------------------------------------------
//--------------------------------------------------------------------------
void main(void)
{
	System_Init();
    PCON  = C_WDT_En | C_LVR_En; 			// Enable WDT & LVR
	SystemStatus = PowerOn;
	StartWuHuaFLag = 1;
	f_EnableSleep = 0;		//开机
	WorkMode = 1;
  while(1)
  {
      CLRWDT();							// Clear WatchDog
      ENI();
      if(f_Timer2ms)			//每2ms 采样AD
      {
      	f_Timer2ms = 0;
		Adc_Sample(); 
      }
     if(f_Timer10ms)
      {
      	f_Timer10ms = 0;
      	Keyscan();					//按键扫描
      	keyHandle();
      	MotorFunc();		//Motor intvial Contrl
      }
  }
}

//--------------------------------------------------------------------------
//
//--------------------------------------------------------------------------
void delay(int count)
{
	int i;
	for(i=1;i<=count;i++)
	;
}