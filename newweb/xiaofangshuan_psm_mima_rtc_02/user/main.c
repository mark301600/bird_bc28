/**************************************************************************//**
 * @file     main.c
 * @version  01
 * 作者：马博
 * 功能：消防栓管理设备
 * 定位模块：ORG1411(3.3V)
 * 存储芯片：24c512
 ******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "Nano1X2Series.h"
#include "NuEdu-Basic01_EEPROM.h"

/*---------------------------------------------------------------------------------------------------------*/
/*  定义全局宏                                                                                         	   */
/*---------------------------------------------------------------------------------------------------------*/
#define DEVICE_ID       										918110003	             				 //定义设备ID号
#define SOFTWARE       											02              				 		 	 //定义固件版本号
//#define UART_ORG1411_BUFF 										1000              						      //定义ORG1411数据缓冲区
//#define UART_GPS_BUFF 											  UART_ORG1411_BUFF              			 //定义GPS数据缓冲区
#define UART_BC28_BUFF 											500              						 	 //定义BC28数据缓冲区
#define UART_NB_BUFF 											  UART_BC28_BUFF              			 //定义NB数据缓冲区

#define UART_BC28 													UART0             						 //定义ORG1411连接串口号
#define UART_BC28_IRQn 											UART0_IRQn             					 //
#define UART_BC28_MODULE 										UART0_MODULE             				 //
#define UART_ORG1411 												UART1             						 //定义BC28连接串口号
#define UART_ORG1411_IRQn 										UART1_IRQn             					 //
#define UART_ORG1411_MODULE 									UART1_MODULE             				 //

#define EEPROM 		   											0x10000            				 		 //存储器大小   0x8000:256k  0x10000:512k
#define PADDR 		   											0x0            				 		 	 //参数存储起始地址
#define NADDR 		   											PADDR            				 		 //条数起始地址
#define AADDR		   											PADDR+2            				 		 //当前数据存储地址
#define DADDR 		   											0x100									 //数据存储起始地址
#define GPS_T 		   											1                						 //定义GPS默认定位周期 单位：h
#define DATA_TX_T 		   										2                						 //定义设备默认发送周期 单位：h
#define GPS_SECOND 		  										60                 				 		 //定义GPS模块单次定位时长  单位:秒
#define GPS_COUNT 		  										3                 				 		 //定义GPS模块单周期定位最多次数
#define SOFT_T 		  											40               				 		 //定义程序运行周期(看门狗周期)
#define DEVICE_T 		  										4               				 		 //定义设备自检周期  单位:小时
#define DATABUFF 		  										128	               				 		 //定义数据处理缓冲
#define TDATABUFF 		  										2*DATABUFF         				 		 //定义长数据处理缓冲
#define VOT  		  											3.3	               				 		 //定义电压门限

#define ON  		  											1	               				 		 //定义软开关开状态
#define OFF  		  											2	               				 		 //定义软开关关状态

#define PC_4  		  											1204	               				 	 //定义IO口所对应值码
#define PC_10 		  											1210	               				 	 
#define PC_11 		  											1211	               				 	
#define PC_12 		  											1212	               				 		
#define PC_13 		  											1213	               				 		
#define PD_11 		  											1311	               				 		
/*---------------------------------------------------------------------------------------------------------*/
/*  定义全局常量                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
char web_ip_port[]="62.234.187.251,8080,";             													 //定义服务器端IP和端口号
char mima[]="123";
char *alarm_it[]=
{
	"11",                          //被盗告警命令
	"12",                          //倾斜告警命令
	"13",                          //正常开告警命令
	"14",                          //正常关告警命令
	"15",                          //检修告警命令
	"16",                          //自检告警命令
};
#define     BEIDAO       0
#define     QINGXIE      1
#define     POWERON      2
#define     POWEROFF     3
#define     JIANXIU      4
#define     ZIJIAN       5

/*---------------------------------------------------------------------------------------------------------*/
/*  定义全局变量                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
typedef struct
{
	uint8_t gps_t;
	uint8_t data_tx_t;
	uint8_t addr1;
	uint8_t addr2;
	uint8_t power;
}STORAGE;
//STORAGE storage={GPS_T,DATA_TX_T,ADDR1,ADDR2,POWER};
//char g_u8RecData[UART_GPS_BUFF]  ={0};																  	  //定义GPS数据缓冲区
//char *buf=g_u8RecData;																  		 			  //定义GPS数据缓冲区指针
char g_u8RecData2[UART_NB_BUFF]  = {0}; 																  //定义NB数据缓冲区
char *buf2=g_u8RecData2;																 				  //定义NB数据缓冲区指针
char databuf[DATABUFF]={0}; 															 				  //定义数据缓冲区
volatile char device_id[11];            																  //保存设备ID
volatile char software[3];            																 	  //保存设备固件版本号
S_RTC_TIME_DATA_T sInitTime,sTime1,sTime2;
volatile uint32_t  g_u32TICK = 0;
char databuffer[DATABUFF];     																		      //待发送数据缓冲区
uint8_t csq,csq1;     																					  //信号强度值
volatile uint8_t u8ADF;																				  	  //AD中断指示符号
volatile uint8_t alarm0,alarm1,alarm2,alarm3,alarm4,alarm5;												  //监测状态指示符号  alarm0:水流报警 alarm1:倾斜报警 alarm2:被盗报警 alarm3:检修报警 alarm4:自检报警 alarm4:密码错误报警
volatile uint8_t alarm0_if=OFF;																	  		  //监测状态指示符号
volatile uint8_t sw=OFF;																				  //开关状态
//volatile uint8_t alarm1=OFF;																				  //设备倾斜状态
volatile uint32_t alarm0_ie=0;																  		  	  //水流监测中断次数统计
volatile uint32_t time=0;																				  //水流计时时间符号
volatile uint32_t time3=0;																				  //IO消抖标志
volatile uint16_t addr=DADDR;																			  //存储地址符号
volatile uint32_t num=0;																				  //记录条数符号
//volatile uint32_t timer0=0;																				  //时钟符号
volatile float advalue;																					  //电压值符号
volatile uint32_t io_val1,io_val2;
//volatile uint32_t io_it,io_it1,io_it2;
volatile uint8_t gps_t,nb_t;
volatile uint8_t mima_num=0;
char MIMA[4]={0};

uint32_t queue[100]={0};
uint32_t *que1=queue;
uint32_t *que2=queue;

/*************************符号数组*******************************/
char *symbol[]=                       
{ 
	 ",",            											//0
	 "E",             											//1
	 "\r\n",             										//2
	 "S",            											//3
	 "+",             											//4
	 "-",             											//5
	 "1",             											//6	
	 "B",             											//7	
	 "C",             											//8
	 "D",             											//9
	 "0",             											//10
};
/*************************参数数组*******************************/
char *para[]=                       
{ 
	 "para",            										//0
	 "success",
	 "0x100,",
	 "0x200,",
	 "0x400,"
};
/***********************服务器参数下行包数据结构******************/
typedef struct               	
{
	char *start;                      							//数据包包头：C
	char *power;                	    						//开关机命令
	char *gps_t;                   		  						//GPS定位周期
	char *nb_t;                   		  						//NB模块发送数据周期
	char *end;                          						//数据包包尾
}DATAQRP;
/***********************服务器参数上行包数据结构******************/
typedef struct               	
{
	char *start;                      							//数据包包头：C
	char *deviceid;                	    						//设备号
	char *para;                   								//参数类型
	char *end;                          						//数据包包尾
}DATAQTP;
/***********************待存储数据包数据结构**********************/
typedef struct                  
{
	char *lat;                        							//纬度值
	char *latit;                      							//纬度标志位       "+"北纬,"-"南纬
	char *lon;                        							//经度值
	char *lonit;                      							//经度标志位       "+"东经,"-"西经
	char *date;                       							//日期
	char *time;                       							//时间
	char *high;                       							//高度值
	char *pdop;                       							//pdop值
	char *fix;                        							//2d,3d定位类型
	char *speed;                      							//速度值
	char *cog;                        							//信号强度
	char *vol;                       							//电压值
	char *tem;                        							//温度值
}DATASTORAGE;
/***********************服务器数据上行包数据结构******************/
typedef struct                     			
{
	char *start;                      							//数据包包头:D
	char *deviceid;                   							//设备号
	DATASTORAGE *datastorage;                   				//待发送数据
	char *software;                   							//固件版本号
    char *end;                          						//数据包包尾
}DATATP;
/*---------------------------------------------------------------------------------------------------------*/
/*  AT指令部分                                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
/*****************BC28**************/
char AT_QCGATT[] = "AT+CGATT?\r\n";																  			//查询网络附着情况
char AT_CGATT[] = "AT+CGATT=1\r\n";																  			//附着网络
char AT_NSOSCF[] = "AT+NSOSCF=0\r\n";
char AT_CFUN0[] = "AT+CFUN=0\r\n";
char AT_CFUN1[] = "AT+CFUN=1\r\n";
char AT_CSCON_Q[] = "AT+CSCON?\r\n";
char AT_CEDRXS[] = "AT+CEDRXS=0,5,\"0101\"\r\n";
char AT_CPSMS[] = "AT+CPSMS=0\r\n";
char AT_CPSMS_SET[] = "AT+CPSMS=1,,,01000011,01000011\r\n";
char AT_CPSMS_Q[] = "AT+CPSMS?\r\n";
char AT_QCPSMS[] = "AT+CPSMS?\r\n";
char AT_NSOCR[] = "AT+NSOCR=DGRAM,17,4587,1\r\n";
char AT_NSOST[] = "AT+NSOST=0,";
char AT_NSOSTF[] = "AT+NSOSTF=0,";
char AT_NSORF[] = "AT+NSORF=0,";
char AT_NSOCL[] = "AT+NSOCL=0\r\n";
char AT_CIMI[] = "AT+CIMI\r\n";
char AT_NCCID[] = "AT+NCCID\r\n";
char AT_CEREG[] = "AT+CEREG=1\r\n";
char AT_QCEREG[] = "AT+CEREG?\r\n";
char AT_CSQ[] = "AT+CSQ\r\n";
char ATI[] = "ATI\r\n";
/*---------------------------------------------------------------------------------------------------------*/
/*  SYS_Init 晶振初始化 （外部32.768k+内部HIRC）                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set HCLK source form HIRC and HCLK source divide 1  */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC,CLK_HCLK_CLK_DIVIDER(1));

    /* Enable LIRC and HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_LXT_EN_Msk | CLK_PWRCTL_HIRC_EN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_LXT_STB_Msk | CLK_CLKSTATUS_HIRC_STB_Msk);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();
	
	SYS->PF_L_MFP = SYS_PF_L_MFP_PF5_MFP_ICE_DAT | SYS_PF_L_MFP_PF4_MFP_ICE_CLK | SYS_PF_L_MFP_PF1_MFP_X32_OUT | SYS_PF_L_MFP_PF0_MFP_X32_IN|SYS_PF_L_MFP_PF3_MFP_GPF3;
    /* Lock protected registers */
    SYS_LockReg();
}
/*---------------------------------------------------------------------------------------------------------*/
/*  延迟函数(不准确)                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void delay(uint32_t ms)
{
	uint32_t i,j;
	for(i=0;i<ms;i++)
	{
		for(j=0;j<5000;j++);
	}
}
/*---------------------------------------------------------------------------------------------------------*/
/*  AD初始化函数                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void Init_Adc(void)
{
	CLK_EnableModuleClock(ADC_MODULE);
	CLK_SetModuleClock(ADC_MODULE,CLK_CLKSEL1_ADC_S_HIRC,CLK_ADC_CLK_DIVIDER(5));
			/* Set PA.4 multi-function pin for ADC channel 4 */
	SYS->PA_L_MFP &= ~(SYS_PA_L_MFP_PA4_MFP_Msk );
    SYS->PA_L_MFP |= SYS_PA_L_MFP_PA4_MFP_ADC_CH4 ;

    SYS->PA_L_MFP = (SYS->PA_L_MFP & ~SYS_PA_L_MFP_PA4_MFP_Msk) | SYS_PA_L_MFP_PA4_MFP_ADC_CH4;
    /* Disable PA.4 digital input path */
    PA->OFFD |= ((1 << 4) << GP_OFFD_OFFD_Pos);
    // Enable channel 4
    ADC_Open(ADC, ADC_INPUT_MODE_SINGLE_END, ADC_OPERATION_MODE_SINGLE, ADC_CH_4_MASK);

    // Set reference voltage to AVDD
    ADC_SET_REF_VOLTAGE(ADC, ADC_REFSEL_POWER);
    // Power on ADC
    ADC_POWER_ON(ADC);
    // Enable ADC ADC_IF interrupt
    ADC_EnableInt(ADC, ADC_ADF_INT);
    NVIC_EnableIRQ(ADC_IRQn);

}
/*---------------------------------------------------------------------------------------------------------*/
/*  AD转换函数                               	                                                           */
/*---------------------------------------------------------------------------------------------------------*/

uint32_t EADC_ConverterVbat(void)
{
	uint32_t i,u32Result=0;
	uint32_t i32ConversionData[10];
	/* Reset the ADC interrupt indicator and trigger sample module 0 to start A/D conversion */

	for(i=0;i<10;i++)
	{
		u8ADF = 0;

		ADC_START_CONV(ADC);

		while (u8ADF == 0);

		i32ConversionData[i] = ADC_GET_CONVERSION_DATA(ADC, 4);

//		ADC_DisableInt(ADC, ADC_ADF_INT);
		
		u32Result+=i32ConversionData[i];
	}
	ADC_DisableInt(ADC, ADC_ADF_INT);
	ADC_Close(ADC);
	return (u32Result/10);
}
/*---------------------------------------------------------------------------------------------------*/
/*  ??????? ????????                                                                          */
/*---------------------------------------------------------------------------------------------------*/
void ProVolt(void)               
{
//    char *res; 
		float i;
//		char buf[4];
	//	ptr=speed;
		Init_Adc();
		i=EADC_ConverterVbat();
		ADC_Close(ADC);
		advalue=((i*3.0*2)/(4096));
//		sprintf(buf,"%.2f",advalue);
//		res=buf;
//		return res;
}
/*---------------------------------------------------------------------------------------------------------*/
/*  电压采样前gpio配置                                                       							   */
/*---------------------------------------------------------------------------------------------------------*/
void V_gpio(void)
{
	PF3=0;
}
/*---------------------------------------------------------------------------------------------------------*/
/*  电压采样后gpio配置                                                       							   */
/*---------------------------------------------------------------------------------------------------------*/
void RV_gpio(void)
{
	PF3=1;	
}
/*---------------------------------------------------------------------------------------------------------*/
/*  定时器0初始化函数               1小时唤醒一次                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void Timer0_Init(uint32_t hour)
{
    //Enable Timer0 clock and select Timer1 clock source
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0_S_LXT, 0);   //分频:将时钟频率变小，拉长周期  0-16分频

    //Initial Timer1 to periodic mode with 1Hz
 //   TIMER_Open(TIMER0, TIMER_TOGGLE_MODE, 1);
	TIMER_SET_PRESCALE_VALUE(TIMER0, 0xff);
    TIMER0->CTL |= TIMER_PERIODIC_MODE;
	
//   timer->CMP = u32Cmpr;

	//??????
	TIMER_SET_CMP_VALUE(TIMER0, 128*3600*hour);
    //Enable Timer1 interrupt
    TIMER_EnableInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);
	NVIC_SetPriority(TMR0_IRQn,0);
	TIMER_EnableWakeup(TIMER0);
}
/*---------------------------------------------------------------------------------------------------------*/
/*  水流时间计数器启动计时            单位:秒                                                    		   */
/*---------------------------------------------------------------------------------------------------------*/
void Timer1_Open()
{
//    uint32_t i=0;
	//Enable Timer1 clock and select Timer1 clock source
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1_S_LXT, 0);
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1);

    // Enable timer interrupt
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);

    // Start Timer 1
    TIMER_Start(TIMER1);
//	while((TIMER1->CTL&0x80) != 0x80)
//	{
//			TIMER1->CTL |= TIMER_CTL_TMR_EN_Msk;i++;
//	}
}
/*---------------------------------------------------------------------------------------------------------*/
/*  水流时间计数器停止计时并计算所用时间                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void Timer1_Close()
{
	TIMER_Close(TIMER1);
//	CLK_DisableModuleClock(TMR1_MODULE);
}
/*---------------------------------------------------------------------------------------------------------*/
/*  密码锁启动时间计数器启动计时            单位:分                                                    	   */
/*---------------------------------------------------------------------------------------------------------*/
void Timer2_Open()
{
//    uint32_t i=0;
	//Enable Timer2 clock and select Timer1 clock source
    CLK_EnableModuleClock(TMR2_MODULE);
    CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL2_TMR2_S_LXT, 0);
	
	TIMER_SET_PRESCALE_VALUE(TIMER2, 0xff);
    TIMER2->CTL |= TIMER_PERIODIC_MODE;
	
//   timer->CMP = u32Cmpr;

	//??????
	TIMER_SET_CMP_VALUE(TIMER2, 128*60);
    //Enable Timer1 interrupt
    TIMER_EnableInt(TIMER2);
    NVIC_EnableIRQ(TMR2_IRQn);
//	NVIC_SetPriority(TMR0_IRQn,0);
	TIMER_EnableWakeup(TIMER2);

    // Start Timer 2
    TIMER_Start(TIMER2);
//	while((TIMER2->CTL&0x80) != 0x80)
//	{
//			TIMER2->CTL |= TIMER_CTL_TMR_EN_Msk;i++;
//	}
}
/*---------------------------------------------------------------------------------------------------------*/
/*  密码锁启动时间计数器停止计时并计算所用时间                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void Timer2_Close()
{
	TIMER_Close(TIMER2);
//	CLK_DisableModuleClock(TMR2_MODULE);
}
/*---------------------------------------------------------------------------------------------------------*/
/*  IO口消抖时间计数器启动计时            单位:秒                                                    	   */
/*---------------------------------------------------------------------------------------------------------*/
void Timer3_Init()
{
//    uint32_t i=0;
	//Enable Timer3 clock and select Timer1 clock source
    CLK_EnableModuleClock(TMR3_MODULE);
    CLK_SetModuleClock(TMR3_MODULE,CLK_CLKSEL2_TMR3_S_LXT, 0);
	TIMER_SET_PRESCALE_VALUE(TIMER3, 0xff);
    TIMER3->CTL |= TIMER_PERIODIC_MODE;
	
//   timer->CMP = u32Cmpr;

	//??????
	TIMER_SET_CMP_VALUE(TIMER3, 128*2);

    // Enable timer interrupt
//    TIMER_EnableInt(TIMER3);
//    NVIC_EnableIRQ(TMR3_IRQn);
//	TIMER_EnableWakeup(TIMER3);
//    // Start Timer 3
//    TIMER_Start(TIMER3);

//	while((TIMER3->CTL&0x80) != 0x80)
//	{
//			TIMER3->CTL |= TIMER_CTL_TMR_EN_Msk;i++;
//	}
}
/*---------------------------------------------------------------------------------------------------------*/
/*  IO口消抖时间计数器启动计时            单位:秒                                                    	   */
/*---------------------------------------------------------------------------------------------------------*/
void Timer3_Open()
{
    uint32_t i=0;
//	//Enable Timer3 clock and select Timer1 clock source
//    CLK_EnableModuleClock(TMR3_MODULE);
//    CLK_SetModuleClock(TMR3_MODULE,CLK_CLKSEL2_TMR3_S_LXT, 0);
//    TIMER_Open(TIMER3, TIMER_PERIODIC_MODE, 10);

    // Enable timer interrupt
    TIMER_EnableInt(TIMER3);
    NVIC_EnableIRQ(TMR3_IRQn);
	TIMER_EnableWakeup(TIMER3);
    // Start Timer 3
    TIMER_Start(TIMER3);

	while((TIMER3->CTL&0x80) != 0x80)
	{
			TIMER3->CTL |= TIMER_CTL_TMR_EN_Msk;i++;
	}
}
/*---------------------------------------------------------------------------------------------------------*/
/*  IO口消抖时间计数器停止计时并计算所用时间                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void Timer3_Close()
{
	TIMER_Close(TIMER3);
//	CLK_DisableModuleClock(TMR3_MODULE);
}
/*---------------------------------------------------------------------------------------------------------*/
/*  字符串解析函数	参数分别为1.待解析字符串地址 2.分隔符 3.第几段字符串	                               */
/*---------------------------------------------------------------------------------------------------------*/
char *strSplict( char * str, const char * seps, int pos)               				
{
    int pos1=0; 
    int count=0; 
    char *ptr,*res; 
    ptr=str; 
    while((pos1=strcspn(ptr,seps))!=strlen(ptr)) 
    { 
        count++; 
        if(count==pos+1) 
        { 
			res=(char*)malloc(pos1+1); 
			strncpy(res,ptr,pos1); 
			
			return res; 
		} 
		ptr+=pos1; 
		ptr+=1; 
    if((*ptr)==0x20)ptr+=1;
    } 
    return ( ptr ); 
}
/*---------------------------------------------------------------------------------------------------------*/
/*  字符串转16进制函数		                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
char *CharToHex(char *buf)
{
	char databuf[TDATABUFF]={0},*p;
	uint32_t i;
	strcpy(databuf,buf);
	p=buf;
	for(i=0;i<strlen(databuf);i++)
	{
		sprintf(p+2*i,"%x",databuf[i]);
		
	}
	return buf;
}
/*---------------------------------------------------------------------------------------------------------*/
/*  整数转字符串函数		                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void itoa (int n,char s[])
{
	int i,j,sign;
	if((sign=n)<0)//????
	n=-n;//?n????
	i=0;
	do{
		 s[i++]=n%10+'0';//??????
	}
	while ((n/=10)>0);//?????
	if(sign<0)
	s[i++]='-';
	s[i]='\0';
	for(j=i;j>=0;j--)//?????????,???????
	printf("%c",s[j]);
}
/*---------------------------------------------------------------------------------------------------------*/
/*  16进制转ASCI字符函数		                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
unsigned char HexToAsc(unsigned char aChar){
  
	if((aChar>=0x30)&&(aChar<=0x39))
	  
		aChar -= 0x30;
	  
	else if((aChar>=0x41)&&(aChar<=0x46))//????
	  
		aChar -= 0x37;
	  
	else if((aChar>=0x61)&&(aChar<=0x66))//????
	  
		aChar -= 0x57;
	  
	else aChar = 0xff;
	  
	return aChar;
  }
/*---------------------------------------------------------------------------------------------------------*/
/*  16进制转字符串函数		                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
char *HexToChar(char *num,char *buf,char *dbuf)
{
	char databuf[TDATABUFF]={0};
	uint32_t i,j,k;
	i=atoi(num);
	strcpy(databuf,buf);
	for(j=0;j<i;j++)
	{
		k=16*HexToAsc(databuf[2*j])+HexToAsc(databuf[2*j+1]);
		*dbuf++=k;
	}
	return dbuf;
}
/*---------------------------------------------------------------------------------------------------------*/
/*  上行参数数据包字符串转换函数		                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
char *DataTxPro(char *buf)
{
	char databuf[TDATABUFF],databuf2[TDATABUFF];
	uint32_t i;
	strcpy(databuf,buf);
	i=strlen(databuf);
	sprintf(databuf2,"%d",i);
	CharToHex(databuf);
	memset(buf,0,TDATABUFF*sizeof(char));
	strcat(buf,databuf2);
	strcat(buf,symbol[0]);
	strcat(buf,databuf);
	return buf;
}
/*---------------------------------------------------------------------------------------------------------*/
/*  MCU发送数据至NB模块                                                                       			   */
/*---------------------------------------------------------------------------------------------------------*/
void SendDataToBc28(char *cmd)
{
	char *buff;
	buff=cmd;
	do{
		UART_WRITE(UART_BC28,*buff);
		while(UART_IS_TX_FULL(UART_BC28));	
		}
	while(*(++buff)!=NULL);
	return;	
}
/*---------------------------------------------------------------------------------------------------------*/
/*  NB模块发送数据至服务器	                           													   */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t SendGpsData(char *buf)
{	
	char databuf[TDATABUFF]={0};
	char at_nsost[2*TDATABUFF]={0};
	char VT[10];
	char CSQ[5];
	uint16_t i;
	DATATP *dp,datatp;
	dp=&datatp;
    dp->start=(char*)symbol[9];
    dp->deviceid=(char*)device_id;
	dp->software=(char*)software;
	dp->end=(char*)symbol[1];
	
	V_gpio();
	ProVolt();
	RV_gpio();
	sprintf(VT, "%.2f",advalue);                    			   			
	sprintf(CSQ, "%d",csq1);

    memset(databuf,0,TDATABUFF*sizeof(uint8_t));
    strcat(databuf,dp->start);
	strcat(databuf,symbol[0]);  
    strcat(databuf,dp->deviceid);
	strcat(databuf,symbol[0]);   
    strcat(databuf,buf);
	strcat(databuf,symbol[0]); 
	strcat(databuf,VT); 
	strcat(databuf,symbol[0]); 
	strcat(databuf,CSQ); 
	strcat(databuf,symbol[0]); 
	strcat(databuf,dp->software);
	strcat(databuf,symbol[0]);
    strcat(databuf,dp->end);
//    databuf[68]=csq;
	do{
			memset(g_u8RecData2,0,UART_NB_BUFF*sizeof(char));
			buf2=g_u8RecData2;
			SendDataToBc28(AT_NSOCR);
			for(i=0;i<200;i++)
			{
				if((strstr(g_u8RecData2,"OK"))!=NULL)
					break;
				else if((strstr(g_u8RecData2,"ERROR"))!=NULL)
					break;
				delay(500);
			}
		}while(strstr(g_u8RecData2,"OK")==NULL);            		        //?????  
	AT_NSOST[9]=g_u8RecData2[2];
	strcat(at_nsost,AT_NSOST);	
	strcat(at_nsost,web_ip_port);
	DataTxPro(databuf);
	strcat(at_nsost,databuf);
	strcat(at_nsost,symbol[2]);
        
	do{
			memset(g_u8RecData2,0,UART_NB_BUFF*sizeof(char));
			buf2=g_u8RecData2;
			SendDataToBc28(at_nsost);
			for(i=0;i<200;i++)
			{
				if((strstr(g_u8RecData2,"OK"))!=NULL)
					break;
				else if((strstr(g_u8RecData2,"ERROR"))!=NULL)
				{
					if(at_nsost[9]=='0'){at_nsost[9]='1';break;}
					if(at_nsost[9]=='1'){at_nsost[9]='0';break;}
					if((at_nsost[9]!='1')&&(at_nsost[9]!='0')){at_nsost[9]='0';break;}
				}
				delay(500);
			}
			}while(strstr(g_u8RecData2,"OK")==NULL);            		        //????? 
	AT_NSOCL[9]=g_u8RecData2[2];
 	do{
			memset(g_u8RecData2,0,UART_NB_BUFF*sizeof(char));
			buf2=g_u8RecData2;
			SendDataToBc28(AT_NSOCL);
			for(i=0;i<200;i++)
			{
				if((strstr(g_u8RecData2,"OK"))!=NULL)
					break;
				else if((strstr(g_u8RecData2,"ERROR"))!=NULL)
				{
					if(AT_NSOCL[9]=='0'){AT_NSOCL[9]='1';break;}
					if(AT_NSOCL[9]=='1'){AT_NSOCL[9]='0';break;}
					if((AT_NSOCL[9]!='1')&&(AT_NSOCL[9]!='0')){AT_NSOCL[9]='0';break;}
				}
				delay(500);
			}
			}while(strstr(g_u8RecData2,"OK")==NULL);            		        //?????    
//    delay(10000);   
    return 0;
}
/*---------------------------------------------------------------------------------------------------------*/
/*  NB模块发送带符号数据至服务器	                           											   */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t SendGpsData_F(char *buf)
{	
	char databuf[TDATABUFF]={0};
	char at_nsost[2*TDATABUFF]={0};
	char VT[10];
	char CSQ[5];
	uint16_t i;
	DATATP *dp,datatp;
	dp=&datatp;
    dp->start=(char*)symbol[9];
    dp->deviceid=(char*)device_id;
	dp->software=(char*)software;
	dp->end=(char*)symbol[1];
	
	V_gpio();
	ProVolt();
	RV_gpio();
	sprintf(VT, "%.2f",advalue);                    			   			
	sprintf(CSQ, "%d",csq1);

    memset(databuf,0,TDATABUFF*sizeof(uint8_t));
    strcat(databuf,dp->start);
	strcat(databuf,symbol[0]);  
    strcat(databuf,dp->deviceid);
	strcat(databuf,symbol[0]);   
    strcat(databuf,buf);
	strcat(databuf,symbol[0]); 
	strcat(databuf,VT); 
	strcat(databuf,symbol[0]); 
	strcat(databuf,CSQ); 
	strcat(databuf,symbol[0]); 
	strcat(databuf,dp->software);
	strcat(databuf,symbol[0]);
    strcat(databuf,dp->end);
//    databuf[68]=csq;
	do{
			memset(g_u8RecData2,0,UART_NB_BUFF*sizeof(char));
			buf2=g_u8RecData2;
			SendDataToBc28(AT_NSOCR);
			for(i=0;i<200;i++)
			{
				if((strstr(g_u8RecData2,"OK"))!=NULL)
					break;
				else if((strstr(g_u8RecData2,"ERROR"))!=NULL)
					break;
				delay(500);
			}
		}while(strstr(g_u8RecData2,"OK")==NULL);            		        //?????  
	AT_NSOSTF[10]=g_u8RecData2[2];
	strcat(at_nsost,AT_NSOSTF);	
	strcat(at_nsost,web_ip_port);
	strcat(at_nsost,para[3]);
	DataTxPro(databuf);
	strcat(at_nsost,databuf);
	strcat(at_nsost,symbol[2]);
        
	do{
			memset(g_u8RecData2,0,UART_NB_BUFF*sizeof(char));
			buf2=g_u8RecData2;
			SendDataToBc28(at_nsost);
			for(i=0;i<200;i++)
			{
				if((strstr(g_u8RecData2,"OK"))!=NULL)
					break;
				else if((strstr(g_u8RecData2,"ERROR"))!=NULL)
				{
					if(at_nsost[9]=='0'){at_nsost[9]='1';break;}
					if(at_nsost[9]=='1'){at_nsost[9]='0';break;}
					if((at_nsost[9]!='1')&&(at_nsost[9]!='0')){at_nsost[9]='0';break;}
				}
				delay(500);
			}
			}while(strstr(g_u8RecData2,"OK")==NULL);            		        //????? 
	AT_NSOCL[9]=g_u8RecData2[2];
 	do{
			memset(g_u8RecData2,0,UART_NB_BUFF*sizeof(char));
			buf2=g_u8RecData2;
			SendDataToBc28(AT_NSOCL);
			for(i=0;i<200;i++)
			{
				if((strstr(g_u8RecData2,"OK"))!=NULL)
					break;
				else if((strstr(g_u8RecData2,"ERROR"))!=NULL)
				{
					if(AT_NSOCL[9]=='0'){AT_NSOCL[9]='1';break;}
					if(AT_NSOCL[9]=='1'){AT_NSOCL[9]='0';break;}
					if((AT_NSOCL[9]!='1')&&(AT_NSOCL[9]!='0')){AT_NSOCL[9]='0';break;}
				}
				delay(500);
			}
			}while(strstr(g_u8RecData2,"OK")==NULL);            		        //?????    
//    delay(10000);   
    return 0;
}
/*---------------------------------------------------------------------------------------------------------*/
/*  待机前io口处理			                           													   */
/*---------------------------------------------------------------------------------------------------------*/
void Restore_io(void)
{
	SYS->PB_H_MFP = 0x00000000;
//	GPIO_SetMode(PD, BIT12, GPIO_PMD_INPUT);
//	GPIO_DISABLE_PULL_UP(PD, BIT12);
//	PD12=1;
	
	GPIO_SetMode(PB, BIT13, GPIO_PMD_INPUT);
	GPIO_DISABLE_PULL_UP(PB, BIT13);
	PB13=0;
	
	GPIO_SetMode(PB, BIT14, GPIO_PMD_INPUT);
	GPIO_DISABLE_PULL_UP(PB, BIT14);
	PB14=0;

	PB->DOUT   = 0xFFFF ;
	PB->PMD   = 0 ;
	PB->PUEN   = 0 ;
}
/*---------------------------------------------------------------------------------------------------------*/
/*  接收服务器下行的参数	                           													   */
/*---------------------------------------------------------------------------------------------------------*/
//uint8_t ReceGpsData(void)
//{	
//    char *ptr,*num;
//    uint32_t i,l;
////	uint32_t gps_t,nb_t;
//    char databuf[TDATABUFF]={0};
//	char parabuf[TDATABUFF]={0};
//	char at_nsost[TDATABUFF]={0};
//	char at_nsorf[TDATABUFF]={0};
//	DATAQTP *dqt,dataqtp;
//	DATAQRP *dqr,dataqrp;
//	dqt=&dataqtp;
//	dqr=&dataqrp;
//	
//	dqt->start=(char*)symbol[8];
//    dqt->deviceid=(char*)device_id;
//	dqt->para=(char*)para[0];
//	dqt->end=(char*)symbol[1];   

//    strcat(databuf,dqt->start);
//    strcat(databuf,symbol[0]);
//	strcat(databuf,dqt->deviceid);
//	strcat(databuf,symbol[0]);
//	strcat(databuf,dqt->para);
//	strcat(databuf,symbol[0]);
//    strcat(databuf,dqt->end);
////	l=strlen(databuf);
////    itoa();
////	sprintf(device_id, "%d", DEVICE_ID); 			   				//???id??????
////	sprintf(software, "%02d", SOFTWARE);
//	do{
//			memset(g_u8RecData2,0,UART_NB_BUFF*sizeof(char));
//			buf2=g_u8RecData2;
//			SendDataToBc28(AT_NSOCR);
//			for(i=0;i<200;i++)
//			{
//				if((strstr(g_u8RecData2,"OK"))!=NULL)
//					break;
//				else if((strstr(g_u8RecData2,"ERROR"))!=NULL)
//					break;
//				delay(500);
//			}
//		}while(strstr(g_u8RecData2,"OK")==NULL);            		        //?????  
//	AT_NSOST[9]=g_u8RecData2[2];
//	strcat(at_nsost,AT_NSOST);

//	strcat(at_nsost,web_ip_port);
//	DataTxPro(databuf);
//	strcat(at_nsost,databuf);
//	strcat(at_nsost,symbol[2]);

//	do{
//			memset(g_u8RecData2,0,UART_NB_BUFF*sizeof(char));
//			buf2=g_u8RecData2;
//			SendDataToBc28(at_nsost);
//			for(i=0;i<200;i++)
//			{
//				if((strstr(g_u8RecData2,"OK"))!=NULL)
//					break;
//				else if((strstr(g_u8RecData2,"ERROR"))!=NULL)
//				{
//					if(at_nsost[9]=='0')at_nsost[9]='1';
//					if(at_nsost[9]=='1')at_nsost[9]='0';
//					break;
//				}
//				delay(500);
//			}
//			}while(strstr(g_u8RecData2,"OK")==NULL);            		        //?????  
//    
//    delay(20000);
//    
//	if((ptr=strstr(g_u8RecData2,"+NSONMI:"))!=NULL)
//    {
//		AT_NSORF[9]=g_u8RecData2[24];
//		ptr=strSplict(ptr,",",1);
//		strcat(at_nsorf,AT_NSORF);
//		strcat(at_nsorf,ptr);
//		do{
//				memset(g_u8RecData2,0,UART_NB_BUFF*sizeof(char));
//				buf2=g_u8RecData2;
//				SendDataToBc28(at_nsorf);
//				for(i=0;i<200;i++)
//				{
//					if((strstr(g_u8RecData2,"OK"))!=NULL)
//						break;
//					else if((strstr(g_u8RecData2,"ERROR"))!=NULL)
//						break;
//					delay(500);
//				}
//		}while(strstr(g_u8RecData2,"OK")==NULL);            		        //????? 
//		num=strSplict(g_u8RecData2,",",3);
//		ptr=strSplict(g_u8RecData2,",",4);
//		strSplict(g_u8RecData2,",",5);
//		HexToChar(num,ptr,parabuf);
//		
//		
//		SpiFlash_WaitReady();
//		SpiFlash_SectorErase(DEVICE_PARA_PAGE);
//		SpiFlash_WaitReady();
//		SpiFlash_PageProgram(parabuf,DEVICE_PARA_PAGE,14);
//		
////		SpiFlash_ReadData(databuf,DEVICE_PARA_PAGE,50);
//		
//		dqr->start=strSplict(parabuf,",",0);
//		dqr->power=strSplict(parabuf,",",1);
//		dqr->gps_t=strSplict(parabuf,",",2);
//		dqr->nb_t=strSplict(parabuf,",",3);
//		dqr->end=strSplict(parabuf,",",4);
//		
//		gps_t=atoi(dqr->gps_t+1);
//		nb_t=atoi(dqr->nb_t+1);
//		
//		if(strstr(dqr->power,"P1")!=NULL)
//		{
//			 on_off_it=POWER_ON;
//		}
//		if(strstr(dqr->power,"P0")!=NULL)
//		{
//			 on_off_it=POWER_OFF;
//		}
//    }
//    return 0;
//}
/*---------------------------------------------------------------------------------------------------------*/
/*  打开BC28函数		                                                                          		   */
/*---------------------------------------------------------------------------------------------------------*/
void Open_Bc28(void)
{
//	PD12=1;	
//	delay(50);
	PB12=1;
	delay(50);
	PB12=0;
}
/*---------------------------------------------------------------------------------------------------------*/
/*  关闭BC28函数                                                      	                                   */
/*---------------------------------------------------------------------------------------------------------*/
void Close_Bc28(void)
{
	uint8_t i;
	do{
		memset(g_u8RecData2,0,UART_NB_BUFF*sizeof(char));
		buf2=g_u8RecData2;
		SendDataToBc28(AT_CSCON_Q);
		for(i=0;i<200;i++)
		{
			if((strstr(g_u8RecData2,"OK"))!=NULL)
				break;
			else if((strstr(g_u8RecData2,"ERROR"))!=NULL)
				break;
			delay(500);
		}
	}while(strstr(g_u8RecData2,"+CSCON:0,0")==NULL);
//	do{
//		memset(g_u8RecData2,0,UART_NB_BUFF*sizeof(char));
//		buf2=g_u8RecData2;
//		SendDataToBc28(AT_CFUN0);
//		for(i=0;i<200;i++)
//		{
//			if((strstr(g_u8RecData2,"OK"))!=NULL)
//				break;
//			else if((strstr(g_u8RecData2,"ERROR"))!=NULL)
//				break;
//			delay(500);
//		}
//	}while(strstr(g_u8RecData2,"OK")==NULL);    
	
	SYS_ResetModule(UART0_RST);
	UART_Close(UART_BC28);
	Restore_io();

	delay(50);

}
/*---------------------------------------------------------------------------------------------------------*/
/*  初始化BC28接口函数                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void Init_Bc28_Interface(void)
{	
	/* Enable IP clock */
	CLK_SetModuleClock(UART_BC28_MODULE, CLK_CLKSEL1_UART_S_HIRC, CLK_UART_CLK_DIVIDER(1));
	CLK_EnableModuleClock(UART_BC28_MODULE);
	/* Set PB multi-function pins for UART0 RXD and TXD  */
	SYS->PB_H_MFP |= SYS_PB_H_MFP_PB14_MFP_UART0_TX | SYS_PB_H_MFP_PB13_MFP_UART0_RX;
	SYS_ResetModule(UART0_RST);
	UART_ENABLE_INT(UART_BC28, (UART_IER_RDA_IE_Msk | UART_IER_RTO_IE_Msk));
	NVIC_EnableIRQ(UART_BC28_IRQn);
	UART_Open(UART_BC28, 9600);
//	Open_Bc28();
}
/*---------------------------------------------------------------------------------------------------------*/
/*  查询BC28信号强度函数                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void Q_Bc28_Csq(void)
{
	char *res;
	uint32_t i;
	do{
			memset(g_u8RecData2,0,UART_NB_BUFF*sizeof(uint8_t));
			buf2=g_u8RecData2;
			SendDataToBc28(AT_CSQ);
			for(i=0;i<200;i++)
			{
				  if((strstr(g_u8RecData2,"+CSQ:"))!=NULL)
					break;
				  else if((strstr(g_u8RecData2,"ERROR"))!=NULL)
					break;
				  delay(500);
			}
		}while(strstr(g_u8RecData2,"+CSQ:")==NULL);            		    
	res=strSplict(g_u8RecData2,":",1);
	i=atoi(res);
	csq1=i;
	if((i==30)||(i==31))csq='A';
	else if((i>=20)&&(i<=29))csq='B';
	else if((i>=10)&&(i<=19))csq='C';
	else if((i>=5)&&(i<=9))csq='D';
	else csq='0';
}
/*---------------------------------------------------------------------------------------------------------*/
/*  NB模块打开函数                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void Nb_Open(void)
{
	Init_Bc28_Interface();
//	delay(10000);
}
/*---------------------------------------------------------------------------------------------------------*/
/*  NB模块初始化函数                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void Nb_Init(void)
{
	uint32_t i;
	Init_Bc28_Interface();
	Open_Bc28();
	delay(10000);
	Q_Bc28_Csq();
	if((csq=='A')||(csq=='B')||(csq=='C')||(csq=='D'))
	{
//		do{
//				memset(g_u8RecData2,0,UART_NB_BUFF*sizeof(char));
//				buf2=g_u8RecData2;
//				SendDataToBc28(AT_CIMI);
//				delay(500);

//				for(i=0;i<100;i++)
//				{
//					if((strstr(g_u8RecData2,"OK"))!=NULL)
//						break;
//					else if((strstr(g_u8RecData2,"ERROR"))!=NULL)
//						break;
//					delay(500);
//				}
//				}while(strstr(g_u8RecData2,"+CEREG:1,1")==NULL);
//		do{
//				memset(g_u8RecData2,0,UART_NB_BUFF*sizeof(char));
//				buf2=g_u8RecData2;
//				SendDataToBc28(AT_CEREG);
//				delay(500);
//				SendDataToBc28(AT_QCEREG);
//				delay(500);
//				for(i=0;i<100;i++)
//				{
//					if((strstr(g_u8RecData2,"OK"))!=NULL)
//						break;
//					else if((strstr(g_u8RecData2,"ERROR"))!=NULL)
//						break;
//					delay(500);
//				}
//			}while(strstr(g_u8RecData2,"+CEREG:1,1")==NULL); AT_NCCID           								//????
		do{
				memset(g_u8RecData2,0,UART_NB_BUFF*sizeof(char));
				buf2=g_u8RecData2;
				SendDataToBc28(AT_CFUN0);
				delay(500);

				for(i=0;i<100;i++)
				{
					if((strstr(g_u8RecData2,"OK"))!=NULL)
						break;
					else if((strstr(g_u8RecData2,"ERROR"))!=NULL)
						break;
					delay(500);
				}
		}while(strstr(g_u8RecData2,"OK")==NULL); 
		do{
				memset(g_u8RecData2,0,UART_NB_BUFF*sizeof(char));
				buf2=g_u8RecData2;
				SendDataToBc28(AT_CEDRXS);
				delay(500);

				for(i=0;i<100;i++)
				{
					if((strstr(g_u8RecData2,"OK"))!=NULL)
						break;
					else if((strstr(g_u8RecData2,"ERROR"))!=NULL)
						break;
					delay(500);
				}
		}while(strstr(g_u8RecData2,"OK")==NULL); 
		do{
				memset(g_u8RecData2,0,UART_NB_BUFF*sizeof(char));
				buf2=g_u8RecData2;
				SendDataToBc28(AT_CPSMS_SET);
				delay(500);

				for(i=0;i<100;i++)
				{
					if((strstr(g_u8RecData2,"OK"))!=NULL)
						break;
					else if((strstr(g_u8RecData2,"ERROR"))!=NULL)
						break;
					delay(500);
				}
		}while(strstr(g_u8RecData2,"OK")==NULL); 
		do{
				memset(g_u8RecData2,0,UART_NB_BUFF*sizeof(char));
				buf2=g_u8RecData2;
				SendDataToBc28(AT_CFUN1);
				for(i=0;i<200;i++)
				{
					if((strstr(g_u8RecData2,"OK"))!=NULL)
						break;
					else if((strstr(g_u8RecData2,"ERROR"))!=NULL)
						break;
					delay(500);
				}
			}while(strstr(g_u8RecData2,"OK")==NULL);            		//??gprs?????? 
		do{
				memset(g_u8RecData2,0,UART_NB_BUFF*sizeof(char));
				buf2=g_u8RecData2;
				SendDataToBc28(AT_CGATT);
				delay(500);
				SendDataToBc28(AT_QCGATT);
				delay(500);
				for(i=0;i<100;i++)
				{
					if((strstr(g_u8RecData2,"OK"))!=NULL)
						break;
					else if((strstr(g_u8RecData2,"ERROR"))!=NULL)
						break;
					delay(500);
				}
			}while(strstr(g_u8RecData2,"+CGATT:1")==NULL);            								//?????  
	}
	else {Close_Bc28();}
}
/*---------------------------------------------------------------------------------------------------------*/
/*  NB模块拨号函数                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void Nb_Bohao(void)
{
	uint32_t i;
//	memset(g_u8RecData2,0,UART_NB_BUFF*sizeof(char));
//	buf2=g_u8RecData2;
	Init_Bc28_Interface();
//	delay(2000);	
//	do{
//	memset(g_u8RecData2,0,UART_NB_BUFF*sizeof(char));
//	buf2=g_u8RecData2;
//	SendDataToBc28(AT_QCGATT);
//	for(i=0;i<200;i++)
//	{
//		if((strstr(g_u8RecData2,"OK"))!=NULL)
//			break;
//		else if((strstr(g_u8RecData2,"ERROR"))!=NULL)
//			break;
//		delay(500);
//	}
//	}while(strstr(g_u8RecData2,"+CGATT:1")==NULL); 
	
	Q_Bc28_Csq();
	if((csq=='A')||(csq=='B')||(csq=='C')||(csq=='D'))
	{

//				do{
//				memset(g_u8RecData2,0,UART_NB_BUFF*sizeof(char));
//				buf2=g_u8RecData2;
//				SendDataToBc28(ATI);
//				delay(500);

//				for(i=0;i<100;i++)
//				{
//					if((strstr(g_u8RecData2,"OK"))!=NULL)
//						break;
//					else if((strstr(g_u8RecData2,"ERROR"))!=NULL)
//						break;
//					delay(500);
//				}
//			}while(strstr(g_u8RecData2,"+CEREG:1,1")==NULL); 
		do{
				memset(g_u8RecData2,0,UART_NB_BUFF*sizeof(char));
				buf2=g_u8RecData2;
				SendDataToBc28(AT_CEREG);
				delay(500);
				SendDataToBc28(AT_QCEREG);
				delay(500);
				for(i=0;i<100;i++)
				{
					if((strstr(g_u8RecData2,"OK"))!=NULL)
						break;
					else if((strstr(g_u8RecData2,"ERROR"))!=NULL)
						break;
					delay(500);
				}
			}while(strstr(g_u8RecData2,"+CEREG:1,1")==NULL);            								//????
//		do{
//				memset(g_u8RecData2,0,UART_NB_BUFF*sizeof(char));
//				buf2=g_u8RecData2;
//				SendDataToBc28(AT_CFUN1);
//				for(i=0;i<200;i++)
//				{
//					if((strstr(g_u8RecData2,"OK"))!=NULL)
//						break;
//					else if((strstr(g_u8RecData2,"ERROR"))!=NULL)
//						break;
//					delay(500);
//				}
//			}while(strstr(g_u8RecData2,"OK")==NULL);            		//??gprs??????  
		do{
				memset(g_u8RecData2,0,UART_NB_BUFF*sizeof(char));
				buf2=g_u8RecData2;
				SendDataToBc28(AT_CGATT);
				delay(500);
				SendDataToBc28(AT_QCGATT);
				delay(500);
				for(i=0;i<100;i++)
				{
					if((strstr(g_u8RecData2,"OK"))!=NULL)
						break;
					else if((strstr(g_u8RecData2,"ERROR"))!=NULL)
						break;
					delay(500);
				}
			}while(strstr(g_u8RecData2,"+CGATT:1")==NULL);            								//?????   
	}
	else {Close_Bc28();}
}
/*---------------------------------------------------------------------------------------------------------*/
/*  GPIO开关初始化函数                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void Init_Gpio_Sw(void)
{
	/* PD11 为设备软开关*/
    GPIO_SetMode(PD, BIT11, GPIO_PMD_INPUT);	
    NVIC_EnableIRQ(GPDEF_IRQn);
    /* Configure PD11 as Input mode and enable interrupt by high and low level */
    GPIO_EnableInt(PD, 11, GPIO_INT_BOTH_EDGE);	

	GPIO_SetMode(PB, BIT12, GPIO_PMD_OUTPUT);             
//	GPIO_SetMode(PD, BIT11, GPIO_PMD_OUTPUT);
//	GPIO_SetMode(PD, BIT12, GPIO_PMD_OUTPUT);
	GPIO_SetMode(PF, BIT3, GPIO_PMD_OUTPUT);
	
	PF3=1;            //AD开关初始状态   关
	PB12=0;            //BC28开关初始状态   关
//	PD11=0;            //BC28电源开关初始状态   关
//	PD12=0;            //GPS电源开关初始状态   关
}
/*---------------------------------------------------------------------------------------------------------*/
/*  GPIO初始化函数                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void Init_Gpio(void)
{
	NVIC_DisableIRQ(GPDEF_IRQn);                             //消除PDEF中断
	GPIO_DisableInt(PD, 11);	        					 //消除PD11中断
	/* PC4 为倾斜报警检测*/
    GPIO_SetMode(PC, BIT4, GPIO_PMD_INPUT);
	/* PC10 为水流报警检测*/
    GPIO_SetMode(PC, BIT10, GPIO_PMD_INPUT);
//	GPIO_ENABLE_DEBOUNCE(PC,BIT10);
	/* PC11,PC12,PC13 为防盗报警检测*/
    GPIO_SetMode(PC, BIT11, GPIO_PMD_INPUT);
	GPIO_SetMode(PC, BIT12, GPIO_PMD_INPUT);
	GPIO_SetMode(PC, BIT13, GPIO_PMD_INPUT);	
    NVIC_EnableIRQ(GPABC_IRQn);
	
//	GPIO_ENABLE_PULL_UP(PC, BIT4);
	GPIO_ENABLE_PULL_UP(PC, BIT10);
//	GPIO_ENABLE_PULL_UP(PC, BIT11);
//	GPIO_ENABLE_PULL_UP(PC, BIT12);
//	GPIO_ENABLE_PULL_UP(PC, BIT13);
    /* Configure PC4,PC10,PC11,PC12,PC13 as Input mode and enable interrupt by high and low level */
	GPIO_EnableInt(PC, 4, GPIO_INT_BOTH_EDGE);
    GPIO_EnableInt(PC, 10, GPIO_INT_BOTH_EDGE);	
	GPIO_EnableInt(PC, 11, GPIO_INT_BOTH_EDGE);	
	GPIO_EnableInt(PC, 12, GPIO_INT_BOTH_EDGE);	
	GPIO_EnableInt(PC, 13, GPIO_INT_BOTH_EDGE);
}
/*---------------------------------------------------------------------------------------------------------*/
/*  更新闹钟函数                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void planNextRTCInterrupt(S_RTC_TIME_DATA_T *sCurTime)
{
    // plan next interrupt timing
    if(sCurTime->u32Minute < 59)
        sCurTime->u32Minute += 1;
    else
    {
        sCurTime->u32Minute = 0;

        if(sCurTime->u32Hour < 23)
            sCurTime->u32Hour += 1;
        else    // next day
        {
            sCurTime->u32Hour = 0;

            // new year first day
            if(sCurTime->u32Month==12 && sCurTime->u32Day==31)
            {
                sCurTime->u32Year += 1;
                sCurTime->u32Month = 1;
                sCurTime->u32Day = 1;
            }
            else if(sCurTime->u32Month==1 ||
                    sCurTime->u32Month==3 ||
                    sCurTime->u32Month==5 ||
                    sCurTime->u32Month==7 ||
                    sCurTime->u32Month==8 ||
                    sCurTime->u32Month==10 ||
                    sCurTime->u32Month==12)   // 1,3,5,7,8,10,12 31-day month
            {
                if(sCurTime->u32Day < 31)
                    sCurTime->u32Day += 1;
                else
                {
                    sCurTime->u32Day = 1;
                    sCurTime->u32Month += 1;
                }
            }
            else if(sCurTime->u32Month==2)     // 2, 28 or 29-day month
            {
                if(RTC_IS_LEAP_YEAR())   // leap year
                {
                    if(sCurTime->u32Day < 29)
                        sCurTime->u32Day += 1;
                    else
                    {
                        sCurTime->u32Day = 1;
                        sCurTime->u32Month += 1;
                    }
                }
                else
                {
                    if(sCurTime->u32Day < 28)
                        sCurTime->u32Day += 1;
                    else
                    {
                        sCurTime->u32Day = 1;
                        sCurTime->u32Month += 1;
                    }
                }
            }
            else if(sCurTime->u32Month==4 ||
                    sCurTime->u32Month==6 ||
                    sCurTime->u32Month==9 ||
                    sCurTime->u32Month==11)   // 4,6,9,11 30-day
            {
                if(sCurTime->u32Day < 30)
                    sCurTime->u32Day += 1;
                else
                {
                    sCurTime->u32Day = 1;
                    sCurTime->u32Month += 1;
                }
            }
        }
    }
    sCurTime->u32Second = 0;

    RTC_SetAlarmDateAndTime(sCurTime);
    RTC_EnableInt(RTC_RIER_AIER_Msk);
    NVIC_EnableIRQ(RTC_IRQn);
}
/*---------------------------------------------------------------------------------------------------------*/
/* RTC Alarm Handle                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void RTC_AlarmHandle(void)
{
	S_RTC_TIME_DATA_T sCurTime;
	alarm4=DEVICE_T;
	RTC_GetDateAndTime(&sCurTime);
	planNextRTCInterrupt(&sCurTime);
}
/*---------------------------------------------------------------------------------------------------------*/
/* RTC Tick Handle                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void RTC_TickHandle(void)
{
	if(*que2==PC_4)
	{
		if(PC4==0)io_val1++;
		if(PC4==1)io_val2++;
	}
	else if(*que2==PC_10)
	{
		if(PC10==0)io_val1++;
		if(PC10==1)io_val2++;
	}
	if(*que2==PC_11)
	{
		if(PC11==0)io_val1++;
		if(PC11==1)io_val2++;
	}
	if(*que2==PC_12)
	{
		if(PC12==0)io_val1++;
		if(PC12==1)io_val2++;
	}
	if(*que2==PC_13)
	{
		if(PC13==0)io_val1++;
		if(PC13==1)io_val2++;
	}
    g_u32TICK++;
}
/**
  * @brief  RTC ISR to handle interrupt event
  * @param  None
  * @retval None
  */
void RTC_IRQHandler(void)
{
    if ( (RTC->RIER & RTC_RIER_AIER_Msk) && (RTC->RIIR & RTC_RIIR_AIF_Msk) )        /* alarm interrupt occurred */
    {
        RTC->RIIR = 0x1;

        RTC_AlarmHandle();
    }
    if ( (RTC->RIER & RTC_RIER_TIER_Msk) && (RTC->RIIR & RTC_RIIR_TIF_Msk) )        /* tick interrupt occurred */
    {
        RTC->RIIR = 0x2;

        RTC_TickHandle();
    }
}
/*---------------------------------------------------------------------------------------------------------*/
/* RTC Alarm Handle                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t RTC_Tt(S_RTC_TIME_DATA_T sTime1,S_RTC_TIME_DATA_T sTime2)
{
	uint32_t t,t1,t2;
	t1=sTime1.u32Hour*3600+sTime1.u32Minute*60+sTime1.u32Second;
	t2=sTime2.u32Hour*3600+sTime2.u32Minute*60+sTime2.u32Second;
	t=t2-t1;
	return t;
}
/*---------------------------------------------------------------------------------------------------------*/
/* RTC 时钟使能                                                                            				   */
/*---------------------------------------------------------------------------------------------------------*/
void RTC_Init(void)
{
	CLK_EnableModuleClock(RTC_MODULE);
}
/*---------------------------------------------------------------------------------------------------------*/
/* RTC Tick Interrupt Enable                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void RTC_TickInterruptEnable(uint32_t u32TickSelection)
{
    /* Set Tick Period */
    RTC_SetTickPeriod(u32TickSelection);

    /* Enable RTC Tick Interrupt */
    RTC_EnableInt(RTC_RIER_TIER_Msk);
    NVIC_EnableIRQ(RTC_IRQn);
}
/*---------------------------------------------------------------------------------------------------------*/
/* RTC Tick Interrupt Disable                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void RTC_TickInterruptDisable(void)
{
    /* Disable RTC Tick Interrupt */
    RTC_DisableInt(RTC_RIER_TIER_Msk);
    NVIC_DisableIRQ(RTC_IRQn);
}
/*---------------------------------------------------------------------------------------------------------*/
/* RTC Alarm Interrupt Enable                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void RTC_AlarmInterruptEnable(void)
{
    /* Enable RTC Alarm Interrupt */
    RTC_EnableInt(RTC_RIER_AIER_Msk);
    NVIC_EnableIRQ(RTC_IRQn);
}
/*---------------------------------------------------------------------------------------------------------*/
/* RTC Alarm Interrupt Disable                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void RTC_AlarmInterruptDisable(void)
{
    /* Disable RTC Alarm Interrupt */
    RTC_DisableInt(RTC_RIER_AIER_Msk);
    NVIC_DisableIRQ(RTC_IRQn);
}
/*---------------------------------------------------------------------------------------------------------*/
/* RTC Alarm Interrupt Disable                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void DetectValue(uint32_t gpionum)
{
    uint32_t gn;
	gn=gpionum;
	if(gn==PC_4)
	{
		if(io_val1<io_val2)alarm1=ON;		
	}
	else if(gn==PC_10)
	{
		if(io_val1>io_val2)
		{
			if(alarm0_if==OFF)
			{
				alarm0=ON;
				alarm0_if=ON;
				RTC_GetDateAndTime(&sTime1);
			}			
		}
		else if(io_val1<io_val2)
		{
			if(alarm0_if==ON)
			{
				alarm0=OFF;
				alarm0_if=OFF;
				RTC_GetDateAndTime(&sTime2);
			}
		}
	}
	else if(gn==PC_11)
	{
		if(io_val1<io_val2)
		{
			MIMA[mima_num]='1';
			mima_num++;
			Timer2_Open();
			if(mima_num==3)
			{
				mima_num=0;
				if(strcmp(MIMA,mima)==0)alarm2=ON;
				if(strcmp(MIMA,mima)==1)alarm3=ON;
				Timer2_Close();
			}
		}		
	}
	else if(gn==PC_12)
	{
		if(io_val1<io_val2)
		{
			MIMA[mima_num]='2';
			mima_num++;
			Timer2_Open();
			if(mima_num==3)
			{
				mima_num=0;
				if(strcmp(MIMA,mima)==0)alarm2=ON;
				if(strcmp(MIMA,mima)==1)alarm3=ON;
				Timer2_Close();
			}
		}			
	}
	else if(gn==PC_13)
	{
		if(io_val1<io_val2)
		{
			MIMA[mima_num]='3';
			mima_num++;
			Timer2_Open();
			if(mima_num==3)
			{
				mima_num=0;
				if(strcmp(MIMA,mima)==0)alarm2=ON;
				if(strcmp(MIMA,mima)==1)alarm3=ON;
				Timer2_Close();
			}
		}			
	}
	io_val1=0;
	io_val2=0;
}
/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
	uint8_t i,j;
//	uint32_t io_val1,io_val2;
	char T_t[10]={0};
//	char CSQ[10]={0};
//	char VT[10]={0};
	char DATA[100]={0};
//	timer0=DEVICE_T;
	SYS_Init();

	/* Time Setting */
    sInitTime.u32Year       = 2019;
    sInitTime.u32Month      = 1;
    sInitTime.u32Day        = 23;
    sInitTime.u32Hour       = 12;
    sInitTime.u32Minute     = 30;
    sInitTime.u32Second     = 0;
    sInitTime.u32DayOfWeek  = RTC_WEDNESDAY;
    sInitTime.u32TimeScale  = RTC_CLOCK_24;
	
	RTC_Init();
	RTC_Open(&sInitTime); //user maybe need Waiting for RTC settings stable
	
//	RTC_AlarmInterruptEnable();
	Init_Gpio_Sw();	
	
	for(;;)
	{
		if(sw==ON)break;
		SYS_UnlockReg();
        CLK_PowerDown();
        SYS_LockReg();		
	}
	
//	Init_Gpio();

	I2C_EEPROM_Init();	

//    I2C_EEPROM_Write(0,0xff);
//    I2C_EEPROM_Write(1,0xff);
//    I2C_EEPROM_Write(2,0xff);
//    I2C_EEPROM_Write(3,0xff);
//    I2C_EEPROM_Write(4,0xff);
//    I2C_EEPROM_Write(5,0xff);
//    I2C_EEPROM_Write(6,0xff);
//    I2C_EEPROM_Write(7,0xff);
//    I2C_EEPROM_Write(8,0xff);
//    I2C_EEPROM_Write(9,0xff);

	i=I2C_EEPROM_Read(NADDR);
	j=I2C_EEPROM_Read(NADDR+1);	
	num=i*256+j;
	if(num==0xffff)num=0;
	
	i=I2C_EEPROM_Read(AADDR);
	j=I2C_EEPROM_Read(AADDR+1);
	addr=i*256+j;
	if(addr==0xffff)addr=DADDR;

	sprintf(device_id, "%d", DEVICE_ID); 			   				//读取设备ID号
	sprintf(software, "%02d", SOFTWARE); 			   				//读取设备固件版本号

	Nb_Init();       			   									//初始化NB模块并拨号连接	
                    			   			
	SendGpsData_F("device is opened"); 
	Close_Bc28();
	
	Timer0_Init(1);	
	TIMER_Start(TIMER0);
//	Timer3_Init();
	Init_Gpio();
	for(;;)
	{
//        if(io_it==1)
//		{
//			io_it=0;
//			io_it2=io_it1;
//			io_val1=0;
//			io_val2=0;
//			RTC_TickInterruptEnable(RTC_TICK_1_128_SEC);
//			while(g_u32TICK!=128);
//			RTC_TickInterruptDisable();
//			g_u32TICK=0;
//			DetectValue(io_it2);
//			io_it2=0;			
//		}
		if(que2!=que1)
		{
//			io_it=*que;
//			if(que2==&queue[100])
//			{
////				memset(queue,0,100*sizeof(uint32_t));
//				que2=queue;				
//			}
//			else que2++;			
			RTC_TickInterruptEnable(RTC_TICK_1_128_SEC);
			while(g_u32TICK!=128);
			RTC_TickInterruptDisable();
			g_u32TICK=0;
			DetectValue(*que2);	
			if(que2==&queue[100])
			{
//				memset(queue,0,100*sizeof(uint32_t));
				que2=queue;				
			}
			else que2++;
		}
	    /*告警优先级次序 a 被盗 b 倾斜 c 使用前后 d 检修 e 自检 */
		if(alarm3==ON)
		{
			alarm3=0;
			Nb_Bohao();                     			   			//打开NB模块并拨号连接			
			SendGpsData(alarm_it[BEIDAO]);			   				//发送被盗告警信息
			Close_Bc28(); 
		}
		if(alarm1==ON)
		{
			alarm1=0;
			Nb_Bohao();                     			   			//打开NB模块并拨号连接		
			SendGpsData_F(alarm_it[QINGXIE]); 			   			//发送倾斜告警信息    		                                    
			Close_Bc28();  
		}
		if(alarm0==ON)
		{
			alarm0=0;
			Nb_Bohao();                     			   			//打开NB模块并拨号连接		
			SendGpsData_F(alarm_it[POWERON]);			   			//发送正常开启告警信息
			Close_Bc28(); 				  			   			    //关闭NB模块	
											
		}
		if(alarm0==OFF)
		{
			alarm0=0;
			time=RTC_Tt(sTime1,sTime2);
			memset(T_t,0,10*sizeof(char));
			sprintf(T_t,"%d",time);
			I2C_EEPROM_Write(addr,time>>24);
			addr++;
			I2C_EEPROM_Write(addr,time>>16);
			addr++;
			I2C_EEPROM_Write(addr,time>>8);
			addr++;
			I2C_EEPROM_Write(addr,time&0xff);
			addr++;
			if(addr>(EEPROM-DADDR-4))addr=DADDR;
			num++;
			I2C_EEPROM_Write(AADDR,addr>>8);
			I2C_EEPROM_Write(AADDR+1,addr&0xff);
			I2C_EEPROM_Write(NADDR,num>>8);
			I2C_EEPROM_Write(NADDR+1,num&0xff);	   			    	//保存当前次消防栓开启时间

			
			Nb_Bohao();                     			   			//打开NB模块并拨号连接	
			memset(DATA,0,100*sizeof(char));
			strcat(DATA,alarm_it[POWEROFF]);			   			//发送正常关闭告警信息
			strcat(DATA,symbol[0]);
			strcat(DATA,T_t);			   							//发送开启时长信息
			SendGpsData_F(DATA);
			Close_Bc28(); 			
		}
		if(alarm2==ON)
		{
			alarm2=0;
			Nb_Bohao();                     			   			//打开NB模块并拨号连接			
			SendGpsData(alarm_it[JIANXIU]);			   				//发送检修告警信息
			Close_Bc28(); 
		}
		if(alarm4==DEVICE_T)
		{
			alarm4=0;
			Nb_Bohao();                     			   			//打开NB模块并拨号连接		
			SendGpsData_F(alarm_it[ZIJIAN]); 			   		    //发送自检告警信息    		                                    
			Close_Bc28();                     			   			//关闭NB模块		
		}
//		if(alarm5==ON)
//		{
//			alarm5=0;
//			Nb_Bohao();                     			   			//打开NB模块并拨号连接			
//			SendGpsData("MIMA is wrong"); 			   		    	//发送密码错误告警信息    	
//			Close_Bc28(); 
//		}	
		if(que2==que1)
		{
			SYS_UnlockReg();
			CLK_PowerDown();
			SYS_LockReg();			
		}	
	}
}

/*---------------------------------------------------------------------------------------------------------*/
/*  AD中断函数                                                                                         	   */
/*---------------------------------------------------------------------------------------------------------*/
void ADC_IRQHandler(void)
{
    uint32_t u32Flag;

    // Get ADC conversion finish interrupt flag
    u32Flag = ADC_GET_INT_FLAG(ADC, ADC_ADF_INT);

    if(u32Flag & ADC_ADF_INT)
		
        u8ADF = 1;

    ADC_CLR_INT_FLAG(ADC, u32Flag);
}
/*---------------------------------------------------------------------------------------------------------*/
/*  TMR0中断服务函数    设备运行时钟  1h中断一次                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void TMR0_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER0) == 1)
    {
        /* Clear Timer1 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER0);
        alarm4++;

    }
}
/*---------------------------------------------------------------------------------------------------------*/
/*  TMR1中断服务函数     水流监测计时时钟  1s中断一次                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void TMR1_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        /* Clear Timer1 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER1);
        time++;

    }

}
/*---------------------------------------------------------------------------------------------------------*/
/*  TMR2中断服务回调函数                                  							                       */
/*---------------------------------------------------------------------------------------------------------*/
void TMR2_TEST_HANDLE(void)
{
    if(TIMER_GetIntFlag(TIMER2) == 1)
    {
        /* Clear Timer1 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER2);
        alarm3=ON;

    }	
}
/*---------------------------------------------------------------------------------------------------------*/
/*  TMR2中断服务函数     密码锁检测计时时钟   1m中断一次                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void TMR2_IRQHandler(void)
{
	TMR2_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  TMR3中断服务回调函数                                  							                       */
/*---------------------------------------------------------------------------------------------------------*/
void TMR3_TEST_HANDLE(void)
{
//	uint32_t io_val1;
	if(TIMER_GetIntFlag(TIMER3) == 1)
    {
        /* Clear Timer1 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER3);
		time3=1;		
    }

}
/*---------------------------------------------------------------------------------------------------------*/
/*  TMR3中断服务函数     IO中断消抖计时时钟   1s中断一次                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void TMR3_IRQHandler(void)
{
	TMR3_TEST_HANDLE();
}
/*---------------------------------------------------------------------------------------------------------*/
/*  PORTABC中断函数                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void GPABC_IRQHandler(void)
{
	uint32_t reg;
    /* To check if PD.11 interrupt occurred */
//	io_it=1;
	if(GPIO_GET_INT_FLAG(PC, BIT11))	   			     //密码锁监测
	{
		GPIO_CLR_INT_FLAG(PC, BIT11); 
//		io_it1=PC_11;
		*que1=PC_11;
		if(que1==&queue[100])que1=queue;
		else 
		que1++;
	}
    else if(GPIO_GET_INT_FLAG(PC, BIT12))	   			//密码锁监测
    {
        GPIO_CLR_INT_FLAG(PC, BIT12);
//		io_it1=PC_12;
		*que1=PC_12;
		if(que1==&queue[100])que1=queue;
		else 
		que1++;
	}
	else if(GPIO_GET_INT_FLAG(PC, BIT13))	   			 //密码锁监测
	{
		GPIO_CLR_INT_FLAG(PC, BIT13);
//		io_it1=PC_13;
		*que1=PC_13;
		if(que1==&queue[100])que1=queue;
		else 
		que1++;
	}
	else if(GPIO_GET_INT_FLAG(PC, BIT4))	   			 //震动监测
	{
		GPIO_CLR_INT_FLAG(PC, BIT4);
//		io_it1=PC_4;
		*que1=PC_4;
		if(que1==&queue[100])que1=queue;
		else 
		que1++;		
	}
	else if(GPIO_GET_INT_FLAG(PC, BIT10))	   			 //水流监测
	{
		GPIO_CLR_INT_FLAG(PC, BIT10);
//		io_it1=PC_10;
		*que1=PC_10;
		if(que1==&queue[100])que1=queue;
		else 
		que1++;		
	}
    else
    {
        /* Un-expected interrupt. Just clear all PORTA, PORTB, PORTC interrupts */
        reg = PA->ISRC;
        PA->ISRC = reg;
        reg = PB->ISRC;
        PB->ISRC = reg;
        reg = PC->ISRC;
        PC->ISRC = reg;
//        printf("Un-expected interrupts. \n");
    }
}
/*---------------------------------------------------------------------------------------------------------*/
/*  PORTDEF中断函数                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void GPDEF_IRQHandler(void)
{
	uint32_t reg;
	if(GPIO_GET_INT_FLAG(PD, BIT11))	   			        //设备软开关
	{
		GPIO_CLR_INT_FLAG(PD, BIT11); 
//		io_val=PD11;
//		io_it=PD_11;
//		Timer3_Open();
		delay(100);
		if(PD11==0)sw=OFF;
		if(PD11==1)sw=ON;
	}	
    else
    {
        /* Un-expected interrupt. Just clear all PORTA, PORTB, PORTC interrupts */
        reg = PD->ISRC;
        PD->ISRC = reg;
        reg = PE->ISRC;
        PE->ISRC = reg;
        reg = PF->ISRC;
        PF->ISRC = reg;
//        printf("Un-expected interrupts. \n");
    }
}
/*---------------------------------------------------------------------------------------------------------*/
/* UART Wake Up Handle function                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_TEST_HANDLE(void)
{
	uint32_t u32IntStatus;
//	if(buf2==&g_u8RecData2[UART_GC65_BUFF-1])buf2=g_u8RecData2;
    u32IntStatus = UART0->ISR;

    /* Wake Up */
    if (u32IntStatus & UART_ISR_WAKE_IS_Msk) {
//        printf("UART_Wakeup. \n");
        UART0->ISR = UART_ISR_WAKE_IS_Msk; //clear status
    }
	if(u32IntStatus & UART_ISR_RDA_IS_Msk){
		UART0->ISR = UART_ISR_RDA_IS_Msk; //clear status
		while(UART_IS_RX_READY(UART0))
        {
            *buf2 = UART_READ(UART0);
            buf2++;
        }
	}
	if(u32IntStatus & UART_IER_RTO_IE_Msk){
		UART0->ISR = UART_ISR_RTO_IS_Msk; //clear status
	}
}
void UART0_IRQHandler(void)
{
    UART0_TEST_HANDLE();
}
