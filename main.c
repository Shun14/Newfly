/*
 * File:   main.c
 * Author: zouzh
 *
 * Created on April 14, 2017, 5:08 PM
 */

// CONFIG1
#pragma config FEXTOSC = OFF    // External Oscillator mode selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT32 // Power-up default value for COSC bits (HFINTOSC with OSCFRQ= 32 MHz and CDIV = 1:1)
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; i/o or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (FSCM timer enabled)

// CONFIG2
#pragma config MCLRE = OFF      // Master Clear Enable bit (MCLR pin function is port defined function)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config LPBOREN = OFF    // Low-Power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = OFF      // Brown-out reset enable bits (Brown-out reset disabled)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (VBOR) set to 1.9V on LF, and 2.45V on F Devices)
#pragma config ZCD = OFF        // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR.)
#pragma config PPS1WAY = ON     // Peripheral Pin Select one-way control (The PPSLOCK bit can be cleared and set only once in software)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a reset)

// CONFIG3
#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled, SWDTEN is ignored)
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4
#pragma config WRT = OFF        // UserNVM self-write protection bits (Write protection off)
#pragma config SCANE = not_available// Scanner Enable bit (Scanner module is not available for use)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (High Voltage on MCLR/Vpp must be used for programming)

// CONFIG5
#pragma config CP = OFF         // UserNVM Program memory code protection bit (Program Memory code protection disabled)
#pragma config CPD = ON         // DataNVM code protection bit (Data EEPROM code protection enabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>


#define  Table_Size  252//(128+10+64)    //数字个数 * 每个字符占据的长度
unsigned int row_now = 0;             //要显示的字库中的行

uint8_t const LOVE_TABLE[Table_Size]= 
{
    0,0,0,0,0,
    255,1,1,1,1,1,1,255,       //U
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,             //
    24,36,66,66,129,129,66,66,
    36,24,0,0,0,0,0,0,          //O
     0,0,0,0,0,0,0,0,           //
    0,0,128,64,64,32,31,32,
    64,64,128,0,0,0,0,0,       //Y
    
     0,0,0,0,0,0,0,0,           //
    0,0,0,255,145,145,145,145,  
    145,145,145,145,  
    145,145,145,145,  
    145,145,145,0,0,0,0,0,     //E
     0,0,0,0,0,0,0,0,          //
    0,0,0,128,64,32,16,8,     //V
    4,2,1,2,4,8,16,32,
    64,128,0,0,0,0,0,0,
     0,0,0,0,0,0,0,0,          //
    0,24,24,36,36,66,66,66,66,129,129,129,129,66,66,66,
    66,36,36,24,24,0,0,0,0,0,        //O
     0,0,0,0,0,0,0,0,         //
    0,0,0,1,1,1,1,1,
    1,1,1,1,1,
    1,1,255,0,0,0,0,0,        //L
     0,0,0,0,0,0,0,0,         //
    0,0,255,0,0,0,0,0,        //I
     0,0,0,0,0,0,0,0,         //
    0,0,0,0,0
};

void OSCInit()       //时钟初始化
{
    OSCCON1bits.NDIV = 0b0011;   //8分频=4M
    OSCCON1bits.NOSC = 0b000;    //32M
}
void GPIOInit()       //IO口初始化
{
	/*
	  A口初始化
	   0,1,2,3,4,5,6,7
	*/
      TRISA  =0;  //方向设置 输出
      ANSELA =0;  //数字地
      LATAbits.LATA7   =1;  //点亮
      LATAbits.LATA6   =1;  //点亮
      LATAbits.LATA5   =1;  //点亮
      LATAbits.LATA4   =1;  //点亮
      LATAbits.LATA3   =1;  //点亮
      LATAbits.LATA2   =1;  //点亮
      LATAbits.LATA1   =1;  //点亮
      LATAbits.LATA0   =1;  //点亮
	/*
	  B口初始化
	   0,1,2,3,4,5
	*/
      TRISB  =0;  //方向设置 输出
      ANSELB =0;  //数字地
      LATBbits.LATB5   =1;  //点亮
      LATBbits.LATB4   =1;
      LATBbits.LATB3   =1;  //点亮
      LATBbits.LATB2   =1;  //点亮
      LATBbits.LATB1   =1;  //点亮
      LATBbits.LATB0   =1;
	/*
	  C口初始化
	   0,1,2,3,4,5,6
	*/
      TRISC  =0;  //方向设置 输出
      ANSELC =0;  //数字地 
      LATCbits.LATC6   =1; 
      LATCbits.LATC5   =1; 
      LATCbits.LATC4   =1; 
      LATCbits.LATC3   =1; 
      LATCbits.LATC2   =1; 
      LATCbits.LATC1   =1;
      LATCbits.LATC0   =1; 
    /*
	  D口初始化
	   0,1,2,3
	*/
      TRISD  =0;  //方向设置 输出
      ANSELD =0;  //数字地  
      LATDbits.LATD3   =1; 
      LATDbits.LATD2   =1; 
      LATDbits.LATD1   =1;
      LATDbits.LATD0   =1; 
    /*
	  E口初始化
	   0,1,2,3
	*/
      TRISE  =0;  //方向设置 输出
      ANSELE =0;  //数字地  
      LATEbits.LATE2   =1; //绿
      LATEbits.LATE1   =1; //红
      LATEbits.LATE0   =1; //蓝
    return;
}
void interruptInit(void)  //中断初始化
{
    INTCONbits.GIE  = 1;  //全局中断
    INTCONbits.PEIE = 1;  //引脚电平跳变中断
}

void IOinterruptinit()   //引脚中断初始化
{
//   IOCEFbits.IOCEF3 = 0;
//  // INTCONbits.IOCIF=0;
//   INTCONbits.PEIE=1;   //引脚中断允许
//   IOCEPbits.IOCEP3=1;  //上升沿中断
//  // OPTION_REGbits.nWPUEN=1;
//   WPUEbits.WPUE3=1;
    
   
  //  IOCCFbits.IOCCF1 = 0;   //清除标识
   // PIR0bits.IOCIF=0;

    IOCCPbits.IOCCP1 = 1;   //允许C1脚上升沿中断
    WPUCbits.WPUC1 = 1;
    
    PIE0bits.IOCIE = 1;     //允许引脚中断
}

void Timer0Init(void)    //定时器2初始化
{
    TMR0L = 0x00;
    TMR0H = 125;
    // 允许中断
    T0CON0bits.T0EN = 1;
    //设置timer0 为 8位计时器
    T0CON0bits.T016BIT = 0;
    //output divider
    T0CON0bits.T0OUTPS = 0b1111;//16 分频
    
    //8 125k   125=1ms
    //64       125 = 8ms
    //64*128    125 = 1.024s
    
    T0CON1bits.T0CS = 0b010;   //fosc/4为时钟源
    T0CON1bits.T0ASYNC = 0;    //使用fosc/4时钟源
    T0CON1bits.T0CKPS = 0b1001; //1:1分频
    
    PIE0bits.TMR0IE = 1;  //使能timer0中断
}
void led_blue(unsigned int num)
{
    LATA = 0x00;
    LATB = 0x00;
    LATC = 0x00;
    LATD = 0x00;
    LATE = 0x00;
    
    
    LATCbits.LATC0 = (num&0x01)?1:0;  //A6=red //A7 = green //C0 = blue
    LATCbits.LATC2 = (num&0x02)?1:0;
    LATEbits.LATE0 = (num&0x04)?1:0;
    LATAbits.LATA3 = (num&0x08)?1:0;
    
    LATAbits.LATA0 = (num&0x10)?1:0;
    LATBbits.LATB5 = (num&0x20)?1:0;
    LATDbits.LATD1 = (num&0x40)?1:0;
    LATCbits.LATC4 = (num&0x80)?1:0;
}
void led_red(unsigned int num)
{
    LATA = 0x00;
    LATB = 0x00;
    LATC = 0x00;
    LATD = 0x00;
    LATE = 0x00;
    
    
    LATAbits.LATA6 = (num&0x01)?1:0;  //A6=red //A7 = green //C0 = blue
    LATCbits.LATC3 = (num&0x02)?1:0;
    LATEbits.LATE1 = (num&0x04)?1:0;
    LATAbits.LATA4 = (num&0x08)?1:0;
    
    LATAbits.LATA1 = (num&0x10)?1:0;
    LATBbits.LATB4 = (num&0x20)?1:0;
    LATDbits.LATD2 = (num&0x40)?1:0;
    LATCbits.LATC5 = (num&0x80)?1:0;
}
void led_green(unsigned int num)
{
    LATA = 0x00;
    LATB = 0x00;
    LATC = 0x00;
    LATD = 0x00;
    LATE = 0x00;
    
    
    LATAbits.LATA7 = (num&0x01)?1:0;  //A6=red //A7 = green //C0 = blue
    LATDbits.LATD0 = (num&0x02)?1:0;
    LATEbits.LATE2 = (num&0x04)?1:0;
    LATAbits.LATA5 = (num&0x08)?1:0;
    
    LATAbits.LATA2 = (num&0x10)?1:0;
    LATBbits.LATB3 = (num&0x20)?1:0;
    LATDbits.LATD3 = (num&0x40)?1:0;
    LATCbits.LATC6 = (num&0x80)?1:0;
}

void main(void) 
{
    OSCInit();        //设置时钟4M
    GPIOInit();       //引脚初始化
    interruptInit();  //总中断初始化
    IOinterruptinit();//霍尔中断初始化
//    Timer0Init();
    led_green(0x00);
    
    LATBbits.LATB0 = 1;
    while(1);
    return;
}
int i = 0;
void interrupt ISR_handler()  //中断处理函数
{
     //定时器2中断处理函数
//    if (PIR4bits.TMR2IF == 1) 
//    {
//        PIR4bits.TMR2IF = 0; //清中断
//        TMR2  = 0;
//        if(i == 0)
//        {
//            led_green(0x0f);
//            i = 1;
//        }
//        else
//        {
//            led_green(0xf0);
//            i = 0;
//        }
//    }
    if ( PIR0bits.IOCIF == 1) {
        IOCCFbits.IOCCF1 = 0;
        PIR0bits.IOCIF = 0;
        if(i == 0)
        {
            led_green(0x0f);
            i = 1;
        }
        else
        {
             led_red(0xf0);
            i = 0;
        }
    }
    
}