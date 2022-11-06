//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//测试硬件：单片机STM32F429IGT6,正点原子Apollo STM32F4/F7开发板,主频180MHZ，晶振12MHZ
//QDtech-TFT液晶驱动 for STM32 IO模拟
//xiao冯@ShenZhen QDtech co.,LTD
//公司网站:www.qdtft.com
//淘宝网站：http://qdtech.taobao.com
//wiki技术网站：http://www.lcdwiki.com
//我司提供技术支持，任何技术问题欢迎随时交流学习
//固话(传真) :+86 0755-23594567 
//手机:15989313508（冯工） 
//邮箱:lcdwiki01@gmail.com    support@lcdwiki.com    goodtft@163.com 
//技术支持QQ:3002773612  3002778157
//技术交流QQ群:324828016
//创建日期:2018/08/09
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 深圳市全动电子技术有限公司 2018-2028
//All rights reserved
/****************************************************************************************************
//=========================================电源接线================================================//
//     LCD模块                STM32单片机
//      VCC          接        DC5V/3.3V        //电源
//      GND          接          GND            //电源地
//=======================================液晶屏数据线接线==========================================//
//本模块数据总线类型为8位并口总线
//     LCD模块                STM32单片机    
//     LCD_D0~D7     接        PE8~PE15         //液晶屏8位并口数据信号
//=======================================液晶屏控制线接线==========================================//
//     LCD模块 				  STM32单片机 
//     LCD_WR        接          PC7           //液晶屏写数据控制信号
//     LCD_RD        接          PC6           //液晶屏读数据控制信号
//     LCD_RS        接          PC8           //液晶屏数据/命令控制信号
//     LCD_RST       接          PC10          //液晶屏复位控制信号
//     LCD_CS        接          PC9           //液晶屏片选控制信号
//=========================================触摸屏触接线=========================================//
//如果模块不带触摸功能或者带有触摸功能，但是不需要触摸功能，则不需要进行触摸屏接线
//	   LCD模块                STM32单片机 
//      T_IRQ        接          PH10        //触摸屏触摸中断信号
//      T_DO         接          PH11          //触摸屏SPI总线读信号
//      T_DIN        接          PH12         //触摸屏SPI总线写信号
//      T_CS         接          PH13         //触摸屏片选控制信号
//      T_CLK        接          PH9         //触摸屏SPI总线时钟信号
**************************************************************************************************/	
 /* @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, QD electronic SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
**************************************************************************************************/			
#ifndef __LED_H
#define __LED_H
#include "sys.h"

//LED端口定义
#define LED0 PBout(1)   //LED0
#define LED1 PBout(0)   //LED1

void LED_Init(void);//初始化		 				    
#endif
