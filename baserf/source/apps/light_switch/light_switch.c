/***********************************************************************************
  Filename: light_switch.c

  Description:  This application function either as a light or a
  switch toggling the ligh. The role of the
  application is chosen in the menu with the joystick at initialisation.

  Push S1 to enter the menu. Choose either switch or
  light and confirm choice with S1.
  Joystick Up: Sends data from switch to light

***********************************************************************************/

/***********************************************************************************
* INCLUDES
*/
#include <hal_lcd.h>
#include <hal_led.h>
#include <hal_joystick.h>
#include <hal_assert.h>
#include <hal_board.h>
#include <hal_int.h>
#include "hal_mcu.h"
#include "hal_button.h"
#include "hal_rf.h"
#include "util_lcd.h"
#include "basic_rf.h"
#include "lowpower.h"


/***********************************************************************************
* CONSTANTS
*/
// Application parameters
#define RF_CHANNEL                25      // 2.4 GHz RF channel

// BasicRF address definitions
#define PAN_ID                0x2007
#define SWITCH_ADDR           0x2520
#define LIGHT_ADDR            0xBEEF
#define APP_PAYLOAD_LENGTH        1
#define LIGHT_NO_CMD              55
#define LIGHT_TOGGLE_CMD          22
#define END_TO_CO                 11

// Application states
#define IDLE                      0
#define SEND_CMD                  1

// Application role
#define NONE                      0
#define SWITCH                    1
#define LIGHT                     2
#define APP_MODES                 2

/***********************************************************************************
* LOCAL VARIABLES
*/
static uint8 ontimes = 0;
static uint8 offtimes = 0;
static uint8 pTxData[APP_PAYLOAD_LENGTH];
static uint8 pTxNoData[APP_PAYLOAD_LENGTH];
static uint8 pRxData[APP_PAYLOAD_LENGTH];
static basicRfCfg_t basicRfConfig;

// Mode menu
static menuItem_t pMenuItems[] =
{
#ifdef ASSY_EXP4618_CC2420
  // Using Softbaugh 7-seg display
  " L S    ", SWITCH,
  " LIGHT  ", LIGHT
#else
  // SRF04EB and SRF05EB
  "Switch",   SWITCH,
  "Light",    LIGHT
#endif
};

static menu_t pMenu =
{
  pMenuItems,
  N_ITEMS(pMenuItems)
};


#ifdef SECURITY_CCM
// Security key
static uint8 key[]= {
    0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7,
    0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf,
};
#endif

/***********************************************************************************
* LOCAL FUNCTIONS
*/
void Delay(uint n);
void InitKeyINT(void);
void InitKey2INT(void);

static void appLight();
static void appSwitch();
static uint8 appSelectMode(void);



/****************************
//延时
*****************************/
void Delay(uint n)
{
	uint i;
	for(i = 0;i<n;i++);
	for(i = 0;i<n;i++);
	for(i = 0;i<n;i++);
	for(i = 0;i<n;i++);
	for(i = 0;i<n;i++);
        for(i = 0;i<n;i++);
	for(i = 0;i<n;i++);
	for(i = 0;i<n;i++);
	for(i = 0;i<n;i++);
	for(i = 0;i<n;i++);
        for(i = 0;i<n;i++);
	for(i = 0;i<n;i++);
	for(i = 0;i<n;i++);
	for(i = 0;i<n;i++);
	for(i = 0;i<n;i++);
        
}

/****************************
//初始化按键为中断输入方式
*****************************/
void InitKeyINT(void)
{
  P0INP &= 0xFE; //上拉    
  P0IEN |= 0X01;   //P00设置为中断方式
  PICTL |= 0X01;   //下降沿触发
  EA = 1;
  IEN1 |= 0X20;   // P0设置为中断方式;
  P0IFG |= 0x00;   //初始化中断标志位
  
}

/****************************
//初始化按键2为中断输入方式
*****************************/
void InitKey2INT(void)
{
  P1INP &= 0xFB;
  P2INP &= 0x9F; //上拉    
  P1IEN |= 0X04;   //P12设置为中断方式
  PICTL &= 0XFD;   //上升沿触发
  EA = 1;
  IEN2 |= 0X10;   // P1设置为中断方式;
  P2IFG |= 0x00;   //初始化中断标志位
  
}

/***********************************************************************************
* @fn          appLight
*
* @brief       Application code for light application. Puts MCU in endless
*              loop waiting for user input from joystick.
*
* @param       basicRfConfig - file scope variable. Basic RF configuration data
*              pRxData - file scope variable. Pointer to buffer for RX data
*
* @return      none
*/
static void appLight()
{
  //  halLcdWriteLine(HAL_LCD_LINE_1, "Light");
   // halLcdWriteLine(HAL_LCD_LINE_2, "Ready");
   uchar i=0; 
   int n = 0;
#ifdef ASSY_EXP4618_CC2420
    halLcdClearLine(1);
    halLcdWriteSymbol(HAL_LCD_SYMBOL_RX, 1);
#endif

    // Initialize BasicRF
    basicRfConfig.myAddr = LIGHT_ADDR;
    if(basicRfInit(&basicRfConfig)==FAILED) {
      HAL_ASSERT(FALSE);
    }
    basicRfReceiveOn();
    //basicRfReceiveOff();
    pTxData[0] = END_TO_CO;
    halBoardInit();
    InitLed();
    InitKeyINT();
    InitKey2INT();
    
    
    // Main loop
    while (TRUE) {
       n = 1000;
       //HAL_LED_CLR_1();
       //LED1 = 0;
      if(basicRfInit(&basicRfConfig)==FAILED) {
        HAL_ASSERT(FALSE);
      }
             
//       while(!basicRfPacketIsReady());             
//        if(basicRfReceive(pRxData, APP_PAYLOAD_LENGTH, NULL)>0) {
//            if(pRxData[0] == LIGHT_TOGGLE_CMD) {
//                 basicRfSendPacket(SWITCH_ADDR, pTxData, APP_PAYLOAD_LENGTH);
//                 LED2 = ~LED2;
//            }
//         }
//      if(halButtonPushed()==HAL_BUTTON_1){
//        basicRfSendPacket(SWITCH_ADDR, pTxData, APP_PAYLOAD_LENGTH);
//        halLedToggle(2);
//        halLedToggle(1);
//      }
        basicRfSendPacket(SWITCH_ADDR, pTxData, APP_PAYLOAD_LENGTH);
        basicRfReceiveOn();
        DelayMS(5);
        while(!basicRfPacketIsReady()&&(n--));
 //       while(!basicRfPacketIsReady());
        basicRfReceiveOff();
        if(basicRfReceive(pRxData, APP_PAYLOAD_LENGTH, NULL)>0) {
            if((pRxData[0] == LIGHT_NO_CMD)&&(offtimes <1)){               
                 LED1 = 0; 
                 offtimes ++;
                 ontimes = 0;
            }   
            else if((pRxData[0] == LIGHT_TOGGLE_CMD)&&(ontimes<1)){
                 LED1 = 1; 
                 ontimes++;
                 offtimes = 0;
               }
             else{
              // times = 0;
             }        
        }       
       Set_ST_Period(2);   //设置睡眠时间,睡眠1秒后唤醒系统
       SysPowerMode(2);    //重新进入睡眠模式PM2
       
    }
}


/***********************************************************************************
* @fn          appSwitch
*
* @brief       Application code for switch application. Puts MCU in
*              endless loop to wait for commands from from switch
*
* @param       basicRfConfig - file scope variable. Basic RF configuration data
*              pTxData - file scope variable. Pointer to buffer for TX
*              payload
*              appState - file scope variable. Holds application state
*
* @return      none
*/
static void appSwitch()
{
 //   halLcdWriteLine(HAL_LCD_LINE_1, "Switch");
  //  halLcdWriteLine(HAL_LCD_LINE_2, "Joystick Push");
  //  halLcdWriteLine(HAL_LCD_LINE_3, "Send Command");
  
#ifdef ASSY_EXP4618_CC2420
    halLcdClearLine(1);
    halLcdWriteSymbol(HAL_LCD_SYMBOL_TX, 1);
#endif

    pTxData[0] = LIGHT_NO_CMD;
    pTxNoData[0] = LIGHT_TOGGLE_CMD;

    // Initialize BasicRF
    basicRfConfig.myAddr = SWITCH_ADDR;
    if(basicRfInit(&basicRfConfig)==FAILED) {
      HAL_ASSERT(FALSE);
    }

    // Keep Receiver off when not needed to save power
    //basicRfReceiveOff();
    basicRfReceiveOn();
    int voicetrigger = 0;
    // Main loop
    while (TRUE) {
      //  if( halJoystickPushed() ) {  //bu qiujie tech     
             
        while(!basicRfPacketIsReady()); 
          
        if(basicRfReceive(pRxData, APP_PAYLOAD_LENGTH, NULL)>0) {
//            if(halButtonPushed()==HAL_BUTTON_1){
//              pTxData[0] = LIGHT_TOGGLE_CMD;
//            }
            if(pRxData[0] == END_TO_CO) {
//              if(halButtonPushed()==HAL_BUTTON_1){
//                 basicRfSendPacket(LIGHT_ADDR, pTxData, APP_PAYLOAD_LENGTH);
//              }
//              else{
//                basicRfSendPacket(LIGHT_ADDR, pTxNoData, APP_PAYLOAD_LENGTH);
//              }
                
                //voicetrigger = MCU_IO_GET(HAL_BOARD_IO_BTN_1_PORT, HAL_BOARD_IO_BTN_1_PIN);
//                if(VOICEIN||BUTTONIN){
//                  basicRfSendPacket(LIGHT_ADDR, pTxData, APP_PAYLOAD_LENGTH);
//                }
//                else {
//                  basicRfSendPacket(LIGHT_ADDR, pTxNoData, APP_PAYLOAD_LENGTH);
//                }
                
                 if((!VOICEIN)||BUTTONIN){
                  basicRfSendPacket(LIGHT_ADDR, pTxNoData, APP_PAYLOAD_LENGTH);
                }
                else {
                  basicRfSendPacket(LIGHT_ADDR, pTxData, APP_PAYLOAD_LENGTH);
                }
              
                
                LED2 = ~LED2;                 
            }
         }
            
    }
}

/****************************
//中断处理函数
*****************************/
#pragma vector = P0INT_VECTOR
 __interrupt void P00_ISR(void)
 {
  if(P0IFG>0)            //按键中断
  {
    P0IFG = 0;
    Delay(100);  
    if(P0IFG==0)         //按键中断
    {
      //SysPowerMode(4);
      BUZZER = 0;
      Delay(100);  
      //KeyTouchtimes = KeyTouchtimes+1;  //每次中断发生时记录按键次数加1  
      P1DIR |= 0x03;
      LED1 = ~LED1;     
     
      BUZZER = 1;
    }  
  }       
  P0IF = 0;             //清中断标志
 }

/****************************
//中断处理函数
*****************************/
#pragma vector = P1INT_VECTOR
 __interrupt void P12_ISR(void)
 {
  if(P1IFG>0)            //按键中断
  {
    P1IFG = 0;
    Delay(100);  
    if(P1IFG==0)         //按键中断
    {
      //SysPowerMode(4); 
      BUZZER = 0;
      Delay(100);  
     // KeyTouchtimes = KeyTouchtimes+1;  //每次中断发生时记录按键次数加1   
      P1DIR |= 0x03;
      LED1 = ~LED1;     
      
       BUZZER = 1;
    }  
  }       
  P1IF = 0;             //清中断标志
 }

/***********************************************************************************
* @fn          main
*
* @brief       This is the main entry of the "Light Switch" application.
*              After the application modes are chosen the switch can
*              send toggle commands to a light device.
*
* @param       basicRfConfig - file scope variable. Basic RF configuration
*              data
*              appState - file scope variable. Holds application state
*
* @return      none
*/
void main(void)
{
     
    uint8 appMode = NONE;
    
    // Config basicRF
    basicRfConfig.panId = PAN_ID;
    basicRfConfig.channel = RF_CHANNEL;
    basicRfConfig.ackRequest = TRUE;
#ifdef SECURITY_CCM
    basicRfConfig.securityKey = key;
#endif

    // Initalise board peripherals
    halBoardInit();
  //  halJoystickInit();//BY QIUJIE 

    // Initalise hal_rf
    if(halRfInit()==FAILED) {
      HAL_ASSERT(FALSE);
    }
   
    
    // Indicate that device is powered
    //halLedSet(1);
    //halLedClear(1);  
     
    
    InitSleepTimer();
    

    // Print Logo and splash screen on LCD
   // utilPrintLogo("Light Switch");

    // Wait for user to press S1 to enter menu
  //  while (halButtonPushed()!=HAL_BUTTON_1);
  //  halMcuWaitMs(350);
  //  halLcdClear();

    // Set application role
   // appMode = appSelectMode();
   // halLcdClear();
    // appMode =  SWITCH;
    // Transmitter application
  //  if(appMode == SWITCH) {
        // No return from here
    
    
    //注：函数appSwitch（）和appLight()只能打开一个
    
    //作为开关板打开此函数（appSwitch）
      // appSwitch();
    
    //被点灯的板打开此函数（appLight）   
   
       appLight();
        
        
        
  //  }
    // Receiver application
  //  else if(appMode == LIGHT) {
        // No return from here
    
  //  }
    // Role is undefined. This code should not be reached
  //  HAL_ASSERT(FALSE);
}


/****************************************************************************************
* @fn          appSelectMode
*
* @brief       Select application mode
*
* @param       none
*
* @return      uint8 - Application mode chosen
*/
static uint8 appSelectMode(void)
{
    halLcdWriteLine(1, "Device Mode: ");

    return utilMenuSelect(&pMenu);
}

/****************************************************************************************
  Copyright 2007 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
***********************************************************************************/
