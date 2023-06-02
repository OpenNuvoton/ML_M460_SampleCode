/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    This is a G-sensor real-time example with TFLITE v2.
 *           This example can change different model.
 *           Please remember that the pose should be: nu-link port is outside user's body.
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "config.h"
#include <vector>
using namespace std;

#include "MainClassify/MainClassify.h"
#include "BufAttributes.h"
#include "MPU6500.h"

#define NAU8822     1

static uint32_t s_au32PcmRxBuff[2][BUFF_LEN] = {{0}};
static DMA_DESC_T DMA_TXDESC[2], DMA_RXDESC[2];

//KWS
//MainClassify *kws;

static volatile uint8_t s_u8TxIdx = 0, s_u8RxIdx = 0;
static volatile uint8_t s_u8CopyData = 0;
static volatile uint8_t s_u8TimeUpFlag = 0;
static volatile uint8_t s_u8CopygsensorData = 0;
static volatile uint16_t s_u16gsensorSize = 0;
volatile uint32_t g_u32Ticks = 0;
#ifdef  __cplusplus
extern  "C" {
#endif 
void TMR0_IRQHandler(void);
void SysTick_Handler(void);
//void PDMA0_IRQHandler(void);
#ifdef  __cplusplus
}
#endif 

void SYS_Init(void);
void PDMA_Init(void);
void I2C2_Init(void);

#if NAU8822
void I2C_WriteNAU8822(uint8_t u8Addr, uint16_t u16Data);
void NAU8822_Setup(void);
#else
uint8_t I2C_WriteMultiByteforNAU88L25(uint8_t u8ChipAddr, uint16_t u16SubAddr, const uint8_t *p, uint32_t u32Len);
uint8_t I2C_WriteNAU88L25(uint16_t u16Addr, uint16_t u16Dat);
void NAU88L25_Reset(void);
void NAU88L25_Setup(void);
#endif

void SysTick_Handler()
{
    g_u32Ticks++;
}


void TMR0_IRQHandler(void)
{
    s_u8TimeUpFlag = 1;
    //printf("TMR0_IRQHandler\n");

   // clear timer interrupt flag
    TIMER_ClearIntFlag(TIMER0);
}

//
// sensor_timer run
//
void sensor_timer_run(void)
{
    // Set timer frequency to 1HZ
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 100);//Sets IMU default sample rate to 100Hz

    // Enable timer interrupt
    TIMER_EnableInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);

    // Start Timer 0
    TIMER_Start(TIMER0);
}

void SYS_Init(void)
{
    /* Set PF multi-function pins for XT1_OUT(PF.2) and XT1_IN(PF.3) */
    SET_XT1_OUT_PF2();
    SET_XT1_IN_PF3();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC and HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HIRC and HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_HXTSTB_Msk);

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock to 200MHz */
    CLK_SetCoreClock(FREQ_200MHZ);

    /* Enable all GPIO clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPCCKEN_Msk | CLK_AHBCLK0_GPDCKEN_Msk |
                    CLK_AHBCLK0_GPECKEN_Msk | CLK_AHBCLK0_GPFCKEN_Msk | CLK_AHBCLK0_GPGCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;
    CLK->AHBCLK1 |= CLK_AHBCLK1_GPICKEN_Msk | CLK_AHBCLK1_GPJCKEN_Msk;

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable I2C2 module clock */
    CLK_EnableModuleClock(I2C2_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();
		
    /* Enable TIMER0 clock */   
    CLK_EnableModuleClock(TMR0_MODULE);
		
    /* Set TIMER0 clock */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);

    /* Set I2C2 multi-function pins */
    SET_I2C2_SDA_PD0();
    SET_I2C2_SCL_PD1();

    /* Enable I2C2 clock pin (PD1) schmitt trigger */
    PD->SMTEN |= GPIO_SMTEN_SMTEN1_Msk;
}


/* Init I2C interface */
void I2C2_Init(void)
{
    /* Open I2C2 and set clock to 100k */
    I2C_Open(I2C2, 100000);

    /* Get I2C2 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C2));
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("+-----------------------------------------------------------------------+\n");
    //printf("|                keyword spotting inference offline with tflite         |\n");
    printf("|                Magic Wand real time inference with TFLM         |\n");
    printf("+-----------------------------------------------------------------------+\n");
    printf("  NOTE: \n");

    /* Use systick to measure inference time */
    SysTick_Config(SystemCoreClock / 1000);
	
    /* Init I2C2 to access codec */
    I2C2_Init();
	
    /*IMU gsensor init*/
    MPU6500_low_levle_init();
   		
    /*gsensor 100Hz timer init*/
    sensor_timer_run();

    const char outputClass[4][8] = {
       "O",
       "L",
       "W",
       "NULL"};
   float threshold = 0.5;

   //int16_t audioBuffer[] ALIGNMENT_ATTRIBUTE = WAVE_DATA;
   //const uint32_t audioBufferElements = sizeof(audioBuffer) / sizeof(int16_t);
			 
   const uint32_t gsensorBufferElements = 128*3;//sizeof(audioBuffer) / sizeof(int16_t);
			 
   MainClassify mainclassify(gsensorBufferElements);

   printf("Classifying..\r\n");

   g_u32Ticks = 0;

   mainclassify.Classify();  // Classify the extracted features.

   int maxIndex = mainclassify.GetTopClass(mainclassify.output);

   printf("Detected %s (%d%%)\r\n", outputClass[maxIndex],
   (static_cast<int>(mainclassify.output[maxIndex]*100)));

    while(1)
    {
       /*
         Every 10ms, a timer triggers to fill the IMU data into data buffer.
       */
       if(s_u8TimeUpFlag)
       {
           s_u8CopygsensorData = mainclassify.FillSensorData();
           s_u8TimeUpFlag = 0;
       }

       /*
	  Once the SensorData Buffer (128 samples, ax,ay,az) is filled, do the inference.
       */
      if(s_u8CopygsensorData)
      {
          /*
           Hand gesture classify
          */
          printf("Classifying..\r\n");
          g_u32Ticks = 0;
          mainclassify.Classify();  
          printf("inference time is %d ms\n", g_u32Ticks);					
          s_u8CopygsensorData = 0;
						
						
          maxIndex = mainclassify.GetTopClass(mainclassify.output);
			
          /*
           Thresholding to reduce false-alarm
          */
          if(mainclassify.output[maxIndex] > threshold)
          {	
              printf("Detected %s (%d%%)\r\n", outputClass[maxIndex],
              (static_cast<int>(mainclassify.output[maxIndex]*100)));
          }
          else
          {
              printf("Detected %s (%d%%), and less than threshold!\r\n", "NULL",
             (static_cast<int>(mainclassify.output[maxIndex]*100)));
          }
       }//if(s_u8CopygsensorData)
    }//while(1)
}
