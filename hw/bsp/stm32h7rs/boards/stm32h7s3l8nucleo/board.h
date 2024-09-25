/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021, Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#ifndef BOARD_H_
#define BOARD_H_

#ifdef __cplusplus
 extern "C" {
#endif

#define LED_PORT              GPIOB
#define LED_PIN               GPIO_PIN_0
#define LED_STATE_ON          1

#define BUTTON_PORT           GPIOC
#define BUTTON_PIN            GPIO_PIN_13
#define BUTTON_STATE_ACTIVE   1

#define UART_DEV              USART3
#define UART_CLK_EN           __HAL_RCC_USART3_CLK_ENABLE
#define UART_GPIO_PORT        GPIOC
#define UART_GPIO_AF          GPIO_AF7_USART3
#define UART_TX_PIN           GPIO_PIN_10
#define UART_RX_PIN           GPIO_PIN_11

// VBUS Sense detection
#define OTG_FS_VBUS_SENSE     0
#define OTG_HS_VBUS_SENSE     1

// STM32F723 has only one USB HS peripheral
// Nucleo board does not have ULPI so USB will operate in FS mode only
// For the rest of the synopsys driver it is FS device however there
// is only USB_OTG_HS defined. Here are required conversions to
// make peripheral FS.
//#define __HAL_RCC_USB2_OTG_FS_CLK_ENABLE __HAL_RCC_USB1_OTG_HS_CLK_ENABLE
//#define GPIO_AF10_OTG2_HS               GPIO_AF10_OTG1_HS
// #define USB_OTG_FS                      USB_OTG_HS
// #define USB_OTG_FS_PERIPH_BASE          USB_OTG_HS_PERIPH_BASE

//--------------------------------------------------------------------+
// RCC Clock
//--------------------------------------------------------------------+
static inline void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
  RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };

  /* The PWR block is always enabled on the H7 series- there is no clock
     enable. For now, use the default VOS3 scale mode (lowest) and limit clock
     frequencies to avoid potential current draw problems from bus
     power when using the max clock speeds throughout the chip. */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY); // Select the power supply
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE0); // VOS0 for max speed

  /* Enable HSE Oscillator and activate PLL1 with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.CSIState = RCC_CSI_OFF;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL1.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL1.PLLSource = RCC_PLLSOURCE_HSE;

  RCC_OscInitStruct.PLL1.PLLM = 12;
  RCC_OscInitStruct.PLL1.PLLN = 300;
  RCC_OscInitStruct.PLL1.PLLP = 1;
  RCC_OscInitStruct.PLL1.PLLQ = 2;
  RCC_OscInitStruct.PLL1.PLLR = 2;
  RCC_OscInitStruct.PLL1.PLLS = 2;
  RCC_OscInitStruct.PLL1.PLLT = 2;
  RCC_OscInitStruct.PLL1.PLLFractional = 0;

  RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL2.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL2.PLLM = 12;
  RCC_OscInitStruct.PLL2.PLLN = 200;
  RCC_OscInitStruct.PLL2.PLLP = 2;
  RCC_OscInitStruct.PLL2.PLLQ = 2;
  RCC_OscInitStruct.PLL2.PLLR = 2;
  RCC_OscInitStruct.PLL2.PLLS = 2;
  RCC_OscInitStruct.PLL2.PLLT = 2;
  RCC_OscInitStruct.PLL2.PLLFractional = 0;
  RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_NONE;

  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK4|RCC_CLOCKTYPE_PCLK5;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;

  /* Unlike on the STM32F4 family, it appears the maximum APB frequencies are
     device-dependent- 120 MHz for this board according to Figure 2 of
     the datasheet. Dividing by half will be safe for now. */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
  RCC_ClkInitStruct.APB5CLKDivider = RCC_APB5_DIV2;

  /* 4 wait states required for 168MHz and VOS0. */
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);

  /* Like on F4, on H7, USB's actual peripheral clock and bus clock are
     separate. However, the main system PLL (PLL1) doesn't have a direct
     connection to the USB peripheral clock to generate 48 MHz, so we do this
     dance. This will connect PLL1's Q output to the USB peripheral clock. */
  RCC_PeriphCLKInitTypeDef RCC_PeriphCLKInitStruct = { 0 };

    RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USBPHYC;
    RCC_PeriphCLKInitStruct.UsbPhycClockSelection = RCC_USBPHYCCLKSOURCE_HSE;
  HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

  /*TODO, CHECK IF THESE ARE APPLIED ELSEWHERE*/
      HAL_PWREx_EnableUSBVoltageDetector();

    /* Peripheral clock enable */
    //__HAL_RCC_USB_OTG_HS_CLK_ENABLE();
    //__HAL_RCC_USBPHYC_CLK_ENABLE();

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(OTG_HS_IRQn, 6, 0); // prio 6 corrrect? with or without freertos?
    //HAL_NVIC_EnableIRQ(OTG_HS_IRQn);
    /*END TODO^*/

  /*
  __HAL_RCC_CRS_CLK_ENABLE();
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB2;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000, 1000);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;
  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);*/
}

static inline void board_stm32h7rs_post_init(void)
{
  // For this board does nothing
}


#ifdef __cplusplus
 }
#endif

#endif
