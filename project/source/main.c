/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016 - 2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_device_registers.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_debug_console.h"

#include "fsl_pint.h"
#include "fsl_common.h"
#include "fsl_inputmux.h"

#if (defined(FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT > 0U))
#include "fsl_sysmpu.h"
#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#include "pin_mux.h"
#include <stdbool.h>
#include "fsl_power.h"
#include "common.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#include "pin_mux.h"
#include <stdbool.h>
#include "fsl_power.h"
#include "common.h"

#include "fsl_spi.h"
#include "fsl_gpio.h"
#include "fsl_spi_freertos.h"

#include "sx1276.h"

#define BUTTON_SW5 kINPUTMUX_GpioPort1Pin1ToPintsel
#define DIO0_IRQ   kINPUTMUX_GpioPort1Pin18ToPintsel
//#define DIO2_IRQ   kINPUTMUX_GpioPort2Pin0ToPintsel

extern TaskHandle_t sx1276_task_handler;
extern TaskHandle_t sx1276_tx_handler;
extern void radio_PttPress();

TaskHandle_t ptt_handler;


/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void
BOARD_InitHardware(void);

void audio_main3(void *handle)
{
//	SX1276_Init(1);
    while (1) {
//		PRINTF("%s\n", __func__);
        taskYIELD()
        ;
    }
}


void ptt_main(void *handle)
{
    while (1) {

        if (GPIO_PinRead(GPIO, 1, 1)) {
            if (ptt_state == PTT_PRESS) {
                ptt_state = PTT_RELEASE;
              //  PRINTF("PTT release\r\n");		//PTT release
              //  radio_PttRelease();
                taskYIELD();
            }
        }
        else {
            if (ptt_state == PTT_RELEASE) {
                ptt_state = PTT_PRESS;
              //  PRINTF("PTT press\r\n");
                radio_PttPress();				//PTT press
            }
        }
        vTaskDelay(1);
    }
}

//ADD_TASK_WORK(NULL, audio_main3,"audio_test3", configMINIMAL_STACK_SIZE, NULL, TASK_AUDIO_PRIORITY);
ADD_TASK_WORK(&ptt_handler, ptt_main, "ptt_main", 1024, NULL, TASK_AUDIO_PRIORITY);

void CreateTasks(void)
{
    TaskWork *ptable;
    TaskWork *table_start;
    TaskWork *table_end;
    extern unsigned int __task_section_table;
    extern unsigned int __task_section_table_end;
    table_start = (TaskWork *) &__task_section_table;
    table_end = (TaskWork *) &__task_section_table_end;
    for (ptable = table_start; ptable < table_end; ++ptable) {
        //	PRINTF("name=%s", ptable->pcName);
        if (xTaskCreate(ptable->pxTaskCode, ptable->pcName,
                ptable->usStackDepth, ptable->pvParameters, ptable->uxPriority,
                ptable->handle) != pdPASS) {
            PRINTF("app task create failed!\r\n");
            return;
        }
    }

}

/*******************************************************************************
 * sx127x SPI controller configure code
 ******************************************************************************/
#define EXAMPLE_SPI_MASTER SPI9
#define EXAMPLE_SPI_MASTER_IRQ FLEXCOMM9_IRQn
#define EXAMPLE_SPI_SSEL kSPI_Ssel1
#define EXAMPLE_MASTER_SPI_SPOL kSPI_SpolActiveAllLow
#define SPI_NVIC_PRIO 2

#define SPI2_MASTER SPI2
#define SPI2_MASTER_IRQ FLEXCOMM2_IRQn
#define SPI2_SSEL kSPI_Ssel0
spi_rtos_handle_t master_rtos_handle;
spi_transfer_t masterXfer = { 0 };
uint8_t masterReceiveBuffer[32];
uint8_t masterSendBuffer[32];

gpio_pin_config_t config_gpio_input = { kGPIO_DigitalInput, 0, };
gpio_pin_config_t config_gpio_output = { kGPIO_DigitalOutput, 0, };

void LPC54018_SPI_Send_Data(unsigned char *TxBuffer, unsigned int num)
{
    status_t status;
    /*Start master transfer*/
    masterXfer.txData = TxBuffer;
    masterXfer.dataSize = num;
    masterXfer.rxData = masterReceiveBuffer;
    masterXfer.configFlags |= kSPI_FrameAssert;
    status = SPI_RTOS_Transfer(&master_rtos_handle, &masterXfer);
   // PRINTF("TxBuffer 0x%x masterReceiveBuffer 0x%x \r\n",TxBuffer[0],masterReceiveBuffer[0]);

}
void LPC54018_SPI_Get_Data(unsigned char *RxBuffer, unsigned int num)
{
    status_t status;
    /*Start master transfer*/
    masterXfer.txData = masterSendBuffer;
    masterXfer.dataSize = num;
    masterXfer.rxData = RxBuffer;
    masterXfer.configFlags |= kSPI_FrameAssert;
    status = SPI_RTOS_Transfer(&master_rtos_handle, &masterXfer);
   // PRINTF("Get data : 0x%x\r\n", RxBuffer[0]);
}

static void Sx127x_Gpio_Init(void)
{
    GPIO_PortInit(GPIO, 1);
    GPIO_PinInit(GPIO, 1, 17, &config_gpio_output);    // SX1278 reset
    GPIO_PinInit(GPIO, 1, 18, &config_gpio_input);     // SX1278 DIO0

   // GPIO_PortInit(GPIO, 0);
   // GPIO_PinInit(GPIO, 0, 1, &config_gpio_output);     // SX1278 NSS

  //  GPIO_PinWrite(GPIO, 0, 1, 0);
  //  GPIO_PinWrite(GPIO, 0, 1, 1);

    GPIO_PortInit(GPIO, 4);
    GPIO_PinInit(GPIO, 4, 6, &config_gpio_output);     // SX1278 NSS

 //   GPIO_PinWrite(GPIO, 4, 6, 0);
 //   GPIO_PinWrite(GPIO, 4, 6, 1);

    GPIO_PortInit(GPIO, 0);
    GPIO_PinInit(GPIO, 0, 5, &config_gpio_output);     // SX1278 TXEN

    GPIO_PortInit(GPIO, 3);
    GPIO_PinInit(GPIO, 3, 13, &config_gpio_output);     // SX1278 RXEN
    GPIO_PinInit(GPIO, 3, 23, &config_gpio_output);     // SX1278 SSN

    GPIO_PortInit(GPIO, 2);
    GPIO_PinInit(GPIO, 2, 0, &config_gpio_input);     // SX1278 DI02

    GPIO_PortInit(GPIO, 1);
    GPIO_PinInit(GPIO, 1, 1, &config_gpio_input);	   // sw5 for test

}

status_t Sx127x_Spi_Ctrl_Init(void)
{
    spi_master_config_t masterConfig;
    status_t status;
    uint32_t sourceClock;

    /* attach 12 MHz clock to SPI9 */
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM9);

    NVIC_SetPriority(EXAMPLE_SPI_MASTER_IRQ, SPI_NVIC_PRIO + 1);

    SPI_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Bps = 5000000;
    masterConfig.sselNum = EXAMPLE_SPI_SSEL;
    masterConfig.sselPol = (spi_spol_t) EXAMPLE_MASTER_SPI_SPOL;

    sourceClock = 10000000;
    status = SPI_RTOS_Init(&master_rtos_handle, EXAMPLE_SPI_MASTER,
            &masterConfig, sourceClock);

    if (status != kStatus_Success) {
        PRINTF("SPI master: error during initialization. \r\n");
        return status;
    }

    return status;
}

status_t Sx127x_Spi2_Ctrl_Init(void)
{
    spi_master_config_t masterConfig;
    status_t status;
    uint32_t sourceClock;

    /* attach 12 MHz clock to SPI2 */
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM2);

    NVIC_SetPriority(SPI2_MASTER_IRQ, SPI_NVIC_PRIO);

    SPI_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Bps = 3000000;
    masterConfig.sselNum = SPI2_SSEL;
    masterConfig.sselPol = (spi_spol_t) EXAMPLE_MASTER_SPI_SPOL;

    sourceClock = 10000000;
    status = SPI_RTOS_Init(&master_rtos_handle, SPI2_MASTER,
            &masterConfig, sourceClock);

    if (status != kStatus_Success) {
        PRINTF("SPI master: error during initialization. \r\n");
        return status;
    }

    return status;
}

/*******************************************************************************
 * sx127x SPI controller configure code
 ******************************************************************************/

void pint_intr_callback(pint_pin_int_t pintr, uint32_t pmatch_status)
{
   // PRINTF("kPINT_PinIntEnableFallEdge%d, ptt_state %d\r\n", pintr, ptt_state);
    ptt_state ^=  ptt_state;
    if (ptt_state == PTT_RELEASE) {
	   PRINTF("PTT release\r\n");		//PTT release
	   radio_PttRelease();
    } else if (ptt_state == PTT_PRESS) {
    	PRINTF("PTT Press\r\n");
    }
}

void dio0_intr_callback(pint_pin_int_t pintr, uint32_t pmatch_status)
{

//    PRINTF("dio0_intr_callback radio_state=%d tick %d\r\n", radio_state ,xTaskGetTickCountFromISR());


    if (RADIO_TX == radio_state) {
        radio_state = RADIO_TX_DONE;
        if (xTaskResumeFromISR(sx1276_tx_handler)) {
            taskYIELD();
        }
    }
    else if(RADIO_RX == radio_state) {
        if (xTaskResumeFromISR(sx1276_task_handler)) {
            taskYIELD();
        }
    }

}

void init_gpio_int()
{
    INPUTMUX_Init(INPUTMUX);

	INPUTMUX_AttachSignal(INPUTMUX, kPINT_PinInt0, BUTTON_SW5);
    INPUTMUX_AttachSignal(INPUTMUX, kPINT_PinInt1, DIO0_IRQ);

    INPUTMUX_Deinit(INPUTMUX);
    PINT_Init(PINT);

	PINT_PinInterruptConfig(PINT, kPINT_PinInt0, kPINT_PinIntEnableBothEdges, pint_intr_callback);
    PINT_PinInterruptConfig(PINT, kPINT_PinInt1, kPINT_PinIntEnableRiseEdge, dio0_intr_callback);

	NVIC_SetPriority(PIN_INT0_IRQn, 3);
    NVIC_SetPriority(PIN_INT1_IRQn, 3);

	PINT_EnableCallbackByIndex(PINT, kPINT_PinInt0);
    PINT_EnableCallbackByIndex(PINT, kPINT_PinInt1);

}

int main(void)
{
    CLOCK_EnableClock(kCLOCK_InputMux);
    CLOCK_EnableClock(kCLOCK_Iocon);
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Gpio1);
    /* USART0 clock */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    BOARD_InitBootPins();
    BOARD_BootClockPLL180M();
    BOARD_InitDebugConsole();
    Sx127x_Gpio_Init();
    Sx127x_Spi2_Ctrl_Init();
    init_gpio_int();
  //  usb_main();

    CreateTasks();

    vTaskStartScheduler();
    return 1U;

}
