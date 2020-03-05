/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_dma.h"
#include "fsl_i2c.h"
#include "fsl_i2s.h"
#include "fsl_i2s_dma.h"
#include "fsl_wm8904.h"
#include "fsl_codec_common.h"
#include "fsl_dmic.h"
#include "fsl_dmic_dma.h"
#include "common.h"
#include "sx1276.h"

#include "pin_mux.h"
#include <stdbool.h>
#include "fsl_codec_adapter.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_I2C (I2C2)
#define DEMO_I2S_MASTER_CLOCK_FREQUENCY (24576000U)
#define DEMO_I2S_TX (I2S0)
#define DEMO_DMA (DMA0)

#define DMAREQ_DMIC1 17U
#define DEMO_DMIC_RX_CHANNEL DMAREQ_DMIC1
#define DEMO_I2S_TX_CHANNEL (13)
#define DEMO_DMIC_CHANNEL kDMIC_Channel1
#define DEMO_DMIC_CHANNEL_ENABLE DMIC_CHANEN_EN_CH1(1)

#define DEMO_AUDIO_BIT_WIDTH (16)
#define DEMO_AUDIO_SAMPLE_RATE (8000)
#define DEMO_AUDIO_PROTOCOL kCODEC_BusI2S
#define FIFO_DEPTH (15U)
#define BUFFER_SIZE (320)
#define BUFFER_NUM (2U)

#define LOG_ENABLE 0

#define I2S_CLOCK_DIVIDER                                                 \
    (CLOCK_GetAudioPllOutFreq() / DEMO_AUDIO_SAMPLE_RATE / 16U /                          \
     2U) /* I2S source clock 24.576MHZ, sample rate 48KHZ, bits width 16, \
single channel, so bitclock should be 48KHZ * 16 * 2 = 1536KHZ, divider should be 24.576MHZ / 1536KHZ / 2 */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
wm8904_config_t wm8904Config = {
		.i2cConfig = {
				.codecI2CInstance = BOARD_CODEC_I2C_INSTANCE,
				.codecI2CSourceClock = BOARD_CODEC_I2C_CLOCK_FREQ
		 },
		.recordSource = kWM8904_RecordSourceLineInput,
		.recordChannelLeft = kWM8904_RecordChannelLeft2,
		.recordChannelRight = kWM8904_RecordChannelRight2,
		.playSource = kWM8904_PlaySourceDAC,
		.slaveAddress = WM8904_I2C_ADDRESS,
		.protocol = kWM8904_ProtocolI2S,
		.format = {
				.sampleRate = kWM8904_SampleRate8kHz,
				.bitWidth = kWM8904_BitWidth16
		 },
		.mclk_HZ = DEMO_I2S_MASTER_CLOCK_FREQUENCY,
		.master = false,
};
codec_config_t boardCodecConfig = {
		.codecDevType = kCODEC_WM8904,
		.codecDevConfig = &wm8904Config
};
static i2s_config_t tx_config;
extern codec_config_t boardCodecConfig;
static uint8_t s_buffer[BUFFER_SIZE * BUFFER_NUM];
static uint32_t volatile s_writeIndex = 0U;
static uint32_t volatile s_emptyBlock = BUFFER_NUM;
static dmic_dma_handle_t s_dmicDmaHandle;
static dma_handle_t s_dmicRxDmaHandle;
static dma_handle_t s_i2sTxDmaHandle;
static i2s_dma_handle_t s_i2sTxHandle;
SDK_ALIGN(dma_descriptor_t s_dmaDescriptorPingpong[2], 16);

extern radio_s radio_state;
extern ppt_state_s ptt_state;

static dmic_transfer_t s_receiveXfer[2U] = {
/* transfer configurations for channel0 */
{
		.data = s_buffer,
		.dataWidth = sizeof(uint16_t),
		.dataSize = BUFFER_SIZE,
		.dataAddrInterleaveSize = kDMA_AddressInterleave1xWidth,
		.linkTransfer = &s_receiveXfer[1],
},

{
		.data = &s_buffer[BUFFER_SIZE],
		.dataWidth = sizeof(uint16_t),
		.dataSize = BUFFER_SIZE,
		.dataAddrInterleaveSize = kDMA_AddressInterleave1xWidth,
		.linkTransfer = &s_receiveXfer[0],
},
};

uint8_t codecHandleBuffer[CODEC_HANDLE_SIZE] = { 0U };
codec_handle_t *codecHandle = (codec_handle_t *) codecHandleBuffer;
/*******************************************************************************
 * Code
 ******************************************************************************/

void dmic_Callback(DMIC_Type *base, dmic_dma_handle_t *handle, status_t status,
		void *userData) {
	static int pre = 0;
//	log_info("%s: %d:%d\r\n", __func__, s_emptyBlock, xTaskGetTickCount()-pre);
	pre = xTaskGetTickCount();
	if (s_emptyBlock) {
		s_emptyBlock--;
	}
}

unsigned char gEncode[48];
void i2s_Callback(I2S_Type *base, i2s_dma_handle_t *handle,
		status_t completionStatus, void *userData) {
	static int pre = 0;
	log_info("%s: %d:%d\r\n", __func__, s_emptyBlock, xTaskGetTickCount()-pre);
	pre = xTaskGetTickCount();
	if (s_emptyBlock < BUFFER_NUM) {
//		s_emptyBlock++;
	}
}

/*!
 * @brief Main function
 */
struct codec2 *C2Handle;
QueueHandle_t AudioMsgSendQueue;
QueueHandle_t AudioMsgRecvQueue;

BaseType_t SendOneFrame(Encodec_t data)
{
	if (NULL == AudioMsgSendQueue){
		AudioMsgSendQueue = xQueueCreate(8, sizeof( Encodec_t ));
	}
	return  xQueueSend(AudioMsgSendQueue, &data, 0);
}

BaseType_t GetThreeFrame(Decodec_t *data){
	if (NULL == AudioMsgRecvQueue){
		AudioMsgRecvQueue = xQueueCreate(8, sizeof( Decodec_t ));
	}
	return xQueueReceive(AudioMsgRecvQueue, data, portMAX_DELAY);
}
void audio_main_capture(void *handle) {
	dmic_channel_config_t dmic_channel_cfg;
	i2s_transfer_t i2sTxTransfer;
    C2Handle = codec2_create(CODEC2_MODE_2400);

	/* Board pin, clock, debug console init */
	pll_config_t audio_pll_config = {
		.desiredRate = 24576000U,
		.inputRate = 12000000U,
	};

	pll_setup_t audio_pll_setup;

	/* I2C clock */
	CLOCK_AttachClk(kFRO12M_to_FLEXCOMM2);

	/* Initialize AUDIO PLL clock */
	CLOCK_SetupAudioPLLData(&audio_pll_config, &audio_pll_setup);
	audio_pll_setup.flags = PLL_SETUPFLAG_POWERUP | PLL_SETUPFLAG_WAITLOCK;
	CLOCK_SetupAudioPLLPrec(&audio_pll_setup, audio_pll_setup.flags);

	/* I2S clocks */
	CLOCK_AttachClk(kAUDIO_PLL_to_FLEXCOMM6);
	CLOCK_AttachClk(kAUDIO_PLL_to_FLEXCOMM7);

	/* DMIC uses audio pll as clock source, divider 8, 24576000/8=3.072MHZ */
	CLOCK_AttachClk(kAUDIO_PLL_to_DMIC);
	CLOCK_SetClkDiv(kCLOCK_DivDmicClk, 48, false);

	/* Attach high speed clock to MCLK for I2S */
	CLOCK_AttachClk(kAUDIO_PLL_to_MCLK);
	SYSCON->MCLKDIV = SYSCON_MCLKDIV_DIV(1 - 1);
	SYSCON->MCLKIO = 1U;

	/* reset FLEXCOMM for I2C */
	RESET_PeripheralReset(kFC2_RST_SHIFT_RSTn);

	/*  If this case runs in RAM, the debuger reset must
	 be selected as Core in case not resetting SRAM after downloading,
	 so the peripherals used will also not be reset by IDE tool.
	 In this case these peripherals used should be reset manually by software
	 */
	RESET_PeripheralReset(kDMA_RST_SHIFT_RSTn);

	/* reset FLEXCOMM for I2S */
	RESET_PeripheralReset(kFC6_RST_SHIFT_RSTn);
	RESET_PeripheralReset(kFC7_RST_SHIFT_RSTn);

	PRINTF("Configure WM8904 codec\r\n");

	/* protocol: i2s
	 * sampleRate: 48K
	 * bitwidth:16
	 */
	if (CODEC_Init(codecHandle, &boardCodecConfig) != kStatus_Success) {
		PRINTF("WM8904_Init failed!\r\n");
	}

	/* Adjust it to your needs, 0x0006 for -51 dB, 0x0039 for 0 dB etc. */
	CODEC_SetVolume(codecHandle,
			kCODEC_PlayChannelHeadphoneLeft | kCODEC_PlayChannelHeadphoneRight,
			0x0032);

	DMA_Init(DEMO_DMA);

	DMA_EnableChannel(DEMO_DMA, DEMO_I2S_TX_CHANNEL);
	DMA_EnableChannel(DEMO_DMA, DEMO_DMIC_RX_CHANNEL);
	DMA_SetChannelPriority(DEMO_DMA, DEMO_I2S_TX_CHANNEL,
			kDMA_ChannelPriority3);
	DMA_SetChannelPriority(DEMO_DMA, DEMO_DMIC_RX_CHANNEL,
			kDMA_ChannelPriority2);
	DMA_CreateHandle(&s_i2sTxDmaHandle, DEMO_DMA, DEMO_I2S_TX_CHANNEL);
	DMA_CreateHandle(&s_dmicRxDmaHandle, DEMO_DMA, DEMO_DMIC_RX_CHANNEL);

	memset(&dmic_channel_cfg, 0U, sizeof(dmic_channel_config_t));

	dmic_channel_cfg.divhfclk = kDMIC_PdmDiv1;
	dmic_channel_cfg.osr = 32U;
	dmic_channel_cfg.gainshft = 3U;
	dmic_channel_cfg.preac2coef = kDMIC_CompValueZero;
	dmic_channel_cfg.preac4coef = kDMIC_CompValueZero;
	dmic_channel_cfg.dc_cut_level = kDMIC_DcCut155;
	dmic_channel_cfg.post_dc_gain_reduce = 1U;
	dmic_channel_cfg.saturate16bit = 1U;
	dmic_channel_cfg.sample_rate = kDMIC_PhyFullSpeed;
	DMIC_Init(DMIC0);
	DMIC_SetIOCFG(DMIC0, kDMIC_PdmDual);
	DMIC_Use2fs(DMIC0, true);
	DMIC_EnableChannelDma(DMIC0, DEMO_DMIC_CHANNEL, true);
	DMIC_ConfigChannel(DMIC0, DEMO_DMIC_CHANNEL, kDMIC_Left, &dmic_channel_cfg);

	DMIC_FifoChannel(DMIC0, DEMO_DMIC_CHANNEL, FIFO_DEPTH, true, true);
	DMIC_EnableChannnel(DMIC0, DEMO_DMIC_CHANNEL_ENABLE);
	PRINTF("Configure I2S\r\n");

	/*
	 * masterSlave = kI2S_MasterSlaveNormalMaster;
	 * mode = kI2S_ModeI2sClassic;
	 * rightLow = false;
	 * leftJust = false;
	 * pdmData = false;
	 * sckPol = false;
	 * wsPol = false;
	 * divider = 1;
	 * oneChannel = false;
	 * dataLength = 16;
	 * frameLength = 32;
	 * position = 0;
	 * fifoLevel = 4;
	 */
	I2S_TxGetDefaultConfig(&tx_config);
	tx_config.divider = I2S_CLOCK_DIVIDER;
	tx_config.oneChannel = true;
	I2S_TxInit(DEMO_I2S_TX, &tx_config);
	I2S_TxTransferCreateHandleDMA(DEMO_I2S_TX, &s_i2sTxHandle,
			&s_i2sTxDmaHandle, i2s_Callback, NULL);
	DMIC_TransferCreateHandleDMA(DMIC0, &s_dmicDmaHandle, dmic_Callback, NULL,
			&s_dmicRxDmaHandle);
	DMIC_InstallDMADescriptorMemory(&s_dmicDmaHandle, s_dmaDescriptorPingpong,
			2U);
	DMIC_TransferReceiveDMA(DMIC0, &s_dmicDmaHandle, s_receiveXfer,
			DEMO_DMIC_CHANNEL);

	while (1) {
		uint32_t volatile s_pemptyBlock = BUFFER_NUM;
		Encodec_t codec;

        int tick1,tick2;
        static int pre = 0;
    
     if (ptt_state == PTT_PRESS) {

 		if (s_emptyBlock < BUFFER_NUM ) {
            tick1 = xTaskGetTickCount();

   /*         if(s_emptyBlock == 0)
			{
				s_emptyBlock == BUFFER_NUM;
				s_pemptyBlock == BUFFER_NUM;
			} */

			i2sTxTransfer.data = s_buffer + s_writeIndex * BUFFER_SIZE;
			i2sTxTransfer.dataSize = BUFFER_SIZE;
            codec2_encode(C2Handle,&codec, i2sTxTransfer.data);
            SendOneFrame(codec);

            if (s_emptyBlock < BUFFER_NUM)
            	s_emptyBlock++;

        	if (++s_writeIndex >= BUFFER_NUM) {
        		s_writeIndex = 0U;
        	}

			tick2 = xTaskGetTickCount();

//			if((xTaskGetTickCount() - pre) < 20)
//			 vTaskDelay(20 - (xTaskGetTickCount() - pre));

			log_info("Send One capture Frame ##### time %d s_emptyBlock %d  tick1 = %d tick2 = %d  tick2 -tick1 = %d\r\n",xTaskGetTickCount() - pre,s_emptyBlock ,tick1 , tick2 ,tick2-tick1);

			pre = xTaskGetTickCount();
		}
      }
     vTaskDelay(3);
	}
}

void audio_main_playback(void *handle) {
	uint8_t buffer[BUFFER_SIZE];
	Decodec_t codec;
	uint8_t frame[FRAME_LEN];
	uint8_t index = 0;
	i2s_transfer_t i2sTxTransfer ={
           .data = buffer,
		   .dataSize = BUFFER_SIZE,
	};

	while (1) {

	   if (radio_state != RADIO_TX) {
		   int tick1,tick2,tick3 ,tick4;
		   tick1 = xTaskGetTickCount();
			if (pdFALSE == GetThreeFrame(&codec)){
				taskYIELD();
				continue;
			} else {

				do{
					tick2 = xTaskGetTickCount();
					memcpy(frame,&codec.ucValue[(index++)*FRAME_LEN],FRAME_LEN);
#if LOG_ENABLE
					uint8_t i;
					PRINTF("play back index %d :\n", index);
					for(i = 0 ; i < FRAME_LEN; i++ )
					{
						PRINTF("0x%x  ",frame[i]);
					}

					PRINTF("\n");
#endif

					codec2_decode(C2Handle, i2sTxTransfer.data, frame);
					if (I2S_TxTransferSendDMA(DEMO_I2S_TX, &s_i2sTxHandle, i2sTxTransfer)
							== kStatus_Success) {
			/*			if (++s_writeIndex >= BUFFER_NUM) {
							s_writeIndex = 0U;
						} */
						//PRINTF("play back receive data ,index %d ,frame[0] 0x%x frame[1] 0x%x frame[2] 0x%x frame[3] 0x%x  \n", index,frame[0],frame[1],frame[2],frame[3]);
					}
					tick3 = xTaskGetTickCount();
		//			vTaskDelay(20 - (tick3 - tick2));
				} while (index < PACK_NUM);
				index = 0;
			}
			tick4 = xTaskGetTickCount();
			//vTaskDelay(60 - (tick4 - tick1));
	   }
		log_info("%s:xxxxxxxxxxxxxxxxxxxxxxx%d:%d:%d\r\n", __func__);
	}
}

//ADD_TASK_WORK(NULL, audio_main_capture, "audio_main_capture", 2000, NULL, TASK_AUDIO_PRIORITY);
//ADD_TASK_WORK(NULL, audio_main_playback, "audio_main_playback", 5000, NULL, TASK_AUDIO_PRIORITY);
