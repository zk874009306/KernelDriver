#include "common.h"
#include "fsl_debug_console.h"
#include "sx1276.h"

/* The software timer period. */
#define TX_TIMER_PERIOD_MS (60 / portTICK_PERIOD_MS)
#define TEST_TIMER_PERIOD_MS (1000 / portTICK_PERIOD_MS)

#define SEND_PACKAGE_LEN        18
#define LOG_ENABLE 0

//uint8_t audio_buff[SEND_PACKAGE_LEN] = "123456789012345678";

TaskHandle_t sx1276_task_handler;
TaskHandle_t sx1276_tx_handler;
TaskHandle_t message_queue_handler;
//TimerHandle_t TxTimerHandle = NULL;

extern QueueHandle_t AudioMsgSendQueue;
extern QueueHandle_t AudioMsgRecvQueue;

Encodec_t encode_frame;
Decodec_t receive_frame;

ppt_state_s ptt_state = PTT_RELEASE;
radio_s radio_state = RADIO_IDLE;


BaseType_t GetOneEncodedFrame(Encodec_t *data){
    if (NULL == AudioMsgSendQueue){
		AudioMsgSendQueue = xQueueCreate(8, sizeof( Encodec_t ));
	}
	return xQueueReceive(AudioMsgSendQueue, data, portMAX_DELAY);
}

BaseType_t PutThreeEncodedFrame(Decodec_t *data){
	if (NULL == AudioMsgRecvQueue){
		AudioMsgRecvQueue = xQueueCreate(8, sizeof( Decodec_t ));
	}
	return xQueueSend(AudioMsgRecvQueue, data, portMAX_DELAY);
}

void radio_Sendmsg()
{
	uint8_t queue_num = 0 ,index = 0;
	uint8_t encode_audio[AUDIO_LEN];
	int pre ,count = 0;

    while(ptt_state == PTT_PRESS)
    {
    	if(AudioMsgSendQueue)
    	{
			queue_num = uxQueueSpacesAvailable(AudioMsgSendQueue);
			if(queue_num <= FREE_QUEUE_LEN){
				do{
					if (pdFALSE == GetOneEncodedFrame(&encode_frame)){
								taskYIELD();
								continue;
					} else {
						log_error("SX1276_GetOneEncodedFrame ##### index %d  ---#####queue num %d \r\n",index ,queue_num);
						memcpy(&encode_audio[(index++)*FRAME_LEN],encode_frame.ucValue,FRAME_LEN);
					}
				}while(index < PACK_NUM);
				index = 0;

				radio_state = RADIO_TX;
			//	count++;
			//	if (count > 5)
				SX1276_SendPacket(encode_audio, AUDIO_LEN);
				log_info("SX1276_SendPacket ##### time %d  ---#####queue num %d \r\n",xTaskGetTickCount() - pre ,queue_num);

				pre = xTaskGetTickCount();
			}
    	}
      //  taskYIELD();
    }
}

void radio_PttPress()
{
	PRINTF("radio_PttPress %d.\r\n", radio_state);
    //start get encoded data
    radio_state = RADIO_TX;
    PRINTF("start get one frame encode audio data\r\n");
    radio_Sendmsg();

}

void radio_PttRelease()
{
    PRINTF("radio_PttRelease %d.\r\n", radio_state);
    if(radio_state == RADIO_TX)
    radio_state = RADIO_IDLE;
}

void RF_Initial()
{
    SX1276_Init(1);            //SX127X module reset and initialize to LoRa mode
    SX1276_SetFreq(23);             //set channel to 23, 433MHz
    SX1276_LoRa_SetDataRate(2);
    SX1276_SetPower(15, 1);         //set RFO PIN output锟?20dBm
    SX1276_SetRxMode();             // receiver mode
    SX1276_WriteReg(0x24, 0);
    radio_state = RADIO_RX;
    PRINTF("sx1278 RF_Initial\n");
}

void RF_RecvHandler(void)
{
    uint8_t length = 0;
    length = SX1276_ReceivePacket(receive_frame.ucValue);

#if LOG_ENABLE
    uint8_t i;
	PRINTF("rcv package lenght  %d :\n", length);
	for(i = 0 ; i < AUDIO_LEN; i++ )
	{
		PRINTF("0x%x  ",receive_frame.ucValue[i]);
	}
	PRINTF("\n");
#endif

/*
    PRINTF("rcv package = %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x, len =%d\r\n", 
    audio_buff[0],audio_buff[1],audio_buff[2],audio_buff[3],audio_buff[4],audio_buff[5],audio_buff[6],
    audio_buff[7],audio_buff[8],audio_buff[9],audio_buff[10],audio_buff[11],audio_buff[12],audio_buff[13],
    audio_buff[14],audio_buff[15],audio_buff[16],audio_buff[17], length);
*/    
    SX1276_SetRxMode();
    radio_state = RADIO_RX;
	PRINTF(" len = %d ,count = %d\r\n", length,receive_frame.ucValue[18]);
}

void radio_rx_main(void *handle)
{
	int t1;
    RF_Initial();
    while (1) {
		vTaskSuspend(NULL);
	//	t1 = xTaskGetTickCount();
	//	PRINTF("radio_rx_main\n");
        RF_RecvHandler();
        /*
        if (pdFALSE == PutThreeEncodedFrame(&receive_frame)){
					taskYIELD();
					continue;
		}*/
	//	PRINTF("radio_rx_main t=%d\n", xTaskGetTickCount() - t1);
    }
}

void message_handle_main(void *handle)
{
	while (1) {
		PRINTF("task message_handle_main\n");
		vTaskSuspend(NULL);
		if (pdFALSE == PutThreeEncodedFrame(&receive_frame)){
					taskYIELD();
					continue;
		}
		PRINTF("task PutThreeEncodedFrame 1111\n");
	}

}

void radio_tx_main(void *handle)
{
    while (1) {
        vTaskSuspend(NULL);
        if (radio_state == RADIO_TX_DONE) {
            radio_state = RADIO_IDLE;
            SX1276_LoRa_SendDone();
        }

        if (ptt_state != PTT_PRESS || radio_state == RADIO_RX) {
            PRINTF("radio_tx_main SX1276_SetRxMode\r\n");
            radio_state = RADIO_RX;
            SX1276_SetRxMode();
        }
    }
}

ADD_TASK_WORK(&sx1276_task_handler, radio_rx_main, "radio_rx", 1024, NULL, TASK_AUDIO_PRIORITY);
ADD_TASK_WORK(&sx1276_tx_handler, radio_tx_main, "radio_tx", 1024, NULL, TASK_AUDIO_PRIORITY);
//ADD_TASK_WORK(&message_queue_handler, message_handle_main, "message_handle", 1024, NULL, TASK_AUDIO_PRIORITY);
