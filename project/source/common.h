/*
 * common.h
 *
 *  Created on: Oct 18, 2019
 *      Author: binnary
 */

#ifndef COMMON_H_
#define COMMON_H_

#include "FreeRTOS.h"
#include "semphr.h"
#include "portmacro.h"
#include "event_groups.h"
#include "task.h"
#include "fsl_i2s.h"
#include "codec2.h"
#include "codec2_internal.h"
#include "logging.h"

#ifdef __cplusplus
extern "C" {
#endif

/*define tasks priority*/
#define TASK_AUDIO_PRIORITY (configMAX_PRIORITIES-1)
#define TASK_GUI_PRIORITY   (configMAX_PRIORITIES-1)
#define MODULE_DESC_LEN   (16)

#define FRAME_LEN 6
#define FREE_QUEUE_LEN 5
#define AUDIO_LEN 18
#define PACK_NUM 3

typedef struct _Encodec_t{
	uint8_t ucValue[6];
}Encodec_t;

typedef struct _Decodec_t{
	uint8_t ucValue[18+1];
}Decodec_t;

typedef struct _TaskWork {
	TaskHandle_t *handle;
	TaskFunction_t pxTaskCode;
	const char pcName[configMAX_TASK_NAME_LEN]; /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
	const configSTACK_DEPTH_TYPE usStackDepth;
	void * const pvParameters;
	UBaseType_t uxPriority;
} TaskWork;

#define WORK_SEC  __attribute__ ((unused,section ("task_work_sec")))

#define ADD_TASK_WORK(handle, func, name, stack, params, proi) \
		TaskWork __work_##func  WORK_SEC = {handle, func, name, stack, params, proi}

#ifdef __cplusplus
}
#endif

extern int USBAcmRecv(uint32_t Instance, uint8_t *buffer, uint32_t size);
extern int USBAcmSend(uint32_t Instance, uint8_t *buffer, uint32_t size);
#endif /* COMMON_H_ */
