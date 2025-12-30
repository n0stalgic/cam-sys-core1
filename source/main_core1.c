/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    main_core1.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_sd.h"
#include "fsl_debug_console.h"
#include "core_printf.h"
#include "core_msgs.h"
#include "app.h"
#include "mcmgr.h"
#include "rpmsg_lite.h"
#include "rpmsg_queue.h"
#include "rpmsg_ns.h"
#include "rpmsg_addr_mem_cfg.h"
#include "icc.h"
#include "periph_mutex.h"
#include "ff.h"
#include "diskio.h"
#include "fsl_sd_disk.h"
#include "sdmmc_config.h"
#include "limits.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "version.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define FREQ_DIV 		1000000U

#define TASK_STACK_SIZE 256U
#define TASK_PRIO       2U

/*! @brief Task stack size. */
#define ACCESSFILE_TASK_STACK_SIZE (2048U)
/*! @brief Task stack priority. */
#define ACCESSFILE_TASK_PRIORITY (3U)

/*! @brief Task stack size. */
#define CARDDETECT_TASK_STACK_SIZE (1024U)  /* Increased for SD init stack usage */
/*! @brief Task stack priority. */
#define CARDDETECT_TASK_PRIORITY (configMAX_PRIORITIES - 1U)

#define IMAGE_METADATA_DIR  "/image_metadata"

#define TASK_GET_SEM_BLOCK_TICKS 1U
#define TASK_ACCESS_SDCARD_TIMES 2U

#define APP_RPMSG_READY_EVENT_DATA 	  (1U)
#define APP_BEGIN_CAPTURE_EVENT		  (2U)

#define RPMSG_LITE_LINK_ID            (RL_PLATFORM_IMXRT1170_M7_M4_LINK_ID)
#define RPMSG_LITE_NS_ANNOUNCE_STRING "rpmsg-camera-sys-chan"

#define ICC_TX_QUEUE_LEN            (32U)   /* Max pending TX requests */
#define IMAGE_READY_QUEUE_LEN       (128U)  /* Max pending image uploads */


extern u8 _pvHeapStart[];
extern u8 _pvHeapLimit[];
extern u32 _HeapSize;

// this is required for heap 5 allocation
HeapRegion_t xHeapRegions[2U];

/*******************************************************************************
 * Globals
 ******************************************************************************/

#if 0
static TaskHandle_t pwr_mgmt_handle;
#endif
static TaskHandle_t backgnd_handle;
static TaskHandle_t icc_hdlr_task_handle;
static TaskHandle_t file_access_task_handle;
static TaskHandle_t card_detect_task_handle;
static QueueHandle_t image_ready_queue;
static QueueHandle_t icc_tx_queue;
static QueueHandle_t sdcard_state_queue;
static SemaphoreHandle_t card_detect_sema4 = NULL;


static FATFS fileSystem; /* File system object */
static FIL file_obj;     /* File object */

static volatile u8 card_inserted = FALSE;
static volatile u8 card_inserted_status = FALSE;

/* Store startup data for RPMsg task */
static volatile u32 rpmsg_startup_data = 0;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!
 * @brief Background task to handle LED and other low-priority tasks
 *
 * @param param Task parameter
 */
void BackgndTask(void* param);

/*!
 * @brief RPMsg/ICC handler task
 *
 * @param param Task parameter
 */
void ICCHdlrTask(void* param);

/*!
 * @brief SD card access task 2.
 *
 * @param param Task parameter.
 */
static void FileAccessTask(void *param);

/*!
 * @brief SD card detect task.
 *
 * @param param Task parameter.
 */
static void CardDetectTask(void *param);

/*!
 * @brief call back function for SD card detect.
 *
 * @param isInserted  true,  indicate the card is insert.
 *                    false, indicate the card is remove.
 * @param userData
 */
static void SDCARD_DetectCallBack(bool isInserted, void *userData);

/*!
 * @brief make filesystem.
 */
static status_t MountFatFs(void);


/*******************************************************************************
 * Code
 ******************************************************************************/

void InitializeHeap(void) {
    // Calculate heap size at runtime
    size_t heap_size = (size_t)(_pvHeapLimit - _pvHeapStart);

    // Initialize heap regions
    xHeapRegions[0].pucStartAddress = (uint8_t*)_pvHeapStart;
    xHeapRegions[0].xSizeInBytes = heap_size;

    // NULL terminator
    xHeapRegions[1].pucStartAddress = NULL;
    xHeapRegions[1].xSizeInBytes = 0;

    // Configure Heap 5
    vPortDefineHeapRegions(xHeapRegions);

    CPRINTF("Heap configured: 0x%08lx, size: %lu bytes\r\n",
            (uint32_t)_pvHeapStart, (uint32_t)heap_size);
}

static void app_nameservice_isr_cb(u32 new_ept, const char *new_ept_name, u32 flags, void *user_data)
{

}

static void SDCARD_DetectCallBack(bool isInserted, void *userData)
{
    card_inserted_status = isInserted;
    xSemaphoreGiveFromISR(card_detect_sema4, NULL);
}

static void CardDetectTask(void *param)
{
    card_detect_sema4 = xSemaphoreCreateBinary();

    BOARD_SD_Config(&g_sd, SDCARD_DetectCallBack, BOARD_SDMMC_SD_HOST_IRQ_PRIORITY, NULL);

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    /* SD host init function */
    if (SD_HostInit(&g_sd) == kStatus_Success)
    {
        while (TRUE)
        {
            /* take card detect semaphore */
            if (xSemaphoreTake(card_detect_sema4, portMAX_DELAY) == pdTRUE)
            {
                if (card_inserted != card_inserted_status)
                {
                    card_inserted = card_inserted_status;

                    if (card_inserted)
                    {
                        CPRINTF("SD card detected.\r\n");
                        /* power off card */
                        SD_SetCardPower(&g_sd, false);
                        /* power on the card */
                        SD_SetCardPower(&g_sd, true);

                        CPRINTF("Mounting filesystem...\r\n");
                        if (MountFatFs() != kStatus_Success)
                        {
                        	CPRINTF("SD card format failure\r\n");
                            continue;
                        }

                        // the system is completely initialized, signal to core1 to begin capture

                        xTaskNotifyGive(file_access_task_handle);
                    	(void)MCMGR_TriggerEvent(kMCMGR_Core0, kMCMGR_RemoteApplicationEvent, APP_BEGIN_CAPTURE_EVENT);

                    }
                }

                if (!card_inserted)
                {
                    PRINTF("\r\nPlease insert a card into board.\r\n");
                }
            }

            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
    else
    {
        PRINTF("\r\nSD host init fail\r\n");
    }

    vTaskSuspend(NULL);
}


void BackgndTask(void* param)
{
	u8 xfer_in_prog = {};

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	while (TRUE)
	{
		LED_TOGGLE();
		vTaskDelay(pdMS_TO_TICKS(LED_FREQ_6HZ));


	}
}

void ICCHdlrTask(void* param)
{
	u8 err = FALSE;
	volatile TestMsg_t local_msg = {0};
	core_message_t core_msg = { 0 };
	rpmsg_core1_attrs_t rpmsg_attrs = {0};

	/* Small delay for stability */
	vTaskDelay(pdMS_TO_TICKS(100));

	/* Initialize RPMsg as remote */
	rpmsg_attrs.rpmsg_inst =
				rpmsg_lite_remote_init((void *)(char *)(platform_patova(rpmsg_startup_data)),
	                                     RPMSG_LITE_LINK_ID, RL_NO_FLAGS);
	GOTO_IF_NULL(rpmsg_attrs.rpmsg_inst, init_err, err);

	/* Signal the master core we are ready */
	CPRINTF("Triggering ready event to core0...\r\n");
	(void)MCMGR_TriggerEvent(kMCMGR_Core0, kMCMGR_RemoteApplicationEvent, APP_RPMSG_READY_EVENT_DATA);

	/* Wait for link up */
	rpmsg_lite_wait_for_link_up(rpmsg_attrs.rpmsg_inst, RL_BLOCK);

	/* Create queue and endpoint */
	rpmsg_attrs.rpmsg_queue = rpmsg_queue_create(rpmsg_attrs.rpmsg_inst);
	GOTO_IF_NULL(rpmsg_attrs.rpmsg_queue , init_err, err);

	rpmsg_attrs.rpmsg_ept =
					rpmsg_lite_create_ept(rpmsg_attrs.rpmsg_inst, CORE1_EPT_ADDR,
							rpmsg_queue_rx_cb, rpmsg_attrs.rpmsg_queue );

	GOTO_IF_NULL(rpmsg_attrs.rpmsg_ept, init_err, err);

	rpmsg_attrs.ns_handle =
				rpmsg_ns_bind(rpmsg_attrs.rpmsg_inst,
						app_nameservice_isr_cb, ((void *)0));

	GOTO_IF_NULL(rpmsg_attrs.ns_handle, init_err, err);

	/* Delay before nameservice announce to ensure master is ready */
	vTaskDelay(pdMS_TO_TICKS(1000));
	(void)rpmsg_ns_announce(rpmsg_attrs.rpmsg_inst,
			rpmsg_attrs.rpmsg_ept, RPMSG_LITE_NS_ANNOUNCE_STRING, (u32)RL_NS_CREATE);
	CPRINTF("Nameservice announce sent\r\n");

	/* RPMsg ping-pong communication loop to test */
	u32 msg_len = {};
	while (local_msg.DATA <= 10U)
	{
		(void)icc_sync_recv(&rpmsg_attrs, (void *)&local_msg, &msg_len, sizeof(TestMsg_t));

		local_msg.DATA++;
		(void)icc_sync_send(&rpmsg_attrs, (void *)&local_msg, sizeof(TestMsg_t));

	}

	// we can communicate with the other core,
	// drain the queue before we start protocol handling
	icc_drain_rx_queue(&rpmsg_attrs);

	CPRINTF("Inter-core link online\r\n");

	// card detection task allows file access task to run
    xTaskNotifyGive(card_detect_task_handle);
    xTaskNotifyGive(backgnd_handle);

	/* Heartbeat tracking */
	static TickType_t last_heartbeat = 0;
	const TickType_t heartbeat_interval = pdMS_TO_TICKS(1000U);  /* 1 second */

	while (TRUE)
	{
		/* Send periodic heartbeat to Core0 */
		TickType_t now = xTaskGetTickCount();
		if ((now - last_heartbeat) >= heartbeat_interval)
		{
			core_message_t hb_msg = {};
			hb_msg.type = MSG_HEARTBEAT;

			icc_tx_req_t hb_req = {};
			hb_req.timeout_ms = 0;
			hb_req.msg = hb_msg;

			if (xQueueSend(icc_tx_queue, &hb_req, 0) != pdTRUE)
			{
				/* Queue full - skip this heartbeat */
			}

			last_heartbeat = now;
		}

		// handle receptions from core0
		u32 msg_len = {};
		if(icc_async_recv(&rpmsg_attrs, &core_msg, &msg_len, sizeof(core_message_t)) == RL_SUCCESS)
		{
			if (msg_len > 0)
			{
				switch(core_msg.type)
				{
					case MSG_IMAGE_READY:
						CPRINTF("Image ready: buf %lu, prio %u\r\n",
								core_msg.payload.image.buffer_id,
								core_msg.payload.image.priority);

						// Queue for saving to sd card
						xQueueSend(image_ready_queue, &core_msg.payload.image, 0);
						break;

					default:
						CPRINTF("Unknown RPMsg type %u\r\n", core_msg.type);
						break;
				}
			}

		}

		icc_tx_req_t tx_req;
		if (xQueueReceive(icc_tx_queue, &tx_req, 0) == pdTRUE)
		{
			s32 rc;
			if (tx_req.timeout_ms == 0U || tx_req.timeout_ms == RL_BLOCK)
			{
				rc = icc_sync_send(&rpmsg_attrs, &tx_req.msg, sizeof(core_message_t));
			}
			else
			{
				rc = icc_async_send(&rpmsg_attrs, &tx_req.msg, sizeof(core_message_t));
			}

			if (rc != RL_SUCCESS)
			{
				CPRINTF("ICC transmit to core0 fail, err %ld\r\n", rc);
			}
		}
		vTaskDelay(pdMS_TO_TICKS(5U));
	}

	init_err:
		if (err)
		{
			if (rpmsg_attrs.rpmsg_ept)
			{
				(void)rpmsg_lite_destroy_ept(rpmsg_attrs.rpmsg_inst, rpmsg_attrs.rpmsg_ept);
				rpmsg_attrs.rpmsg_ept = NULL;
			}

			if (rpmsg_attrs.rpmsg_queue)
			{
				(void)rpmsg_queue_destroy(rpmsg_attrs.rpmsg_inst, rpmsg_attrs.rpmsg_queue);
				rpmsg_attrs.rpmsg_queue = NULL;
			}

			if (rpmsg_attrs.ns_handle)
			{
				(void)rpmsg_ns_unbind(rpmsg_attrs.rpmsg_inst, rpmsg_attrs.ns_handle);
			}

			if (rpmsg_attrs.rpmsg_inst)
			{
				(void)rpmsg_lite_deinit(rpmsg_attrs.rpmsg_inst);
				rpmsg_attrs.rpmsg_inst = NULL;
			}

			CPRINTF("RPMsg deinitialized, task ending\r\n");

		}

		vTaskDelete(NULL); // Task complete, delete itself
}

static status_t MountFatFs(void)
{
	FRESULT error;
    const TCHAR driverNumberBuffer[3U] = {SDDISK + '0', ':', '/'};

    error = f_mount(&fileSystem, driverNumberBuffer, 0U);
    if (error)
    {
        CPRINTF("Mount volume failed (err %d)\r\n", error);
        return kStatus_Fail;
    }

    error = f_chdrive((char const *)&driverNumberBuffer[0U]);
    if (error)
    {
        PRINTF("Change drive failed (err %d) \r\n", error);
        return kStatus_Fail;
    }

    /* Open the metadata file once - keep it open for the entire session */
    error = f_open(&file_obj, "/image_metadata/test_pattern_0.dat", FA_WRITE | FA_CREATE_ALWAYS);
    if (error)
    {
        CPRINTF("Open metadata file failed (err %d)\r\n", error);
        return kStatus_Fail;
    }

    CPRINTF("Metadata file opened successfully\r\n");
    return kStatus_Success;
}


static void FileAccessTask(void *param)
{
    UINT bytes_written   = 0U;
    FRESULT error;
    image_metadata_t metadata;

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    while (TRUE)
    {
		if ( card_inserted == TRUE && xQueueReceive(image_ready_queue, &metadata, portMAX_DELAY) == pdTRUE)
		{
			/* Monitor resources every 10 writes */
			static uint32_t write_count = 0;
			if (++write_count % 10 == 0) {
				UBaseType_t stack_remaining = uxTaskGetStackHighWaterMark(NULL);
				CPRINTF("Write #%lu, Heap: %u bytes, Stack: %u words\r\n",
				        write_count, xPortGetFreeHeapSize(), stack_remaining);
			}

			/* Close and reopen file every 50 writes to prevent FatFS buffer buildup */
			if (write_count % 50 == 0) {
				f_close(&file_obj);
				error = f_open(&file_obj, "/image_metadata/metadata.dat", FA_WRITE | FA_OPEN_APPEND);
				if (error) {
					CPRINTF("Reopen failed (err %d)\r\n", error);
					continue;
				}
				CPRINTF("File reopened at write #%lu\r\n", write_count);
			}

			/* Write metadata to the open file */
			error = f_write(&file_obj, &metadata, sizeof(image_metadata_t), &bytes_written);
			if ((error) || (bytes_written != sizeof(image_metadata_t)))
			{
				CPRINTF("Write failed (err %d, wrote %u bytes)\r\n", error, bytes_written);
				continue;
			}

			/* Sync to ensure data is written to SD card */
			error = f_sync(&file_obj);
			if (error)
			{
				CPRINTF("Sync failed (err %d)\r\n", error);
				continue;
			}

			CPRINTF("Write complete (total size: %lu bytes)\r\n", f_size(&file_obj));

			/* Notify core0 that buffer is now free */
			core_message_t core_msg;
			core_msg.type = MSG_BUFFER_FREE;
			core_msg.payload.buffer_id = metadata.buffer_id;

			/* Flow control: Wait for queue space instead of failing */
			while(xQueueSend(icc_tx_queue, &core_msg, pdMS_TO_TICKS(100)) != pdTRUE)
			{
				CPRINTF("ICC TX queue full, waiting for Core0 to drain...\r\n");
			}
		}
    }

    vTaskSuspend(NULL);
}

void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                    char *pcTaskName )
{
	CPRINTF("Stack overflow detected!!! Task %s\r\n", pcTaskName);

}


/*
 * @brief   Application entry point.
 */
int main(void)
{
	char freqBuf[40];
	u32 startupData;
	mcmgr_status_t status;
	mcmgr_core_t core;\

	/* Small delay to let core0 finish its startup messages */
	SDK_DelayAtLeastUs(100000U, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY); // 100ms

	/* Init board hardware */
	BOARD_InitHardware();

	InitializeHeap();

	/* Initialize MCMGR */
	(void)MCMGR_Init();

	/* Get the startup data from CM7 */
	do
	{
		core = MCMGR_GetCurrentCore();
		status = MCMGR_GetStartupData(core, &startupData);
	} while (status != kStatus_MCMGR_Success);

	/* Initialize peripheral mutex for inter-core synchronization */
	periph_mutex_init();

	/* Store startup data for RPMsg task */
	rpmsg_startup_data = startupData;

	/* Configure LED */
	LED_INIT();

	/* Print startup banner */
	CPRINTF("Build: %s %s %s\r\n", __MAIN_CORE_1_FW_VER__, __DATE__, __TIME__);
	print_banner("Status", "[ONLINE]");
	sprintf(freqBuf, "%lu MHz", CLOCK_GetCoreSysClkFreq() / FREQ_DIV);
	print_banner("Core Clk", freqBuf);

	/* Create queues */
	image_ready_queue = xQueueCreate(IMAGE_READY_QUEUE_LEN, sizeof(image_metadata_t));
	CHECK_NULL(image_ready_queue);

	icc_tx_queue = xQueueCreate(ICC_TX_QUEUE_LEN, sizeof(icc_tx_req_t));
	CHECK_NULL(icc_tx_queue);

	CHECK_RTOS(xTaskCreate(ICCHdlrTask,   "Core1_ICCHdlrTask"  , TASK_STACK_SIZE * 4, NULL, 5U              , &icc_hdlr_task_handle));
	CHECK_RTOS(xTaskCreate(BackgndTask,   "Core1_BackgndTask"  , TASK_STACK_SIZE,     NULL, tskIDLE_PRIORITY, &backgnd_handle));
	CHECK_RTOS(xTaskCreate(CardDetectTask,"Core1_CardDetectTask", CARDDETECT_TASK_STACK_SIZE, NULL, 4U, &card_detect_task_handle));
	CHECK_RTOS(xTaskCreate(FileAccessTask,"Core1_FileAccessTask", ACCESSFILE_TASK_STACK_SIZE, NULL, ACCESSFILE_TASK_PRIORITY, &file_access_task_handle));

#if 0
	card_detect_task_handle = xTaskCreateStatic(CardDetectTask,"Core1_CardDetectTask",
			CARDDETECT_TASK_STACK_SIZE,    NULL, 4U,  card_detect_stack, &card_detect_tcb);

	CHECK_NULL(card_detect_task_handle);

	file_access_task_handle = xTaskCreateStatic(FileAccessTask, "Core1_FileAccessTask",
			ACCESSFILE_TASK_STACK_SIZE, NULL, ACCESSFILE_TASK_PRIORITY, file_access_stack , &file_access_tcb);

	CHECK_NULL(file_access_task_handle);
#endif
	vTaskStartScheduler();

	/* Should never reach here */
	CPRINTF("ERROR: Scheduler returned!\r\n");
	while(TRUE)
	{
		__asm volatile("nop");
	}
}
