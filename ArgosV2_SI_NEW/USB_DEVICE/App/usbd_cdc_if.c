/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : usbd_cdc_if.c
 * @version        : v1.0_Cube
 * @brief          : Usb device for Virtual Com Port.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "main.h"
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define BACKUP_FLASH_ADDR 0x080E0000

#define TARGET_PULSE_NUMBER		1

#define DIVISOR					1

#define ENCODER_PULSE_COUNT		2000

#define TARGET_DISTANCE			1.0f

extern void can_transmit(int data);

// Auto Trigger Flag
extern uint8_t run_f;

// DMI variable, Encoder variable
extern int a_pls_cnt, b_pls_cnt;
extern int input_enc_val;

// USB Flag, Buffer
extern uint8_t usb_sel_f, enterkey_f; // USB Select Flag, Enter key input
extern uint8_t usb_tx_buf[300]; // USB Tx, ADC print buf
uint8_t usb_rx_buf[200]; // USB Rx
extern int usbselect;

// ADC variable
extern uint8_t setcdsvalue;
extern int adc_value[4]; // ADC DATA SAVE

// Strobe Count, Check
extern uint16_t stb_cnt0, stb_cnt1, stb_cnt2, stb_cnt3, stb_cnt_all; // STROBE Count
extern uint8_t stb_chk0, stb_chk1, stb_chk2, stb_chk3; // STROBE CHECK FLAG

// Camera
extern uint8_t spi_cam_select;
extern uint8_t spi_cam_value;

// buffer pointer
char *bufptr;
// User Interface Manual Print
char ui_cmd_manual[] = "M . Manual\r\n"
		"P . Print Status\r\n" // >> Argos I version : V
		"2 . USB 2.0 >> Default : 2.0 \r\n"
		"3 . USB 3.0 \r\n\r\n"

		"I . Input Encoder Pulse Value for Taking a Picture\r\n"
		"H . Ha... Save data\r\n"
		"F . Force Triggering\r\n"
		"A . Auto Triggering Start\r\n"
		"T . Auto Triggering Stop\r\n\r\n";

// Serial Communication Command Manual Print
char serial_cmd_manual[] = "#M . Manual\r\n"
		"#P . Print Status\r\n" // >> Argos I version : V
		"#2 . USB 2.0 >> Default : 2.0 \r\n"
		"#3 . USB 3.0 \r\n\r\n"

		"#I . Input Encoder Pulse Value for Taking a Picture\r\n"
		"#H . Ha... Save data\r\n"
		"#F . Force Triggering\r\n"
		"#A . Auto Triggering Start\r\n"
		"#T . Auto Triggering Stop\r\n\r\n";

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
 * @brief Usb device library.
 * @{
 */

/** @addtogroup USBD_CDC_IF
 * @{
 */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
 * @brief Private types.
 * @{
 */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
 * @}
 */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
 * @brief Private defines.
 * @{
 */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* USER CODE END PRIVATE_DEFINES */

/**
 * @}
 */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
 * @brief Private macros.
 * @{
 */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
 * @}
 */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
 * @brief Private variables.
 * @{
 */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
 * @}
 */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
 * @brief Public variables.
 * @{
 */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
 * @}
 */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
 * @brief Private functions declaration.
 * @{
 */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t *pbuf, uint32_t *Len);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */
void ui_cmd_func(uint8_t *Buf, uint32_t *Len)
{
	wheel_param_t wp;
	memcpy(&wp, (void*) (BACKUP_FLASH_ADDR), sizeof(wheel_param_t));

	// Manual
	if (strncmp((const char*) Buf, "m", 1) == 0
			|| strncmp((const char*) Buf, "M", 1) == 0)
	{
		CDC_Transmit_FS((uint8_t*) ui_cmd_manual, strlen(ui_cmd_manual));
		bufptr = (char*) usb_rx_buf;
	}

	// Print Status
	else if (strncmp((const char*) Buf, "p", 1) == 0
			|| strncmp((const char*) Buf, "P", 1) == 0)
	{
		sprintf((char*) usb_tx_buf,
				"Input : Encoder value for photo distance = %d\r\n"
						"Memory : Encoder value for photo distance = %d\r\n"
						"USB Select= %d.0 (default : 2.0)\r\n"
						"Auto triggering= %s\r\n", input_enc_val,
				wp.enc_val_for_photo_dist, usbselect,
				(run_f == true) ? ("Started\r\n") : ("Disabled\r\n"));
		CDC_Transmit_FS((uint8_t*) usb_tx_buf,
				strlen((const char*) usb_tx_buf));
	}

	// Input Data
	if (strncmp((const char*) Buf, "i", 1) == 0
			|| strncmp((const char*) Buf, "I", 1) == 0)
	{
		usb_sel_f = 1;
	}

	// Save
	else if (strncmp((const char*) Buf, "h", 1) == 0
			|| strncmp((const char*) Buf, "H", 1) == 0)
	{
		/*
		 wP.encoderTargetCount = input_enc_val;
		 HAL_StatusTypeDef FlashStatus = HAL_OK;
		 HAL_FLASH_Unlock();
		 {


		 FLASH_EraseInitTypeDef fler;
		 uint32_t perr;
		 fler.TypeErase = FLASH_TYPEERASE_SECTORS;
		 fler.Banks = FLASH_BANK_1;
		 fler.Sector = FLASH_SECTOR_11;
		 fler.NbSectors = 1;

		 HAL_FLASHEx_Erase(&fler, &perr);
		 register uint32_t *_targetAddr = (uint32_t*) (&wP);
		 for (uint8_t i = 0; i <= (sizeof(WheelParam) * 2); i +=
		 sizeof(uint32_t))
		 {
		 FlashStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
		 BACKUP_FLASH_ADDR + i, _targetAddr[i / sizeof(uint32_t)]);


		 //while (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
		 //BACKUP_FLASH_ADDR + i, _targetAddr[i / sizeof(uint32_t)]) != HAL_OK)
		 //;

		 }
		 }
		 HAL_FLASH_Lock();
		 */
		wp.enc_val_for_photo_dist = input_enc_val;
		uint32_t SectorError = 0;
		HAL_FLASH_Unlock();
		FLASH_EraseInitTypeDef EraseInitStruct;
		EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
		EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
		EraseInitStruct.Banks = FLASH_BANK_1;
		EraseInitStruct.Sector = FLASH_SECTOR_11;
		EraseInitStruct.NbSectors = 1;
		if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
		{
			int errorcode = HAL_FLASH_GetError();
			sprintf((char*) usb_tx_buf, "Error Code : %d\r\n", errorcode);
			CDC_Transmit_FS((uint8_t*) usb_tx_buf,
					strlen((const char*) usb_tx_buf));
			return HAL_ERROR;
		}

		/* Clear cache for flash */
		__HAL_FLASH_DATA_CACHE_DISABLE();
		__HAL_FLASH_INSTRUCTION_CACHE_DISABLE();

		__HAL_FLASH_DATA_CACHE_RESET();
		__HAL_FLASH_INSTRUCTION_CACHE_RESET();

		__HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
		__HAL_FLASH_DATA_CACHE_ENABLE();

		HAL_StatusTypeDef FlashStatus = HAL_OK;
		register uint32_t *_targetAddr = (uint32_t*) (&wp);
		/*
		 uint32_t Address = FLASH_USER_START_ADDR;
		 while (Address < FLASH_USER_END_ADDR)
		 {
		 if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, _targetAddr)
		 == HAL_OK)
		 {
		 Address += sizeof(uint32_t);
		 _targetAddr += sizeof(uint32_t);
		 }
		 else
		 {
		 uint32_t errorcode = HAL_FLASH_GetError();
		 return HAL_ERROR;
		 }
		 }
		 */
		for (uint8_t i = 0; i <= (sizeof(wheel_param_t) * 2); i +=
				sizeof(uint32_t))
		{
			FlashStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
			BACKUP_FLASH_ADDR + i, _targetAddr[i / sizeof(uint32_t)]);

			//while (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
			//FLASH_USER_START_ADDR + i, _targetAddr[i / sizeof(uint32_t)])
			//	!= HAL_OK)
			//;
		}
		HAL_FLASH_Lock();
		sprintf((char*) usb_tx_buf, "Save %s",
				(FlashStatus == HAL_OK) ? ("Complete\r\n") : ("Fail\r\n"));
		CDC_Transmit_FS((uint8_t*) usb_tx_buf,
				strlen((const char*) usb_tx_buf));
	}

	// USB 2.0
	else if (strncmp((const char*) Buf, "2", 1) == 0)
	{
		CDC_Transmit_FS((uint8_t*) "USB Select 2.0\r\n", 16);
		usbselect = 2;
		can_transmit(usbselect);
	}
	// USB 3.0
	else if (strncmp((const char*) Buf, "3", 1) == 0)
	{
		CDC_Transmit_FS((uint8_t*) "USB Select 3.0\r\n", 16);
		usbselect = 3;
		can_transmit(usbselect);
	}

	// Force Trigger
	else if (strncmp((const char*) Buf, "f", 1) == 0
			|| strncmp((const char*) Buf, "F", 1) == 0)
	{
		HAL_GPIO_WritePin(SIG_AUTOFOCUS_GPIO_Port, SIG_AUTOFOCUS_Pin,
				GPIO_PIN_SET);
		HAL_GPIO_WritePin(SIG_SHUTTER_GPIO_Port, SIG_SHUTTER_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(STROBE_DRV_GPIO_Port, STROBE_DRV_Pin, GPIO_PIN_RESET);
		//LL_TIM_ClearFlag_UPDATE(TIM3);
		//LL_TIM_EnableCounter(TIM3);
	}

	// Auto Trigger
	else if (strncmp((const char*) Buf, "a", 1) == 0
			|| strncmp((const char*) Buf, "A", 1) == 0)
	{
		// CW
		TIM8->CNT = 0;
		a_pls_cnt = 0;
		b_pls_cnt = 0;

		// CCW
		/*
		 TIM8->CNT = 65535;
		 A_PLS_CNT = 65535;
		 B_PLS_CNT = 65535;
		 */
		run_f = 1;
		CDC_Transmit_FS((uint8_t*) "AutoTrigger Started!\r\n", 22); // ACK
	}
	// Stop
	else if (strncmp((const char*) Buf, "t", 1) == 0
			|| strncmp((const char*) Buf, "T", 1) == 0)
	{
		run_f = 0;
		CDC_Transmit_FS((uint8_t*) "Stopped!\r\n", 10); // ACK
	}
	else
	{
		CDC_Transmit_FS((uint8_t*) ui_cmd_manual, strlen(ui_cmd_manual));
	}

}

void serial_cmd_func(uint8_t *Buf, uint32_t *Len)
{
	wheel_param_t wp;
	memcpy(&wp, (void*) (BACKUP_FLASH_ADDR), sizeof(wheel_param_t));

	// Manual
	if (strncmp((const char*) Buf, "#m", 2) == 0
			|| strncmp((const char*) Buf, "#M", 2) == 0)
	{
		CDC_Transmit_FS((uint8_t*) serial_cmd_manual,
				strlen(serial_cmd_manual));
		bufptr = (char*) usb_rx_buf;
	}

	// Print Status
	else if (strncmp((const char*) Buf, "#p", 2) == 0
			|| strncmp((const char*) Buf, "#P", 2) == 0)
	{
		sprintf((char*) usb_tx_buf,
				"#Input : Encoder value for photo distance = %d\r\n"
						"#Memory : Encoder value for photo distance = %d\r\n"
						"#USB Select= %d.0 (default : 2.0)\r\n"
						"#Auto triggering= %s\r\n", input_enc_val,
				wp.enc_val_for_photo_dist, usbselect,
				(run_f == true) ? ("#Started\r\n") : ("#Disabled\r\n"));
		CDC_Transmit_FS((uint8_t*) usb_tx_buf,
				strlen((const char*) usb_tx_buf));
	}

	// Input Data
	if (strncmp((const char*) Buf, "#i", 2) == 0)
	{
		sscanf((const char*) Buf, "#i%d\r\n", &input_enc_val);
	}
	else if (strncmp((const char*) Buf, "#I", 2) == 0)
	{
		sscanf((const char*) Buf, "#I%d\r\n", &input_enc_val);
	}

	// Save
	else if (strncmp((const char*) Buf, "#h", 2) == 0)
	{
		/*
		 wP.encoderTargetCount = input_enc_val;
		 HAL_StatusTypeDef FlashStatus = HAL_OK;
		 HAL_FLASH_Unlock();
		 {


		 FLASH_EraseInitTypeDef fler;
		 uint32_t perr;
		 fler.TypeErase = FLASH_TYPEERASE_SECTORS;
		 fler.Banks = FLASH_BANK_1;
		 fler.Sector = FLASH_SECTOR_11;
		 fler.NbSectors = 1;

		 HAL_FLASHEx_Erase(&fler, &perr);
		 register uint32_t *_targetAddr = (uint32_t*) (&wP);
		 for (uint8_t i = 0; i <= (sizeof(WheelParam) * 2); i +=
		 sizeof(uint32_t))
		 {
		 FlashStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
		 BACKUP_FLASH_ADDR + i, _targetAddr[i / sizeof(uint32_t)]);


		 //while (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
		 //BACKUP_FLASH_ADDR + i, _targetAddr[i / sizeof(uint32_t)]) != HAL_OK)
		 //;

		 }
		 }
		 HAL_FLASH_Lock();
		 */
		wp.enc_val_for_photo_dist = input_enc_val;
		uint32_t SectorError = 0;
		HAL_FLASH_Unlock();
		FLASH_EraseInitTypeDef EraseInitStruct;
		EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
		EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
		EraseInitStruct.Banks = FLASH_BANK_1;
		EraseInitStruct.Sector = FLASH_SECTOR_11;
		EraseInitStruct.NbSectors = 1;
		if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
		{
			int errorcode = HAL_FLASH_GetError();
			sprintf((char*) usb_tx_buf, "Error Code : %d\r\n", errorcode);
			CDC_Transmit_FS((uint8_t*) usb_tx_buf,
					strlen((const char*) usb_tx_buf));
			return HAL_ERROR;
		}

		/* Clear cache for flash */
		__HAL_FLASH_DATA_CACHE_DISABLE();
		__HAL_FLASH_INSTRUCTION_CACHE_DISABLE();

		__HAL_FLASH_DATA_CACHE_RESET();
		__HAL_FLASH_INSTRUCTION_CACHE_RESET();

		__HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
		__HAL_FLASH_DATA_CACHE_ENABLE();

		HAL_StatusTypeDef FlashStatus = HAL_OK;
		register uint32_t *_targetAddr = (uint32_t*) (&wp);
		/*
		 uint32_t Address = FLASH_USER_START_ADDR;
		 while (Address < FLASH_USER_END_ADDR)
		 {
		 if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, _targetAddr)
		 == HAL_OK)
		 {
		 Address += sizeof(uint32_t);
		 _targetAddr += sizeof(uint32_t);
		 }
		 else
		 {
		 uint32_t errorcode = HAL_FLASH_GetError();
		 return HAL_ERROR;
		 }
		 }
		 */
		for (uint8_t i = 0; i <= (sizeof(wheel_param_t) * 2); i +=
				sizeof(uint32_t))
		{
			FlashStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
			BACKUP_FLASH_ADDR + i, _targetAddr[i / sizeof(uint32_t)]);

			//while (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
			//FLASH_USER_START_ADDR + i, _targetAddr[i / sizeof(uint32_t)])
			//	!= HAL_OK)
			//;
		}
		HAL_FLASH_Lock();
		sprintf((char*) usb_tx_buf, "#Save %s",
				(FlashStatus == HAL_OK) ? ("Complete\r\n") : ("Fail\r\n"));
		CDC_Transmit_FS((uint8_t*) usb_tx_buf,
				strlen((const char*) usb_tx_buf));
	}

	// USB 2.0
	else if (strncmp((const char*) Buf, "#2", 2) == 0)
	{
		CDC_Transmit_FS((uint8_t*) "#USB Select 2.0\r\n", 17);
		usbselect = 2;
		can_transmit(usbselect);
	}
	// USB 3.0
	else if (strncmp((const char*) Buf, "#3", 2) == 0)
	{
		CDC_Transmit_FS((uint8_t*) "#USB Select 3.0\r\n", 17);
		usbselect = 3;
		can_transmit(usbselect);
	}

	// Force Trigger
	else if (strncmp((const char*) Buf, "#f", 2) == 0
			|| strncmp((const char*) Buf, "#F", 2) == 0)
	{
		HAL_GPIO_WritePin(SIG_AUTOFOCUS_GPIO_Port, SIG_AUTOFOCUS_Pin,
				GPIO_PIN_SET);
		HAL_GPIO_WritePin(SIG_SHUTTER_GPIO_Port, SIG_SHUTTER_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(STROBE_DRV_GPIO_Port, STROBE_DRV_Pin, GPIO_PIN_RESET);
		//LL_TIM_ClearFlag_UPDATE(TIM3);
		//LL_TIM_EnableCounter(TIM3);
	}

	// Auto Trigger
	else if (strncmp((const char*) Buf, "#a", 2) == 0
			|| strncmp((const char*) Buf, "#A", 2) == 0)
	{
		// CW
		TIM8->CNT = 0;
		a_pls_cnt = 0;
		b_pls_cnt = 0;

		// CCW
		/*
		 TIM8->CNT = 65535;
		 A_PLS_CNT = 65535;
		 B_PLS_CNT = 65535;
		 */
		run_f = 1;
		CDC_Transmit_FS((uint8_t*) "#AutoTrigger Started!\r\n", 23); // ACK
	}
	// Stop
	else if (strncmp((const char*) Buf, "#t", 2) == 0
			|| strncmp((const char*) Buf, "#T", 2) == 0)
	{
		run_f = 0;
		CDC_Transmit_FS((uint8_t*) "#Stopped!\r\n", 11); // ACK
	}
	else
	{
		CDC_Transmit_FS((uint8_t*) serial_cmd_manual,
				strlen(serial_cmd_manual));
	}

}

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
 * @}
 */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{ CDC_Init_FS, CDC_DeInit_FS, CDC_Control_FS, CDC_Receive_FS };

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Initializes the CDC media low layer over the FS USB IP
 * @retval USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_Init_FS(void)
{
	/* USER CODE BEGIN 3 */
	/* Set Application Buffers */
	USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
	USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
	return (USBD_OK);
	/* USER CODE END 3 */
}

/**
 * @brief  DeInitializes the CDC media low layer
 * @retval USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_DeInit_FS(void)
{
	/* USER CODE BEGIN 4 */
	return (USBD_OK);
	/* USER CODE END 4 */
}

/**
 * @brief  Manage the CDC class requests
 * @param  cmd: Command code
 * @param  pbuf: Buffer containing command data (request parameters)
 * @param  length: Number of data to be sent (in bytes)
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
	/* USER CODE BEGIN 5 */
	switch (cmd)
	{
	case CDC_SEND_ENCAPSULATED_COMMAND:

		break;

	case CDC_GET_ENCAPSULATED_RESPONSE:

		break;

	case CDC_SET_COMM_FEATURE:

		break;

	case CDC_GET_COMM_FEATURE:

		break;

	case CDC_CLEAR_COMM_FEATURE:

		break;

		/*******************************************************************************/
		/* Line Coding Structure                                                       */
		/*-----------------------------------------------------------------------------*/
		/* Offset | Field       | Size | Value  | Description                          */
		/* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
		/* 4      | bCharFormat |   1  | Number | Stop bits                            */
		/*                                        0 - 1 Stop bit                       */
		/*                                        1 - 1.5 Stop bits                    */
		/*                                        2 - 2 Stop bits                      */
		/* 5      | bParityType |  1   | Number | Parity                               */
		/*                                        0 - None                             */
		/*                                        1 - Odd                              */
		/*                                        2 - Even                             */
		/*                                        3 - Mark                             */
		/*                                        4 - Space                            */
		/* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
		/*******************************************************************************/
		static uint8_t line_coding[7] // 115200bps, 1stop, no parity, 8bit
		=
		{ 0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08 }; // 0001c200 >> 115200 arr[3],arr[2],arr[1],arr[0]
	case CDC_SET_LINE_CODING:
		memcpy(line_coding, pbuf, sizeof(line_coding));
		break;

	case CDC_GET_LINE_CODING:
		memcpy(pbuf, line_coding, sizeof(line_coding));
		break;

	case CDC_SET_CONTROL_LINE_STATE:

		break;

	case CDC_SEND_BREAK:

		break;

	default:
		break;
	}

	return (USBD_OK);
	/* USER CODE END 5 */
}

/**
 * @brief  Data received over USB OUT endpoint are sent over CDC interface
 *         through this function.
 *
 *         @note
 *         This function will issue a NAK packet on any OUT packet received on
 *         USB endpoint until exiting this function. If you exit this function
 *         before transfer is complete on CDC interface (ie. using DMA controller)
 *         it will result in receiving more data while previous ones are still
 *         not sent.
 *
 * @param  Buf: Buffer of data to be received
 * @param  Len: Number of data received (in bytes)
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_Receive_FS(uint8_t *Buf, uint32_t *Len)
{
	/* USER CODE BEGIN 6 */
	USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
	USBD_CDC_ReceivePacket(&hUsbDeviceFS);

	if (strncmp((const char*) Buf, "#", 1) == 0)
	{
		serial_cmd_func(Buf, Len);
	}
	else if (strncmp((const char*) Buf, "#", 1) != 0 && usb_sel_f == 0)
	{
		ui_cmd_func(Buf, Len);

	}

	else
	{
		for (uint16_t i = 0; i < *Len; i++)
		{
			*bufptr = Buf[i];
			CDC_Transmit_FS((uint8_t*) bufptr, 1);
			bufptr++;
			if (Buf[i] == '\r' || Buf[i] == '\n')
			{
				enterkey_f = 1;
				bufptr = (char*) usb_rx_buf;
			}
			else if (Buf[i] == '\b')
			{
				if (bufptr != (char*) usb_rx_buf)
				{
					bufptr--;
				}
			}
		}
	}
	return (USBD_OK);
	/* USER CODE END 6 */
}

/**
 * @brief  CDC_Transmit_FS
 *         Data to send over USB IN endpoint are sent over CDC interface
 *         through this function.
 *         @note
 *
 *
 * @param  Buf: Buffer of data to be sent
 * @param  Len: Number of data to be sent (in bytes)
 * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
 */
uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len)
{
	uint8_t result = USBD_OK;
	/* USER CODE BEGIN 7 */
	USBD_CDC_HandleTypeDef *hcdc =
			(USBD_CDC_HandleTypeDef*) hUsbDeviceFS.pClassData;
	if (hcdc->TxState != 0)
	{
		return USBD_BUSY;
	}
	USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
	result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
	/* USER CODE END 7 */
	return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
