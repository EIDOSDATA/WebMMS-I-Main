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
extern int16_t stbtim0, stbtim1, stbtim2, stbtim3, stbtimall; // Strobe Timer
extern char buf[130]; // ADC print buf
extern uint16_t stb0, stb1, stb2, stb3, stb_all; // STROBE CHECK FLAG
extern uint8_t UFlag, EnterFlag; // USB Select Flag Enter key input
extern bool bFlag;
extern uint8_t setcdsvalue;
extern char UTxbuf[300];
char URxbuf[200];
extern int A_PLS_CNT, B_PLS_CNT;
char *bufptr;

// WebMMS S Version
char Manual[] = "M . Manual\r\n"
		"P or V . Print Status\r\n" // >> Argos I version : V
		"W or I . Set Wheel Radius\r\n"// >> Argos I version : I
		//"L . Set Encoder Pulse Count\r\n"
		//"D . Set Trigger Distance\r\n"
		"C . Camera Configuration Select\r\n"
		"X . Camera Configuration Value\r\n"
		"S . Save data\r\n"
		"2 . USB 2.0 >> Default : 2.0 \r\n"
		"3 . USB 3.0 \r\n"
		"F . Force Triggering\r\n"// >> Argos I version : F
		"R or A . Auto Triggering Start\r\n"// >> Argos I version : A
		"T or S . Auto Triggering Stop\r\n\r\n";// >> Argos I version : S

/*
 // WebMMS I Version
 char Manual[] = "M . Manual\r\n"
 "V . View status\r\n"
 "I . Input Value status\r\n"
 "A . Auto triggering start\r\n"
 "S . Auto triggering stop\r\n";
 "F . Force triggering\r\n"
 "Q . SAVE\r\n"
 "C . Camera Configuration Select\r\n"
 "X . Camera Configuration Value\r\n"
 "U . USB Control >> Default : 2.0 \r\n\r\n";
 */

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
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS
};

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
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
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
	case CDC_SET_LINE_CODING:

		break;

	case CDC_GET_LINE_CODING:

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
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
	USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
	USBD_CDC_ReceivePacket(&hUsbDeviceFS);
	// WebMMS S Version
	if (UFlag == 0)
	{
		// UI INPUT DATA
		if (Buf[0] == '#')
		{
			if (strncmp(Buf, "#fuck", 4) == 0) // TEST code
			{
				HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
			}
			if (strncmp(Buf, "#shoot", 6) == 0) // USB 2.0
			{
				HAL_GPIO_WritePin(SIG_AUTOFOCUS_GPIO_Port, SIG_AUTOFOCUS_Pin,
						SET);
				HAL_GPIO_WritePin(SIG_SHUTTER_GPIO_Port, SIG_SHUTTER_Pin, SET);
				HAL_GPIO_WritePin(STROBE_DRV_GPIO_Port, STROBE_DRV_Pin, RESET);
				LL_TIM_ClearFlag_UPDATE(TIM3);
				LL_TIM_EnableCounter(TIM3);
			}
			if (strncmp(Buf, "#USB2", 5) == 0) // USB 2.0
			{
				CDC_Transmit_FS("USB Select 2.0\r\n\r\n", 16);
				UFlag = 8;
			}
			if (strncmp(Buf, "#USB3", 5) == 0) // USB 3.0
			{
				CDC_Transmit_FS("USB Select 3.0\r\n\r\n", 16);
				UFlag = 9;
			}
			if (strncmp(Buf, "#dparam", 7) == 0) // Set Radius Parameter
			{
				CDC_Transmit_FS("Input wheel radius.. > ", 23);
				UFlag = 2;
			}
			if (strncmp(Buf, "#dsave", 6) == 0) // Save Parameter
			{
				CDC_Transmit_FS("Saving current param...\r\n", 25);
				UFlag = 7;
			}
			if (strncmp(Buf, "#setcds", 7) == 0) // Save Parameter
			{
				CDC_Transmit_FS("Set Number of CDS Sensor...\r\n", 29);
				char data = Buf[7];
				setcdsvalue = atoi(&data);
			}
			if (strncmp(Buf, "#stget", 6) == 0) // Save Parameter
			{
				sprintf(UTxbuf, "#st,%d,%d,%d,%d,%d\r\n", stb0, stb1, stb2,
						stb3, stb_all);
				//sprintf(UTxbuf, "#st,%d,%d,%d,%d\r\n", stb0, stb1, stb2, stb3);
				CDC_Transmit_FS(UTxbuf, strlen(UTxbuf));
			}
			if (strncmp(Buf, "#stinit", 7) == 0) // Save Parameter
			{
				stb0 = 0;
				stb1 = 0;
				stb2 = 0;
				stb3 = 0;
				stb_all = 0;
				LL_GPIO_ResetOutputPin(LD1_GPIO_Port, LD1_Pin);
				LL_GPIO_ResetOutputPin(LD2_GPIO_Port, LD2_Pin);
				LL_GPIO_ResetOutputPin(LD3_GPIO_Port, LD3_Pin);
				//LL_GPIO_WriteOutputPort(GPIOx, PortValue);
			}
		}
		else
		{

			switch (Buf[0])
			{
			// SHELL INPUT DATA
			case 'M':
			case 'm':

				CDC_Transmit_FS((uint8_t*) Manual, strlen(Manual));
				bufptr = URxbuf;
				break;
			case 'V':
			case 'v':
			case 'P':
			case 'p':
				UFlag = 1;
				break;
			case 'I':
			case 'i':
			case 'W':
			case 'w':
				UFlag = 2;
				break;
			case 'L':
			case 'l':
				UFlag = 3;
				break;
			case 'D':
			case 'd':
				UFlag = 4;
				break;
			case 'C':
			case 'c':
				UFlag = 5;
				break;
			case 'X':
			case 'x':
				UFlag = 6;
				break;
			case 'S':
			case 's':
				UFlag = 7;
				break;
			case '2':
				UFlag = 8;
				break;
			case '3':
				UFlag = 9;
				break;
			case 'F':
			case 'f':
				HAL_GPIO_WritePin(SIG_AUTOFOCUS_GPIO_Port, SIG_AUTOFOCUS_Pin,
						SET);
				HAL_GPIO_WritePin(SIG_SHUTTER_GPIO_Port, SIG_SHUTTER_Pin, SET);
				HAL_GPIO_WritePin(STROBE_DRV_GPIO_Port, STROBE_DRV_Pin, RESET);

				LL_TIM_ClearFlag_UPDATE(TIM3);
				LL_TIM_EnableCounter(TIM3);
				break;
			case 'A':
			case 'a':
			case 'R':
			case 'r':
				TIM8->CNT = 0;
				A_PLS_CNT = 0;
				B_PLS_CNT = 0;
				bFlag = 1;
				CDC_Transmit_FS((uint8_t*) "AutoTrigger Started!\r\n", 22); // ACK
				break;
				//case 'S':
				//case 's':
			case 'T':
			case 't':
				bFlag = 0;
				CDC_Transmit_FS((uint8_t*) "Stopped!\r\n", 10); // ACK
				break;
			default:
				CDC_Transmit_FS((uint8_t*) Manual, strlen(Manual));
				break;
			}
		}
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
				EnterFlag = 1;
				bufptr = URxbuf;
			}
			else if (Buf[i] == '\b')
			{
				if (bufptr != URxbuf)
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
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
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
