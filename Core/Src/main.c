/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * NMEA2000 Switch Controller
  *
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Switch function definition
struct SWITCH_T {
	uint16_t		switchType;
	uint16_t		switchState;
	uint16_t		switchOnId;
	GPIO_TypeDef	*switchOnPort;
	uint16_t		switchOffId;
	GPIO_TypeDef	*switchOffPort;
	uint16_t		ledNo;
	uint16_t		switchOnTime;
	uint8_t			relayId;
	uint8_t			relayNo;
};

// Bring led port definitions to an array structure
struct LED_T {
	GPIO_TypeDef	*gpio;
	uint16_t		pin;
};

// NMEA2K network operational NAME parameters
struct NMEA2K_NETWORK {
	uint8_t			senderId;
	uint8_t			linkState;
	uint16_t		heartbeatTime;
	uint8_t			*nameData;
	uint8_t			linkRetries;
};

// Queue type for ISO Transport Messages being received
struct ISO_TRANSPORT_MESSAGE {
	uint8_t			senderId;
	uint8_t			isoCommand;
	uint8_t			packetsExpected;
	uint8_t			packetsReceived;
	uint16_t		messageSize;
	uint32_t		pgn;
	uint8_t			*data;
	struct ISO_TRANSPORT_MESSAGE *prev;
	struct ISO_TRANSPORT_MESSAGE *next;
};

// Queue type for timer actions
struct TIMER_T {
	int 			type;
	int				timeout;
	int 			countdown;
	void 			(*callback)(void *);
	void 			*payload;
	struct TIMER_T	*prev;
	struct TIMER_T	*next;
};

// Queue type for jobs to be executed
struct JOB_QUEUE_T {
	void 			(*callback)(void *);
	void 			*payload;
	struct JOB_QUEUE_T	*next;
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define N2K_ID				192		/* Initial NMEA2K sender id */
#define SWITCH_CHANNELS		8		/* Number of switches on this board */

// Initial ISO NAME parameters
#define MFG_CODE			666		/* Manufacturer code (0-2048) 666 for homemade */
#define DEV_INST			0		/* Device instance */
#define DEV_FUNCT			135		/* Switch Interface */
#define DEV_CLASS			110		/* Human Interface */
#define SYS_INST			0		/* System instance */
#define IND_GROUP			4		/* Industry group = 4 (Marine Industry) */
#define ARB_ADDRESS			1		/* Arbitrary address capable */
#define ISO_MAX_RETRIES		8		/* Maximum number of address claim retries */
#define ISO_HEARTBEAT		2000	/* Heartbeat interval = 20 sec (2000 * 10ms) */

// NMEA2K Link States
#define LINK_STATE_IDLE		0
#define LINK_STATE_CLAIMED	1
#define LINK_STATE_ACTIVE	2
#define LINK_STATE_ABORT	99

// Switch types
#define SWITCH_DISABLED		0
#define SWITCH_ON_OFF		1
#define SWITCH_PUSHBUTTON	2
#define SWITCH_MOMENTARY	3
#define SWITCH_MOM_TIME		4

// Switch states
#define SWITCH_IDLE			0
#define SWITCH_ON_1			1
#define SWITCH_OFF_1		2
#define SWITCH_ON_2			3
#define SWITCH_OFF_2		4
#define SWITCH_ON			5
#define SWITCH_OFF			6

#define TIMER_ONESHOT		1
#define TIMER_RELOAD		2

#define MODEL_ID			"GJK NMEA2000 Switch Controller"
#define SOFTWARE_VERSION	"V 1.0"
#define HARDWARE_VERSION	"V 1.0"

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
volatile uint16_t ledStatus;

// Configuration of switches
struct SWITCH_T switches[SWITCH_CHANNELS] = {
		{SWITCH_ON_OFF, SWITCH_IDLE, SW1_ON_Pin, SW1_ON_GPIO_Port, SW1_OFF_Pin, SW1_OFF_GPIO_Port, 0, 0, 0x8C, 0},
		{SWITCH_ON_OFF, SWITCH_IDLE, SW2_ON_Pin, SW2_ON_GPIO_Port, SW2_OFF_Pin, SW2_OFF_GPIO_Port, 1, 0, 0x8C, 4},
		{SWITCH_PUSHBUTTON, SWITCH_IDLE, SW3_ON_Pin, SW3_ON_GPIO_Port, SW3_OFF_Pin, SW3_OFF_GPIO_Port, 2, 0, 0x8C, 1},
		{SWITCH_PUSHBUTTON, SWITCH_IDLE, SW4_ON_Pin, SW4_ON_GPIO_Port, SW4_OFF_Pin, SW4_OFF_GPIO_Port, 3, 0, 0x8C, 5},
		{SWITCH_MOMENTARY, SWITCH_IDLE, SW5_ON_Pin, SW5_ON_GPIO_Port, SW5_OFF_Pin, SW5_OFF_GPIO_Port, 4, 0, 0x8C, 2},
		{SWITCH_MOMENTARY, SWITCH_IDLE, SW6_ON_Pin, SW6_ON_GPIO_Port, SW6_OFF_Pin, SW6_OFF_GPIO_Port, 5, 0, 0x8C, 6},
		{SWITCH_MOM_TIME, SWITCH_IDLE, SW7_ON_Pin, SW7_ON_GPIO_Port, SW7_OFF_Pin, SW7_OFF_GPIO_Port, 6, 300, 0x8C, 3},
		{SWITCH_MOM_TIME, SWITCH_IDLE, SW8_ON_Pin, SW8_ON_GPIO_Port, SW8_OFF_Pin, SW8_OFF_GPIO_Port, 7, 500, 0x8C, 7}
};

// Configuration of leds
struct LED_T leds[SWITCH_CHANNELS] = {
		{LED0_GPIO_Port, LED0_Pin},
		{LED1_GPIO_Port, LED1_Pin},
		{LED2_GPIO_Port, LED2_Pin},
		{LED3_GPIO_Port, LED3_Pin},
		{LED4_GPIO_Port, LED4_Pin},
		{LED5_GPIO_Port, LED5_Pin},
		{LED6_GPIO_Port, LED6_Pin},
		{LED7_GPIO_Port, LED7_Pin}
};

struct NMEA2K_NETWORK nmea2kNetwork;
struct TIMER_T *timers = NULL;
struct JOB_QUEUE_T *jobs = NULL;

char productInformation[128];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

void Send_ISOAddressClaim();
void Send_ISOAcknowledgement(uint8_t dest, uint8_t control, uint8_t group, uint32_t pgn);
void Send_HeartBeat(void * payload);
void Send_SwitchBankControl(uint8_t *data);
void Send_SwitchOn(void * payload);
void Send_SwitchOff(void * payload);
void Send_ProductInformation(void * payload);
int Send_FFMessage(CAN_TxHeaderTypeDef *TxHeader, uint8_t *data, int seqNo, int len);
void Handle_ISOTransportMessages(uint32_t N2KPgn, uint8_t senderId, uint8_t *data);
void Handle_ISOCommandedAddress(uint8_t *data);
void Set_Led(struct SWITCH_T * pSwitch, uint8_t *data);
int Compare_NameWeight(uint8_t *data);
int CheckSwitch(struct SWITCH_T *pSwitch);
struct TIMER_T * Add_Timer(int type, int timeout, void(*callback)(), void * payload);
void Delete_Timer(struct TIMER_T *timer);
struct JOB_QUEUE_T * Add_Job(void(*callback)(), void * payload);
void Delete_Job(struct JOB_QUEUE_T *job);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	ledStatus = 0;

	// Initialize Product Information record
	for (int i = 0; i < 128; i++)
		productInformation[i] = 0xFF;

	sprintf(productInformation, "%s", MODEL_ID);
	sprintf(productInformation + 32, "%s", SOFTWARE_VERSION);
	sprintf(productInformation + 64, "%s", HARDWARE_VERSION);
	sprintf(productInformation + 96, "%lu", (HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetUIDw2()));

	// Initialise the data array with the ISO NAME info
	uint8_t nameData[8];

	*(uint32_t *)nameData = (MFG_CODE << 21) | ((HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetUIDw2()) & 0x1FFFFF);
	nameData[4] = DEV_INST;
	nameData[5] = DEV_FUNCT;
	nameData[6] = (DEV_CLASS << 1) | 1;
	nameData[7] = ((ARB_ADDRESS == 0)?0:0x80) | ((IND_GROUP << 4) & 0x70) | ((SYS_INST) & 0x0F);

	// Populate initial N2K network structure
	nmea2kNetwork.senderId = N2K_ID;
	nmea2kNetwork.linkState = LINK_STATE_IDLE;
	nmea2kNetwork.heartbeatTime = ISO_HEARTBEAT;
	nmea2kNetwork.nameData = nameData;
	nmea2kNetwork.linkRetries = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  // Initialize continues timers
  HAL_TIM_Base_Stop_IT(&htim3);

  // Heartbeat timer
  if (Add_Timer(TIMER_RELOAD, nmea2kNetwork.heartbeatTime, &Send_HeartBeat, NULL) == NULL) {
	  Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		// If the NMEA2K link is idle, try to (re)activate it
		if (nmea2kNetwork.linkState == LINK_STATE_IDLE) {
			// Stop the seconds timer
			HAL_TIM_Base_Stop_IT(&htim3);
			// Check the retry counter against max retries
			if (nmea2kNetwork.linkRetries > ISO_MAX_RETRIES) {
				// ERROR, no valid address
				nmea2kNetwork.linkState = LINK_STATE_ABORT;
				Error_Handler();
			}
			// Try a PGN 60928 - ISO Address Claim
			Send_ISOAddressClaim();
			// Set link state to CLAIMED
			nmea2kNetwork.linkState = LINK_STATE_CLAIMED;
			// Wait for 250 ms for the claim
			HAL_Delay(250);
			// Check if the claim is still valid
			if (nmea2kNetwork.linkState == LINK_STATE_CLAIMED) {
				// Claim is valid, activate network
				nmea2kNetwork.linkState = LINK_STATE_ACTIVE;
				// Clear the retry counter
				nmea2kNetwork.linkRetries = 0;
				// (re)start the seconds timer
				HAL_TIM_Base_Start_IT(&htim3);
			}
		}

		// If the link is up, do the switch polling
		if (nmea2kNetwork.linkState == LINK_STATE_ACTIVE) {
			// Check all the switches for state changes
			for (int i = 0; i < SWITCH_CHANNELS; i++)
				CheckSwitch(&switches[i]);

			// Run job queue
			while (jobs != NULL) {
				(jobs->callback)(jobs->payload);
				Delete_Job(jobs);
			}

			// Wait 10 ms for debounce
			HAL_Delay(10);
		}
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */
	CAN_FilterTypeDef  sFilterN2K127501, sFilterN2K060928;

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
	// Filter for Binary Switch Bank Status messages
	uint32_t can_id = 127501 << 8;

	sFilterN2K127501.FilterBank = 1;
	sFilterN2K127501.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterN2K127501.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterN2K127501.FilterIdHigh = (can_id >> 13) & 0xFFFF;
	sFilterN2K127501.FilterIdLow = ((can_id << 3) & 0xFFF8) | 4;
	sFilterN2K127501.FilterMaskIdHigh = 0x0FFF;
	sFilterN2K127501.FilterMaskIdLow = 0xF804;
	sFilterN2K127501.FilterFIFOAssignment = CAN_FILTER_FIFO1;
	sFilterN2K127501.FilterActivation = CAN_FILTER_ENABLE;

	HAL_CAN_ConfigFilter(&hcan, &sFilterN2K127501);

	// Filter for ISO messages
	can_id = 0x0E800 << 8;

	sFilterN2K060928.FilterBank = 0;
	sFilterN2K060928.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterN2K060928.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterN2K060928.FilterIdHigh = (can_id >> 13) & 0xFFFF;
	sFilterN2K060928.FilterIdLow = ((can_id << 3) & 0xFFF8) | 4;
	sFilterN2K060928.FilterMaskIdHigh = 0x0FC0;
	sFilterN2K060928.FilterMaskIdLow = 0x0004;
	sFilterN2K060928.FilterFIFOAssignment = CAN_FILTER_FIFO1;
	sFilterN2K060928.FilterActivation = CAN_FILTER_ENABLE;

	HAL_CAN_ConfigFilter(&hcan, &sFilterN2K060928);

	// Start CAN bus
	HAL_CAN_Start(&hcan);
	// Enable RX interrupts
	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK) {
		Error_Handler();
	}

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 6399;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED0_Pin|LED1_Pin|LED2_Pin|LED3_Pin
                          |LED4_Pin|LED5_Pin|LED6_Pin|LED7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED0_Pin LED1_Pin LED2_Pin LED3_Pin
                           LED4_Pin LED5_Pin LED6_Pin LED7_Pin */
  GPIO_InitStruct.Pin = LED0_Pin|LED1_Pin|LED2_Pin|LED3_Pin
                          |LED4_Pin|LED5_Pin|LED6_Pin|LED7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SW1_ON_Pin SW1_OFF_Pin SW4_OFF_Pin SW5_ON_Pin
                           SW8_OFF_Pin SW2_ON_Pin SW2_OFF_Pin SW3_ON_Pin
                           SW3_OFF_Pin SW4_ON_Pin */
  GPIO_InitStruct.Pin = SW1_ON_Pin|SW1_OFF_Pin|SW4_OFF_Pin|SW5_ON_Pin
                          |SW8_OFF_Pin|SW2_ON_Pin|SW2_OFF_Pin|SW3_ON_Pin
                          |SW3_OFF_Pin|SW4_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SW5_OFF_Pin SW6_ON_Pin SW6_OFF_Pin SW7_ON_Pin
                           SW7_OFF_Pin SW8_ON_Pin */
  GPIO_InitStruct.Pin = SW5_OFF_Pin|SW6_ON_Pin|SW6_OFF_Pin|SW7_ON_Pin
                          |SW7_OFF_Pin|SW8_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/******************************************************************************
 * HAL_TIM_PeriodElapsedCallback
 *
 * HAL function every 0.01 sec called by the TIM1 timer
 * Used for timers
 *
 *****************************************************************************/
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)
{
	struct TIMER_T *current, *temp;

	__HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);

	current = timers;
	while (current != NULL) {
		// Countdown on timer
		if (current->countdown != 0) {
			current->countdown -= 1;
			current = current->next;
		}
		else {
			// Submit timer job to the job queue
 			Add_Job(current->callback, current->payload);

 			// Reload contignious timer
			if (current->type == TIMER_RELOAD) {
				current->countdown = current->timeout;
				current = current->next;
			}
			// Cleanup one-shot timer
			else if (current->type == TIMER_ONESHOT) {
				temp = current;
				current = current->next;
				Delete_Timer(temp);
			}
		}
	}
}


/******************************************************************************
 * HAL_CAN_RxFifo1MsgPendingCallback
 *
 * HAL function called when a CAN message is received through the filters
 * Determines which action must be taken as a reaction
 *
 *****************************************************************************/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef	RxHeader;
	uint8_t				RxData[8];
	uint32_t			N2KPgn;
	uint8_t				senderId;

	// Get the message from the FIFO
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData) != HAL_OK) {
		Error_Handler();
	}

	// Get the NMEA2000 PGN and the sender id from the extended CAN id
	N2KPgn = (RxHeader.ExtId >> 8) & 0x01FFFF;
	senderId = RxHeader.ExtId & 0xFF;

	// Check for Switch Bank Control message addressed to our channelID
	if (N2KPgn == 127501) {
		for (int i = 0; i < SWITCH_CHANNELS; i++) {
			if (RxData[0] == switches[i].relayId)
				Set_Led(&switches[i], RxData);
		}
	}

	// Check for ISO Address Claim message with our senderId
	if (((N2KPgn & 0x1FE00) == 0x0EE00) && (senderId == nmea2kNetwork.senderId)) {
		// If we are waiting for the claimed address, address is in use, send a new claim
		if (nmea2kNetwork.linkState == LINK_STATE_CLAIMED) {
			nmea2kNetwork.linkState = LINK_STATE_IDLE;
			nmea2kNetwork.linkRetries += 1;
			nmea2kNetwork.senderId += nmea2kNetwork.linkRetries;
		}
		// If our address is already active, check a couple of things
		if (nmea2kNetwork.linkState == LINK_STATE_ACTIVE) {
			// If the other contender can't change his senderID or has a higher NAME score, we change our senderID
			if (Compare_NameWeight(RxData)) {
				Send_ISOAddressClaim();
			}
			else {
				nmea2kNetwork.linkState = LINK_STATE_IDLE;
				nmea2kNetwork.linkRetries += 1;
				nmea2kNetwork.senderId += nmea2kNetwork.linkRetries;
			}
		}
	}

	// Check for ISO Request to our senderId or broadcast id
	if (((N2KPgn & 0x1FE00) == 0x0EA00) && (((N2KPgn & 0xFF) == nmea2kNetwork.senderId) || ((N2KPgn & 0xFF) == 0xFF))) {
		uint32_t requestPGN = *(uint32_t *)RxData & 0x00FFFFFF;

		// Check for a ISO Address Claim request
		if (requestPGN == 0x00EE00) {
			Add_Job(&Send_ISOAddressClaim, NULL);
		}

		// Check for a Product Information request
		if (requestPGN == 0x01F014) {
			Add_Job(&Send_ProductInformation, NULL);
		}
	}

	// Check for ISO Transport Protocol messages
	if (((N2KPgn & 0x1FF00) == 0x0EC00) || ((N2KPgn & 0x1FF00) == 0x0EB00)) {
		Handle_ISOTransportMessages(N2KPgn, senderId, RxData);
	}
}


/******************************************************************************
 * Send_ISOAddressClaim
 *
 * Send a ISO Address Claim message to claim our sender ID
 *
 *****************************************************************************/
void Send_ISOAddressClaim() {
	CAN_TxHeaderTypeDef	TxHeader;
	uint32_t			TxMailbox = CAN_TX_MAILBOX0;
	uint32_t			N2KId;

	// Initialise the CAN header with the NMEA2K info
	// Can ID = PRIO(3) - 0 - PGN(17) - Sender ID(8)
	N2KId = (6 << 26) | (0x0EEFF << 8) | nmea2kNetwork.senderId;
	TxHeader.ExtId = N2KId;
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 8;
	TxHeader.TransmitGlobalTime = DISABLE;

	// Send the NMEA 60928 ISO Address Claim message
	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, nmea2kNetwork.nameData, &TxMailbox) != HAL_OK) {
		Error_Handler();
	}

}


/******************************************************************************
 * Send_ISOAcknowledgement
 *
 * Send a ISO Acknowledgement message
 *
 * Parameters:
 * - dest		Destination address
 * - control	Acknowledgement type
 * - group		Group ID
 * - pgn		PGN to acknowledge
 *
 *****************************************************************************/
void Send_ISOAcknowledgement(uint8_t dest, uint8_t control, uint8_t group, uint32_t pgn) {
	CAN_TxHeaderTypeDef	TxHeader;
	uint32_t			TxMailbox = CAN_TX_MAILBOX0;
	uint32_t			N2KId;

	// Initialise the CAN header with the NMEA2K info
	// Can ID = PRIO(3) - 0 - PGN(17) - Sender ID(8)
	N2KId = (6 << 26) | (0x0E800 << 8) | (dest << 8) | nmea2kNetwork.senderId;
	TxHeader.ExtId = N2KId;
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 8;
	TxHeader.TransmitGlobalTime = DISABLE;

	// Initialise the data array with the ISO NAME info
	uint8_t TxData[] = {control, group, 0xFF, 0xFF, 0xFF, (pgn & 0xFF), ((pgn >> 8) & 0xFF), ((pgn >> 16) & 0xFF)};

	// Send the NMEA 60928 ISO Address Claim message
	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
		Error_Handler();
	}

}


/******************************************************************************
 * Send_SwitchBankControl
 *
 * Send a NMEA2000 Switch Bank Control message to control the relay board
 *
 * Parameters:
 *  - *TxData	Pointer to a CAN TxData[8] structure
 *
 *****************************************************************************/
void Send_SwitchBankControl(uint8_t *data)
{
	CAN_TxHeaderTypeDef	TxHeader;
	uint32_t			TxMailbox = CAN_TX_MAILBOX0;
	uint32_t			N2KId;

	// Initialize the CAN header with the NMEA2K info
	// Can ID = PRIO(3) - 0 - PGN(17) - Sender ID(8)
	N2KId = (3 << 26) | (127502 << 8) | nmea2kNetwork.senderId;
	TxHeader.ExtId = N2KId;
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 8;
	TxHeader.TransmitGlobalTime = DISABLE;

	// Send the NMEA 127501 Binary Switch Bank Status message
	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, data, &TxMailbox) != HAL_OK) {
		Error_Handler();
	}
}


/******************************************************************************
 * Send_HeartBeat
 *
 * Send a NMEA2000 Heartbeat message to the network
 *
 *****************************************************************************/
void Send_HeartBeat(void * payload) {
	static int			sequence = 0;
	CAN_TxHeaderTypeDef	TxHeader;
	uint32_t			TxMailbox = CAN_TX_MAILBOX0;
	uint32_t			N2KId;

	// Initialise the CAN header with the NMEA2K info
	// Can ID = PRIO(3) - 0 - PGN(17) - Sender ID(8)
	N2KId = (6 << 26) | (0x1F011 << 8) | nmea2kNetwork.senderId;
	TxHeader.ExtId = N2KId;
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 8;
	TxHeader.TransmitGlobalTime = DISABLE;

	// Initialise the data array with the heartbeat info
	uint8_t TxData[8];

	*((uint16_t *)TxData) = nmea2kNetwork.heartbeatTime;
	TxData[2] = sequence++;
	TxData[3] = 0b01010011;
	*((uint32_t *)(TxData + 4)) = 0xFFFFFFFF;

	if (sequence > 252)
		sequence = 0;

	// Send the NMEA 60928 ISO Address Claim message
	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
		Error_Handler();
	}

}


void Send_SwitchOn(void * payload) {
	int offset, index;
	struct SWITCH_T *pSwitch = (struct SWITCH_T *)payload;
	uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

	if (nmea2kNetwork.linkState == LINK_STATE_ACTIVE) {
		// Reset the data array
		data[0] = pSwitch->relayId;
		offset = 2 * (pSwitch->relayNo % 4);
		index = 1 + (pSwitch->relayNo / 4);
		data[index] &= ~(0b10 << offset);
		Send_SwitchBankControl(data);
	}
}


void Send_SwitchOff(void * payload) {
	int offset, index;
	struct SWITCH_T *pSwitch = (struct SWITCH_T *)payload;
	uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

	if (nmea2kNetwork.linkState == LINK_STATE_ACTIVE) {
		// Reset the data array
		data[0] = pSwitch->relayId;
		offset = 2 * (pSwitch->relayNo % 4);
		index = 1 + (pSwitch->relayNo / 4);
		data[index] &= ~(0b11 << offset);
		Send_SwitchBankControl(data);
	}
}


void Send_ProductInformation(void * payload) {
	CAN_TxHeaderTypeDef	TxHeader;
	uint32_t			N2KId;
	uint8_t				*data;

	// Initialise the CAN header with the NMEA2K info
	// Can ID = PRIO(3) - 0 - PGN(17) - Sender ID(8)
	N2KId = (6 << 26) | (0x1F014 << 8) | nmea2kNetwork.senderId;
	TxHeader.ExtId = N2KId;
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 8;
	TxHeader.TransmitGlobalTime = DISABLE;

	// Allocate data structure
	if ((data = malloc(134 * sizeof(uint8_t))) == NULL) {
		Error_Handler();
	}

	// Populate data structure
	*(uint16_t *)data = 3000;					// NMEA2000 v3.000
	*(uint16_t *)(data + 2) = 101;				// Model number 1.01
	memcpy(data + 4, productInformation, 128);	// 128 bytes of product information
	*(data + 132) = 0xFF;						// No official NMEA2000 certification
	*(data + 133) = 4;							// Guaranteed max load = 4 * 50mA = 200mA

	// Send as Fast Frame message
	Send_FFMessage(&TxHeader, data, 0, 134);
	free(data);
}


int Send_FFMessage(CAN_TxHeaderTypeDef *TxHeader, uint8_t *data, int seqNo, int len) {
	uint32_t TxMailbox = CAN_TX_MAILBOX0;
	int offset = 0, frameCounter = 0, pkgLen;
	uint8_t TxData[8];

	// Check validity of len parameter
	if ((len > 223) || (len < 9))
		return frameCounter;

	// Frame length is always 8 data bytes
	TxHeader->DLC = 8;

	// Fill TxData for first package
	TxData[0] = (seqNo << 5) | (frameCounter++ & 0x1F);
	TxData[1] = len & 0xFF;
	memcpy(&TxData[2], data, ((len - offset) < 6)?(len - offset):6);
	offset += ((len - offset) < 6)?(len - offset):6;

	// Send the NMEA2000 FF Frame
	if (HAL_CAN_AddTxMessage(&hcan, TxHeader, TxData, &TxMailbox) != HAL_OK) {
		Error_Handler();
	}

	while (offset < len) {
		// Delay for 1 ms to prevent buffer flooding (non critical)
		HAL_Delay(1);

		// Fill TxData for additional packages
		TxData[0] = (seqNo << 5) | (frameCounter++ & 0x1F);
		pkgLen = ((len - offset) < 7)?(len - offset):7;
		memcpy(&TxData[1], data + offset, pkgLen);
		offset += pkgLen;

		// Pad till 8 bytes
		for (int i = pkgLen; i < 7; i++) {
			TxData[i + 1] = 0xFF;
		}

		// Send the NMEA2000 FF Frame
		if (HAL_CAN_AddTxMessage(&hcan, TxHeader, TxData, &TxMailbox) != HAL_OK) {
			Error_Handler();
		}
	}

	// Return number of frames send
	return frameCounter;
}


void Handle_ISOTransportMessages(uint32_t N2KPgn, uint8_t senderId, uint8_t *data) {
	struct ISO_TRANSPORT_MESSAGE *msgList = NULL, *currentMsg, *newMsg;
	int offset, byteCount;

	// BAM announcement
	if (((N2KPgn & 0x1FF00) == 0x0EC00) && (data[0] == 0x20)) {
		// Find the last message to insert after
		if ((currentMsg = msgList) != NULL) {
			while (currentMsg->next != NULL)
				currentMsg = currentMsg->next;
		}

		// Create the new message structure
		if ((newMsg = malloc(sizeof(struct ISO_TRANSPORT_MESSAGE))) == NULL) {
			Error_Handler();
		}

		// Fill the structure
		newMsg->senderId = senderId;
		newMsg->isoCommand = data[0];
		newMsg->packetsExpected = data[3];
		newMsg->packetsReceived = 0;
		newMsg->messageSize = (uint16_t)data[1];
		newMsg->pgn = (((uint32_t)data[4]) >> 8);
		newMsg->prev = currentMsg;
		newMsg->next = NULL;

		// Create the data structure
		if ((newMsg->data = malloc(newMsg->messageSize * sizeof(uint8_t))) == NULL) {
			Error_Handler();
		}

		// Link newMsg in the list
		if (currentMsg == NULL) {
			msgList = newMsg;
		}
		else {
			currentMsg->prev = newMsg;
		}
	}

	// Data package
	if ((N2KPgn & 0x1FF00) == 0x0EB00) {
		currentMsg = msgList;
		// Find the right message
		while(currentMsg != NULL) {
			if (currentMsg->senderId == senderId) {
				offset = data[0] - 1;
				if ((byteCount = currentMsg->messageSize - (data[0] * 7)) > 7)
					byteCount = 7;
				memcpy(currentMsg->data + offset, data, byteCount);
				currentMsg->packetsReceived += 1;

				// Check if message is complete
				if (currentMsg->packetsReceived == currentMsg->packetsExpected) {
					// Check for ISO Commanded Address
					if (currentMsg->pgn == 65240)
						Handle_ISOCommandedAddress(currentMsg->data);

					// Cleanup memory and restore linked list
					if (currentMsg->prev == NULL)
						msgList = currentMsg->next;
					else
						currentMsg->prev->next = currentMsg->next;

					if (currentMsg->next != NULL)
						currentMsg->next->prev = currentMsg->prev;

					free(currentMsg->data);
					free(currentMsg);
				}
				break;
			}
		}
	}
}


void Handle_ISOCommandedAddress(uint8_t *data) {
	if (*(uint64_t *)nmea2kNetwork.nameData == *(uint64_t *)data) {
		nmea2kNetwork.senderId = data[8];
		nmea2kNetwork.nameData[7] &= ~(1 << 7);
		nmea2kNetwork.linkState = LINK_STATE_IDLE;
	}
}


void Set_Led(struct SWITCH_T * pSwitch, uint8_t *data) {
	int offset, index;

	// Determine index and offset for the parameters in the data structure
	offset = 2 * (pSwitch->relayNo % 4);
	index = 1 + (pSwitch->relayNo / 4);

	if ((data[index] >> offset) & 0x01) {
		// Parameter is 1, light the LED
		ledStatus |= (1 << pSwitch->ledNo);
		HAL_GPIO_WritePin(leds[pSwitch->ledNo].gpio, leds[pSwitch->ledNo].pin, GPIO_PIN_SET);
	}
	else {
		// Parameter is 0, dim the LED
		ledStatus &= ~(1 << pSwitch->ledNo);
		HAL_GPIO_WritePin(leds[pSwitch->ledNo].gpio, leds[pSwitch->ledNo].pin, GPIO_PIN_RESET);
	}
}


/******************************************************************************
 * Compare_NameWeight
 *
 * Helper function to compare our ISO NAME values against a contender
 *
 * Parameters:
 * 	- *data		Pointer to a CAN RxData[8] structure
 *
 * Returns:
 * 	- (-1)		Our ISO NAME value is greater, reclaim our address
 * 	- (0)		The ISO NAME values are the same, increase deviceInstance and
 * 				reclaim our address (We were first! :-P )
 * 	- (+1)		Our contender has a greater ISO NAME value, we do a modified
 * 				address claim
 *
 *****************************************************************************/
int Compare_NameWeight(uint8_t *data) {
	return (*(uint64_t *)nmea2kNetwork.nameData <= *(uint64_t *)data);
}


/******************************************************************************
 * CheckSwitch
 *
 * Check the state of the switch. Debounce by state machine, this also mekes
 * shure the switch only fires once
 *
 * Parameters:
 *  - switchNo		Number of the switch to be checked
 *****************************************************************************/
int CheckSwitch(struct SWITCH_T *pSwitch) {
	GPIO_PinState swOn, swOff;
	swOn = HAL_GPIO_ReadPin(pSwitch->switchOnPort, pSwitch->switchOnId);
	swOff = HAL_GPIO_ReadPin(pSwitch->switchOffPort, pSwitch->switchOffId);

	switch (pSwitch->switchType) {
	case SWITCH_ON_OFF:
		if ((swOn == GPIO_PIN_RESET) && (swOff == GPIO_PIN_SET)) {
			switch (pSwitch->switchState) {
			case SWITCH_IDLE:
				pSwitch->switchState = SWITCH_ON_1;
				break;
			case SWITCH_ON_1:
				pSwitch->switchState = SWITCH_ON_2;
				break;
			case SWITCH_ON_2:
 				pSwitch->switchState = SWITCH_ON;
 				Send_SwitchOn(pSwitch);
				return 1;
			}
		}
		if ((swOn == GPIO_PIN_SET) && (swOff == GPIO_PIN_RESET)) {
			switch (pSwitch->switchState) {
			case SWITCH_IDLE:
				pSwitch->switchState = SWITCH_OFF_1;
				break;
			case SWITCH_OFF_1:
				pSwitch->switchState = SWITCH_OFF_2;
				break;
			case SWITCH_OFF_2:
				pSwitch->switchState = SWITCH_OFF;
 				Send_SwitchOff(pSwitch);
				return -1;
			}
		}
		// Switch is released, set it back to idle
		if ((swOn == GPIO_PIN_SET) && (swOff == GPIO_PIN_SET) && (pSwitch->switchState != SWITCH_IDLE))
			pSwitch->switchState = SWITCH_IDLE;

		break;
	case SWITCH_PUSHBUTTON:
		if ((swOn == GPIO_PIN_RESET) || (swOff == GPIO_PIN_RESET)) {
			switch (pSwitch->switchState) {
			case SWITCH_IDLE:
				pSwitch->switchState = SWITCH_ON_1;
				break;
			case SWITCH_ON_1:
				pSwitch->switchState = SWITCH_ON_2;
				break;
			case SWITCH_ON_2:
				pSwitch->switchState = SWITCH_ON;
				if (ledStatus & (0x01 << pSwitch->ledNo)) {
					// Switch is On, set to Off
	 				Send_SwitchOff(pSwitch);
					return -1;
				}
				else {
					// Switch is Off, turn it On
	 				Send_SwitchOn(pSwitch);
					return 1;
				}
			}
		}
		// Switch is released, set it back to idle
		if ((swOn == GPIO_PIN_SET) && (swOff == GPIO_PIN_SET) && (pSwitch->switchState != SWITCH_IDLE))
			pSwitch->switchState = SWITCH_IDLE;

		break;
	case SWITCH_MOMENTARY:
		if ((swOn == GPIO_PIN_RESET) || (swOff == GPIO_PIN_RESET)) {
			switch (pSwitch->switchState) {
			case SWITCH_IDLE:
				pSwitch->switchState = SWITCH_ON_1;
				break;
			case SWITCH_ON_1:
				pSwitch->switchState = SWITCH_ON_2;
				break;
			case SWITCH_ON_2:
				pSwitch->switchState = SWITCH_ON;
 				Send_SwitchOn(pSwitch);
				return 1;
			}
		}
		// Switch is released, set it back to idle and turn the channel Off
		if ((swOn == GPIO_PIN_SET) && (swOff == GPIO_PIN_SET) && (pSwitch->switchState != SWITCH_IDLE)) {
			pSwitch->switchState = SWITCH_IDLE;
			Send_SwitchOff(pSwitch);
			return -1;
		}

		break;
	case SWITCH_MOM_TIME:
		if ((swOn == GPIO_PIN_RESET) || (swOff == GPIO_PIN_RESET)) {
			switch (pSwitch->switchState) {
			case SWITCH_IDLE:
				pSwitch->switchState = SWITCH_ON_1;
				break;
			case SWITCH_ON_1:
				pSwitch->switchState = SWITCH_ON_2;
				break;
			case SWITCH_ON_2:
				pSwitch->switchState = SWITCH_ON;
 				Send_SwitchOn(pSwitch);
				if (Add_Timer(TIMER_ONESHOT, pSwitch->switchOnTime, &Send_SwitchOff, pSwitch) == NULL)
					Error_Handler();
				return 1;
			}
		}
		// Switch is released, set it back to idle
		if ((swOn == GPIO_PIN_SET) && (swOff == GPIO_PIN_SET) && (pSwitch->switchState != SWITCH_IDLE))
			pSwitch->switchState = SWITCH_IDLE;

		break;
	}
	return 0;
}


struct TIMER_T * Add_Timer(int type, int timeout, void(*callback)(), void * payload) {
	struct TIMER_T *current, *new;

	if ((new = malloc(sizeof(struct TIMER_T))) == NULL)
		return NULL;

	new->type = type;
	new->timeout = timeout;
	new->countdown = timeout;
	new->callback = callback;
	new->payload = payload;

	if (timers == NULL) {
		new->prev = NULL;
		new->next = NULL;
		timers = new;
	}
	else {
		current = timers;
		while (current->next != NULL)
			current = current->next;
		new->prev = current;
		new->next = NULL;
		current->next = new;
	}

	return new;;
}


void Delete_Timer(struct TIMER_T *timer) {
	if (timer->prev == NULL)
		timers = timer->next;
	else
		timer->prev->next = timer->next;

	if (timer->next != NULL)
		timer->next->prev = timer->prev;

	free(timer);
}


struct JOB_QUEUE_T * Add_Job(void(*callback)(), void * payload) {
	struct JOB_QUEUE_T *current, *new;

	if ((new = malloc(sizeof(struct JOB_QUEUE_T))) == NULL)
		return NULL;

	new->callback = callback;
	new->payload = payload;
	new->next = NULL;

	if (jobs == NULL)
		jobs = new;
	else {
		current = jobs;
		while (current->next != NULL)
			current = current->next;
		current->next = new;
	}

	return new;
}

void Delete_Job(struct JOB_QUEUE_T *job) {
	struct JOB_QUEUE_T *current;

	if (jobs == NULL) {}	// Do nothing
	else if (jobs == job)
		jobs = job->next;
	else {
		current = jobs;
		while (current->next != job)
			current = current->next;
		current->next = job->next;
	}

	free(job);
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  volatile int a = 1, b = 7;
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
	  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  for (int i = 0; i < 2000000; i++) {
		  a = a * b;
	  }
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
