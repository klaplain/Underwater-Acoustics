/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "wav.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ADC -----------------------------------------------------------------------*/
#define ADCBUFLEN 32768
#define BUFFER_FULL 1
#define BUFFER_EMPTY 0
/* SPI Message codes ---------------------------------------------------------*/
#define DIRECTORY 0x02
#define DATETIME 0x03
#define SAVE 0x05
#define FORMAT 0x06
#define DELETE 0x07
#define REC2 0x09

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd1;

SPI_HandleTypeDef hspi6;

UART_HandleTypeDef huart3;

MDMA_HandleTypeDef hmdma_mdma_channel0_sdmmc1_end_data_0;
/* USER CODE BEGIN PV */
/* ADC Variables--------------------------------------------------------------*/
uint16_t adc_buf[ADCBUFLEN];
uint16_t adc_lower_status = BUFFER_EMPTY;
uint16_t adc_upper_status = BUFFER_EMPTY;
uint32_t millisecs_to_record=0;
int sampling_frequency_kHz=0;
uint32_t end_acq_ms;
uint32_t bytes_written=0;
uint32_t bytes_read=0;
uint32_t total_bytes_written=0;
uint32_t total_blocks_written=0;
uint32_t read_value=0;
uint32_t adcbuf_index=0;

uint16_t max_save_time_ms;
uint16_t start_time_ms,ms_taken;
int ADC_overrun =0; // set as FALSE
int overrun_count;

/* SD Card FatFS Variables-----------------------------------------------------*/
FRESULT result; /* FatFs function common result code */
FILINFO fno;
char full_path_name[256];
char filename[200];
char fullfilename[200];
char filenumber[6];
char filenametosave[15];
char filenametodelete[15];

FATFS *fs;
DWORD fre_clust, fre_sect, tot_sect;

BYTE work[_MAX_SS]; // for formatting SD

char *SD_Directory2;
int directory_lines=6;
char SD_space_description[150];

uint16_t current_max_filenumber=0;
uint32_t filelength;
uint16_t filereadint;
uint32_t file_byte_count=0;
int16_t filevalue;

/* SPI Variables---------------------------------------------------------------*/
char SPI_buffer[20];
char filename_buffer[50];
uint16_t filesize_buffer[4];
uint16_t SPI_Buffer[4];
uint16_t SPI6_NCS;
uint16_t previous_CS;
uint16_t current_CS;

/* WAV File--------------------------------------------------------------------*/
/*
 * Structure of Header and header variable
 */
typedef struct wavfile_header_s
{
	char    ChunkID[4];     /*  4   */
	uint32_t ChunkSize;      /*  4   */
	char    Format[4];      /*  4   */

	char    Subchunk1ID[4]; /*  4   */
	uint32_t Subchunk1Size;  /*  4   */
	uint16_t AudioFormat;    /*  2   */
	uint16_t NumChannels;    /*  2   */
	uint32_t SampleRate;     /*  4   */
	uint32_t ByteRate;       /*  4   */
	uint16_t BlockAlign;     /*  2   */
	uint16_t BitsPerSample;  /*  2   */

	char    Subchunk2ID[4];
	uint32_t Subchunk2Size;
} wavfile_header_t;

/* RTC Variables---------------------------------------------------------------*/
RTC_DateTypeDef gDate;
RTC_TimeTypeDef gTime;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_MDMA_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI6_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
void directory_request_handler();
FRESULT get_SD_directory (char* path);
void datetime_request_handler();
void recording_request_handler2();
int write_wav_header(int32_t SampleRate,int32_t FrameCount);
void delete_file_handler();
void save_request_handler();
void set_ADC_clock_prescalar(int sampling_fr);

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
	/* ADC Testing and writing to SD Card*/
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

	HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_MDMA_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  MX_USART3_UART_Init();
  MX_SPI6_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

	printf("\r\nProgram Start\r\n");
//	HAL_GPIO_WritePin(SIG_ATTEN_GPIO_Port, SIG_ATTEN_Pin, GPIO_PIN_SET);  // Set attenuator to "no attenuation"

	// Let's check the file system and mount it
	f_mount(0, "", 0);
	if(f_mount(&SDFatFS, (TCHAR const*)SDPath, 0) != FR_OK)
	{
		printf("PANIC: Cannot mount SD Card\r\n");
		for(;;){}
	}

	HAL_GPIO_WritePin(Acq_Busy_GPIO_Port, Acq_Busy_Pin, GPIO_PIN_RESET);  // Tell the RASPI we are Not Busy
	previous_CS=HAL_GPIO_ReadPin (SPI6_NCS_GPIO_Port, SPI6_NCS_Pin);  // Need to detect and wait for change in SPI CS (Chip Select)
	current_CS = previous_CS;

	for(;;){

		HAL_GPIO_WritePin(Acq_Busy_GPIO_Port, Acq_Busy_Pin, GPIO_PIN_RESET);  // Tell RASPI we are Ready
		current_CS=HAL_GPIO_ReadPin (SPI6_NCS_GPIO_Port, SPI6_NCS_Pin);

		if(previous_CS && !current_CS){
			HAL_SPI_Receive(&hspi6, (uint8_t *)SPI_buffer, 8, 100);
			HAL_GPIO_WritePin(Acq_Busy_GPIO_Port, Acq_Busy_Pin, GPIO_PIN_SET);  // Tell the RASPI we are Busy
			printf("\r\n%02x %02x %02x %02x %02x %02x %02x %02x\r\n",SPI_buffer[0],SPI_buffer[1],SPI_buffer[2],SPI_buffer[3],SPI_buffer[4],SPI_buffer[5],SPI_buffer[6],SPI_buffer[7]);
			switch(SPI_buffer[0]){

			case DIRECTORY: //Get SD Card directory contents
				printf("SD Directory Request\r\n");
				directory_request_handler();
				break;

			case DATETIME:
				printf("Set DateTime\r\n");
				datetime_request_handler();
				break;

			case SAVE:  // Save file to raspberry pi
				printf("Save\r\n");
				save_request_handler();
				break;

			case FORMAT:// Format SD card
				printf("Format SD - Sector size %d\r\n",sizeof(work));
				if((SPI_buffer[2] == FORMAT) && (SPI_buffer[4] == FORMAT) && (SPI_buffer[6] == FORMAT)){
					result= f_mkfs("", FM_ANY, 0, work, sizeof(work));
					directory_lines=6;
					printf("Formated SD");
				}
				break;

			case DELETE:// Delete file
				printf("Delete File\r\n");
				delete_file_handler();
				break;

			case REC2: // Record analog input
				printf("Recording Request2\r\n");
				recording_request_handler2();
				break;

			case 0:
				for(;;);
				break;

			}
		}
		previous_CS=current_CS;

	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 1;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 2;
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief SPI6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI6_Init(void)
{

  /* USER CODE BEGIN SPI6_Init 0 */

  /* USER CODE END SPI6_Init 0 */

  /* USER CODE BEGIN SPI6_Init 1 */

  /* USER CODE END SPI6_Init 1 */
  /* SPI6 parameter configuration*/
  hspi6.Instance = SPI6;
  hspi6.Init.Mode = SPI_MODE_SLAVE;
  hspi6.Init.Direction = SPI_DIRECTION_2LINES;
  hspi6.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi6.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi6.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi6.Init.NSS = SPI_NSS_SOFT;
  hspi6.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi6.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi6.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi6.Init.CRCPolynomial = 0x0;
  hspi6.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi6.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi6.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi6.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi6.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi6.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi6.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi6.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi6.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi6.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI6_Init 2 */

  /* USER CODE END SPI6_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 576000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * Enable MDMA controller clock
  * Configure MDMA for global transfers
  *   hmdma_mdma_channel0_sdmmc1_end_data_0
  */
static void MX_MDMA_Init(void)
{

  /* MDMA controller clock enable */
  __HAL_RCC_MDMA_CLK_ENABLE();
  /* Local variables */

  /* Configure MDMA channel MDMA_Channel0 */
  /* Configure MDMA request hmdma_mdma_channel0_sdmmc1_end_data_0 on MDMA_Channel0 */
  hmdma_mdma_channel0_sdmmc1_end_data_0.Instance = MDMA_Channel0;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.Request = MDMA_REQUEST_SDMMC1_END_DATA;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.TransferTriggerMode = MDMA_BUFFER_TRANSFER;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.Priority = MDMA_PRIORITY_LOW;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.SourceInc = MDMA_SRC_INC_BYTE;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.DestinationInc = MDMA_DEST_INC_BYTE;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.SourceDataSize = MDMA_SRC_DATASIZE_BYTE;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.DestDataSize = MDMA_DEST_DATASIZE_BYTE;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.BufferTransferLength = 1;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.SourceBlockAddressOffset = 0;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.DestBlockAddressOffset = 0;
  if (HAL_MDMA_Init(&hmdma_mdma_channel0_sdmmc1_end_data_0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure post request address and data masks */
  if (HAL_MDMA_ConfigPostRequestMask(&hmdma_mdma_channel0_sdmmc1_end_data_0, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* MDMA interrupt initialization */
  /* MDMA_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MDMA_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(MDMA_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RED_LED_Pin|RTL8720CHIPEN_Pin|GAINA0_Pin|GAINA1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Acq_Busy_GPIO_Port, Acq_Busy_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SIG_ATTEN_GPIO_Port, SIG_ATTEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RED_LED_Pin RTL8720CHIPEN_Pin GAINA0_Pin GAINA1_Pin */
  GPIO_InitStruct.Pin = RED_LED_Pin|RTL8720CHIPEN_Pin|GAINA0_Pin|GAINA1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Acq_Busy_Pin */
  GPIO_InitStruct.Pin = Acq_Busy_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Acq_Busy_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI6_NCS_Pin */
  GPIO_InitStruct.Pin = SPI6_NCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SPI6_NCS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Record_Enable_Pin */
  GPIO_InitStruct.Pin = Record_Enable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Record_Enable_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CARD_DETECT_Pin */
  GPIO_InitStruct.Pin = SD_CARD_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(SD_CARD_DETECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SIG_ATTEN_Pin */
  GPIO_InitStruct.Pin = SIG_ATTEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SIG_ATTEN_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
	(void)file;
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		HAL_UART_Transmit(&huart3,(uint8_t*)ptr++,1,1);// Sending in normal mode
	}
	return len;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	adc_lower_status = BUFFER_FULL;
	if(adc_upper_status == BUFFER_FULL)  // Overflow detect
	{
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
		ADC_overrun=1; //Set overrun flag
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	adc_upper_status = BUFFER_FULL;
	if(adc_lower_status == BUFFER_FULL)  // Overflow detect
	{
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
		ADC_overrun=1; //Set overrun flag
	}
	//
}


void directory_request_handler()
{
	HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);				/* Get the RTC current Date */
	HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);				/* Display time Format: hh:mm:ss */
	printf("RTC Date Time  %02d:%02d:%02d  %02d-%02d-%2d\r\n",gTime.Hours, gTime.Minutes, gTime.Seconds,gDate.Month,gDate.Date,2000 + gDate.Year);				/* Display date Format: dd-mm-yy */

	/* allocate memory for directory listing */
	SD_Directory2 = malloc( 100 * directory_lines++ );

	if( SD_Directory2 == NULL ) {
		printf("PANIC: unable to allocate required memory\r\n");
	}
	/* Get volume information and free clusters of drive 1 */
	result = f_getfree("0:", &fre_clust, &fs);
	if (result!= FR_OK){
		printf("PANIC: unable to determine free space on SD\r\n");
	}
	/* Get total sectors and free sectors */
	tot_sect = (fs->n_fatent - 2) * fs->csize;
	fre_sect = fre_clust * fs->csize;
	/* Print free space in unit of KB (assuming 512 bytes/sector) */
	sprintf(SD_space_description,"%lu KB total drive space.     %lu KB available.\r\n",tot_sect / 2, fre_sect / 2);
	strcpy( SD_Directory2, SD_space_description);

	/* Get directory contents */
	strcpy(full_path_name, "/");

	result= get_SD_directory(full_path_name);
	printf("%s", SD_Directory2);
	strcat( SD_Directory2, "\f");

	/* Send directory content to raspi */
	int buffer_index=0;
	do {
		SPI_Buffer[0]=SD_Directory2[buffer_index];
		HAL_GPIO_WritePin(Acq_Busy_GPIO_Port, Acq_Busy_Pin, GPIO_PIN_RESET);  // Tell RASPI we are Ready
		HAL_SPI_Transmit(&hspi6, (uint8_t *)SPI_Buffer, 1, 5000);
		HAL_GPIO_WritePin(Acq_Busy_GPIO_Port, Acq_Busy_Pin, GPIO_PIN_SET);  // Tell RASPI we are busy
	}
	while(SD_Directory2[buffer_index++] != 12);

	printf("Directory Uploaded\r\n");
	free(SD_Directory2);
	directory_lines=5;

}

FRESULT get_SD_directory (char* path)        /* Start node to be scanned (***also used as work area***) */
{
	FRESULT res;
	DIR dir;
	UINT i;
	static FILINFO fno;
	char this_file[280];

	res = f_opendir(&dir, path);                       /* Open the directory */
	if (res == FR_OK) {
		for (;;) {
			res = f_readdir(&dir, &fno);                   /* Read a directory item */
			if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
			if (fno.fattrib & AM_DIR) {                    /* It is a directory */
				i = strlen(path);
				sprintf(&path[i], "/%s", fno.fname);
				res = get_SD_directory(path);                    /* Enter the directory */
				if (res != FR_OK) break;
				path[i] = 0;
			} else {                                       /* It is a file. */
				sprintf(&this_file[0],"%s/%s\t%ld\t%u-%02u-%02u\t%02u:%02u\r\n", path, fno.fname,fno.fsize,(fno.fdate >> 9) + 1980, fno.fdate >> 5 & 15, fno.fdate & 31,fno.ftime >> 11, fno.ftime >> 5 & 63);
				SD_Directory2 = realloc(SD_Directory2, 100 * directory_lines++ );
				if( SD_Directory2 == NULL ) {
					printf("PANIC: unable to allocate required memory\r\n");
				} else {
					strcat( SD_Directory2, this_file);
				}
			}
		}
		f_closedir(&dir);
	}
	return res;
}
void datetime_request_handler(){
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	sTime.Hours = SPI_buffer[1]; // set hours
	sTime.Minutes = SPI_buffer[2]; // set minutes
	sTime.Seconds = SPI_buffer[3]; // set seconds
	sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
	{
		printf("FAIL: Problem setting time\r\n");
	}
	sDate.WeekDay = 4;
	sDate.Month = SPI_buffer[5];
	sDate.Date = SPI_buffer[6]; // date
	sDate.Year = SPI_buffer[7]; // year
	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
	{
		printf("FAIL: Problem setting date\r\n");
	}
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2); // backup register
	printf("DateTime Set Complete\r\n");
}

void save_request_handler(){
	int filename_byte_length;

	// Now read the number of bytes needed for the filename
	HAL_GPIO_WritePin(Acq_Busy_GPIO_Port, Acq_Busy_Pin, GPIO_PIN_RESET);  // Tell the RASPI we are Ready for the filename
	HAL_SPI_Receive(&hspi6, (uint8_t *)SPI_buffer, 2, 2000);
	HAL_GPIO_WritePin(Acq_Busy_GPIO_Port, Acq_Busy_Pin, GPIO_PIN_SET);  // Tell the RASPI we are Busy
	filename_byte_length = (SPI_buffer[0]<<8) + (SPI_buffer[1]);

	// now read the filename
	HAL_GPIO_WritePin(Acq_Busy_GPIO_Port, Acq_Busy_Pin, GPIO_PIN_RESET);  // Tell the RASPI we are Ready for the filename
	HAL_SPI_Receive(&hspi6, (uint8_t *)filename_buffer, filename_byte_length, 2000);
	HAL_GPIO_WritePin(Acq_Busy_GPIO_Port, Acq_Busy_Pin, GPIO_PIN_SET);  // Tell the RASPI we are Busy

	printf("opening file %s to upload\r\n", filename_buffer);
	if(f_open(&SDFile, filename_buffer, FA_READ) != FR_OK)  				//Open file for reading and uploading
	{
		printf("FAIL: Cannot open file for reading uploading\r\n");
	}
	else
	{
		SPI_Buffer[0]=f_size(&SDFile);
		SPI_Buffer[1]=f_size(&SDFile)>>16;
		HAL_GPIO_WritePin(Acq_Busy_GPIO_Port, Acq_Busy_Pin, GPIO_PIN_RESET);  // Tell RASPI we are Ready
		HAL_SPI_Transmit(&hspi6, (uint8_t *)SPI_Buffer, 4, 5000);
		file_byte_count=0;
		HAL_GPIO_WritePin(Acq_Busy_GPIO_Port, Acq_Busy_Pin, GPIO_PIN_SET);  // Tell RASPI we are Busy

		file_byte_count=0;
		while(!f_eof(&SDFile)){
			result = f_read(&SDFile, &filereadint,2 , (void *)&bytes_read);
			if((bytes_read == 0) || (result != FR_OK))
			{
				printf("FAIL: Cannot read file to save upload\r\n");
			}
			else
			{
				file_byte_count++;
				filevalue=filereadint;
				if(file_byte_count>sizeof(wavfile_header_t)){
					filevalue = filereadint-32768;
				}

				SPI_Buffer[0]= filevalue;

				HAL_GPIO_WritePin(Acq_Busy_GPIO_Port, Acq_Busy_Pin, GPIO_PIN_RESET);  // Tell RASPI we are Ready
				HAL_SPI_Transmit(&hspi6, (uint8_t *)SPI_Buffer, 2, 5000);
				HAL_GPIO_WritePin(Acq_Busy_GPIO_Port, Acq_Busy_Pin, GPIO_PIN_SET);  // Tell RASPI we are Busy
			}
		}
	}
	HAL_GPIO_WritePin(Acq_Busy_GPIO_Port, Acq_Busy_Pin, GPIO_PIN_SET);  // Tell RASPI we are busy
	printf("%s Uploaded\r\n",filename_buffer);
	f_close(&SDFile);

}

void delete_file_handler(){

	int filename_byte_length;

	// Now read the number of bytes needed for the filename
	HAL_GPIO_WritePin(Acq_Busy_GPIO_Port, Acq_Busy_Pin, GPIO_PIN_RESET);  // Tell the RASPI we are Ready for the filename
	HAL_SPI_Receive(&hspi6, (uint8_t *)SPI_buffer, 2, 2000);
	HAL_GPIO_WritePin(Acq_Busy_GPIO_Port, Acq_Busy_Pin, GPIO_PIN_SET);  // Tell the RASPI we are Busy
	filename_byte_length = (SPI_buffer[0]<<8) + (SPI_buffer[1]);

	// now read the filename
	HAL_GPIO_WritePin(Acq_Busy_GPIO_Port, Acq_Busy_Pin, GPIO_PIN_RESET);  // Tell the RASPI we are Ready for the filename
	HAL_SPI_Receive(&hspi6, (uint8_t *)filename_buffer, filename_byte_length, 2000);
	HAL_GPIO_WritePin(Acq_Busy_GPIO_Port, Acq_Busy_Pin, GPIO_PIN_SET);  // Tell the RASPI we are Busy

	printf("deleting file %s  %d\r\n", filename_buffer,filename_byte_length);
	if(f_unlink(filename_buffer) != FR_OK)  				//delete file
	{
		printf("FAIL: Cannot delete file\r\n");
	}
	else{
		printf("%s Deleted\r\n",filename_buffer);
	}
}


void recording_request_handler2(){

	millisecs_to_record = (SPI_buffer[4]<<8)+SPI_buffer[5];
	sampling_frequency_kHz = ((SPI_buffer[1]<<8)+SPI_buffer[2]);

	int filename_byte_length;

	// Now read the number of bytes needed for the filename
	HAL_GPIO_WritePin(Acq_Busy_GPIO_Port, Acq_Busy_Pin, GPIO_PIN_RESET);  // Tell the RASPI we are Ready for the filename
	HAL_SPI_Receive(&hspi6, (uint8_t *)SPI_buffer, 2, 100);
	HAL_GPIO_WritePin(Acq_Busy_GPIO_Port, Acq_Busy_Pin, GPIO_PIN_SET);  // Tell the RASPI we are Busy
	filename_byte_length = (SPI_buffer[0]<<8) + (SPI_buffer[1]);

	// now read the filename
	HAL_GPIO_WritePin(Acq_Busy_GPIO_Port, Acq_Busy_Pin, GPIO_PIN_RESET);  // Tell the RASPI we are Ready for the filename
	HAL_SPI_Receive(&hspi6, (uint8_t *)filename_buffer, filename_byte_length, 1000);
	HAL_GPIO_WritePin(Acq_Busy_GPIO_Port, Acq_Busy_Pin, GPIO_PIN_SET);  // Tell the RASPI we are Busy

	printf("Record: Sampling %3dkHz  Gain %1d   Duration %lumS  %s \r\n", sampling_frequency_kHz, SPI_buffer[3], millisecs_to_record,filename_buffer);

	if(f_open(&SDFile, filename_buffer, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)  //Open file for writing (Create)
	{
		printf("FAIL: Cannot open file for recording\r\n");
	}
	else
	{
		write_wav_header(sampling_frequency_kHz*1000,sampling_frequency_kHz*millisecs_to_record); 							// Write WAV Header Sampling at 786000 Hz
		for(adcbuf_index=0;adcbuf_index < ADCBUFLEN;adcbuf_index++) // Clear ADC Buffer
		{
			adc_buf[adcbuf_index]=0;
		}
		total_blocks_written=0;
		total_bytes_written=0;
		adc_lower_status = BUFFER_EMPTY;
		adc_upper_status = BUFFER_EMPTY;
		overrun_count=0;
		max_save_time_ms=0;
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
		set_ADC_clock_prescalar(sampling_frequency_kHz);
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADCBUFLEN); 	// Enable ADC DMA and add contents
		end_acq_ms=uwTick+millisecs_to_record;
		do{
			if(adc_lower_status == BUFFER_FULL)
			{
				start_time_ms=uwTick;
				f_write(&SDFile, &adc_buf[0], ADCBUFLEN, (void *)&bytes_written);
				ms_taken=uwTick-start_time_ms;
				if(ms_taken > max_save_time_ms){
					max_save_time_ms = ms_taken;
				}
				total_bytes_written = total_bytes_written+bytes_written;
				adc_lower_status = BUFFER_EMPTY;
				total_blocks_written++;
				if(ADC_overrun){
					HAL_ADC_Stop_DMA(&hadc1);
					f_write(&SDFile, &adc_buf[ADCBUFLEN/2], ADCBUFLEN, (void *)&bytes_written);
					total_bytes_written = total_bytes_written+bytes_written;
					adc_upper_status = BUFFER_EMPTY;
					total_blocks_written++;
					ADC_overrun=0;
					overrun_count++;
					HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADCBUFLEN); 	// Enable ADC DMA and add contents
				}
			}
			if(adc_upper_status == BUFFER_FULL)
			{
				start_time_ms=uwTick;
				f_write(&SDFile, &adc_buf[ADCBUFLEN/2], ADCBUFLEN, (void *)&bytes_written);
				ms_taken=uwTick-start_time_ms;
				if(ms_taken > max_save_time_ms){
					max_save_time_ms = ms_taken;
				}
				total_bytes_written = total_bytes_written+bytes_written;
				adc_upper_status = BUFFER_EMPTY;
				total_blocks_written++;
				if(ADC_overrun){
					HAL_ADC_Stop_DMA(&hadc1);
					f_write(&SDFile, &adc_buf[0], ADCBUFLEN, (void *)&bytes_written);
					total_bytes_written = total_bytes_written+bytes_written;
					adc_lower_status = BUFFER_EMPTY;
					total_blocks_written++;
					ADC_overrun=0;
					overrun_count++;
					HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADCBUFLEN); 	// Enable ADC DMA and add contents
				}
			}
		}
		while(uwTick< end_acq_ms);
		HAL_ADC_Stop_DMA(&hadc1);   								// Halt ADC DMA

		// Now update WAV file with data length
		f_lseek(&SDFile,0x28);
		f_write(&SDFile,&total_bytes_written, 4, (void *)&bytes_written);

		f_close(&SDFile);  											// Writing complete so close file
		printf("Total Blocks Written %lu  Total Bytes Written %lu MaxSaveTime %d  Overruns %d\r\n",total_blocks_written,total_bytes_written,max_save_time_ms,overrun_count);
		printf("Recording Complete\r\n");

		strcpy(fullfilename,"//");
		strcat(fullfilename,filename_buffer);
		HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);				/* Get the RTC current Date */
		HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);				/* Display time Format: hh:mm:ss */
		fno.fdate = (WORD)(((gDate.Year+20) * 512U) | gDate.Month * 32U | gDate.Date);
		fno.ftime = (WORD)(gTime.Hours * 2048U | gTime.Minutes * 32U | gTime.Seconds / 2U);
		result=f_utime(fullfilename, &fno);
	}
}

/*
 * Function to write header of WAV file
 */
/*Return 0 on success and -1 on failure*/
int write_wav_header(int32_t SampleRate,int32_t FrameCount)
{
	int ret=0;

	struct wavfile_header_s wav_header;
	int32_t subchunk2_size;
	int32_t chunk_size;

	//    size_t write_count;

	uint32_t bytes_written=0;

	FRESULT result; /* FatFs function common result code */

	subchunk2_size  = FrameCount * NUM_CHANNELS * BITS_PER_SAMPLE / 8;
	chunk_size      = 4 + (8 + SUBCHUNK1SIZE) + (8 + subchunk2_size);

	wav_header.ChunkID[0] = 'R';
	wav_header.ChunkID[1] = 'I';
	wav_header.ChunkID[2] = 'F';
	wav_header.ChunkID[3] = 'F';

	wav_header.ChunkSize = chunk_size;

	wav_header.Format[0] = 'W';
	wav_header.Format[1] = 'A';
	wav_header.Format[2] = 'V';
	wav_header.Format[3] = 'E';

	wav_header.Subchunk1ID[0] = 'f';
	wav_header.Subchunk1ID[1] = 'm';
	wav_header.Subchunk1ID[2] = 't';
	wav_header.Subchunk1ID[3] = ' ';

	wav_header.Subchunk1Size = SUBCHUNK1SIZE;
	wav_header.AudioFormat = AUDIO_FORMAT;
	wav_header.NumChannels = NUM_CHANNELS;
	wav_header.SampleRate = SampleRate;
	wav_header.ByteRate = SampleRate <<1;
	wav_header.BlockAlign = BLOCK_ALIGN;
	wav_header.BitsPerSample = BITS_PER_SAMPLE;

	wav_header.Subchunk2ID[0] = 'd';
	wav_header.Subchunk2ID[1] = 'a';
	wav_header.Subchunk2ID[2] = 't';
	wav_header.Subchunk2ID[3] = 'a';
	wav_header.Subchunk2Size = subchunk2_size;

	printf("chunk_size %ld %lx\r\n", chunk_size,chunk_size);
	printf("subchunk2_size %ld %lx\r\n", subchunk2_size,subchunk2_size);
	//Open file for writing (Create)
	result = f_write(&SDFile, &wav_header, sizeof(wavfile_header_t), (void *)&bytes_written);
	if((bytes_written == 0) || (result != FR_OK))
	{
		Error_Handler();
	}

	// Flush the file buffers
	result = f_sync(&SDFile);
	if(result != FR_OK)
	{
		Error_Handler();
	}
	return ret;
}

void set_ADC_clock_prescalar(int sampling_fr){
	ADC_MultiModeTypeDef multimode = {0};
	switch(sampling_fr){
	case 800:
		hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
		break;
	case 533:
		hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV6;
		break;
	case 400:
		hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
		break;
	case 320:
		hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV10;
		break;
	case 266:
		hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV12;
		break;
	case 200:
		hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV16;
		break;
	case 100:
		hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV32;
		break;
	case 50:
		hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV64;
		break;
	case 25:
		hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV128;
		break;
	case 12:
		hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV256;
		break;
	}
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
	{
		Error_Handler();
	}

}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
