/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "defines.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
struct SignDescriptor {
	uint8_t structure;
	uint8_t type;
	uint32_t attribute;
	uint16_t unit;
	uint32_t period;
	uint8_t dimension;
	uint32_t coef;
};

uint8_t buf_rx[1040], buf_tx[1040];
uint8_t data[1040];
uint16_t count_rx;
uint8_t receivedByte;
uint16_t dataLength;
uint8_t deviceInfo[] = {'S', 'T', 'M', '3', '2', 'F', '4', '0', '5', 'R', 'G', 'T', '6', 0x00};
uint8_t selfTest[] = {'S', 'e', 'l', 'f', '-', 't', 'e', 's', 't', ' ', 'c', 'o', 'm', 'p', 'l', 'e', 't', 'e', 'd', ' ', 
's', 'u', 'c', 'c', 'e', 's', 's', 'f', 'u', 'l','l', 'y',0x00};
uint8_t firmwareName[] = {'F', 'i', 'r', 'm', 'w', 'a', 'r', 'e', '1', 0x00};
uint8_t scan_mode = 0;
uint8_t buf_write[WRITE_BLOCK_SIZE];

uint16_t num_signals[4], data_signals[4];

uint8_t s = 0;

uint8_t first = 1;

struct SignDescriptor Sign12 = {STRUCT_XXX, SIGN_TYPE_UINT, ATTRIBUTE_READING|ATTRIBUTE_TELEMETRY,   
	UNIT_0, PERIOD, DIM_ANG_DEGREE, COEFFICIENT};   // дескриптор сигналов

struct SignDescriptor Sign0 = {STRUCT_XXX, SIGN_TYPE_UINT, ATTRIBUTE_READING|ATTRIBUTE_TELEMETRY,
	UNIT_NONE, PERIOD, DIM_NONE, COEFFICIENT};   // дескриптор версии

uint16_t vers = 0;
uint16_t angle1 = 1, angle2 = 2;
uint32_t spi_value;
	uint32_t spi_value1;
	uint32_t spi_value2;
	uint8_t spi_buf[3];
	uint8_t spi_buf1[3];
	uint8_t spi_buf2[3];
	int count1;
	int count2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t Read_Data(SPI_HandleTypeDef *hspi, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{ 
	//uint8_t spi_buf[3]; // 3 байта данных от датчика
	//uint32_t spi_value; // 22-битное значение данных
	uint16_t angle;     // значение угла	
	
	while(__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE));       // wait RXNE=0
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);     // set RS1 or RS2
	HAL_SPI_Receive(hspi, spi_buf, 3, 100);               // reading data
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);   // reset RS1 or RS2
	uint32_t spi_value = (uint32_t)(spi_buf[0]) << 16|(uint32_t)spi_buf[1] << 8| spi_buf[2];
	if(spi_buf[0] & 0x80) // Position Valid Flag. Set to 1 when data is valid
	{
		//spi_value = (uint32_t)(spi_buf[0] & 0x3F) << 16|(uint32_t)spi_buf[1] << 8| spi_buf[2];
	}
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
	//for(int i = 0; i < 3; i++)                              // to clear buffer SPI
		//spi_buf[i]= hspi->Instance->DR;
	
	return spi_value;
}

uint32_t Read_Data1()
{ count1++;
	SPI_HandleTypeDef *hspi = &hspi1;
		for(int i = 0; i < 3; i++)                              // to clear buffer SPI
		spi_buf1[i]= hspi->Instance->DR;
	while(__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_RXNE));       // wait RXNE=0
	
	HAL_GPIO_WritePin(GPIOA, RS1_EN_Pin, GPIO_PIN_SET);     // set RS1 or RS2
	HAL_SPI_Receive(&hspi1, spi_buf1, 3, 100);               // reading data
	HAL_GPIO_WritePin(GPIOA, RS1_EN_Pin, GPIO_PIN_RESET);   // reset RS1 or RS2
	
	if(spi_buf1[0] & 0x80) // Position Valid Flag. Set to 1 when data is valid
	{
		spi_value1 = (uint32_t)(spi_buf1[0] & 0x3F) << 16|(uint32_t)spi_buf1[1] << 8| spi_buf1[2];
	}
	
	return spi_value1;
}

uint32_t Read_Data2()
{ count2++;
	SPI_HandleTypeDef *hspi = &hspi3;
		for(int i = 0; i < 3; i++)                              // to clear buffer SPI
		spi_buf2[i]= hspi->Instance->DR;
	while(__HAL_SPI_GET_FLAG(&hspi3, SPI_FLAG_RXNE));       // wait RXNE=0
	
	HAL_GPIO_WritePin(GPIOC, RS2_EN_Pin, GPIO_PIN_SET);     // set RS1 or RS2
	HAL_SPI_Receive(&hspi3, spi_buf2, 3, 100);               // reading data
	HAL_GPIO_WritePin(GPIOC, RS2_EN_Pin, GPIO_PIN_RESET);   // reset RS1 or RS2
	//uint32_t spi_value = (uint32_t)(spi_buf1[0] & 0x0F ) << 16|(uint32_t)spi_buf1[1] << 8| spi_buf1[2];
	if(spi_buf2[0] & 0x80) // Position Valid Flag. Set to 1 when data is valid
	{
		spi_value2 = (uint32_t)(spi_buf2[0] & 0x3F) << 16|(uint32_t)spi_buf2[1] << 8| spi_buf2[2];
	}
	
	return spi_value2;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		//angle1++;
		//angle2++;
		//angle1 = Read_Data(&hspi1, GPIOA, RS1_EN_Pin);
		//angle2 = Read_Data(&hspi3, GPIOC, RS2_EN_Pin);
	}
}

void angleChange()
{
	angle1++;
		angle2++;
}

uint8_t XOR_calc(uint8_t length)								//Checksum XOR
{
	uint8_t sum = 0;
	for(int j = 6; j < length; j++)
		sum ^= buf_tx[j];
  return sum;	
}

uint16_t CRC_calc(uint8_t *buf, uint16_t length)								//Checksum package
{
	uint32_t sum = 0;
	for(int k = 1; k < length; k += 2)
		sum += ((uint16_t)buf[k-1] << 8) | buf[k]; 
	while(sum >> 16)
		sum = (sum & 0xFFFF) + (sum >> 16);
	sum = (uint16_t)(~sum);
	return sum;
}

void WrapCreate(void)							//Creation of a wrapper 
{
	buf_tx[0] = STARTBYTE;
	buf_tx[1] = 0;
	if(dataLength % 2 == 0) buf_tx[3] &= ~0x80;		
	else
	{
		dataLength = dataLength + 1;
		buf_tx[dataLength+5] = 0;		
		buf_tx[3] |= 0x80;
		
	}
	buf_tx[2] = dataLength/2;
	buf_tx[3] |= (dataLength/2)/256;
	buf_tx[4] = CRC_calc(buf_tx, 4)/256;
	buf_tx[5] = CRC_calc(buf_tx, 4);
	buf_tx[6] = 0;
}

void CreatePacketTX(void)
{
	WrapCreate();
	
	if(buf_tx[3] & 0x80) 
	{
		for(int i = 0; i < dataLength - 3; i++)
			buf_tx[i+7] = data[i];
		buf_tx[dataLength+4]	= XOR_calc(dataLength+4);	
	}
	else 
	{
		for(int i = 0; i < dataLength - 2; i++)
			buf_tx[i+7] = data[i];
		buf_tx[dataLength+5]	= XOR_calc(dataLength+5);			
	}
	
	buf_tx[dataLength+6] = CRC_calc(buf_tx, dataLength+6)/256;
	buf_tx[dataLength+7] = CRC_calc(buf_tx, dataLength+6);
	
	HAL_GPIO_WritePin(GPIOA, RS_EN_Pin, GPIO_PIN_SET);
	for(int i = 0; i < dataLength + 8; i++)
		HAL_UART_Transmit(&huart1, &buf_tx[i], 1, 1000);
  HAL_GPIO_WritePin(GPIOA, RS_EN_Pin, GPIO_PIN_RESET);	
}

void ConvertData(int32_t dat, uint8_t type, uint8_t k)
{
	for(int i = 0; i < type; i+=8)
	{
		data[k] = dat >> i;
		k++;
	}
}



void CreateSignDescriptor(struct SignDescriptor signal, uint8_t *signName, uint8_t x)
{
//	for(int i = 0; i < sizeof(signal.name); i++)
//		data[i] = signal.name[i];	
	for(int i = 0; i < x; i++)
		data[i] = signName[i];	
	data[x] = signal.structure;	
  data[x+1] = signal.type;	
	ConvertData(signal.attribute, 32, x+2);	
	ConvertData(signal.unit, 16, x+6);	
	ConvertData(signal.period, 32, x+8);
	data[x+12] = signal.dimension;
	data[x+13] = 0x00;
	data[x+14] = 0x00;
	data[x+15] = 0x80;
	data[x+16] = 0x3F;
  //ConvertData(signal.coef, 32, x+12);
}

void CheckPacketRX(void)
{
	uint16_t cmd = (uint16_t)(buf_rx[6] << 8) | buf_rx[7];
	switch(cmd)
	{
		//--------------Служебные команды---------------
		case CMD_CHECK_CHANNEL: dataLength = 10;
														for(int i = 0; i < dataLength - 2; i++)
																data[i] = buf_rx[i+8]; break;			
		case CMD_RESET: dataLength = 2;	break;				
		case CMD_DEVICE_INFO: dataLength = sizeof(deviceInfo) + 2;
													for(int i = 0; i < sizeof(deviceInfo); i++) 
														data[i] = deviceInfo[i]; break;			
		case CMD_SCAN_MODE: dataLength = 2;
												if(buf_rx[8] == 1) scan_mode = 1;
												else scan_mode = 0;	break;		
		case CMD_SELF_TEST: dataLength = sizeof(selfTest) + 2;	
												for(int i = 0; i < sizeof(selfTest); i++)
													data[i] = selfTest[i]; break;						
		case CMD_SAVE_SETTING: dataLength = 2; break;							
		case CMD_TEST_CHANNEL: if(buf_rx[3] & 0x80)
													 {
														 for(int i = 0; i < dataLength - 4; i++)
															 data[i] = buf_rx[i+8];													
														 dataLength = dataLength - 2;	
													 }	
													 else	
													 {
														 for(int i = 0; i < dataLength - 3; i++)
															 data[i] = buf_rx[i+8];														
													 }	break;
													 
		//--------------Программирование---------------											 
  	case CMD_NUM_FIRMWARE: dataLength = 3; data[0] = 0; break;
		case CMD_PROG_DESCRIPTOR: dataLength = sizeof(firmwareName) + 19; 
													    for(int i = 0; i < sizeof(firmwareName); i++)
																data[i] = firmwareName[i];
													    ConvertData(WRITE_BLOCK_SIZE, 32, sizeof(firmwareName));
													    ConvertData(READ_BLOCK_SIZE, 32, sizeof(firmwareName)+4);
													    ConvertData(MAX_SIZE, 32, sizeof(firmwareName)+8);
															ConvertData(WRITE_TIMEOUT, 16, sizeof(firmwareName)+12);
													    ConvertData(PREP_TIMEOUT, 16, sizeof(firmwareName)+14);
													    data[sizeof(firmwareName)+16] = RESTART_DEVICE; break;
													    //ConvertData(VERSION_FIRMWARE, 32, sizeof(firmwareName)+17);	break;
		case CMD_READ_ID: dataLength = 6; ConvertData(ID, 32, 0); break;	
		case CMD_READING: dataLength = 2 + READ_BLOCK_SIZE; 
											for(int i = 0; i < READ_BLOCK_SIZE; i++)
												data[i] = buf_write[i];
													 break;	
    case CMD_ERASE: dataLength = 2; break;
    case CMD_WRITE: dataLength = 2; 
										for(int i = 0; i < WRITE_BLOCK_SIZE; i++)
											buf_write[i] = buf_rx[i+8]; 
													 break;	
    case CMD_END_PROG: dataLength = 2; break;		
    case CMD_RESTART_DEVICE: dataLength = 2; break;		
		case CMD_SET_POINTER: dataLength = 2; break;		
		//--------------Сигналы---------------	
		case CMD_NUM_SIGNALS: dataLength = 4; data[0] = N_SIGNALS; data[1] = N_SIGNALS >> 8 ; /*ConvertData(N_SIGNALS, 16, 0);*/ break;	
		case CMD_SIGN_DESCRIPTOR: if(buf_rx[8] == 0)
															{
																dataLength = 33; //sizeof(signal1Name) + 14;
																uint8_t signalName[] = {'V', 'e', 'r', 's', 'i', 'o', 'n', ' ', 'V', '1', '.', '0', '0',0x00};
																for(int i = 0; i < 14; i++)
																	data[i] = signalName[i];
																CreateSignDescriptor(Sign0, signalName, 14);	
															}
															else if(buf_rx[8] == 1)
															{
																dataLength = 28; //sizeof(signal1Name) + 14;
																uint8_t signalName[] = {'Z', 'e', 't', 't', 'l', 'e', 'x', '1', 0x00};
																for(int i = 0; i < 9; i++)
																	data[i] = signalName[i];
																CreateSignDescriptor(Sign12, signalName, 9);
															}
															else if(buf_rx[8] == 2)
															{
																dataLength = 28; 
																uint8_t signalName[] = {'Z', 'e', 't', 't', 'l', 'e', 'x', '2', 0x00};
																for(int i = 0; i < 9; i++)
																	data[i] = signalName[i];
																CreateSignDescriptor(Sign12, signalName, 9);	
															}
															 break;
	  case CMD_CONTROL: dataLength = 2; break;
	  case CMD_SIGNATURE: dataLength = 2; break;
		case CMD_READ_VALUE: dataLength = 4; 
												 if(buf_rx[8] == 0)
												 {
													 data[0] = vers; 
													 data[1] = vers >> 8; 
												 }
												 else if(buf_rx[8] == 1)
												 {
													 data[0] = angle1; 
													 data[1] = angle1 >> 8;
												 }
												 else if(buf_rx[8] == 2) 
												 {
													 data[0] = angle2; 
													 data[1] = angle2 >> 8;
												 }	break;
		//--------------Телеметрия---------------
		case CMD_TEL_DESCRIPTOR: dataLength = 14;
														 ConvertData(PERIOD_TEL, 32, 0); 													
														 ConvertData(MAX_SIGN_TEL, 16, 4); 
														 ConvertData(SIZE_FRAME, 16, 6); 
														 ConvertData(ATTRIBUTE_TEL, 32, 8); break;
		case CMD_TEL_MODE: dataLength = 2; 
											 if(first == 0)
											 {
												 if(buf_rx[8] == 0)
													 s = 1;
											 }
		case CMD_ADD_SIGNAL: dataLength = 2; if(s != 0) first = 0; num_signals[s++] = buf_rx[8]; break;												 
											 
		case CMD_READ_DATA: /*if(!scan_mode)
												{
													for(int i = 1; i < s; i++)
													{
														switch(num_signals[i])
														{
															case 0: data_signals[i-1] = vers; break;
															case 1: data_signals[i-1] = angle1; break;
															case 2: data_signals[i-1] = angle2; break;
														}
													}
													data[0] = N_ITER;
													data[1] = N_ITER/256;
													
													data[2] = STATUS_OK;
												
													for(int i = 0, j = 3; i < s-1; i++, j+=2)
													{
														data[j] = data_signals[i];
														data[j+1] = data_signals[i]/256;
													}									  
																								
													dataLength = 5+2*(s-1);	
                            													
												}
												else
												{
														data[0] = 0;
														data[1] = 0;
														
														data[2] = STATUS_OK;
																												
														dataLength = 5;
														
												}
													break;*/
													
													if(!scan_mode){
														angleChange();
														data[0] = N_ITER;
													  data[1] = N_ITER/256;
														data[2] = STATUS_OK;
														data[3] = angle1;
														data[4] = angle1/256;
														data[5] = angle2;
														data[6] = angle2/256;
														dataLength = 9;	
													}
													else
													{
														data[0] = 0;
														data[1] = 0;
														
														data[2] = STATUS_OK;
																												
														dataLength = 5;
													}
	}
	
	CreatePacketTX();
}

void UartHandling(void)
{
	if (buf_rx[0] == STARTBYTE && buf_rx[1] == DEVICEADDRESS)
	{	
		dataLength = (uint16_t)((buf_rx[3] & 0x0F) << 8) | buf_rx[2];
		dataLength *=2;
	}	
	else 
		count_rx=0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	buf_rx[count_rx] = receivedByte;
	count_rx++;
	if(count_rx == 4)
			UartHandling(); 
	else if (count_rx == dataLength + 8)
  {	
		count_rx = 0;
		if(CRC_calc(buf_rx, dataLength+8) == 0)
			CheckPacketRX();
  }
	HAL_UART_Receive_IT(&huart1, &receivedByte, 1);	 
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);  // on LED
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_UART_Receive_IT(&huart1, &receivedByte,1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//spi_value1 = Read_Data1();
		//HAL_Delay(1000);
		//spi_value2 = Read_Data2();
		//HAL_Delay(1000);
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 156;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 77;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 499;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_9B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_EVEN;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RS1_EN_Pin|RS_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RS2_EN_GPIO_Port, RS2_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : RS1_EN_Pin */
  GPIO_InitStruct.Pin = RS1_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(RS1_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : RS_EN_Pin */
  GPIO_InitStruct.Pin = RS_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(RS_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RS2_EN_Pin */
  GPIO_InitStruct.Pin = RS2_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(RS2_EN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
