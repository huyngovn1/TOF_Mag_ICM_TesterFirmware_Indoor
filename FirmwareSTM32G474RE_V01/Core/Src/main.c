/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ADDR_OVERFLASH  (0x08080000) //512k
#define IMU_HEADING (0)
#define AVG_TIME (4)
#define GY_NOISE (8)
#include <stdio.h>
#include <math.h>
#include "bmp3.h"
#include "QMC5883.h"
#include "ICM-42670.h"
#include "BSP.h"
#include "IMU.h"
#include "Rx.h"
#include "Gps.h"
#include "crsf.h"
#include "FC.h"
#include "AutoPilot.h"
#include <I2CDev.h>
#include "VL53L1X_api.h"
#include "vl53l1_platform.h"
#include "AutoPilotInput.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
__IO int IMU_Status,TestVal;
extern float gyroOffsetFist[3],accel_offsetZ;
extern float  Angle[];
extern float sinHMC,cosHMC;
extern Fc_t Fc;
__IO Imu_t Imu;
int16_t  TempRaw,TempAvg;
int16_t GyroAvg[3], AcellAvg[3];
int32_t AcellSum[3], GyroSum[3], AcellSum2[3], GyroSum2[3],TempSum;
uint8_t  ImuData[16];
uint8_t RxBuffer1[64];
uint8_t Calib=0;
uint8_t Rx1Buffer[64];
uint8_t Rx2Buffer[64];
uint8_t Rx3Buffer[256];
uint8_t Rx4Buffer[64];
uint8_t Rx5Buffer[64];

uint8_t Tx1Buffer[256];
uint8_t Tx2Buffer[256];
uint8_t Tx3Buffer[256];
uint8_t Tx4Buffer[256];
uint8_t Tx5Buffer[256];

uint8_t RxUpdated=0;

uint32_t  RxUpdatedCount;
__IO uint16_t ThroNow;
extern int16_t X_CON,Y_CON,Z_CON;
extern uint8_t Armed;
extern float sinangle0,sinangle1,cosangle0,cosangle1,cosangle01;
extern float AngleFlybar[];
UBX_NAV_PVT_t UBX_NAV_PVT;
extern FcConfig_t FcConfig;
Rx_t Rx;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
__IO struct bmp3_dev dev;
__IO struct bmp3_data bmp3_comp_data;
int32_t AltiMmSum=0;
#define TOF_COUNT              5
#define VL53_DEFAULT_ADDR      0x52

#define TOF_READ_SLOT_MS       10

#define TOF_ADDR_1             0x54
#define TOF_ADDR_2             0x56
#define TOF_ADDR_3             0x58
#define TOF_ADDR_4             0x5A
#define TOF_ADDR_5             0x5C
//VL53L0X TofU, TofF;
//I2C_HandleTypeDef I2C;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;
I2C_HandleTypeDef hi2c4;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_uart5_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
typedef struct
{
    I2C_HandleTypeDef *i2c;

    GPIO_TypeDef *xshut_port;
    uint16_t xshut_pin;

    uint16_t addr;

    volatile uint16_t distance_mm;
    volatile uint8_t range_status;
    volatile uint8_t read_status;
    volatile uint8_t online;
} TOF_Sensor_t;

TOF_Sensor_t tof[TOF_COUNT] =
{
    {
        &hi2c1,
        XSHUT1_GPIO_Port,
        XSHUT1_Pin,
        TOF_ADDR_1,
        0, 0, 0, 0
    },

    {
        &hi2c2,
        XSHUT2_GPIO_Port,
        XSHUT2_Pin,
        TOF_ADDR_2,
        0, 0, 0, 0
    },

    {
        &hi2c2,
        XSHUT2B_GPIO_Port,
        XSHUT2B_Pin,
        TOF_ADDR_3,
        0, 0, 0, 0
    },

    {
        &hi2c3,
        XSHUT3_GPIO_Port,
        XSHUT3_Pin,
        TOF_ADDR_4,
        0, 0, 0, 0
    },

    {
        &hi2c3,
        XSHUT3B_GPIO_Port,
        XSHUT3B_Pin,
        TOF_ADDR_5,
        0, 0, 0, 0
    }
};

volatile uint8_t tof_init_error = 0;
volatile uint8_t tof_online_count = 0;
volatile uint8_t tof_current_index = 2;

uint32_t tof_read_tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_I2C4_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
__IO uint32_t CountTest,CountError,CountTestIT,CountTestIT2;
__IO TOF_t TOF;
int16_t MagRaw[3];
uint16_t TestVal16;
uint8_t GyroCalibrationDone=0;
extern uint32_t RxTimOut;
extern float AngleFlybar[];
extern float VX_Accel,VY_Accel;
uint8_t *UBX_PtrUpdate;
extern GPS_t GPS;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int8_t rslt = BMP3_OK;

/* Sensor_0 interface over SPI with native chip select line */
uint8_t dev_addr = 0;
MTF_t MTF;

void user_delay_us(uint32_t period, void *intf_ptr)
{    __IO uint32_t delay=period*16;
     for (uint32_t i=0;i<delay;i++);
}

uint8_t spi2Transfer(SPI_TypeDef *SPIx, uint8_t data)
{
   // 1. Chờ cho đến khi bộ đệm truyền trống (TXE = 1)
   while( !(SPIx->SR & SPI_SR_TXE) );

   // 2. Ép kiểu ghi 8-bit vào DR để kích hoạt bộ phát (Tránh kích hoạt nhầm 16-bit FIFO)
   *(__IO uint8_t *)&SPIx->DR = data;

   // 3. Chờ cho đến khi nhận đủ 1 byte dữ liệu về (RXNE = 1)
   // Phải đợi RXNE trước khi đợi BSY!
   while( !(SPIx->SR & SPI_SR_RXNE) );

   // 4. Đọc dữ liệu ra từ DR (Thao tác này tự động xóa cờ RXNE)
   return (*(__IO uint8_t *)&SPIx->DR);
}

int8_t  user_spi_read(uint8_t reg_addr, uint8_t *read_data, uint32_t len, void *intf_ptr)
{
    delay(15);
	BMP_CS0;
    delay(15);

    spi2Transfer(SPI2, reg_addr | 0x80);
	for( uint16_t i=0;i<len;i++) read_data[i] = spi2Transfer(SPI2, 0x00);
	BMP_CS1;
    return 0;
}


int8_t  user_spi_write(uint8_t reg_addr, const uint8_t *read_data, uint32_t len,
                                                void *intf_ptr)
{

	    delay(15);
		BMP_CS0;
         delay(15);
        spi2Transfer(SPI2, reg_addr);          // Device Reset
		for( uint16_t i=0;i<len;i++) spi2Transfer(SPI2, read_data[i]);
		BMP_CS1;
    return 0;
}

//float getAltitude(double press, double temp) {
//  return ((pow((1013.25 / press), 1/5.257) - 1.0) * (temp + 273.15)) / 0.0065;
//}


uint32_t PageErrorFlash = 0;
void WriteFLASH_all_Parameters(void){
    FLASH_EraseInitTypeDef EraseInitStruct;
        // ----------------------------------------------------------------------
        // PHẦN 1: NGỪNG TOÀN BỘ HỆ THỐNG
        // ----------------------------------------------------------------------
    __disable_irq();

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
    HAL_FLASH_Unlock();
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Page      = 127;
    EraseInitStruct.NbPages   = 1;
    EraseInitStruct.Banks     = FLASH_BANK_1 ;
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageErrorFlash) != HAL_OK) {
        //CountError = HAL_FLASH_GetError();
    	while(1);
    }

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,ADDR_OVERFLASH-8 ,0x01) != HAL_OK) {
        	PageErrorFlash = HAL_FLASH_GetError();
    }

    HAL_FLASH_Lock();
    __enable_irq();
}
static void TOF_All_XSHUT_Off(void)
{
    for (uint8_t i = 0; i < TOF_COUNT; i++)
    {
        HAL_GPIO_WritePin(
            tof[i].xshut_port,
            tof[i].xshut_pin,
            GPIO_PIN_RESET
        );
    }

    HAL_Delay(50);
}

static uint8_t TOF_Init_Fail(uint8_t i, uint8_t error)
{
    tof[i].online = 0;
    tof[i].distance_mm = 0;
    tof[i].read_status = error;

    HAL_GPIO_WritePin(
        tof[i].xshut_port,
        tof[i].xshut_pin,
        GPIO_PIN_RESET
    );

    return error;
}

static uint8_t TOF_Init_One(uint8_t i)
{
    uint8_t status;
    uint8_t boot_state = 0;
    uint8_t model_id = 0;
    uint32_t timeout_tick;

    /* Chọn I2C của cảm biến */
    VL53L1_SelectI2C(tof[i].i2c);

    /* Bật riêng cảm biến */
    HAL_GPIO_WritePin(
        tof[i].xshut_port,
        tof[i].xshut_pin,
        GPIO_PIN_SET
    );

    HAL_Delay(10);

    timeout_tick = HAL_GetTick();

    /* Chờ cảm biến khởi động */
    while (boot_state == 0)
    {
        status = VL53L1X_BootState(
            VL53_DEFAULT_ADDR,
            &boot_state
        );

        if (status != 0)
        {
            return TOF_Init_Fail(i, 10 + i);
        }

        if ((HAL_GetTick() - timeout_tick) > 1000)
        {
            return TOF_Init_Fail(i, 20 + i);
        }

        HAL_Delay(2);
    }

    /* Kiểm tra Model ID */
    status = VL53L1_RdByte(
        VL53_DEFAULT_ADDR,
        VL53L1__MODELID_INDEX,
        &model_id
    );

    if ((status != 0) ||
        (model_id != VL53L1__MODELID_VALUE))
    {
        return TOF_Init_Fail(i, 30 + i);
    }

    status = VL53L1X_SensorInit(VL53_DEFAULT_ADDR);

    if (status != 0)
    {
        return TOF_Init_Fail(i, 40 + i);
    }

    status = VL53L1X_SetDistanceMode(
        VL53_DEFAULT_ADDR,
        2
    );

    if (status != 0)
    {
        return TOF_Init_Fail(i, 50 + i);
    }

    status = VL53L1X_SetTimingBudgetInMs(
        VL53_DEFAULT_ADDR,
        50
    );

    if (status != 0)
    {
        return TOF_Init_Fail(i, 60 + i);
    }

    status = VL53L1X_SetInterMeasurementInMs(
        VL53_DEFAULT_ADDR,
        60
    );

    if (status != 0)
    {
        return TOF_Init_Fail(i, 70 + i);
    }

    /* Đổi sang địa chỉ riêng */
    status = VL53L1X_SetI2CAddress(
        VL53_DEFAULT_ADDR,
        tof[i].addr
    );

    if (status != 0)
    {
        return TOF_Init_Fail(i, 80 + i);
    }

    HAL_Delay(5);

    status = VL53L1X_StartRanging(tof[i].addr);

    if (status != 0)
    {
        return TOF_Init_Fail(i, 90 + i);
    }

    tof[i].online = 1;
    tof[i].read_status = 0;
    tof[i].distance_mm = 0;

    return 0;
}

static uint8_t TOF_Init_All(void)
{
    uint8_t first_error = 0;

    tof_online_count = 0;
    tof_current_index = 0;

    TOF_All_XSHUT_Off();

    for (uint8_t i = 1; i < TOF_COUNT; i++)
    {
        tof[i].online = 0;
        tof[i].distance_mm = 0;
        tof[i].range_status = 0;
        tof[i].read_status = 0;

        uint8_t error = TOF_Init_One(i);

        if (error == 0)
        {
            tof_online_count++;
        }
        else if (first_error == 0)
        {
            first_error = error;
        }
    }

    return first_error;
}

static void TOF_Read_One_By_One(void)
{
    uint8_t ready = 0;
    uint8_t range_status = 0;
    uint16_t distance = 0;

    uint8_t i = tof_current_index;

    if ((HAL_GetTick() - tof_read_tick) < TOF_READ_SLOT_MS)
    {
        return;
    }

    tof_read_tick = HAL_GetTick();

    if (tof[i].online == 1)
    {
        VL53L1_SelectI2C(tof[i].i2c);

        tof[i].read_status =
            VL53L1X_CheckForDataReady(
                tof[i].addr,
                &ready
            );

        if ((tof[i].read_status == 0) &&
            (ready != 0))
        {
            tof[i].read_status =
                VL53L1X_GetRangeStatus(
                    tof[i].addr,
                    &range_status
                );

            tof[i].range_status = range_status;

            if ((tof[i].read_status == 0) &&
                (range_status <= 2))
            {
                tof[i].read_status =
                    VL53L1X_GetDistance(
                        tof[i].addr,
                        &distance
                    );

                if (tof[i].read_status == 0)
                {
                    tof[i].distance_mm = distance;
                }
            }

            VL53L1X_ClearInterrupt(tof[i].addr);
        }
    }

    if (++tof_current_index >= TOF_COUNT)
    {
        tof_current_index = 1;
    }
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USB_Device_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_I2C4_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET);

  IMU_Status=1;
  WriteFLASH_all_Parameters();
	__enable_irq();

	M10N_GPS_Config(&huart3);

  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)Rx1Buffer, 64);
  __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);

  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)Rx2Buffer, 64);
  __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);

  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *)Rx3Buffer, 256);
  __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);

  HAL_UARTEx_ReceiveToIdle_DMA(&huart4, (uint8_t *)Rx4Buffer, 64);
  __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);

  HAL_UARTEx_ReceiveToIdle_DMA(&huart5, (uint8_t *)Rx5Buffer, 64);
  __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);


  // 1. Bật Channel 1 của TIM1
  TIM1->CCR1=4400;
  TIM1->CCR2=4400;
  TIM1->CCR3=4400;
  TIM1->CCR4=4400;
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH4);
  LL_TIM_EnableAllOutputs(TIM1);
  LL_TIM_EnableCounter(TIM1);

  HAL_Delay(500);

  tof_init_error = TOF_Init_All();
  IMU_Status=2;
 // M8N_GPS_Config();

  // Remove: tof[0],tof[1],

/*  TIM1->CCR4=4560;   HAL_Delay(1000);
  TIM1->CCR3=4560;   HAL_Delay(1000);
  TIM1->CCR1=4560;   HAL_Delay(1000);
  TIM1->CCR2=4560;   HAL_Delay(1000);*/

  IMU_Status=3;
  QMC5883_Init();
//  SPI1->CR1 |= SPI_CR1_SPE;
   IMU_Status=4;

   HAL_Delay(200);
   //HAL_GPIO_WritePin(XSHUT1_GPIO_Port, XSHUT1_Pin,GPIO_PIN_SET);
 //  HAL_GPIO_WritePin(XSHUT2_GPIO_Port, XSHUT2_Pin,GPIO_PIN_SET);
 //  HAL_Delay(24);

//   I2C=hi2c2;
//   setup_VL53L0X(&TofU); //Set up the VL53L0X struct with default address and timeout status.
////   //changeAddresses(); //Reset the sensor and initialise it. Then change its address to something unique from other sensors.
////
//   init(&TofU,true); //attempt to initialise it with the necessary settings for normal operation. Returns 0 if fail, 1 if success.
////
//   setTimeout(&TofU,500);
////
//   setVcselPulsePeriod(&TofU,VcselPeriodPreRange, 14);    //  VcselPeriodPreRange:  12 to 18 (initialized default: 14)
//   setVcselPulsePeriod(&TofU,VcselPeriodFinalRange, 10);  //  VcselPeriodFinalRange: 8 to 14 (initialized default: 10)
////
//   startContinuous(&TofU,0); //start the sensor in continuous back-to-back reading mode.

 //  #if defined LONG_RANGE
 //  // lower the return signal rate limit (default is 0.25 MCPS)
 //   setSignalRateLimit(&TofU,0.25);
 //  #endif
 //
 //  #if defined HIGH_SPEED
 //   // reduce timing budget to 20 ms (default is about 33 ms)
 //   setMeasurementTimingBudget(&TofU,20000);
 //  #elif defined HIGH_ACCURACY
 //          // increase timing budget to 200 ms
 //   setMeasurementTimingBudget(&TofU,200000);
 //  #endif


   IMU_Status=5;
   LL_SPI_Enable(SPI2);
	dev.intf_ptr = &dev_addr;
	dev.intf = BMP3_SPI_INTF;
	dev.read = &user_spi_read;
	dev.write = &user_spi_write;
	dev.delay_us = &user_delay_us;
	dev.dummy_byte = 1;
	//
	//  dev.chip_id = cs_pin;
	//  dev.intf_ptr = spi_dev;


	rslt=bmp3_init(&dev);


	if (rslt != BMP3_OK)
	{
	//sendUsart1("BMP_Error\n");
		IMU_Status=0;
	}
	else 	IMU_Status=6;
	//sprintf(temUsart, "BMP_OK: %d\n", dev.chip_id); sendUsart1(temUsart);

	// Configure settings for drone use case as per datasheet
	dev.settings.press_en = BMP3_ENABLE;
	dev.settings.temp_en = BMP3_ENABLE;
	dev.settings.odr_filter.press_os = BMP3_OVERSAMPLING_8X;
	dev.settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
	dev.settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_1;
	dev.settings.odr_filter.odr = BMP3_ODR_50_HZ;
	bmp3_set_sensor_settings(BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_IIR_FILTER | BMP3_SEL_ODR, &dev);

	// Set the power mode to normal mode
	dev.settings.op_mode = BMP3_MODE_NORMAL;
	bmp3_set_op_mode(&dev);


    IMU_Status=7;
    ICM42670_Init();
    IMU_Status=8;
    RxTimOut=0;
    	  TIM1->CCR1=4400; //M1
    	  TIM1->CCR2=4400; //M2
    	  TIM1->CCR4=4400; //M3
    	  TIM1->CCR3=4400; //M4

/*    		while(1){
    			HAL_Delay(1000);
    	    	  TIM1->CCR1=4700;
      			HAL_Delay(1000);
      	    	  TIM1->CCR2=4700;

      			HAL_Delay(1000);
      	    	  TIM1->CCR4=4700;
      			HAL_Delay(1000);
      	    	  TIM1->CCR3=4700;

    		}*/

	while(1){
				static uint8_t MotorTestDone = 0;

				if (MotorTestDone == 0)
				{
				    MotorTestDone = 1;

				    // Ga thap nhat - cho ESC arm
				    TIM1->CCR1 = 4000;
				    TIM1->CCR2 = 4000;
				    TIM1->CCR3 = 4000;
				    TIM1->CCR4 = 4000;
				    HAL_Delay(3000);

				    // M1 quay 1 giay
				    TIM1->CCR1 = 4600;
				    HAL_Delay(1000);
				    TIM1->CCR1 = 4000;
				    HAL_Delay(300);

				    // M2 quay 1 giay
				    TIM1->CCR2 = 4600;
				    HAL_Delay(1000);
				    TIM1->CCR2 = 4000;
				    HAL_Delay(300);

				    // M3 quay 1 giay
				    TIM1->CCR4 = 4600;
				    HAL_Delay(1000);
				    TIM1->CCR4 = 4000;
				    HAL_Delay(300);

				    // M4 quay 1 giay
				    TIM1->CCR3 = 4600;
				    HAL_Delay(1000);
				    TIM1->CCR3 = 4000;
				    HAL_Delay(500);

				    // 4 MOTOR TANG GA DAN
				    for (uint16_t pwm = 4600; pwm <= 5300; pwm += 20)
				    {
				        TIM1->CCR1 = pwm;
				        TIM1->CCR2 = pwm;
				        TIM1->CCR3 = pwm;
				        TIM1->CCR4 = pwm;

				        HAL_Delay(30);
				    }

				    // Giu ga cao 0.5 giay
				    HAL_Delay(5000);

				    // 4 MOTOR HA GA DAN
				    for (int32_t pwm = 5300; pwm >= 4000; pwm -= 20)
				    {
				        TIM1->CCR1 = pwm;
				        TIM1->CCR2 = pwm;
				        TIM1->CCR3 = pwm;
				        TIM1->CCR4 = pwm;

				        HAL_Delay(30);
				    }

				    // DUNG CA 4 MOTOR
				    TIM1->CCR1 = 4000;
				    TIM1->CCR2 = 4000;
				    TIM1->CCR3 = 4000;
				    TIM1->CCR4 = 4000;
				}

	    // Dieu khien 4 dong co
		  if( SPI_ByteRead(DATA_RDY_INT)){ //1600Hz
			      SPI_BytesRead(0x09,16,ImuData);

			      Imu.Temp= (ImuData[0]<<8)|ImuData[1];

	              int16_t AcellRaw[3],GyroRaw[3];

		          AcellRaw[0]= (ImuData[2]<<8)|ImuData[3];
				  AcellRaw[1]= (ImuData[4]<<8)|ImuData[5];
				  AcellRaw[2]= (ImuData[6]<<8)|ImuData[7];

			      GyroRaw[0]= (ImuData[8]<<8)|ImuData[9];
				  GyroRaw[1]= (ImuData[10]<<8)|ImuData[11];
				  GyroRaw[2]= (ImuData[12]<<8)|ImuData[13];

				  Imu.Accel[0]=0-AcellRaw[1];
				  Imu.Accel[1]=0- AcellRaw[0];
				  Imu.Accel[2]= 0-AcellRaw[2];

				  Imu.Gyro[0]= -GyroRaw[1];
				  Imu.Gyro[1]= -GyroRaw[0];
			      Imu.Gyro[2]= -GyroRaw[2];
		  }
		HAL_Delay(20);
		IMU_Status++;
		RxTimOut=0;
	    TOF_Read_One_By_One();
	    HAL_Delay(20);

		  					 QMC5883_Get_Values(MagRaw); //50Hz
		  				     bmp3_get_sensor_data(BMP3_ALL, &bmp3_comp_data, &dev); //10Hz
		 				     AltiMmSum= (int32_t)(getAltitude(bmp3_comp_data.pressure/100.,
		 				     bmp3_comp_data.temperature/ 100.)*1000.0f) ;


	      //  XyPosition3Sensors2(TOF.distance,&VX_Accel,&VY_Accel);
	}


  //---------------------------Waiting Trasmitter--------------------------

	while(1){  //Waiting Rx
	  HAL_Delay(1);
	  if(RxUpdated) {
		   if(RxUpdateChannel (&RxUpdated, Rx2Buffer,Rx.ChannelValue )){
			   RxTimOut=0;
				  if((Rx.ChannelValue[1]>(RX_CENTRE+400))|| (Rx.ChannelValue[2]>(RX_CENTRE+300))|| (Rx.ChannelValue[1]<(RX_CENTRE-300))|| (Rx.ChannelValue[2]<(RX_CENTRE-300))|| (Rx.ChannelValue[4]>(RX_CENTRE+300))|| (Rx.ChannelValue[4]<(RX_CENTRE-300)))RxUpdatedCount=0;
				  else if(++RxUpdatedCount>=100){
					 Rx.ChannelCenter[1]=Rx.ChannelValue[1];	// Ail
					 Rx.ChannelCenter[2]=Rx.ChannelValue[2];	   	// Ele
					 Rx.ChannelCenter[4]=Rx.ChannelValue[4];			// Rud
					 break;
				  }
			}

	  }
	}

	  IMU_Status=9;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  static uint8_t ImuUpdated=0,MagUpdate=0,BaroUpdate=0;
	   static uint32_t LowTime=0;
	  if(RxUpdated) {
			  if(RxUpdateChannel (&RxUpdated, Rx2Buffer,Rx.ChannelValue )){
				    RxTimOut=0;
			           Rx.TxMove[0]=Rx.ChannelValue[1]-Rx.ChannelCenter[1];
			           Rx.TxMove[1]=Rx.ChannelValue[2]-Rx.ChannelCenter[2];
			           Rx.TxMove[2]=Rx.ChannelCenter[4]-Rx.ChannelValue[4];

					if(GyroCalibrationDone) {
					   if(Rx.TxMove[2]<-1200 && Rx.ChannelValue[3]<5000&&Rx.TxMove[0]<-1200&&Rx.TxMove[1]<-1200){
						  // if(Armed==2)Armed=0;
						  // else
							  //IMU_Status++;
						   if(Armed==0)Armed=1;
						   LowTime=0;
					   }
					   if(Armed==1&&Rx.TxMoveDelay[0]>-100 && Rx.TxMoveDelay[0]<100&&Rx.TxMoveDelay[1]<100&&Rx.TxMoveDelay[1]>-100&&Rx.TxMoveDelay[2]<100&&Rx.TxMoveDelay[2]>-100){
						   Armed=2;
						   AngleFlybar[0]=0;
						   AngleFlybar[1]=0;
						   AngleFlybar[2]=0;
						   Rx.TxMoveDelay[0]=0;
						   Rx.TxMoveDelay[1]=0;
						   Rx.TxMoveDelay[2]=0;
					   }
				   }
					  RxUpdatedCount++;
		  }

	  }

	  if( SPI_ByteRead(DATA_RDY_INT)){ //1600Hz
		      SPI_BytesRead(0x09,16,ImuData);

			  TempRaw= (ImuData[0]<<8)|ImuData[1];
			  TempSum+=TempRaw;

              int16_t AcellRaw[3],GyroRaw[3];

	          AcellRaw[0]= (ImuData[2]<<8)|ImuData[3];
			  AcellRaw[1]= (ImuData[4]<<8)|ImuData[5];
			  AcellRaw[2]= (ImuData[6]<<8)|ImuData[7];

		      GyroRaw[0]= (ImuData[8]<<8)|ImuData[9];
			  GyroRaw[1]= (ImuData[10]<<8)|ImuData[11];
			  GyroRaw[2]= (ImuData[12]<<8)|ImuData[13];

			  Imu.Accel[0]=0-AcellRaw[1];
			  Imu.Accel[1]=0- AcellRaw[0];
			  Imu.Accel[2]= 0-AcellRaw[2];

			  Imu.Gyro[0]= -GyroRaw[1];
			  Imu.Gyro[1]= -GyroRaw[0];
		      Imu.Gyro[2]= -GyroRaw[2];

		      AcellSum[0]+= Imu.Accel[0];
		      AcellSum[1]+= Imu.Accel[1];
		      AcellSum[2]+= Imu.Accel[2];

		      GyroSum[0]+= Imu.Gyro[0];
		      GyroSum[1]+= Imu.Gyro[1];
		      GyroSum[2]+= Imu.Gyro[2];

		      static uint32_t countImu=0;
		  			   if(++countImu==AVG_TIME){ //AVG_TIME 400Hz
		  				   TempAvg=(TempSum/AVG_TIME);
		  				   AcellAvg[0]=AcellSum[0]/AVG_TIME;
		  				   AcellAvg[1]=AcellSum[1]/AVG_TIME;
		  				   AcellAvg[2]=AcellSum[2]/AVG_TIME;

		  			       GyroAvg[0]=0-GyroSum[0]/AVG_TIME;
		  				   GyroAvg[1]=0-GyroSum[1]/AVG_TIME;
		  				   GyroAvg[2]=GyroSum[2]/AVG_TIME;

		  				   TempSum=0;
		  				   AcellSum[0]=0;  AcellSum[1]=0;  AcellSum[2]=0;
		  				   GyroSum[0]=0;  GyroSum[1]=0;  GyroSum[2]=0;
		  				   countImu=0;
		  				   ImuUpdated=1;

		  				   if(GyroCalibrationDone){  // Ready To Fight
		  					  FlightControlLoop();
		  		             // AutoPilot(); //Only Have Plug GPS module
							  ThroNow=Rx.ChannelValue[3];

							  if(Armed<2){
								  ThroNow=4440;
							  }
							  else if(ThroNow<4600){
								  ThroNow=4600;
								  if(++LowTime==1200)Armed=0;// 3s
							  }
							  else LowTime=0;
							 // TIM1->CCR4=ThroNow;
							 // TIM1->CCR3=ThroNow;
							  //TIM1->CCR1=ThroNow;
							 // TIM1->CCR2=ThroNow;
			  		           Mixer(X_CON,Y_CON,Z_CON,ThroNow);  //AutoPitch
			  		          FlightControlLoop2();
		  				   }
		  				   else {
		  					   //GyroCalibrationDone= GyroOffsetLoopRaw(GyroAvg , AcellAvg,(float*)gyroOffsetFist,&accel_offsetZ);
		  				   }
		  			   } //--------End 400Hz------------------------

	  				   uint32_t Start=HAL_GetTick();
		  			   static uint32_t count1600Hz=0;
		  			   if(++count1600Hz==8){   //50Hz
		  				    TOF_Read_One_By_One();
		  				    TOF.distance[0]=tof[4].distance_mm;// Left Target
		  				    TOF.distance[1]=tof[3].distance_mm;  // Back Target
		  				    TOF.distance[2]=MTF.distance;
		  				    TOF.distance[3]=tof[2].distance_mm; // Right Target
		  				    TOF.distance[4]=tof[1].distance_mm;// Front Target
		  				    TOF.distance[5]=tof[0].distance_mm; // Up
		  				    TOF.Updated=1;
		  				    TestVal=HAL_GetTick()-Start;
		  			   }
		  			   else    if(count1600Hz==16){
		  				      MagUpdate=1;
		  					  QMC5883_Get_Values(MagRaw); //50Hz
		  				      TestVal=HAL_GetTick()-Start;
		  			   }
		  			   else    if(count1600Hz==32){
		  				     bmp3_get_sensor_data(BMP3_ALL, &bmp3_comp_data, &dev); //10Hz
		 				     AltiMmSum= (int32_t)(getAltitude(bmp3_comp_data.pressure/100.,
		 				     bmp3_comp_data.temperature/ 100.)*1000.0f) ;
		 				     BaroUpdate=1;
		 				     count1600Hz=0;
		  			   }

	        }  //End 1600Hz

	  if(UBX_PtrUpdate){ // GPS Updated
			    #ifdef   VISUAL_GPS
			      VisualGPSTest();
			      GPS.FrameRateMs=100;
			      GPS.TimeoutGPS=0;
			      GPS.Updated=1;
			      UBX_NAV_PVT.numSV=21;
			   #else

			      if (Parse_And_Update_PVT((uint8_t *)UBX_PtrUpdate, &UBX_NAV_PVT)) {
	//		    	  GPS.Speed=UBX_NAV_PVT.gSpeed*0.1f;
	//		    	  TestFloat= GPS.Speed;
	//		    	  CountTestIT++;

			    	  if(UBX_NAV_PVT.hAcc <100000&& UBX_NAV_PVT.vAcc <100000){
			    		  static int32_t LasthMSL=0;
			    		  int32_t Since=(int32_t)UBX_NAV_PVT.iTOW-(int32_t)GPS.TimeWeek;
			    		  if(Since<0)Since+=604800000;
			    		  GPS.TimeWeek=UBX_NAV_PVT.iTOW;
	                      if(Since>20 && Since<1500){
	                    	  GPS.FrameRateMs=Since;
							  UBX_NAV_PVT.headVeh=Fc.Heading*ONE_RAD*100000;

							  GPS.LatOld=GPS.Lat;
							  GPS.LonOld=GPS.Lon;

	                          GPS.Lon=UBX_NAV_PVT.lon+(int32_t)((float)FcConfig.GpsPos *sinHMC); // Dich Offet vi tri lap GPS
	                          GPS.Lat=UBX_NAV_PVT.lat+(int32_t)((float)FcConfig.GpsPos *cosHMC);

							  GPS.HDOP=UBX_NAV_PVT.hAcc;
							  GPS.VDOP=UBX_NAV_PVT.vAcc;
							  GPS.pDOP=UBX_NAV_PVT.pDOP;

						      int16_t CourseLast=GPS.Course;
							  GPS.Course=UBX_NAV_PVT.headMot/1000;
	                     	  int16_t dAngle=GPS.Course-CourseLast;
	                     	  if(dAngle>18000)dAngle-=36000;
	                     	  else if(dAngle<-18000)dAngle+=36000;

	                          if(dAngle<-6000 || dAngle>6000)GPS.Stopped=1;

			    		    GPS.vx=UBX_NAV_PVT.velE*0.1f;
			    		    GPS.vy=UBX_NAV_PVT.velN*0.1f;

			    		    GPS.vUp=(float)UBX_NAV_PVT.velD*-0.1f;

							if(GPS.vUp>800)GPS.vUp=800;
							else  if(GPS.vUp<-800)GPS.vUp=-800;

			    		    float dPosCm=(float)(UBX_NAV_PVT.hMSL-LasthMSL)*0.1f;
			    		    float vNoise=dPosCm*1000.0f/(float)GPS.FrameRateMs;

			    			if(cosangle01<0.9781f || NOTGOOD_GPS_SIGNAL||GPS.VDOP>600 ||(Rx.TxMoveDelay[3]==0&&((GPS.vUp>=0&&((vNoise>(GPS.vUp*1.3f+30)) || (vNoise<(GPS.vUp*0.75f-30)))) ||(GPS.vUp<0&&((vNoise<(GPS.vUp*1.3f-30)) || (vNoise>(GPS.vUp*0.75f+30))))))){
			    		        GPS.AltitudeDoppler+=GPS.vUp*(float)GPS.FrameRateMs/1000.0f;
			    			}
			    			else if(vNoise>-800 && vNoise<800){
			    				GPS.AltitudeDoppler+=dPosCm;
			    			} // else Error

			    			 GPS.AltitudeCm=(float)UBX_NAV_PVT.hMSL*0.1f;//-(float)FcConfig.GpsPos*sinangle1;
							 GPS.AltiKitCenter=GPS.AltitudeDoppler;//-(float)FcConfig.GpsPos*sinangle1;

							 //--------------Compensation Elevator Rate--------------
	//						 GPS.vUp-=Imu.GyroRate[1]*FcConfig.GpsPos;  // vUp center Kit

			    		     GPS.TimeoutGPS=0;
			    	         GPS.Updated=1;
	                      }
					      LasthMSL=UBX_NAV_PVT.hMSL;
			    	  } //else CountError++;
			      } //else CountError++;


			   #endif

	           UBX_PtrUpdate=NULL;
	      }

	  //if(CountTest>=44 && CountTest<=60) {
		//  LL_TIM_OC_SetCompareCH1(TIM1, IMU_Status*100);
		  //LL_TIM_OC_SetCompareCH2(TIM1, IMU_Status*100);
		  //LL_TIM_OC_SetCompareCH3(TIM1, IMU_Status*100);
		  //LL_TIM_OC_SetCompareCH4(TIM1, IMU_Status*100);
	  //}
//	  TestVal16=readRangeContinuousMillimeters(&TofU);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 42;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV8;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
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
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR_ADC1;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x60505F8C;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x60505F8C;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x60505F8C;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x60505F8C;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

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

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**SPI1 GPIO Configuration
  PA5   ------> SPI1_SCK
  PA6   ------> SPI1_MISO
  PA7   ------> SPI1_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV128;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 7;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_DisableNSSPulseMgt(SPI1);
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**SPI2 GPIO Configuration
  PB13   ------> SPI2_SCK
  PB14   ------> SPI2_MISO
  PB15   ------> SPI2_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_14;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV64;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 7;
  LL_SPI_Init(SPI2, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI2, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_DisableNSSPulseMgt(SPI2);
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
  LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  TIM_InitStruct.Prescaler = 41;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 9999;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM1);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 4400;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH4);
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH4);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
  LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM1);
  TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
  TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
  TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
  TIM_BDTRInitStruct.DeadTime = 0;
  TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
  TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
  TIM_BDTRInitStruct.BreakFilter = LL_TIM_BREAK_FILTER_FDIV1;
  TIM_BDTRInitStruct.BreakAFMode = LL_TIM_BREAK_AFMODE_INPUT;
  TIM_BDTRInitStruct.Break2State = LL_TIM_BREAK2_DISABLE;
  TIM_BDTRInitStruct.Break2Polarity = LL_TIM_BREAK2_POLARITY_HIGH;
  TIM_BDTRInitStruct.Break2Filter = LL_TIM_BREAK2_FILTER_FDIV1;
  TIM_BDTRInitStruct.Break2AFMode = LL_TIM_BREAK_AFMODE_INPUT;
  TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_ENABLE;
  LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  /**TIM1 GPIO Configuration
  PC0   ------> TIM1_CH1
  PC1   ------> TIM1_CH2
  PC2   ------> TIM1_CH3
  PC3   ------> TIM1_CH4
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  TIM_InitStruct.Prescaler = 41;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 12011;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM3);
  LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM3, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM3, LL_TIM_CHANNEL_CH3);
  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM3);
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**TIM3 GPIO Configuration
  PB0   ------> TIM3_CH3
  PB4   ------> TIM3_CH1
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 100000;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_2;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXINVERT_INIT;
  huart2.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  huart3.Init.BaudRate = 9600;
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
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  /* DMA2_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);
  /* DMA2_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);
  /* DMA2_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED1_Pin|LED2_Pin|XSHUT1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, XSHUT2_Pin|XSHUT2B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, XSHUT3_Pin|XSHUT3B_Pin|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_BMP_GPIO_Port, CS_BMP_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin XSHUT1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|XSHUT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : XSHUT2_Pin XSHUT2B_Pin */
  GPIO_InitStruct.Pin = XSHUT2_Pin|XSHUT2B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_CS_Pin */
  GPIO_InitStruct.Pin = IMU_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(IMU_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : XSHUT3_Pin XSHUT3B_Pin PB8 */
  GPIO_InitStruct.Pin = XSHUT3_Pin|XSHUT3B_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_BMP_Pin */
  GPIO_InitStruct.Pin = CS_BMP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CS_BMP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    CountError++; // Tăng tổng số lần dính lỗi UART

    // BƯỚC 1: Dừng triệt để DMA đang kẹt ngay lập tức
    HAL_UART_DMAStop(huart);

    // BƯỚC 2: Clear cờ lỗi phần cứng (ORE, NE, FE, PE) qua thanh ghi ICR & Đọc RDR xả đệm
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);

    volatile uint32_t temp = huart->Instance->RDR;
    (void)temp;

    // BƯỚC 3: Ép Reset sạch sẽ trạng thái phần mềm của HAL
    huart->RxState = HAL_UART_STATE_READY;
    huart->gState  = HAL_UART_STATE_READY;
    huart->Lock    = HAL_UNLOCKED;
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    // BƯỚC 4: TỰ ĐỘNG BẬT LẠI RECEIVER CHO BẤT KỲ CỔNG NÀO ĐANG CHẠY HALF-DUPLEX
    if ((huart->Instance->CR3 & USART_CR3_HDSEL) != 0U)
    {
        HAL_HalfDuplex_EnableReceiver(huart);
    }

    // BƯỚC 5: Tự động phân luồng Buffer và kích hoạt lại DMA RX IDLE
    uint8_t *pBuffer = NULL;
    uint16_t bufferSize = 0;

    if (huart->Instance == USART1)
    {
        pBuffer = Rx1Buffer;
        bufferSize = sizeof(Rx1Buffer);
    }
    else if (huart->Instance == USART2)
    {
        pBuffer = Rx2Buffer;
        bufferSize = sizeof(Rx2Buffer);
    }
    else if (huart->Instance == USART3)
    {
        pBuffer = Rx3Buffer;
        bufferSize = sizeof(Rx3Buffer);
    }
    else if (huart->Instance == UART4)
    {
        pBuffer = Rx4Buffer;
        bufferSize = sizeof(Rx4Buffer);
    }

    else if (huart->Instance == UART5)
    {
        pBuffer = Rx5Buffer;
        bufferSize = sizeof(Rx5Buffer);
    }

    // BƯỚC 6: Kích hoạt lại DMA nếu buffer hợp lệ
    if (pBuffer != NULL && bufferSize > 0)
    {
        if (HAL_UARTEx_ReceiveToIdle_DMA(huart, pBuffer, bufferSize) == HAL_OK)
        {
            if (huart->hdmarx != NULL)
            {
                __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
            }
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    // CHỈ XỬ LÝ KHI NGOẠI VI THỰC SỰ ĐANG CHẠY HẢI-DUPLEX (SRXL2 / SBUS-2 / Single-wire)
    // Nếu là Full-Duplex (CRSF), bit HDSEL = 0 -> Bỏ qua hoàn toàn, bộ RX nền vẫn đang tự chạy DMA.
    if ((huart->Instance->CR3 & USART_CR3_HDSEL) != 0U)
    {
        // 1. Chuyển đường truyền IO về lại chế độ Receiver
        HAL_HalfDuplex_EnableReceiver(huart);

        // 2. Ép Reset trạng thái phần mềm RX về READY (Chống dính cờ BUSY của HAL)
        huart->RxState = HAL_UART_STATE_READY;

        // 3. Tự động xác định Buffer và Kích hoạt lại DMA RX IDLE
        uint8_t *pBuffer = NULL;
        uint16_t bufferSize = 0;

        if (huart->Instance == USART1)
        {
            pBuffer = Rx1Buffer;
            bufferSize = sizeof(Rx1Buffer);
        }
        else if (huart->Instance == USART2)
        {
            pBuffer = Rx2Buffer;
            bufferSize = sizeof(Rx2Buffer);
        }
        else if (huart->Instance == USART3)
        {
            pBuffer = Rx3Buffer;
            bufferSize = sizeof(Rx3Buffer);
        }
        else if (huart->Instance == UART4)
        {
            pBuffer = Rx4Buffer;
            bufferSize = sizeof(Rx4Buffer);
        }

        else if (huart->Instance == UART5)
        {
            pBuffer = Rx5Buffer;
            bufferSize = sizeof(Rx5Buffer);
        }


        // 4. Kích hoạt DMA RX nếu Buffer hợp lệ
        if (pBuffer != NULL && bufferSize > 0)
        {
            if (HAL_UARTEx_ReceiveToIdle_DMA(huart, pBuffer, bufferSize) == HAL_OK)
            {
                if (huart->hdmarx != NULL)
                {
                    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
                }
            }
        }
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	 if(huart->Instance == USART1){
		 if(Rx1Buffer[0]==0 &&Size==18){

		 }
		 HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)Rx1Buffer, 64);
	     __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
	 }

	 if(huart->Instance == USART2){ // ELRS PORT
		 if(Size==25&&Rx2Buffer[0]==0x0F){ // SBUS
			  RxUpdated=2;
		 }
		 else if (Rx1Buffer[0] == CRSF_ADDRESS_RX &&Rx1Buffer[2] == CRSF_FRAMETYPE_RC_CHANNELS) {
			  RxUpdated=4;
		 }
		 HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)Rx2Buffer, 64);
	     __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
	 }

	 if(huart->Instance == USART3){ // GPS

		 if(Size==100 && Rx3Buffer[0]==0xB5 && Rx3Buffer[1]==0x62){// GPS data on Packet
					 UBX_PtrUpdate=Rx3Buffer;
					 CountTestIT++;
		 } //else CountError++;

		 HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *)Rx3Buffer, 256);
	     __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);

	 }

	 if(huart->Instance == UART4){

		 if(Rx4Buffer[0]==0xEF && Rx4Buffer[1]==0x0F&& Rx4Buffer[3]==0x51) {// MTF-02

		 }
		 else if(Size==32&& Rx4Buffer[0]==36 && Rx4Buffer[1]==88 && Rx4Buffer[2]==60 && Rx4Buffer[5]==31) {// MSP-V2
		     static float AngleLast[2];
			 MTF.time_ms=HAL_GetTick();
			 MTF.flow_quality=Rx4Buffer[8];
			 MTF.flow_vel_y = *(int16_t*)&Rx4Buffer[9];
			 MTF.flow_vel_x = *(int16_t*)&Rx4Buffer[13];

			 MTF.moved_x+=MTF.flow_vel_x;
			 MTF.moved_y+=MTF.flow_vel_y;
			 MTF.distance=Rx4Buffer[27] | (Rx4Buffer[28]<<8);
			 MTF.vx=MTF.flow_vel_x + 32*((float)MTF.distance*(Angle[0]-AngleLast[0]));

			 AngleLast[0]=Angle[0];
			 AngleLast[1]=Angle[1];

			 CountTestIT2++;

			 MTF.Updated=1;
		 }


		 HAL_UARTEx_ReceiveToIdle_DMA(&huart4, (uint8_t *)Rx4Buffer, 64);
	     __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);


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
