/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
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
#include "dma.h"
#include "fatfs.h"
#include "sai.h"
#include "sdmmc.h"
#include "usart.h"
#include "gpio.h"
//
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "waverecorder.h"
#include <stdio.h>
#include "ai_platform.h"
#include "network.h"
#include "network_data.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct __attribute__((packed)) {
    char ChunkID[4];
    uint32_t ChunkSize;
    char Format[4];
    char Subchunk1ID[4];
    uint32_t Subchunk1Size;
    uint16_t AudioFormat;
    uint16_t NumChannels;
    uint32_t SampleRate;
    uint32_t ByteRate;
    uint16_t BlockAlign;
    uint16_t BitsPerSample;
    char Subchunk2ID[4];
    uint32_t Subchunk2Size;
} WAV_Header;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Audio recording RELATED VARIABLES */
AUDIO_ApplicationTypeDef appli_state = APPLICATION_IDLE;
AUDIO_PLAYBACK_StateTypeDef AudioState;
AUDIO_ErrorTypeDef  status;

/* SD card RELATED VARIABLES */
#define WORK_BUFFER_SIZE 512
BYTE workBuffer_init[WORK_BUFFER_SIZE];
extern FIL WavFile;
static AUDIO_OUT_BufferTypeDef BufferCtl;
FRESULT res;
UINT bytesRead;
WAV_Header header;

/* AI model RELATED VARIABLES */
ai_handle network;
float aiInData[AI_NETWORK_IN_1_SIZE];
float aiOutData[AI_NETWORK_OUT_1_SIZE];
ai_u8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];
const char* activities[AI_NETWORK_OUT_1_SIZE] = {
  "down", "go", "left", "right", "stop", "up"
};
ai_buffer * ai_input;
ai_buffer * ai_output;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


/* Needed to send messages easier to terminal for debugging */
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker-
>Libraries->Small printf
set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/* =========================================================*/

/* Needed to initialize the SD Card */
void SDCard_InitAndFormat(void);

/* Needed to create a txt in the SD Card */
void new_log(FIL *fp, const char *filename, const char *content);

/* Needed to create a folder in the SD Card */
void new_folder(const char *foldername);

void ReadWAVFileInfo(const char *filename);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint8_t button_pressed = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_SAI1_Init();
  MX_USART1_UART_Init();
  MX_FATFS_Init();
  MX_SDMMC1_SD_Init();
  /* USER CODE BEGIN 2 */

  /* We format the SD card */
  printf("SD card init...\r\n");
  SDCard_InitAndFormat();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  /* Standby : waiting for the button to be pressed to start recording */
	  check_button_release();
	  printf("Waiting for input to record...\r\n");
	  HAL_Delay(1000);

	  if (button_pressed == 1)
	  {
		  /* Toggle the green led to visually show action */
		  HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
		  HAL_Delay(100);
		  HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
		  HAL_Delay(100);

		  /* If the program is not already recording... */
		  if(AudioState == AUDIO_STATE_IDLE)
		  {
		    /* Configure the audio recorder: sampling frequency, bits-depth, number of channels */
		    AUDIO_REC_Start();
		  }

		  /* While recording, we loop the recording process */
		  while(AudioState == AUDIO_STATE_RECORD)
		  {
		    status = AUDIO_REC_Process();
		  }

		  /* Once we stop recording, we correctly close the .WAV */
		  if (AudioState == AUDIO_STATE_STOP)
		  {
			status = AUDIO_REC_Process();
		    printf("Recording stopped.\r\n");
		  }

		  ReadWAVFileInfo("WAVE.wav");
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* Needed to send messages easier to terminal for debugging */
PUTCHAR_PROTOTYPE
{
/* Place your implementation of fputc here */
/* e.g. write a character to the USART2 and Loop until the end
of transmission */
HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
return ch;
}
/* ======================================================== */


/* User button interruption and variable change */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);

  if (GPIO_Pin == USR_BTN_Pin && button_pressed == 0)
      {
	  	  button_pressed = 1;
	  	  printf("Button pressed...\r\n");
      }
}
/* ======================================================== */


/* Used to verify the state of the button */
void check_button_release()
{
    if (HAL_GPIO_ReadPin(GPIOI, USR_BTN_Pin) == GPIO_PIN_RESET) button_pressed = 0;
}
/* ======================================================== */


/* Used to initialize the SD card */
void SDCard_InitAndFormat(void) {
    FRESULT res;

    // Mount the file system
    res = f_mount(&SDFatFS, (TCHAR const *)SDPath, 0);
    if (res != FR_OK) {
        printf("Error: Failed to mount SD card (Code: %d).\r\n", res);
        Error_Handler();
    }

    // Format the SD card
    res = f_mkfs((TCHAR const *)SDPath, FM_ANY, 0, workBuffer_init, sizeof(workBuffer_init));
    if (res != FR_OK) {
        printf("Error: Failed to format SD card (Code: %d).\r\n", res);
        Error_Handler();
    }

    // Unmount the file system to complete formatting
    res = f_mount(NULL, (TCHAR const *)SDPath, 0);
    if (res != FR_OK) {
        printf("Error: Failed to unmount SD card after formatting (Code: %d).\r\n", res);
        Error_Handler();
    }



    printf("SD card initialized and formatted successfully.\r\n");
}
/* ======================================================== */


/* Used to create a new txt file */
void new_log(FIL *fp, const char *filename, const char *content){
	FRESULT res;
	uint32_t byteswritten;
	if(f_open(fp, filename, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
	{
		Error_Handler();
	}
	else
	{
		res = f_write(fp, content, strlen(content), (void *)&byteswritten);

		if((byteswritten == 0) || (res != FR_OK))
		{
			Error_Handler();
		}
		else
		{
			f_close(fp);
		}
	}
}
/* ======================================================== */


/* Used to create a new folder */
void new_folder(const char *foldername){
	if(f_mkdir(foldername) != FR_OK)
	{
		Error_Handler();
	}
	else
	{
		printf("Folder created!\r\n");
	}
}
/* ======================================================== */


/* Debug function : reads the characteristics of a .wav file in the SD card */
void ReadWAVFileInfo(const char *filename) {
    FIL file;               // File object
    WAV_Header header;      // WAV file header
    UINT bytesRead;         // Number of bytes read
    FRESULT res;

    // Open the WAV file
    res = f_open(&file, filename, FA_READ);
    if (res != FR_OK) {
        printf("Error: Failed to open file '%s' (Code: %d).\n", filename, res);
        return;
    }

    // Read the WAV file header
    res = f_read(&file, &header, sizeof(WAV_Header), &bytesRead);
    if (res != FR_OK || bytesRead != sizeof(WAV_Header)) {
        printf("Error: Failed to read WAV file header (Code: %d).\n", res);
        f_close(&file);
        return;
    }

    // Print WAV file information
    printf("WAV File Info:\r\n");
    printf("  ChunkID: %.4s\r\n", header.ChunkID);
    printf("  Format: %.4s\r\n", header.Format);
    printf("  Audio Format: %d\r\n", header.AudioFormat);
    printf("  Number of Channels: %d\r\n", header.NumChannels);
    printf("  Sample Rate: %d Hz\r\n", header.SampleRate);
    printf("  Byte Rate: %d\r\n", header.ByteRate);
    printf("  Block Align: %d\r\n", header.BlockAlign);
    printf("  Bits Per Sample: %d\r\n", header.BitsPerSample);
    printf("  Subchunk2ID: %.4s\r\n", header.Subchunk2ID);
    printf("  Subchunk2Size: %d bytes\r\n", header.Subchunk2Size);

    // Close the file
    f_close(&file);
}
/* ======================================================== */


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
