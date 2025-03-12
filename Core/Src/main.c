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
#include "crc.h"
#include "dma.h"
#include "fatfs.h"
#include "sai.h"
#include "sdmmc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* General INCLUDES */
#include <stdio.h>
/* Voice recording INCLUDES */
#include "waverecorder.h"
/* Cube AI INCLUDES */
#include "ai_datatypes_defines.h"
#include "ai_platform.h"
#include "network.h"
#include "network_data.h"
/* Signal processing INCLUDES */
#include "arm_math.h"
#include "dsp/transform_functions.h"
/* Wireless transmission INCLUDES */
#include "MY_NRF24.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Signal processing INCLUDES */
#define BUFFER_SIZE 16000  // Same as sample rate, variables store 16000 values.
#define FFT_SIZE 256

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Audio recording VARIABLES */
AUDIO_ApplicationTypeDef appli_state = APPLICATION_IDLE;
AUDIO_PLAYBACK_StateTypeDef AudioState;
AUDIO_ErrorTypeDef  status;

/* SD card VARIABLES */
#define WORK_BUFFER_SIZE 512
BYTE workBuffer_init[WORK_BUFFER_SIZE];
extern FIL WavFile;
FRESULT res;
UINT bytesRead, bw;
FIL file;

/* AI model VARIABLES */
ai_handle network;
float aiInData[AI_NETWORK_IN_1_SIZE];
float aiOutData[AI_NETWORK_OUT_1_SIZE];
ai_u8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];
const char* activities[AI_NETWORK_OUT_1_SIZE] = {
	"down", "go", "left", "right", "stop", "up"
};
ai_buffer * ai_input;
ai_buffer * ai_output;

/* Audio processing VARIABLES */
static int16_t stereo_waveform[BUFFER_SIZE*2];
static int16_t waveform[BUFFER_SIZE] __attribute__((section(".sdram")));
static float float_waveform[BUFFER_SIZE] __attribute__((section(".sdram")));
static float spectrogram[124][129] __attribute__((section(".sdram")));
const static uint32_t frame_step = 128;
arm_rfft_fast_instance_f32 fft;

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

/* Needed to initialize the SD Card */
void SDCard_InitAndFormat(void);
int read_wav_file(const char *filename, int16_t *buffer);

/* Audio processing */
void arm_hanning_f32(float32_t * pDst, uint32_t blockSize);

/* AI functions */
static void AI_Init(void);
static void AI_Run(float *pIn, float *pOut);
static uint32_t argmax(const float * values, uint32_t len);
void softmax(float *values, uint32_t len);

/* Wireless transmission */
void HAL_GPIO_send_command(const char* command);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Wireless transmission RELATED VARIABLES */
volatile uint8_t button_pressed = 0;

uint64_t TxpipeAddrs = 0x11223344AA;
char myTxData[32] = "Hello world!";
char myRxData[32];

char droite[32] = "right";
char gauche[32] = "left";
char stop[32] = "stop";
char go[32] = "go";
char back[32] = "down";


volatile uint8_t button_pressed_right = 0;
volatile uint8_t button_pressed_left = 0;
volatile uint8_t button_pressed_stop = 0;
volatile uint8_t button_pressed_go = 0;
volatile uint8_t button_pressed_back = 0;
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
  MX_CRC_Init();
  MX_FMC_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  /* Init */
  printf("SD card init...\r\n");
  SDCard_InitAndFormat();
  BSP_SDRAM_Init();
  AI_Init();

  NRF24_begin(GPIOA, CSN_PIN_Pin, CE_PIN_Pin, hspi2);
  nrf24_DebugUART_Init(huart1);

  /* Set values of arrays to 0 for a clean start */
  memset(waveform, 0, sizeof(waveform));
  memset(stereo_waveform, 0, sizeof(stereo_waveform));
  memset(float_waveform, 0, sizeof(float_waveform));
  memset(spectrogram, 0, sizeof(spectrogram));

  /* Wireless module config */

  /* Good practice : in case we were in receive mode just before */
  NRF24_stopListening();
  /* We open the writing pipe (hex address) */
  NRF24_openWritingPipe(TxpipeAddrs);
  /* We turn off ack and select a channel 0 - 127 */
  NRF24_setAutoAck(false);
  NRF24_setChannel(52);
  /* We set a payload size of 32 bytes   (max) */
  NRF24_setPayloadSize(32);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    	/* We wait for the user button on the Discovery board to be pressed (BLUE BUTTON) */
        check_button_release();
        printf("Waiting for input to record...\r\n");
        HAL_Delay(1000);

        SCB_DisableDCache();
        SCB_DisableICache();

        /* If the button is pressed, execute main program */
        if (button_pressed == 1)
        {
            /* If the program is not already recording... */
            if (AudioState == AUDIO_STATE_IDLE)
            {
                /* Configure the audio recorder: sampling frequency, bits-depth, number of channels */
                AUDIO_REC_Start();
            }

            /* While recording, we loop the recording process */
            while (AudioState == AUDIO_STATE_RECORD)
            {
                status = AUDIO_REC_Process();
            }

            /* Once we stop recording, we correctly close the .WAV */
            if (AudioState == AUDIO_STATE_STOP)
            {
                status = AUDIO_REC_Process();
                printf("Recording stopped.\r\n");
            }

            /* Audio processing step*/

            	/* We read the .wav file stored in the SD card and assign its content to an array */
				if (read_wav_file("WAVE.WAV", stereo_waveform) != 0) {
					printf("ERROR : cannot read .wav file\r\n");
					return;
				}

				/* Stereo to mono audio transformation */
				for (uint32_t i = 0; i < BUFFER_SIZE; i++) {
					waveform[i] = (stereo_waveform[2 * i] + stereo_waveform[2 * i + 1]) / 2;  // Moyenne des deux canaux
				}


				// -------------------- NORMALIZATION --------------------
				printf("Audio normalization...\r\n");

				/* Find min and max */

				float min_val = 32767.0f;
				float max_val = -32768.0f;

				for (uint32_t i = 0; i < BUFFER_SIZE; i++) {
					if (waveform[i] < min_val) min_val = waveform[i];
					if (waveform[i] > max_val) max_val = waveform[i];
				}

				/* Display the found values */
				printf("Min: %.2f, Max: %.2f\n", min_val, max_val);

				/* Normalization process */
				float range = max_val - min_val;
				if (range == 0) range = 1.0f;  // To not divide by 0

				for (uint32_t i = 0; i < BUFFER_SIZE; i++) {
					float_waveform[i] = 2.0f * (waveform[i] - min_val) / range - 1.0f; // Normalization [-1,1]
				}

				/* Display some values */
				printf("Premières valeurs normalisées : ");
				for (uint32_t i = 0; i < 10; i++) {
					printf("%.6f ", float_waveform[i]);
				}
				printf("\r\n");

				// -------------------- HANNING & FFT --------------------

				/* Hanning window creation */
				static float32_t hanning_window[FFT_SIZE];
				arm_hanning_f32(hanning_window, FFT_SIZE);

				for (uint32_t idx = 0; idx < 124; idx++) {
					float frame[FFT_SIZE];
					float mag[FFT_SIZE / 2 + 1]; // Complex values magnitude
					float sum = 0.0f;

					/* Hanning extraction and application */
					for (uint32_t i = 0; i < FFT_SIZE; i++) {
						frame[i] = float_waveform[idx * frame_step + i] * hanning_window[i];
						sum += frame[i];
					}

					/* Suppression of the DC bias */
					float mean = sum / FFT_SIZE;
					for (uint32_t i = 0; i < FFT_SIZE; i++) {
						frame[i] -= mean;
					}

					/* Declare RFFT */
					if (arm_rfft_fast_init_f32(&fft, FFT_SIZE) != ARM_MATH_SUCCESS) {
						printf("Erreur : Échec de l'initialisation de la FFT !\r\n");
						Error_Handler();
					}

					/* Compute RFFT */
					float dst[FFT_SIZE];
					arm_rfft_fast_f32(&fft, frame, dst, 0);

					/* Processing of the complex values magnitudes */
					arm_cmplx_mag_f32(dst, mag, FFT_SIZE / 2 + 1);

					/* Fill the spectrogram array */
					for (uint32_t i = 0; i < FFT_SIZE / 2 + 1; i++) {
						spectrogram[idx][i] = mag[i];
					}
				}

				// -------------------- SD card save --------------------

//				printf("Saving spectrogram...\r\n");
//
//				res = f_open(&file, "data.txt", FA_WRITE | FA_CREATE_ALWAYS);
//				if (res == FR_OK) {
//				    f_write(&file, "[\n", 2, &bw);
//				    char buffer[32];
//
//				    for (uint32_t i = 0; i < 124; i++) {
//				        f_write(&file, " [", 2, &bw);
//				        for (uint32_t j = 0; j < 129; j++) {
//				            sprintf(buffer, "%.8f", spectrogram[i][j]);
//				            f_write(&file, buffer, strlen(buffer), &bw);
//
//				            if (j < 128) {  // Add space between values but no comma
//				                f_write(&file, " ", 1, &bw);
//				            }
//				        }
//				        f_write(&file, "]", 1, &bw);
//
//				        if (i < 123) { // New line for the next row except the last one
//				            f_write(&file, "\n", 1, &bw);
//				        }
//				    }
//
//				    f_write(&file, "\n]", 2, &bw);
//				    f_close(&file);
//				    printf("Saving success !\r\n");
//				} else {
//				    printf("Saving FAILED !\r\n");
//				}

				// -------------------- AI format preparation --------------------

				/* Fit spectrogram into aiInData */
				for (uint32_t i = 0; i < 124; i++) {
					for (uint32_t j = 0; j < FFT_SIZE / 2 + 1; j++) {
						aiInData[i * (FFT_SIZE / 2 + 1) + j] = spectrogram[i][j];
					}
				}

				/* Vérification avant passage au modèle */
				printf("Premières valeurs envoyées au modèle : ");
				for (uint32_t i = 0; i < 10; i++) {
					printf("%.6f ", aiInData[i]);
				}
				printf("\r\n");

				// -------------------- INFERENCE --------------------
				printf("Inference...\r\n");
				AI_Run(aiInData, aiOutData);

				// -------------------- SOFTMAX AND PREDICTION --------------------
				softmax(aiOutData, AI_NETWORK_OUT_1_SIZE);

				/* Check sum of probabilities */
				float sum_softmax = 0;
				for (uint32_t i = 0; i < AI_NETWORK_OUT_1_SIZE; i++) {
					sum_softmax += aiOutData[i];
				}
				printf("Softmax probability sum : %f\r\n", sum_softmax);

				/* Pick the class with the highest probability */
				uint32_t class_idx = argmax(aiOutData, AI_NETWORK_OUT_1_SIZE);
				printf("Detected command : %s (Confidence : %.2f%%)\r\n", activities[class_idx], aiOutData[class_idx] * 100);

				/* Map the detected word to the correct command */
				const char* command_to_send = NULL;

				if (strcmp(activities[class_idx], "down") == 0) {
				    command_to_send = back;
				    button_pressed_back = 0;
				} else if (strcmp(activities[class_idx], "go") == 0) {
				    command_to_send = go;
				    button_pressed_go = 0;
				} else if (strcmp(activities[class_idx], "left") == 0) {
				    command_to_send = gauche;
				    button_pressed_left = 0;
				} else if (strcmp(activities[class_idx], "right") == 0) {
				    command_to_send = droite;
				    button_pressed_right = 0;
				} else if (strcmp(activities[class_idx], "stop") == 0) {
				    command_to_send = stop;
				    button_pressed_stop = 0;
				} else if (strcmp(activities[class_idx], "up") == 0) {
				    command_to_send = stop;
				    button_pressed_stop = 0;
				}

				/* Send the mapped command */
				if (command_to_send != NULL) {
				    HAL_GPIO_send_command(command_to_send);
				}

				}
        }

        HAL_Delay(100);  // Small delay for stability
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */


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


/* Used to verify the state of the button */
void check_button_release()
{
    if (HAL_GPIO_ReadPin(GPIOI, USR_BTN_Pin) == GPIO_PIN_RESET) button_pressed = 0;
}


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

/* Used to read the wav file in the SD card and store its content inside an array */
int read_wav_file(const char *filename, int16_t *buffer) {
    FIL file;
    UINT bytes_read;

    FRESULT result = f_open(&file, filename, FA_READ);

    if (result != FR_OK) {
        return -1;  // File open error
    }

    // Skip WAV header (44 bytes)
    f_lseek(&file, 44);

    // Read audio samples into the buffer
    result = f_read(&file, buffer, (BUFFER_SIZE*2) * sizeof(int16_t), &bytes_read);

    if (result != FR_OK) {
        f_close(&file);
        return -1;  // Read error
    }

    f_close(&file);
    printf("file successfully read! \r\n");
    return 0;  // Success
}

/* Used for signal processing : Hanning window formula */
void arm_hanning_f32(float32_t * pDst, uint32_t blockSize) {
   float32_t k = 2.0f / ((float32_t) blockSize);
   float32_t w;

   for(uint32_t i=0;i<blockSize;i++)
   {
     w = PI * i * k;
     w = 0.5f * (1.0f - cosf (w));
     pDst[i] = w;
   }
}

/* Used for initializing the AI model */
static void AI_Init(void)
{
  ai_error err;

  /* Create a local array with the addresses of the activations buffers */
  const ai_handle act_addr[] = { activations };
  /* Create an instance of the model */
  err = ai_network_create_and_init(&network, act_addr, NULL);
  if (err.type != AI_ERROR_NONE) {
    printf("ai_network_create error - type=%d code=%d\r\n", err.type, err.code);
    Error_Handler();
  }
  ai_input = ai_network_inputs_get(network, NULL);
  ai_output = ai_network_outputs_get(network, NULL);
}

/* Used to send an input to the AI model, and get an answer */
static void AI_Run(float *pIn, float *pOut)
{
  ai_i32 batch;
  ai_error err;

  /* Update IO handlers with the data payload */
  ai_input[0].data = AI_HANDLE_PTR(pIn);
  ai_output[0].data = AI_HANDLE_PTR(pOut);

  batch = ai_network_run(network, ai_input, ai_output);
  if (batch != 1) {
    err = ai_network_get_error(network);
    printf("AI ai_network_run error - type=%d code=%d\r\n", err.type, err.code);
    Error_Handler();
  }
}

/* Used to pick the highest probability result of the AI model */
static uint32_t argmax(const float * values, uint32_t len)
{
  float max_value = values[0];
  uint32_t max_index = 0;
  for (uint32_t i = 1; i < len; i++) {
    if (values[i] > max_value) {
      max_value = values[i];
      max_index = i;
    }
  }
  return max_index;
}

/* Used for probability calculation */
void softmax(float *values, uint32_t len) {
    // Find the maximum value in the logits for numerical stability
    float max_val = values[0];
    for (uint32_t i = 1; i < len; i++) {
        if (values[i] > max_val) {
            max_val = values[i];
        }
    }

    // Subtract the max value from all logits to prevent overflow/underflow
    for (uint32_t i = 0; i < len; i++) {
        values[i] -= max_val;
    }

    // Compute the sum of exponentiated values
    float sum = 0.0f;
    for (uint32_t i = 0; i < len; i++) {
        values[i] = expf(values[i]);  // Exponentiate each value
        sum += values[i];             // Sum the exponentiated values
    }
}

/* Used for message transmission */
void HAL_GPIO_send_command(const char* command) // Fonction de transmission des commandes
{
    if (strcmp(command, droite) == 0 && button_pressed_right == 0) {
        button_pressed_right = 1;

		if (NRF24_write(droite, 32) == 1) {
			printf("Transmission success: command %s\r\n", command);
		} else {
			printf("Transmission failed for command %s\r\n", command);
		}

    } else if (strcmp(command, gauche) == 0 && button_pressed_left == 0) {
        button_pressed_left = 1;

        // Send the command using NRF24
		if (NRF24_write(gauche, 32) == 1) {
			printf("Transmission success: command %s\r\n", command);
		} else {
			printf("Transmission failed for command %s\r\n", command);
		}

    } else if (strcmp(command, stop) == 0 && button_pressed_stop == 0) {
        button_pressed_stop = 1;

		if (NRF24_write(stop, 32) == 1) {
			printf("Transmission success: command %s\r\n", command);
		} else {
			printf("Transmission failed for command %s\r\n", command);
		}

    } else if (strcmp(command, go) == 0 && button_pressed_go == 0) {
        button_pressed_go = 1;

		if (NRF24_write(go, 32) == 1) {
			printf("Transmission success: command %s\r\n", command);
		} else {
			printf("Transmission failed for command %s\r\n", command);
		}
    } else if (strcmp(command, back) == 0 && button_pressed_back == 0) {
        button_pressed_back = 1;
		if (NRF24_write(back, 32) == 1) {
			printf("Transmission success: command %s\r\n", command);
		} else {
			printf("Transmission failed for command %s\r\n", command);
		}
    } else {
        return; // Command not recognized, do nothing
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
