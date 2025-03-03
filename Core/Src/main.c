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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "waverecorder.h"
#include <stdio.h>


#include "ai_datatypes_defines.h"
#include "ai_platform.h"
#include "network.h"
#include "network_data.h"

#include "arm_math.h"  // For DSP functions like FFT
#include "dsp/transform_functions.h"

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
#define BUFFER_SIZE 16000  // 1 second of audio at 16kHz
#define FFT_SIZE 256  // Choose a suitable size based on your model's input
#define NUM_MEL_BINS 40  // Optional: if using Mel spectrograms
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
//ai_handle network;
//float aiInData[AI_NETWORK_IN_1_SIZE];
//float aiOutData[AI_NETWORK_OUT_1_SIZE];
//ai_u8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];
//const char* activities[AI_NETWORK_OUT_1_SIZE] = {
//  "down", "go", "left", "right", "stop", "up"
//};
//ai_buffer * ai_input;
//ai_buffer * ai_output;

/* Audio processing RELATED VARIABLES */
static uint16_t waveform[BUFFER_SIZE];
static uint16_t last_ffts[125];

const static uint32_t frame_step = 128;

static arm_rfft_fast_instance_f32 fft;

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
void ReadWAVFileInfo_fromSD(const char *filename);

void AI_Init(void);
int AI_Process(const float* input_data);

int read_wav_file(const char *filename, int16_t *buffer);

void arm_hanning_f32(float32_t * pDst, uint32_t blockSize);

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
	char buf[50];
	int buf_len = 0;
	uint32_t timestamp;
	float y_val;

	const char* activities[AI_NETWORK_OUT_1_SIZE] = {
	    "down", "go", "left", "right", "stop", "up"
	};

	// Buffers for input and output data
	AI_ALIGNED(4) float in_data[AI_NETWORK_IN_1_SIZE];
	AI_ALIGNED(4) float out_data[AI_NETWORK_OUT_1_SIZE];

	// Chunk of memory used to hold intermediate values for the neural network
	AI_ALIGNED(4) ai_u8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];

	// Neural network handle
	ai_handle network = AI_HANDLE_NULL;

	// Wrapper structs for input and output buffers
	ai_buffer ai_input[AI_NETWORK_IN_NUM];
	ai_buffer ai_output[AI_NETWORK_OUT_NUM];

	/**
	 * @brief Initialize the AI model
	 */
	void AI_Init(void) {
	    ai_error ai_err;

	    // Set working memory and get weights/biases from the model
	    ai_network_params ai_params = {
	        AI_NETWORK_DATA_WEIGHTS(ai_network_data_weights_get()),
	        AI_NETWORK_DATA_ACTIVATIONS(activations)
	    };

	    // Create and initialize the network
	    ai_err = ai_network_create(&network, AI_NETWORK_DATA_CONFIG);
	    if (ai_err.type != AI_ERROR_NONE) {
	        printf("AI network creation failed: type=%d, code=%d\r\n", ai_err.type, ai_err.code);
	        Error_Handler();
	    }

	    if (!ai_network_init(network, &ai_params)) {
	        ai_err = ai_network_get_error(network);
	        printf("AI network initialization failed: type=%d, code=%d\r\n", ai_err.type, ai_err.code);
	        Error_Handler();
	    }

	    // Retrieve input and output buffer structures dynamically
	    ai_network_inputs_get(network, ai_input);
	    ai_network_outputs_get(network, ai_output);

	    printf("AI model initialized successfully.\r\n");
	}


	/**
	 * @brief Process the AI model
	 * @param input_data Pointer to input data buffer
	 * @return Index of the predicted activity
	 */
	int AI_Process(const float* input_data) {
	    // Copy processed spectrogram (stored in waveform) to input buffer in_data
	    // Ensure input_data matches the shape expected by the AI model (for example, AI_NETWORK_IN_1_SIZE)
	    // Typically, you'll need to reshape or flatten your spectrogram data to match the input size of the model

	    memcpy(in_data, input_data, sizeof(in_data));  // Assuming input_data is the spectrogram of correct size

	    // Run the AI model (inference)
	    ai_i32 nbatch = ai_network_run(network, ai_input, ai_output);
	    if (nbatch != 1) {
	        ai_error ai_err = ai_network_get_error(network);
	        printf("AI model inference failed: type=%d, code=%d\r\n", ai_err.type, ai_err.code);
	        Error_Handler();
	    }

	    // Find the index of the predicted activity
	    float max_val = out_data[0];
	    int max_idx = 0;
	    for (int i = 1; i < AI_NETWORK_OUT_1_SIZE; i++) {
	        if (out_data[i] > max_val) {
	            max_val = out_data[i];
	            max_idx = i;
	        }
	    }

	    // Print predicted activity
	    printf("Predicted activity: %s (confidence: %.2f)\r\n", activities[max_idx], max_val);
	    return max_idx;
	}

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
  /* USER CODE BEGIN 2 */

  /* We format the SD card */
  printf("SD card init...\r\n");
  SDCard_InitAndFormat();

  AI_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t write_index = 0; // AI stuff
    while (1)
    {
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

            //ReadWAVFileInfo("WAVE.wav");

            /* Audio processing step*/

            // We read the contents of the file, save the info in the "audio_buffer" variable
            read_wav_file("WAVE.wav", waveform);

            printf("Shape of audio_buffer: (%u,)\r\n", sizeof(waveform) / sizeof(waveform[0]));

            // We create a fast fft instance (lookup)
        	if (arm_rfft_fast_init_f32(&fft, FFT_SIZE) != ARM_MATH_SUCCESS) {
        		printf("Failed to init RFFT");
        	}

        	// We create a hanning window, of size 256
            static float32_t hanning[FFT_SIZE];
        	arm_hanning_f32(hanning, FFT_SIZE);

        	printf("Normalizing audio\r\n");
        	// Normalisation de l'audio
        	float min = 32767.0f;  // Set min to the maximum positive value for 16-bit signed integer
        	float max = -32768.0f; // Set max to the minimum negative value for 16-bit signed integer

        	for (uint32_t i = 0; i < sizeof(waveform) / sizeof(waveform[0]); i++) {
        	    int16_t val = waveform[i];  // Directly access the int16_t sample
        	    if ((float)val < min) min = (float)val;  // Compare values and update min
        	    if ((float)val > max) max = (float)val;  // Compare values and update max
        	}
        	printf("Normalizing audio OK\r\n");

        	printf("Min value: %.2f, Max value: %.2f\r\n", min, max);

        	printf("Conversion, bias removal and hanning application\r\n");
        	// Conversion waveform to float (-1 to 1), Remove DC bias (mean subtraction), applies Hanning window
        	for (uint32_t idx = 0; idx < 124; idx++) {
        	    float dst[FFT_SIZE];
        	    static float mag[FFT_SIZE + 1];
        	    double sum = 0;
        	    static float* signal_chunk = mag;

        	    for (uint32_t i = 0; i < FFT_SIZE; i++) {
        	        signal_chunk[i] = (float)((uint16_t)waveform[idx * frame_step + i]);
        	        // Normalize from -1 to 1
        	        signal_chunk[i] = (2.0f * (signal_chunk[i] - min) / (max - min)) - 1;
        	        sum += signal_chunk[i];
        	    }

    			// Remove DC component
    			float mean = (float)(sum / (double)FFT_SIZE);
    			for (uint32_t i = 0; i < FFT_SIZE; i++) {
    				signal_chunk[i] = signal_chunk[i] - mean;

    				// Apply window function
    				signal_chunk[i] *= hanning[i];
    			}

    			// Compute FFT
    			arm_rfft_fast_f32(&fft, signal_chunk, dst, 0);

    			// The first "complex" is actually to reals, X[0] and X[N/2]
				float first_real = (dst[0] < 0.0f) ? (-1.0f * dst[0]) : dst[0];
				float second_real = (dst[1] < 0.0f) ? (-1.0f * dst[1]) : dst[1];

				// Take the magnitude for all the complex values in between
				arm_cmplx_mag_f32(dst + 2, mag + 1, FFT_SIZE / 2);

				// Fill in the two real numbers at 0 and N/2
				mag[0] = first_real;
				mag[128] = second_real;

				for (uint32_t i = 0; i < 129; i++) {
					// We can't override waveform[129 * idx + 128] yet
					// because we need it for the next iteration, so we need to store
					// it separately
					if (i < 128) {
						((uint16_t*)waveform)[128 * idx + i] = (uint8_t)(mag[i] * 8.0f);
					}
					else {
						last_ffts[idx] = (uint8_t)(mag[i] * 8.0f);
					}
				}
			}

			// Spectrogram formatting
			uint8_t* input_tensor = waveform;
			uint32_t input_tensor_len = 124 * 129;

			for (uint32_t idx = 123; idx > 0; idx--){
			    uint8_t tmp[128];
			    memcpy(tmp, waveform + (idx * 128), 128);
			    memcpy(waveform + (idx * 128 + idx), tmp, 128);
			    waveform[idx * 128 + (idx - 1)] = last_ffts[idx - 1];
        	}

//			for (uint32_t i = 0; i < input_tensor_len; i++){
//				printf("%u\r\n", input_tensor[i]);
//			}

			printf("Conversion, bias removal and hanning application OK\r\n");

			int predicted_word_index = AI_Process((float*)waveform);
			printf("Detected word: %s\r\n", activities[predicted_word_index]);

        }

        HAL_Delay(100);  // Small delay for stability
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
}
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
void ReadWAVFileInfo_fromSD(const char *filename) {
    FIL file;               // File object
    WAV_Header header;      // WAV file header
    UINT bytesRead;         // Number of bytes read
    FRESULT res;

    res = f_mount(&SDFatFS, (TCHAR const *)SDPath, 0);
    if (res != FR_OK) {
        printf("Error: Failed to mount SD card (Code: %d).\r\n", res);
        Error_Handler();
    }

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


/* Debug function : reads the characteristics of a .wav file in the SD card */
void ReadWAVFileInfo(const char *filename) {
    FIL file;               // File object
    WAV_Header header;      // WAV file header
    UINT bytesRead;         // Number of bytes read
    FRESULT res;

    SCB_DisableDCache();
    SCB_DisableICache();

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

//    SCB_EnableDCache();
//    SCB_EnableICache();

    // Close the file
    f_close(&file);
}
/* ======================================================== */

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
    result = f_read(&file, buffer, BUFFER_SIZE * sizeof(int16_t), &bytes_read);

    if (result != FR_OK) {
        f_close(&file);
        return -1;  // Read error
    }

    f_close(&file);
    printf("file successfully read! \r\n");
    return 0;  // Success
}


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
