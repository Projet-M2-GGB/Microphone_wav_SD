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
#include "fmc.h"

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
UINT bytesRead, bw;
WAV_Header header;
FIL file;
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

/* Audio processing RELATED VARIABLES */

static int16_t stereo_waveform[BUFFER_SIZE*2];
static int16_t waveform[BUFFER_SIZE] __attribute__((section(".sdram")));
static float float_waveform[BUFFER_SIZE] __attribute__((section(".sdram")));
static float spectrogram[124][129] __attribute__((section(".sdram")));


static uint16_t last_ffts[125];

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
/* =========================================================*/

/* Needed to initialize the SD Card */
void SDCard_InitAndFormat(void);

/* Needed to create a txt in the SD Card */
void new_log(FIL *fp, const char *filename, const char *content);

/* Needed to create a folder in the SD Card */
void new_folder(const char *foldername);

void ReadWAVFileInfo(const char *filename);
void ReadWAVFileInfo_fromSD(const char *filename);

int read_wav_file(const char *filename, int16_t *buffer);

void arm_hanning_f32(float32_t * pDst, uint32_t blockSize);

static void AI_Init(void);
static void AI_Run(float *pIn, float *pOut);
static uint32_t argmax(const float * values, uint32_t len);
void softmax(float *values, uint32_t len);
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
  MX_CRC_Init();
  MX_FMC_Init();

  /* USER CODE BEGIN 2 */

  /* We format the SD card */
  printf("SD card init...\r\n");
  SDCard_InitAndFormat();
  BSP_SDRAM_Init();
  AI_Init();

  memset(waveform, 0, sizeof(waveform));
  memset(stereo_waveform, 0, sizeof(stereo_waveform));
  memset(float_waveform, 0, sizeof(float_waveform));
  memset(spectrogram, 0, sizeof(spectrogram));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  	uint32_t write_index = 0;
    while (1)
    {
        check_button_release();
        printf("Waiting for input to record...\r\n");
        HAL_Delay(1000);

        SCB_DisableDCache();
        SCB_DisableICache();

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

//            // We read the contents of the file, save the info in the "audio_buffer" variable
//            read_wav_file("WAVE.wav", stereo_waveform);
//            //Début traitement

				// We read the contents of the file, save the info in the "audio_buffer" variable
				// Charger le fichier WAV et vérifier mono/stéréo
				// Charger le fichier WAV et vérifier mono/stéréo
				if (read_wav_file("WAVE.WAV", stereo_waveform) != 0) {
					printf("Erreur : Impossible de lire le fichier WAV\r\n");
					return;
				}

				// Si le fichier est stéréo, on le convertit en mono en moyennant les canaux

				for (uint32_t i = 0; i < BUFFER_SIZE; i++) {
					waveform[i] = (stereo_waveform[2 * i] + stereo_waveform[2 * i + 1]) / 2;  // Moyenne des deux canaux
				}


				// -------------------- NORMALISATION --------------------
				printf("Normalisation de l'audio...\r\n");

				// Trouver le min et le max
				float min_val = 32767.0f;
				float max_val = -32768.0f;

				for (uint32_t i = 0; i < BUFFER_SIZE; i++) {
					if (waveform[i] < min_val) min_val = waveform[i];
					if (waveform[i] > max_val) max_val = waveform[i];
				}

				// Vérifier que les valeurs sont valides
				printf("Min: %.2f, Max: %.2f\n", min_val, max_val);

				// Calcul de la normalisation
				float range = max_val - min_val;
				if (range == 0) range = 1.0f;  // Éviter division par zéro

				for (uint32_t i = 0; i < BUFFER_SIZE; i++) {
					float_waveform[i] = 2.0f * (waveform[i] - min_val) / range - 1.0f; // Normalisation [-1,1]
				}

				// Vérification des valeurs normalisées
				printf("Premières valeurs normalisées : ");
				for (uint32_t i = 0; i < 10; i++) {
					printf("%.6f ", float_waveform[i]);
				}
				printf("\r\n");

				// -------------------- APPLICATION HANNING & FFT --------------------

				// Création de la fenêtre de Hanning
				static float32_t hanning_window[FFT_SIZE];
				arm_hanning_f32(hanning_window, FFT_SIZE);

				printf("Application de la fenêtre de Hanning et calcul FFT...;\r\n");

				for (uint32_t idx = 0; idx < 124; idx++) {
					float frame[FFT_SIZE];
					float mag[FFT_SIZE / 2 + 1]; // Magnitude des valeurs complexes
					float sum = 0.0f;

					// Extraction et application de Hanning
					for (uint32_t i = 0; i < FFT_SIZE; i++) {
						frame[i] = float_waveform[idx * frame_step + i] * hanning_window[i];
						sum += frame[i];
					}

					// Suppression du biais DC
					float mean = sum / FFT_SIZE;
					for (uint32_t i = 0; i < FFT_SIZE; i++) {
						frame[i] -= mean;
					}


					if (arm_rfft_fast_init_f32(&fft, FFT_SIZE) != ARM_MATH_SUCCESS) {
						printf("Erreur : Échec de l'initialisation de la FFT !\r\n");
						Error_Handler();
					}


					// Calcul FFT
					float dst[FFT_SIZE];
					arm_rfft_fast_f32(&fft, frame, dst, 0);

					// Calcul de la magnitude des valeurs complexes
					arm_cmplx_mag_f32(dst, mag, FFT_SIZE / 2 + 1);

					// Stockage dans le spectrogramme
					for (uint32_t i = 0; i < FFT_SIZE / 2 + 1; i++) {
						spectrogram[idx][i] = mag[i];
					}
				}

				// -------------------- NORMALISATION DU SPECTROGRAMME --------------------

				printf("Normalisation du spectrogramme...\r\n");
//				float min_spec = 1e6, max_spec = -1e6;
//
//				// Trouver min et max du spectrogramme
//				for (uint32_t i = 0; i < 124; i++) {
//					for (uint32_t j = 0; j < FFT_SIZE / 2 + 1; j++) {
//						if (spectrogram[i][j] < min_spec) min_spec = spectrogram[i][j];
//						if (spectrogram[i][j] > max_spec) max_spec = spectrogram[i][j];
//					}
//				}
//
//				float spec_range = max_spec - min_spec;
//				if (spec_range == 0) spec_range = 1.0f;
//
//				// Appliquer la normalisation
//				for (uint32_t i = 0; i < 124; i++) {
//					for (uint32_t j = 0; j < FFT_SIZE / 2 + 1; j++) {
//						spectrogram[i][j] = (spectrogram[i][j] - min_spec) / spec_range;
//					}
//				}

				// Vérification des valeurs normalisées
				printf("Premières valeurs normalisées du spectrogramme : ");
				for (uint32_t i = 0; i < 10; i++) {
					printf("%.6f ", spectrogram[0][i]);
				}
				printf("\r\n");

				// -------------------- SAUVEGARDE SUR SD --------------------

				printf("Sauvegarde du spectrogramme...\r\n");

				res = f_open(&file, "data.txt", FA_WRITE | FA_CREATE_ALWAYS);
				if (res == FR_OK) {
				    f_write(&file, "[\n", 2, &bw);
				    char buffer[32];

				    for (uint32_t i = 0; i < 124; i++) {
				        f_write(&file, " [", 2, &bw);
				        for (uint32_t j = 0; j < 129; j++) {
				            sprintf(buffer, "%.8f", spectrogram[i][j]);
				            f_write(&file, buffer, strlen(buffer), &bw);

				            if (j < 128) {  // Add space between values but no comma
				                f_write(&file, " ", 1, &bw);
				            }
				        }
				        f_write(&file, "]", 1, &bw);

				        if (i < 123) { // New line for the next row except the last one
				            f_write(&file, "\n", 1, &bw);
				        }
				    }

				    f_write(&file, "\n]", 2, &bw);
				    f_close(&file);
				    printf("Sauvegarde réussie !\r\n");
				} else {
				    printf("Échec de la sauvegarde du spectrogramme !\r\n");
				}

				// -------------------- PRÉPARATION POUR L'IA --------------------

				// Mise à plat du spectrogramme dans aiInData
				for (uint32_t i = 0; i < 124; i++) {
					for (uint32_t j = 0; j < FFT_SIZE / 2 + 1; j++) {
						aiInData[i * (FFT_SIZE / 2 + 1) + j] = spectrogram[i][j];
					}
				}

				// Vérification avant passage au modèle
				printf("Premières valeurs envoyées au modèle : ");
				for (uint32_t i = 0; i < 10; i++) {
					printf("%.6f ", aiInData[i]);
				}
				printf("\r\n");

				// -------------------- INFÉRENCE AVEC IA --------------------
				printf("Exécution de l'inférence...\r\n");
				AI_Run(aiInData, aiOutData);

				// -------------------- SOFTMAX ET PRÉDICTION --------------------
				softmax(aiOutData, AI_NETWORK_OUT_1_SIZE);

				// Vérifier la somme des probabilités
				float sum_softmax = 0;
				for (uint32_t i = 0; i < AI_NETWORK_OUT_1_SIZE; i++) {
					sum_softmax += aiOutData[i];
				}
				printf("Somme des probabilités Softmax : %f\r\n", sum_softmax);

				// Trouver la classe avec la probabilité max
				uint32_t class_idx = argmax(aiOutData, AI_NETWORK_OUT_1_SIZE);
				printf("Mot détecté : %s (Confiance : %.2f%%)\r\n", activities[class_idx], aiOutData[class_idx] * 100);
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

//    FRESULT res = f_mount(&SDFatFS, (TCHAR const *)SDPath, 0);
//    if (res != FR_OK) {
//        printf("Error: Failed to mount SD card (Code: %d).\r\n", res);
//        Error_Handler();
//    }

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

//    // Normalize by dividing each value by the sum to get probabilities
//    for (uint32_t i = 0; i < len; i++) {
//        values[i] /= sum;
//    }
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
